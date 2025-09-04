# main_calibration.py

import numpy as np
import cv2
import time
import os
from sklearn.linear_model import RANSACRegressor
import scipy.optimize as optimize

# Attempt to import the Royale SDK library
try:
    import roypy
except ImportError:
    print("Error: The 'roypy' library was not found.")
    print("Please make sure the Royale SDK is installed and the Python bindings are in your PYTHONPATH.")
    exit()

# --- 1. Camera Handling Class ---

class Camera(roypy.IDepthDataListener):
    """
    A class to handle capturing depth data from the Pico Flexx camera.
    """
    def __init__(self):
        super().__init__()
        self.frame = None
        self.lock = False
        self.cam = None

    def onNewData(self, data):
        if not self.lock:
            self.lock = True
            # The data from the SDK is a 1D array, reshape it
            p = data.points
            self.frame = np.reshape(p, (data.height, data.width, 3))
            self.lock = False

    def open(self):
        # This will open the first camera found
        self.cam = roypy.CameraManager().createCamera()
        self.cam.registerDataListener(self)
        self.cam.startCapture()
        print("Camera capture started.")
        # Allow some time for the camera to start streaming
        time.sleep(1)

    def close(self):
        if self.cam:
            self.cam.stopCapture()
            print("Camera capture stopped.")

    def get_frame(self):
        while self.frame is None:
            time.sleep(0.01)
        # Return the Z-channel (depth) in meters
        return self.frame[:, :, 2]

# --- 2. Core Calibration Functions ---

def deproject_to_3d(depth_frame, intrinsics):
    """
    Converts a depth frame to a 3D point cloud using camera intrinsics.
    
    Args:
        depth_frame (np.array): The 2D depth image.
        intrinsics (dict): Dictionary with fx, fy, cx, cy.

    Returns:
        np.array: An array of (X, Y, Z) points.
    """
    fx, fy = intrinsics['fx'], intrinsics['fy']
    cx, cy = intrinsics['cx'], intrinsics['cy']
    
    height, width = depth_frame.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    
    Z = depth_frame
    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy
    
    # Filter out invalid depth points (Z=0)
    valid_mask = Z > 0
    points_3d = np.dstack((X, Y, Z))[valid_mask]
    
    return points_3d

def fit_plane_ransac(points_3d):
    """
    Fits a plane to a 3D point cloud using RANSAC.
    The plane equation is ax + by + cz + d = 0.
    
    Args:
        points_3d (np.array): The Nx3 point cloud.

    Returns:
        tuple: The plane parameters (a, b, c, d). Returns None if fitting fails.
    """
    if len(points_3d) < 100: # Need enough points to fit a plane
        return None

    # We want to predict 'z' from 'x' and 'y'
    X_data = points_3d[:, :2]
    y_data = points_3d[:, 2]

    ransac = RANSACRegressor(min_samples=100, max_trials=100, residual_threshold=0.01)
    
    try:
        ransac.fit(X_data, y_data)
    except ValueError:
        return None

    # Get the plane coefficients from the fitted model
    # The model gives z = coef[0]*x + coef[1]*y + intercept
    # So, the plane equation is: coef[0]*x + coef[1]*y - 1*z + intercept = 0
    a, b = ransac.estimator_.coef_
    d = ransac.estimator_.intercept_
    c = -1.0
    
    # Normalize the plane equation
    norm = np.sqrt(a**2 + b**2 + c**2)
    return a/norm, b/norm, c/norm, d/norm

def cost_function(params, data, initial_intrinsics):
    """
    The cost function to be minimized by the optimizer.
    It calculates the total point-to-plane distance error.
    """
    # Unpack parameters
    fx, fy, cx, cy, k1, k2, c1, c2 = params
    
    height, width = data[0]['depth_frame'].shape
    
    total_error = []

    for sample in data:
        depth_frame = sample['depth_frame']
        plane_params = sample['plane_params']
        
        # 1. Apply depth correction
        # d_true = d_meas + c1*d_meas^2 + c2*d_meas^3
        corrected_depth = depth_frame + c1 * (depth_frame**2) + c2 * (depth_frame**3)
        
        # 2. De-project to 3D using the current guess of intrinsics
        # Note: A full de-projection would also undistort the (u,v) coordinates.
        # For simplicity in this script, we apply a simplified model.
        # A more complex model would use cv2.undistortPoints here.
        
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        # Normalize coordinates
        x_norm = (u - cx) / fx
        y_norm = (v - cy) / fy
        
        # Apply radial distortion
        r2 = x_norm**2 + y_norm**2
        dist_factor = (1 + k1 * r2 + k2 * r2**2)
        
        x_distorted = x_norm * dist_factor
        y_distorted = y_norm * dist_factor
        
        # Compute 3D points
        X = x_distorted * corrected_depth
        Y = y_distorted * corrected_depth
        Z = corrected_depth
        
        valid_mask = Z > 0
        points_3d = np.dstack((X, Y, Z))[valid_mask]
        
        # 3. Calculate point-to-plane distance
        # dist = |ax + by + cz + d|
        a, b, c, d = plane_params
        distances = np.abs(points_3d @ np.array([a, b, c]) + d)
        
        total_error.extend(distances)
        
    # Return a large error value if something went wrong
    if not total_error:
        return [1e6] * len(data) * 1000

    return np.array(total_error)


# --- 3. Main Execution Block ---

def main():
    """Main function to run the calibration workflow."""
    
    # --- Part A: Data Capture ---
    print("--- ToF Intrinsic Calibration ---")
    
    output_dir = "calibration_data"
    os.makedirs(output_dir, exist_ok=True)
    
    capture_data = []
    
    print("\nStep 1: Data Capture")
    print("Point the camera at a flat wall.")
    
    try:
        camera = Camera()
        camera.open()
        
        while True:
            dist_str = input("Enter the measured distance to the wall in meters (or 'q' to finish): ")
            if dist_str.lower() == 'q':
                break
            
            try:
                distance = float(dist_str)
            except ValueError:
                print("Invalid input. Please enter a number.")
                continue

            print("Capturing frame... Press 's' to save, any other key to retry.")
            while True:
                depth_frame = camera.get_frame()
                # Display the depth frame for visualization
                depth_display = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                depth_display = cv2.applyColorMap(depth_display, cv2.COLORMAP_JET)
                cv2.imshow("Depth View", depth_display)
                key = cv2.waitKey(10) & 0xFF
                
                if key == ord('s'):
                    filename = os.path.join(output_dir, f"depth_{distance:.2f}m_{time.time():.0f}.npy")
                    np.save(filename, depth_frame)
                    capture_data.append({'distance': distance, 'file': filename})
                    print(f"Saved to {filename}")
                    break
                elif key != 255: # Any other key press
                    print("Retrying capture...")

    finally:
        camera.close()
        cv2.destroyAllWindows()

    if not capture_data:
        print("No data was captured. Exiting.")
        return

    # --- Part B: Data Processing & Plane Fitting ---
    print("\nStep 2: Processing captured data and fitting planes...")
    
    # Use default intrinsics for initial plane fitting.
    # IMPORTANT: You should find these for your specific camera model,
    # but these are reasonable starting points for the Pico Flexx.
    initial_intrinsics = {'fx': 225.0, 'fy': 225.0, 'cx': 112.0, 'cy': 84.0}
    
    processed_data = []
    for sample in capture_data:
        depth_frame = np.load(sample['file'])
        points_3d = deproject_to_3d(depth_frame, initial_intrinsics)
        plane_params = fit_plane_ransac(points_3d)
        
        if plane_params is not None:
            processed_data.append({
                'depth_frame': depth_frame,
                'plane_params': plane_params
            })
            print(f"Successfully fitted plane for {os.path.basename(sample['file'])}")
        else:
            print(f"Warning: Could not fit plane for {os.path.basename(sample['file'])}. Skipping.")
    
    if len(processed_data) < 5:
        print("Error: Not enough valid planes were fitted. Need at least 5 captures. Exiting.")
        return
        
    # --- Part C: Optimization ---
    print("\nStep 3: Running non-linear optimization to find parameters...")
    
    # Initial guess for all parameters [fx, fy, cx, cy, k1, k2, c1, c2]
    # k1, k2 are lens distortion (start at 0)
    # c1, c2 are depth distortion (start at 0)
    initial_params = np.array([
        initial_intrinsics['fx'],
        initial_intrinsics['fy'],
        initial_intrinsics['cx'],
        initial_intrinsics['cy'],
        0.0, 0.0, # k1, k2
        0.0, 0.0  # c1, c2
    ])
    
    # Run the optimizer
    result = optimize.least_squares(
        cost_function,
        initial_params,
        args=(processed_data, initial_intrinsics),
        verbose=2,
        method='trf'
    )
    
    optimized_params = result.x
    
    # --- Part D: Display Results ---
    print("\n--- Calibration Complete! ---")
    param_names = ['fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'c1 (depth)', 'c2 (depth)']
    
    print("\nOptimized Intrinsic Parameters:")
    for name, val in zip(param_names, optimized_params):
        print(f"  {name:<12}: {val:.6f}")
        
    # Save results to a file
    output_file = "tof_intrinsics.yml"
    fs = cv2.FileStorage(output_file, cv2.FILE_STORAGE_WRITE)
    fs.write("fx", optimized_params[0])
    fs.write("fy", optimized_params[1])
    fs.write("cx", optimized_params[2])
    fs.write("cy", optimized_params[3])
    fs.write("k1", optimized_params[4])
    fs.write("k2", optimized_params[5])
    fs.write("c1_depth", optimized_params[6])
    fs.write("c2_depth", optimized_params[7])
    fs.release()
    print(f"\nResults saved to '{output_file}'")


if __name__ == "__main__":
    main()