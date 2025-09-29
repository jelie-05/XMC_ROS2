#!/usr/bin/env python3
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

# Make Royale SDK findable even from ROS/VS Code
RR = os.getenv("ROYALE_ROOT", os.path.expanduser("~/Documents/libroyale-4.24.0.1201-LINUX-x86-64Bit"))
sys.path.append(f"{RR}/build-pywrap")
sys.path.append(f"{RR}/build-pywrap/bin")
os.environ["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH", "") + f":{RR}/bin"

import roypy  # noqa: E402


def _num_points(frame):
    fn = getattr(frame, "getNumPoints", None)
    if callable(fn):
        try:
            return int(fn())
        except Exception:
            pass
    n = getattr(frame, "npoints", None)
    if isinstance(n, int):
        return n
    # fallback: width*height if available
    w = getattr(frame, "width", None)
    h = getattr(frame, "height", None)
    if isinstance(w, int) and isinstance(h, int):
        return w * h
    return 0


class Listener(roypy.IDepthDataListener):
    def __init__(self, node: Node):
        super().__init__()
        self.n = node
        self.pub_depth = node.create_publisher(Image, "royale/depth_image", 10)
        self.pub_cloud = node.create_publisher(PointCloud2, "royale/points", 10)
        self.pub_gray  = node.create_publisher(Image, "royale/gray_image", 10)
        self.frame_id = node.declare_parameter("frame_id", "royale_optical_frame").get_parameter_value().string_value
        self.publish_cloud = node.declare_parameter("publish_cloud", True).get_parameter_value().bool_value
        self._printed_frame_info = False

    def onNewData(self, frame):
        try:
            # # Debug print only once
            # if not self._printed_frame_info:
            #     self._printed_frame_info = True
            #     print("Frame type:", type(frame))
            #     print("Frame attributes/methods:", dir(frame))
            #     print("Callable methods:", [m for m in dir(frame) if callable(getattr(frame, m))])
                
            # Some SDKs expose hasDepth()
            has_depth = True
            hd = getattr(frame, "hasDepth", None)
            if callable(hd):
                try:
                    has_depth = bool(hd())
                except Exception:
                    has_depth = True
            if not has_depth:
                return

            # width/height are attributes in your SDK
            w = int(getattr(frame, "width", 0))
            h = int(getattr(frame, "height", 0))
            npts = _num_points(frame)
            if npts <= 0 or w <= 0 or h <= 0:
                self.n.get_logger().warn("Invalid frame dims/points; skipping")
                return

            # Ensure consistency: if npts != w*h, we still publish depth as 1D padded/truncated
            # but normally npts should equal w*h for image layout.
            depth_vals = [0.0] * npts
            pts = []

            getX = frame.getX
            getY = frame.getY
            getZ = frame.getZ
            getConf = getattr(frame, "getDepthConfidence", None)

            for i in range(npts):
                # Per-index getters provided by your SDK
                x = float(getX(i))
                y = float(getY(i))
                z = float(getZ(i))
                conf = float(getConf(i)) if callable(getConf) else 1.0

                depth_vals[i] = z if z > 0.0 else 0.0
                if self.publish_cloud and z > 0.0:
                    pts.append([x, y, z])

            # If sizes mismatch, pad/trim depth to w*h so RViz doesn't complain
            target_len = w * h
            if len(depth_vals) < target_len:
                depth_vals.extend([0.0] * (target_len - len(depth_vals)))
            elif len(depth_vals) > target_len:
                depth_vals = depth_vals[:target_len]

            # Build and publish 32FC1 depth image
            img = Image()
            img.header = Header()
            img.header.stamp = self.n.get_clock().now().to_msg()
            img.header.frame_id = self.frame_id
            img.height, img.width = h, w
            img.encoding = "32FC1"
            img.is_bigendian = False
            img.step = w * 4
            import array
            img.data = array.array('f', depth_vals).tobytes()
            self.pub_depth.publish(img)

            # Optional PointCloud2
            if self.publish_cloud and pts:
                fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                ]
                cloud = pc2.create_cloud(img.header, fields, pts)
                self.pub_cloud.publish(cloud)

            # Publishing Gray Scale
            getGray = getattr(frame, "getGrayValue", None)
            have_gray = callable(getGray)

            if have_gray:
                gray_vals = [0] * npts
                for i in range(npts):
                    gv = getGray(i)
                    try:
                        gray_vals[i] = int(gv)
                    except Exception:
                        gray_vals[i] = int(float(gv))

                # Pad/trim grayscale list to match w*h
                target_len = w * h
                if len(gray_vals) < target_len:
                    gray_vals.extend([0] * (target_len - len(gray_vals)))
                elif len(gray_vals) > target_len:
                    gray_vals = gray_vals[:target_len]

                # Inspect range to decide encoding
                gmin, gmax = min(gray_vals), max(gray_vals)
                # self.n.get_logger().info(f"Gray range: min={gmin}, max={gmax}")

                gray_img = Image()
                gray_img.header = img.header  # reuse same header as depth
                gray_img.height, gray_img.width = h, w
                gray_img.is_bigendian = False

                if gmax < 256:
                    gray_img.encoding = "mono8"
                    gray_img.step = w * 1
                    gray_img.data = bytes(bytearray(gray_vals))
                else:
                    gray_img.encoding = "mono16"
                    gray_img.step = w * 2
                    import array as _arr
                    gray_img.data = _arr.array('H', gray_vals).tobytes()

                self.pub_gray.publish(gray_img)
            # else:
            #     self.n.get_logger().warn("No Gray Scale Image")

        except Exception as e:
            self.n.get_logger().error(f"Royale callback error: {e}")


class RoyaleNode(Node):
    def __init__(self):
        super().__init__("royale_node")
        self.get_logger().info("Starting Royale node...")
        cm = roypy.CameraManager()
        cams = cm.getConnectedCameraList()
        if cams.size() == 0:
            raise RuntimeError("No Royale camera found")
        cam_id = str(cams[0])
        self.get_logger().info(f"Using camera: {cam_id}")
        self.cam = cm.createCamera(cam_id)

        # Initialize FIRST (loads calibration)
        self.cam.initialize()

        # Optional: choose use case by name
        # for uc in self.cam.getUseCases():
        #     if "MODE_9_5FPS" in uc:
        #         self.cam.setUseCase(uc)

        self.listener = Listener(self)
        self.cam.registerDataListener(self.listener)

        self._run = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        self.cam.startCapture()
        self.get_logger().info("Capture started.")
        try:
            while self._run:
                time.sleep(0.01)
        finally:
            try:
                self.cam.stopCapture()
            except Exception:
                pass
            self.get_logger().info("Capture stopped.")

    def destroy_node(self):
        self._run = False
        try:
            if self._th.is_alive():
                self._th.join(timeout=1.0)
        except Exception:
            pass
        try:
            self.cam.unregisterDataListener()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = RoyaleNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(str(e))
        else:
            print(e)
    finally:
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
