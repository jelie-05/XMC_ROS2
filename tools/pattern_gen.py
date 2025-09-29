#!/usr/bin/env python3
import cv2
import numpy as np

# Pretty names for common ArUco dictionaries
DICT_NAMES = {
    cv2.aruco.DICT_4X4_50: "DICT_4X4_50",
    cv2.aruco.DICT_4X4_100: "DICT_4X4_100",
    cv2.aruco.DICT_4X4_250: "DICT_4X4_250",
    cv2.aruco.DICT_4X4_1000: "DICT_4X4_1000",
    cv2.aruco.DICT_5X5_50: "DICT_5X5_50",
    cv2.aruco.DICT_5X5_100: "DICT_5X5_100",
    cv2.aruco.DICT_5X5_250: "DICT_5X5_250",
    cv2.aruco.DICT_5X5_1000: "DICT_5X5_1000",
    cv2.aruco.DICT_6X6_50: "DICT_6X6_50",
    cv2.aruco.DICT_6X6_100: "DICT_6X6_100",
    cv2.aruco.DICT_6X6_250: "DICT_6X6_250",
    cv2.aruco.DICT_6X6_1000: "DICT_6X6_1000",
    cv2.aruco.DICT_7X7_50: "DICT_7X7_50",
    cv2.aruco.DICT_7X7_100: "DICT_7X7_100",
    cv2.aruco.DICT_7X7_250: "DICT_7X7_250",
    cv2.aruco.DICT_7X7_1000: "DICT_7X7_1000",
    cv2.aruco.DICT_ARUCO_ORIGINAL: "DICT_ARUCO_ORIGINAL",
}

def put_footer(canvas, text, dpi=300, margin_mm=5.0):
    """Draw a white strip and black text at the bottom; font size scales with DPI."""
    H, W = canvas.shape[:2]
    px_per_mm = dpi / 25.4

    # Footer strip ~12 mm tall
    strip_h_px = int(round(12 * px_per_mm))
    y0 = H - strip_h_px
    cv2.rectangle(canvas, (0, y0), (W, H), (255, 255, 255), thickness=-1)

    font = cv2.FONT_HERSHEY_SIMPLEX
    # Start scale proportional to DPI (≈2 mm text height)
    scale = 0.2 * px_per_mm
    thickness = max(1, int(round(scale * 1.5)))

    # Fit text to width if needed
    target_width = int(W * 0.95)
    while True:
        (tw, th), _ = cv2.getTextSize(text, font, scale, thickness)
        if tw <= target_width or scale <= 0.3:
            break
        scale -= 0.05

    x = (W - tw) // 2
    y = H - int(round(margin_mm * px_per_mm))
    cv2.putText(canvas, text, (x, y), font, scale, (0, 0, 0), thickness, cv2.LINE_AA)



def generate_charuco(square_size_mm=30.0, marker_size_mm=20.0, squares_x=5, squares_y=7,
                     dpi=300, margin_mm=10.0, dictionary_id=cv2.aruco.DICT_5X5_1000,
                     out_png="charuco_custom.png", orientation="portrait"):
    # ---------- page settings ----------
    A4_MM_PORTRAIT = (210.0, 297.0)  # width, height in mm
    if str(orientation).lower() == "landscape":
        A4_MM = (A4_MM_PORTRAIT[1], A4_MM_PORTRAIT[0])  # swap for landscape
    else:
        A4_MM = A4_MM_PORTRAIT

    MM_PER_INCH = 25.4
    px_per_mm = dpi / MM_PER_INCH
    a4_w_px, a4_h_px = int(A4_MM[0] * px_per_mm), int(A4_MM[1] * px_per_mm)

    # ---------- convert input sizes to pixels ----------
    square_len_px = int(round(square_size_mm * px_per_mm))
    marker_len_px = int(round(marker_size_mm * px_per_mm))

    # ---------- build the board ----------
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    # Use positional args for maximum compatibility (OpenCV 4.7 contrib)
    board = aruco.CharucoBoard_create(
        squares_x, squares_y,
        square_len_px, marker_len_px,
        dictionary
    )

    # ---------- draw board image ----------
    try:
        board_img = board.generateImage((squares_x * square_len_px, squares_y * square_len_px))
    except AttributeError:
        board_img = board.draw((squares_x * square_len_px, squares_y * square_len_px))

    # ---------- place onto A4 canvas centered ----------
    canvas = np.full((a4_h_px, a4_w_px), 255, dtype=np.uint8)
    off_x = (a4_w_px - board_img.shape[1]) // 2
    off_y = (a4_h_px - board_img.shape[0]) // 2
    canvas[off_y:off_y + board_img.shape[0], off_x:off_x + board_img.shape[1]] = board_img

    # ---------- footer text ----------
    dict_name = DICT_NAMES.get(dictionary_id, str(dictionary_id))
    footer = f"ChArUco {squares_x}x{squares_y} | square {square_size_mm:.2f} mm | marker {marker_size_mm:.2f} mm | {dict_name} | {dpi} DPI"
    put_footer(canvas, footer, dpi=dpi, margin_mm=5.0)

    # ---------- save ----------
    cv2.imwrite(out_png, canvas)
    print("Saved:", out_png)
    print(f"Square length: {square_size_mm:.2f} mm, Marker length: {marker_size_mm:.2f} mm, Dict: {dict_name}, DPI: {dpi}")

# Example usage:
if __name__ == "__main__":
    generate_charuco(square_size_mm=50.0, marker_size_mm=35.0, squares_x=6, squares_y=4, dpi=600,
                     out_png="tools/pattern/charuco_A4_6x4_600dpi.png", orientation="landscape")
