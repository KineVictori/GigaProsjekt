import cv2
import numpy as np

def get_available_cameras(maxPorts=5):
    availablePorts = []
    for i in range(maxPorts):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            availablePorts.append(i)
        cap.release()
    return availablePorts

def empty(a):
    pass

def setup_trackbars():
    cv2.namedWindow("HSV Trackbars")
    cv2.resizeWindow("HSV Trackbars", 400, 300)
    cv2.createTrackbar("H Min", "HSV Trackbars", 0, 179, empty)
    cv2.createTrackbar("H Max", "HSV Trackbars", 179, 179, empty)
    cv2.createTrackbar("S Min", "HSV Trackbars", 0, 255, empty)
    cv2.createTrackbar("S Max", "HSV Trackbars", 255, 255, empty)
    cv2.createTrackbar("V Min", "HSV Trackbars", 0, 255, empty)
    cv2.createTrackbar("V Max", "HSV Trackbars", 255, 255, empty)

def get_trackbar_values():
    h_min = cv2.getTrackbarPos("H Min", "HSV Trackbars")
    h_max = cv2.getTrackbarPos("H Max", "HSV Trackbars")
    s_min = cv2.getTrackbarPos("S Min", "HSV Trackbars")
    s_max = cv2.getTrackbarPos("S Max", "HSV Trackbars")
    v_min = cv2.getTrackbarPos("V Min", "HSV Trackbars")
    v_max = cv2.getTrackbarPos("V Max", "HSV Trackbars")
    return {"hmin": h_min, "hmax": h_max,
            "smin": s_min, "smax": s_max,
            "vmin": v_min, "vmax": v_max}

def main():
    print("Searching for available cameras")
    available_ports = get_available_cameras()

    if not available_ports:
        print("No available cameras detected")
        return

    print(f"Available cameras: {available_ports}")

    CAMERA_PORT = -1
    RESOLUTION = (640, 480)

    while CAMERA_PORT not in available_ports:
        try:
            CAMERA_PORT = int(input(f"Choose a camera from this list ({available_ports}): "))
        except ValueError:
            print("Invalid. Try again.")

    camera = cv2.VideoCapture(CAMERA_PORT)
    if not camera.isOpened():
        print(f"Error: Could not open camera on port {CAMERA_PORT}")
        return

    camera.set(3, RESOLUTION[0])
    camera.set(4, RESOLUTION[1])

    setup_trackbars()

    print("Starting camera stream.")

    while True:
        ret, img = camera.read()
        if not ret:
            print("Error: Could not read image from camera")
            break

        # Blur for å jevne ut støy
        img_blur = cv2.GaussianBlur(img, (7, 7), 0)

        # HSV-basert fargefiltrering
        hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
        
        hsv_vals = get_trackbar_values()
        lower = np.array([hsv_vals["hmin"], hsv_vals["smin"], hsv_vals["vmin"]])
        upper = np.array([hsv_vals["hmax"], hsv_vals["smax"], hsv_vals["vmax"]])

        mask = cv2.inRange(hsv, lower, upper)
        result = cv2.bitwise_and(img, img, mask = mask)

        # Finn og tegn konturer
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                x, y, w, h = cv2.boundingRect(cnt)
                cx, cy = x + w // 2, y + h // 2
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)

        # Vis resultater
        cv2.imshow("Calibration - Original", img)
        cv2.imshow("Calibration - Mask", mask)
        cv2.imshow("Calibration - Result", result)
        
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
