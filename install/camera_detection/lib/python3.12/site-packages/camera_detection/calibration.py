import cv2
import numpy as np
#import cvzone
#from cvzone.ColorModule import ColorFinder

def availableCameras(maxPorts=5):
    availablePorts = []
    for i in range(maxPorts):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            availablePorts.append(i)
        cap.release()
    return availablePorts

def empty(a):
    pass

def setupTrackbars():
    cv2.namedWindow("HSV Trackbars")
    cv2.resizeWindow("HSV Trackbars", 400, 300)
    cv2.createTrackbar("H Min", "HSV Trackbars", 0, 179, empty)
    cv2.createTrackbar("H Max", "HSV Trackbars", 179, 179, empty)
    cv2.createTrackbar("S Min", "HSV Trackbars", 0, 255, empty)
    cv2.createTrackbar("S Max", "HSV Trackbars", 255, 255, empty)
    cv2.createTrackbar("V Min", "HSV Trackbars", 0, 255, empty)
    cv2.createTrackbar("V Max", "HSV Trackbars", 255, 255, empty)

def getTrackbarValues():
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
    print("Søker etter tilgjengelige kamera")
    availablePorts = availableCameras()

    if not availablePorts:
        print("Ingen tilgjengelige kamera detektert")
        return

    print(f"Tilgjengelige kamera-porter: {availablePorts}")

    CAMERA_PORT = -1
    RESOLUTION = (640, 480)

    while CAMERA_PORT not in availablePorts:
        try:
            CAMERA_PORT = int(input(f"Velg et tilgjengelig kamera fra denne lista ({availablePorts}): "))
        except ValueError:
            print("Ugyldig input. Prøv igjen.")

    #myColorFinder = ColorFinder(True)  # Kalibreringsmodus = True

    camera = cv2.VideoCapture(CAMERA_PORT)
    if not camera.isOpened():
        print(f"Feil: Kunne ikke åpne kamera på port {CAMERA_PORT}")
        return

    camera.set(3, RESOLUTION[0])
    camera.set(4, RESOLUTION[1])

    print("Starter kamerastream. Trykk 's' for å lagre HSV-verdier, 'q' for å avslutte.")

    while True:
        ret, img = camera.read()
        if not ret:
            print("Feil: Kunne ikke lese ramme fra kamera.")
            break

        # Blur for å jevne ut støy
        img = cv2.GaussianBlur(img, (7, 7), 0)

        # HSV-basert fargefiltrering
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        hsv_vals = getTrackbarValues()
        lower = np.array([hsv_vals["hmin"], hsv_vals["smin"], hsv_vals["vmin"]])
        upper = np.array([hsv_vals["hmax"], hsv_vals["smax"], hsv_vals["vmax"]])

        mask = cv2.inRAnge(hsv_vals, lower, upper)

        # Finn konturer med lavere minArea
        imgContour, _ = cv2.findContours(img, mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE, minArea=100)

        # Vis resultater
        cv2.imshow("Kalibrering - Juster HSV verdier", imgContour)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("Lagrer verdier:", hsv_vals)
            break
        elif key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()