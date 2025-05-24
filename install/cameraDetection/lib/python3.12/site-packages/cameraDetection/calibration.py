import cv2
import cvzone
from cvzone.ColorModule import ColorFinder

def availableCameras(maxPorts=5):
    availablePorts = []
    for i in range(maxPorts):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            availablePorts.append(i)
        cap.release()
    return availablePorts

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

    myColorFinder = ColorFinder(True)  # Kalibreringsmodus = True

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
        imgColor, mask = myColorFinder.update(img)

        # Finn konturer med lavere minArea
        imgContour, _ = cvzone.findContours(img, mask, minArea=100)

        # Vis resultater
        cv2.imshow("Kalibrering - Juster HSV verdier", imgContour)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('s'):
            print("Lagrer verdier:", myColorFinder.hsvVals)
            break
        elif key == ord('q'):
            break

    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()