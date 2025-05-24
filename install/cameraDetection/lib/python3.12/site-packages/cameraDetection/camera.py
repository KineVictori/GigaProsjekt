import cv2
import cvzone
from cvzone.ColorModule import ColorFinder

# Funksjon for å liste opp tilgjengelige kamera-porter
def availableCameras(maxPorts = 5):
    availablePorts = []
    for i in range(maxPorts):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            availablePorts.append(i)
        cap.release()
    return availablePorts

def getBallPosition(cap, hsvVals, centerPoint, myColorFinder, resolution):
    """
    Finn ballens posisjon basert på HSV-verdier.
    :return: (x, y, z) posisjon eller (None, None, None) hvis ingen ball funnet.
    """
    ret, img = cap.read()
    if not ret:
        return None, None, None

    # Prosesser bildet for alle farger i hsvVals og slå sammen maskene
    mask = None
    for name, hsv in hsvVals.items():
        _, currentMask = myColorFinder.update(img, hsv)
        if mask is None:
            mask = currentMask
        else:
            mask = cv2.bitwise_or(mask, currentMask)

    # Finn konturer på masken
    imgContour, contours = cvzone.findContours(img, mask)

    # Vis bilde med sporing
    cv2.imshow("Ballsporing", imgContour)
    cv2.waitKey(1)

    # Sjekk om det finnes konturer (ballen funnet)
    if contours:
        # Beregn avstand i x, y i forhold til senterpunktet
        x = contours[0]['center'][0] - centerPoint[0]
        y = (resolution[1] - contours[0]['center'][1]) - centerPoint[1]
        # Estimer z ut fra areal (størrelse) av konturen
        z = (contours[0]['area'] - centerPoint[2]) / 1000

        return x, y, z

    return None, None, None

def main():
    # Viser hvilken kamera som er tilgjengelige
    print("Søker etter tilgjengelige kamera")
    availablePorts = availableCameras()

    if not availablePorts:
        print("Ingen tilgjengelige kamera detektert")
        return

    print(f"Tilgjengelige kamera-porter: {availablePorts}")

    # Velge kamera-port manuelt
    CAMERA_PORT = -1
    RESOLUTION = (640, 480)

    while CAMERA_PORT not in availablePorts:
        try:
            CAMERA_PORT = int(input(f"Velg et tilgjengelig kamera fra denne lista ({availablePorts}): "))
        except ValueError:
            print("Ugyldig input. Prøv igjen.")

    # Bruk de kalibrerte HSV-verdiene fra kalibreringen
    hsvVals = {
        "Rød": {'hmin': 160, 'smin': 144, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 0},
        "Blå": {'hmin': 111, 'smin': 0, 'vmin': 0, 'hmax': 179, 'smax': 255, 'vmax': 255},
        "Grønn": {'hmin': 31, 'smin': 46, 'vmin': 0, 'hmax': 179, 'smax': 76, 'vmax': 185}
    }

    # Opprett ColorFinder (ikke i kalibreringsmodus)
    myColorFinder = ColorFinder(False)

    # Åpne kamera
    camera = cv2.VideoCapture(CAMERA_PORT)

    if not camera.isOpened():
        print(f"Feil: Kunne ikke åpne kamera på port {CAMERA_PORT}")
        return

    # Sett ønsket oppløsning
    camera.set(3, RESOLUTION[0])  # Bredde
    camera.set(4, RESOLUTION[1])  # Høyde

    # Senterpunkt for referanse (kan settes til 0,0,0 hvis du ikke har en kjent referanse)
    centerPoint = (RESOLUTION[0]//2, RESOLUTION[1]//2, 1000)

    print("Starter ballsporing. Trykk 'q' for å avslutte.")

    # Hovedløkke for sporing
    while True:
        x, y, z = getBallPosition(camera, hsvVals, centerPoint, myColorFinder, RESOLUTION)

        if x is not None and y is not None and z is not None:
            print(f"Ballposisjon: X={x}, Y={y}, Z={z}")
        else:
            print("Ingen ball funnet")

        # Trykk 'q' for å avslutte
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Frigjør ressurser
    camera.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()