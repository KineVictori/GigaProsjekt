import cv2
import cvzone
from cvzone.ColorModule import ColorFinder

#Funksjon for å liste opp tilgjengelige kamera-porter
def availableCameras(maxPorts = 5):
    availablePorts = []
    for i in range(maxPorts):
        cap = cv2.VideoCapture(i)
        if cap.read()[0]:
            availablePorts.append(i)
        cap.release()
    return availablePorts

#Viser hvilken kamera som er tilgjengelige
print("Søker etter tilgjengelige kamera")
availablePorts = availableCameras()

if not availablePorts:
    print("Ingen tilgjengelige kamera detektert")
    exit()

print(f"Tilgjengelige kamera-porter: {availablePorts}")
0
#Velge kamera-port manuelt
CAMERA_PORT = -1
RESOLUTION = (640, 480)

while CAMERA_PORT not in availablePorts:
    CAMERA_PORT = int(input(f"Velg et tilgjengelig kamera fra denne lista ({availablePorts}):"))

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
    exit()

camera.set(3, RESOLUTION[0])  # Bredde
camera.set(4, RESOLUTION[1])  # Høyde

# Senterpunkt for referanse (kan settes til 0,0,0 hvis du ikke har en kjent referanse)
centerPoint = (RESOLUTION[0]//2, RESOLUTION[1]//2, 1000)  

def getBallPosition(cap, hsvVals, centerPoint):
    """
    Finn ballens posisjon basert på HSV-verdier.
    :return: (x, y, z) posisjon eller (None, None, None) hvis ingen ball funnet.
    """
    ret, img = cap.read()
    if not ret:
        return None, None, None

    # Prosesser bildet
    for name, hsv in hsvVals.items():
        imgColor, mask = myColorFinder.update(img, hsv)
    imgContour, contours = cvzone.findContours(img, mask)

    # Vis bilde med sporing
    cv2.imshow("Ballsporing", imgContour)
    cv2.waitKey(1)

    # Sjekk om det finnes konturer (ballen funnet)
    if contours:
        x = contours[0]['center'][0] - centerPoint[0]
        y = (RESOLUTION[1] - contours[0]['center'][1]) - centerPoint[1]
        z = (contours[0]['area'] - centerPoint[2]) / 1000  # Skalert areal

        return x, y, z

    return None, None, None


# Hovedløkke for sporing
while True:
    x, y, z = getBallPosition(camera, hsvVals, centerPoint)
    
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