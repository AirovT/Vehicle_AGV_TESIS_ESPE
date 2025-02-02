import numpy as np
import cv2 as cv
import glob
import pickle
import os

################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################
chessboardSize = (9,6)
frameSize = (640,480)

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0],0:chessboardSize[1]].T.reshape(-1,2)

size_of_chessboard_squares_mm = 25
objp = objp * size_of_chessboard_squares_mm

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('Calibracion2.0/*.png')

for image in images:

    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Encontrar las esquinas del tablero de ajedrez
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    # Si encontrados, añade puntos de objeto, puntos de imagen (después de refinarlos).
    if ret == True:

        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners)

        # Dibuja y muestra las esquinas.
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)

cv.destroyAllWindows()

############## CALIBRACION #######################################################

ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

#Guarda el resultado de la calibración de la cámara para uso posterior (no nos preocuparemos por rvecs / tvecs).
path = "Calibracion"
name = "calibration.pkl"
name2 = "cameraMatrix.pkl"
name3 = "dist.pkl"

# Crear rutas de archivo separadas
ruta1 = os.path.join(path, name)
ruta2 = os.path.join(path, name2)
ruta3 = os.path.join(path, name3)

# Asegurarse de que el directorio exista
os.makedirs(path, exist_ok=True)

# Guardar los datos de calibración
pickle.dump((cameraMatrix, dist), open(ruta1, "wb"))

# Guardar la matriz de la cámara
pickle.dump(cameraMatrix, open(ruta2, "wb"))

# Guardar los coeficientes de distorsión
pickle.dump(dist, open(ruta3, "wb"))

# Función para cargar y mostrar los datos de un archivo pickle
def load_and_print_pickle(ruta):
    with open(ruta, 'rb') as file:
        data = pickle.load(file)
    print(f"Datos de {ruta}:")
    print(data)
    print()  # Línea en blanco para separar las salidas

# Cargar y mostrar los datos de cada archivo
#load_and_print_pickle(ruta1)
load_and_print_pickle(ruta2)
load_and_print_pickle(ruta3)

############## UNDISTORTION #####################################################

img = cv.imread('Calibracion/cali5.png')

if img is None:
    print("❌ ERROR: No se pudo abrir la imagen. Verifica la ruta y que el archivo existe.")
    exit()


h,  w = img.shape[:2]
newCameraMatrix, roi = cv.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))


# Undistort
dst = cv.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# crop the image
x, y, w, h = roi
dst = dst[y:y+h, x:x+w]
name4='caliResult1.png'
ruta4 = os.path.join(path, name4)
cv.imwrite(ruta4, dst)


# Undistort with Remapping
mapx, mapy = cv.initUndistortRectifyMap(cameraMatrix, dist, None, newCameraMatrix, (w,h), 5)

# Verificar si mapx y mapy son válidos
if mapx is None or mapy is None:
    print("❌ ERROR: Los mapas de remapeo (mapx, mapy) son inválidos.")
    exit()

# Verificar si los mapas tienen dimensiones correctas
print(f"mapx shape: {mapx.shape}, mapy shape: {mapy.shape}")

dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

# Verificar si dst está vacío
if dst is None or dst.size == 0:
    print("❌ ERROR: `cv.remap()` devolvió una imagen vacía. Verifica los mapas y la imagen de entrada.")
    exit()

# Verificar el tamaño de la imagen resultante
print(f"dst shape: {dst.shape}")

# crop the image
x, y, w, h = roi

print(f"ROI: (x={x}, y={y}, w={w}, h={h})")

# Verificar si el recorte está dentro de los límites de la imagen
if x < 0 or y < 0 or x + w > dst.shape[1] or y + h > dst.shape[0]:
    print("❌ ERROR: El ROI está fuera de los límites de la imagen.")
    exit()

# Recortar la imagen
dst = dst[y:y+h, x:x+w]
name5='caliResult2.png'
ruta5 = os.path.join(path, name5)
cv.imwrite(ruta5, dst)


# Reprojection Error
mean_error = 0

for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print( "total error: {}".format(mean_error/len(objpoints)) )