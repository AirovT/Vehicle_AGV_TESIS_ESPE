import pickle
import cv2
import numpy as np


# Read in the saved objpoints and imgpoints
cameraMatrix, dist = pickle.load(open( r"C:\Users\jairo\Documents\U_ESPE\Tesis\Nueva tesis ESPE\GitHub Tesis\Vehicle_AGV_TESIS_ESPE\Python\Calibracion/calibration.pkl", "rb" ))
#dist_pickle = pickle.load( open( "wide_dist_pickle.p", "rb" ) )

# Imprimir las matrices de la cámara y distorsión
print("Camera Matrix (Matriz de la cámara):")
print(cameraMatrix)

print("\nDistortion Coefficients (Coeficientes de distorsión):")
print(dist)

# Read in an image
img = cv2.imread(r'C:\Users\jairo\Documents\U_ESPE\Tesis\Nueva tesis ESPE\GitHub Tesis\Vehicle_AGV_TESIS_ESPE\Python\Calibracion/cali5.png')
h,  w = img.shape[:2]
newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))

# Undistort
undistorted = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)

# crop the image
x, y, w, h = roi
undistorted = undistorted[y:y+h, x:x+w]
undistorted = cv2.resize(undistorted,(1920,1080))
cv2.imshow("Imagen Original", img)

cv2.imshow("Imagen sin distorsion", undistorted)

cv2.waitKey(0)
# Destroy all the windows
cv2.destroyAllWindows()

# Reprojection Error
mean_error = 0
try:
    for i in range(len(objpoints)):
        # Verificar si los puntos 3D y 2D tienen el mismo tamaño
        if len(objpoints[i]) == len(imgpoints[i]):
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], cameraMatrix, dist)
            error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
            mean_error += error
        else:
            raise ValueError(f"El número de puntos 3D y 2D no coinciden para la imagen {i}")

    print("Total reprojection error: {}".format(mean_error / len(objpoints)))

except NameError as e:
    print(f"Error en las variables no definidas: {e}")
except ValueError as e:
    print(f"Error en las dimensiones de los puntos: {e}")
except Exception as e:
    print(f"Ocurrió un error inesperado: {e}")