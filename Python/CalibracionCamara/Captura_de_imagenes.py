import cv2 as cv
import os
import keyboard
import threading

CHESS_BOARD_DIM = (9, 6)  # Define la dimensión del tablero de ajedrez como una tupla de dos valores
n = 0  # Variable declarada para almacenar la cantidad de imagenes
dir = "Calibracion2.0"  # Establece la ruta del directorio donde se guardarán las imágenes.
full_dir_path = os.path.abspath(dir)

CHECK_DIR = os.path.isdir(dir)  # Comprueba si el directorio de imágenes ya existe.
if not CHECK_DIR:  # Verifica si el directorio de imágenes no existe.
    os.makedirs(dir)  # Si el directorio no existe, lo crea.
    print(f'"{dir}" Directory is created')
else:
    print(f'"{dir}" Directory already Exists.')

criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)  # Establece los criterios de terminación para el algoritmo de búsqueda de esquinas del tablero

def detect_checker_board(image, grayImage, criteria, boardDimension):  # Define una función para detectar el tablero de ajedrez en una imagen dada
    ret, corners = cv.findChessboardCorners(grayImage, boardDimension)
    if ret == True:
        corners1 = cv.cornerSubPix(grayImage, corners, (3, 3), (-1, -1), criteria)
        image = cv.drawChessboardCorners(image, boardDimension, corners1, ret)
    return image, ret

# Cambia aquí tu RTSP URL si es necesario
rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
cap = cv.VideoCapture(rtsp_url)  # Inicializa la captura desde la cámara RTSP

if not cap.isOpened():
    print("Error: No se puede abrir la cámara")
    exit()

frame = None
stop_thread = False

def capture_frames():
    global frame, stop_thread
    while not stop_thread:
        ret, frame_temp = cap.read()
        if ret:
            frame = frame_temp

capture_thread = threading.Thread(target=capture_frames)
capture_thread.start()

if __name__ == "__main__":
    while True:
        if frame is not None:
            copyFrame = frame.copy()  # Realiza una copia del fotograma capturado.
            gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)  # Convierte el fotograma a escala de grises

            image, board_detected = detect_checker_board(frame, gray, criteria, CHESS_BOARD_DIM)  # Utiliza la función detect_checker_board para detectar el tablero de ajedrez en la imagen

            # Agrega un texto al fotograma mostrando el número de imagen guardada
            cv.putText(frame, f"saved_img : {n}", (30, 40), cv.FONT_HERSHEY_PLAIN, 1.4, (255, 0, 0), 2, cv.LINE_AA)
            cv.imshow("frame", frame)  # Muestra el fotograma en una ventana con el nombre "frame".
            cv.imshow("copyframe", copyFrame)  # Muestra la copia del fotograma en otra ventana con el nombre "copyFrame".

            if keyboard.is_pressed('q'):
                break
            if keyboard.is_pressed('s') and board_detected == True:  # Si se presiona la tecla 's' y se detecta un tablero de ajedrez en la imagen, guarda la imagen.
                cv.imwrite(f"{dir}/image{n}.png", copyFrame)  # Guarda la imagen en el directorio de imágenes
                print(f"imagen {n}")
                n += 1  # Incrementa el contador de imágenes guardadas.

            cv.waitKey(1)  # Espera 1 milisegundo para actualizar las ventanas

    stop_thread = True
    capture_thread.join()
    cap.release()  # Libera los recursos de la cámara.
    cv.destroyAllWindows()  # Cierra todas las ventanas abiertas por OpenCV.

print("Images guardadas:", n)  # Imprime el número total de imágenes guardadas
