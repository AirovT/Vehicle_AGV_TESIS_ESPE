from calibrate import CharucoCalibrator
import cv2

if __name__ == "__main__":
    # Configuración de parámetros para el tablero ChArUco
    aruco_dict_id = cv2.aruco.DICT_6X6_250
    squares_vertically = 7
    squares_horizontally = 5
    square_length = 0.04  # en metros
    marker_length = 0.02  # en metros

    # Inicializar el calibrador
    calibrator = CharucoCalibrator(
        aruco_dict=aruco_dict_id,
        squares_vertically=squares_vertically,
        squares_horizontally=squares_horizontally,
        square_length=square_length,
        marker_length=marker_length
    )

    # Generar y mostrar el tablero ChArUco
    calibrator.generate_charuco_board()
