import cv2
import numpy as np
import math

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0
    return np.array([x, y, z])

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    rvecs, tvecs = [], []
    for c in corners:
        _, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
    return rvecs, tvecs

def Estimation(ID_Objetivo):
    scale_percent = 50  # Escalar al 50% del tamaño original
    marker_size = 180
    

    camera_matrix = np.array([[1.17390158e+03, 0.00000000e+00, 9.70673620e+02],
                              [0.00000000e+00, 1.17020056e+03, 4.92174250e+02],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
    camera_distortion = np.array([[ -2.97179698e-01,
                                    6.95602029e-02,
                                    2.42497888e-03,
                                     1.90931850e-04,
                                    2.42043840e-03]])  

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
    # rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif&rtsp_transport=udp"
    
    cap = cv2.VideoCapture(rtsp_url)    
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Corregir la distorsión de la imagen
        frame_undistorted = cv2.undistort(frame, camera_matrix, camera_distortion)

        # Procesar el frame original (sin corregir distorsión)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco_detector.detectMarkers(gray_frame)
        frame_corrected = frame.copy()

        if ids is not None and ID_Objetivo in ids:
            indice = np.where(ids == ID_Objetivo)[0][0]
            corners = [corners[indice]]
            rvec_list, tvec_list = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec, tvec = rvec_list[0], tvec_list[0]
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)
            rvec_flipped = rvec * -1
            tvec_flipped = tvec * -1
            rotation_matrix, _ = cv2.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
            x, y, z = round(realworld_tvec[0, 0], 3), round(realworld_tvec[1, 0], 3), round(realworld_tvec[2, 0], 3)
            yaw = round(math.degrees(yaw), 2)
            pitch = round(math.degrees(pitch), 2)
            roll = round(math.degrees(roll), 2)
            tvec_str = f"P x={x} y={y} z={z} pitch={pitch} roll={roll} yaw={yaw}"
            cv2.putText(frame_corrected, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)

        # Redimensionar las imágenes
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        resized_frame_corrected = cv2.resize(frame_corrected, (width, height), interpolation=cv2.INTER_AREA)
        resized_frame_undistorted = cv2.resize(frame_undistorted, (width, height), interpolation=cv2.INTER_AREA)

        # Dibujar líneas guía en el frame corregido (anotaciones)
        height, width = resized_frame_corrected.shape[:2]
        cv2.line(resized_frame_corrected, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)  # Línea vertical central
        cv2.line(resized_frame_corrected, (0, height // 2), (width, height // 2), (0, 255, 0), 2)  # Línea horizontal central
        for i in range(1, 3):
            offset = height // 4 * i
            cv2.line(resized_frame_corrected, (0, offset), (width, offset), (255, 0, 0), 1)  # Líneas guía horizontales
            cv2.line(resized_frame_corrected, (0, height - offset), (width, height - offset), (255, 0, 0), 1)

        # Dibujar líneas guía en el frame sin distorsión
        height, width = resized_frame_undistorted.shape[:2]
        cv2.line(resized_frame_undistorted, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)  # Línea vertical central
        cv2.line(resized_frame_undistorted, (0, height // 2), (width, height // 2), (0, 255, 0), 2)  # Línea horizontal central
        for i in range(1, 3):
            offset = height // 4 * i
            cv2.line(resized_frame_undistorted, (0, offset), (width, offset), (255, 0, 0), 1)  # Líneas guía horizontales
            cv2.line(resized_frame_undistorted, (0, height - offset), (width, height - offset), (255, 0, 0), 1)

        # Mostrar las ventanas
        cv2.imshow('Frame Original', resized_frame)
        cv2.imshow('Frame Corregido (Anotaciones)', resized_frame_corrected)
        cv2.imshow('Frame Corregido (Sin Distorsion)', resized_frame_undistorted)

        if cv2.waitKey(3) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    Estimation(0)