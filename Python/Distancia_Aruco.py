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

marker_size = 120  
camera_matrix = np.array([[1.25893332e+03, 0.00000000e+00, 5.22304630e+02],
                          [0.00000000e+00, 1.27293638e+03, 4.92206435e+02],
                          [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
camera_distortion = np.array([[-0.4894947, 0.20673354, -0.0178347, 0.05254465, -0.06985189]])

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
cap = cv2.VideoCapture(rtsp_url)
cap.set(3, 640)
cap.set(4, 480)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco_detector.detectMarkers(gray_frame)
    
    if ids is not None:
        for i in range(len(ids)):
            cv2.aruco.drawDetectedMarkers(frame, corners)
            rvec_list_all, tvec_list_all, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec = rvec_list_all[i]
            tvec = tvec_list_all[i]
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)
            
            rvec_flipped = rvec * -1
            tvec_flipped = tvec * -1
            rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
            
            x = round(realworld_tvec[0, 0], 3)
            y = round(realworld_tvec[1, 0], 3)
            z = round(realworld_tvec[2, 0], 3)
            yaw = round(math.degrees(yaw), 3)
            
            print(f"ID: {ids[i]} | x={x}, y={y}, z={z}, yaw={yaw}")
            
            tvec_str = f"ID: {ids[i]} x={x} y={y} z={z} yaw={yaw}"
            cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
    
    cv2.imshow('DETECCION DE ARUCO', frame)
    
    if cv2.waitKey(3) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
