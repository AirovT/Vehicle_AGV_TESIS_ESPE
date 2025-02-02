import math
import cv2
import numpy as np
from typing import List, Tuple, Any
import paho.mqtt.client as mqtt
from tb_device_mqtt import TBDeviceMqttClient, TBPublishInfo
import time
import logging

import socket 
import time
import json
import serial
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO)

counter = 0

Puerto_BT = 'COM13'

# Funcion para calculas la estimacion de la distancia con respecto del aruco y la camara, se envia el ID objetivo
#ID objetivo corresponde al que vamos a buscar
def Estimation(ID_Objetivo, BTPort, BANDERA_):
    pid = PIDController(0.01, 0.01, 0.1 , 200, 0)
    pid.Conectar_Blu_Auto(BTPort)
    # Función para verificar si una matriz es una matriz de rotación válida
    coordenadas = [0,0,0,0]
    
    def isRotationMatrix(R):
        Rt = np.transpose(R)  # Transponer la matriz de rotación
        shouldBeIdentity = np.dot(Rt, R)  # Producto punto de la transpuesta con la matriz original
        I = np.identity(3, dtype=R.dtype)  # Matriz de identidad 3x3
        n = np.linalg.norm(I - shouldBeIdentity)  # Norma de la diferencia entre I y shouldBeIdentity
        return n < 1e-6  # Devuelve True si la norma es menor que 1e-6

    # Función para convertir una matriz de rotación en ángulos de Euler
    def rotationMatrixToEulerAngles(R):
        assert (isRotationMatrix(R))  # Verificar si la matriz de rotación es válida
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])  # Calcular sy
        singular = sy < 1e-6  # Verificar si sy es singular
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])  # Calcular x
            y = math.atan2(-R[2, 0], sy)  # Calcular y
            z = math.atan2(R[1, 0], R[0, 0])  # Calcular z
        else:
            x = math.atan2(-R[1, 2], R[1, 1])  # Calcular x en caso de singularidad
            y = math.atan2(-R[2, 0], sy)  # Calcular y en caso de singularidad
            z = 0  # Establecer z en 0 en caso de singularidad
        return np.array([x, y, z])  # Devolver ángulos de Euler como un array

    marker_size = 120  # Tamaño del marcador ArUco
    camera_matrix = np.array([[1.25893332e+03, 0.00000000e+00, 5.22304630e+02],
                              [0.00000000e+00, 1.27293638e+03, 4.92206435e+02],
                              [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
    camera_distortion = np.array([[ -0.4894947,
                                    0.20673354,
                                    -0.0178347,
                                    0.05254465,
                                    -0.06985189]])  
    # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_50)  # Definir diccionario de marcadores ArUco
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # cap = cv2.VideoCapture(0)  # Inicializar captura de vídeo desde la dirección
    rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
    cap = cv2.VideoCapture(rtsp_url)  # Inicializa la captura desde la cámara RTSP
    cap.set(3, 640)  # Establecer el ancho del fotograma de la cámara
    cap.set(4, 480)  # Establecer el alto del fotograma de la cámara
    
    counter = 0
    Posicionado = False
    Acercado = False
    # Uso de la clase FiltroDatos
    filtro = FiltroDatos()
    

    while True:  # Bucle infinito para capturar y procesar los fotogramas
        
        cap = cv2.VideoCapture(rtsp_url)
        ret, frame = cap.read()  # Leer un fotograma del vídeo
        if not ret:  # Verificar si la lectura del fotograma fue exitosa
            break  # Salir del bucle si la lectura no fue exitosa

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Convertir el fotograma a escala de grises
        corners, ids, rejected = aruco_detector.detectMarkers(gray_frame)  # Detectar marcadores ArUco

        # Verificar si se detectaron marcadores y si el ID objetivo está en la lista de IDs
        if ids is not None and ID_Objetivo in ids:
            # Obtener el índice del marcador con el ID objetivo
            indice = np.where(ids == ID_Objetivo)[0][0]
            # Filtrar los marcadores y los IDs para mantener solo la información del marcador objetivo
            corners = [corners[indice]]
            ids = [ID_Objetivo]

            # Dibujar los marcadores detectados en el fotograma
            cv2.aruco.drawDetectedMarkers(frame, corners)
            # Estimar la pose del marcador objetivo
            rvec_list_all, tvec_list_all, _objPoints = my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            rvec = rvec_list_all[0]
            tvec = tvec_list_all[0]
            # Dibujar ejes de coordenadas en el marcador objetivo
            cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 100)
            rvec_flipped = rvec * -1
            tvec_flipped = tvec * -1
            rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            pitch, roll, yaw = rotationMatrixToEulerAngles(rotation_matrix)
            #Agregar las coordenadas y el ángulo a la lista
            x = round(realworld_tvec[0, 0], 3)  # Redondea el valor de la primera fila
            y = round(realworld_tvec[1, 0], 3)  # Redondea el valor de la segunda fila
            z = round(realworld_tvec[2, 0], 3)  # Redondea el valor de la tercera fila
            yaw = round(math.degrees(yaw),3)
           # Imprimir las coordenadas formateadas
            print("ENCONTRADO")
            # print(x)
            # print(y)
            # print(z)
            # print(theta)
            x_filtrado, y_filtrado, z_filtrado, yaw_filtrado = filtro.filtrar_datos(x, y, z, yaw)
            print(f"Filtrado: x={x_filtrado}, y={y_filtrado}, z={z_filtrado}, yaw={yaw_filtrado}")

            tvec_str = "P x=%2.0f y=%2.0f  z=%2.0f y =%2.0f" % (x_filtrado, y_filtrado, z_filtrado, yaw_filtrado)
            # cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
            
            # Calcular el offset
            puntos_referencia = [(475,-22), (1000,-64), (1500, -138)]  # Tres puntos de (z, error)
            x_off = calcular_offset(z_filtrado, puntos_referencia)

            # Mostrar el texto con los valores filtrados en el frame
            cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)

            # Agregar el offset debajo del texto actual
            offset_text = "Offset x: %.2f" % x_off
            cv2.putText(frame, offset_text, (20, 500), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)
            
            x_seteado = x_filtrado - x_off
            x_offset_text = " Set x: %.2f" % x_seteado
            cv2.putText(frame, x_offset_text, (20, 540), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)

            if Posicionado == False:    
                Posicionado = pid.Posicionamiento(int(x_seteado/2), yaw_filtrado)

            if Acercado == False:
                Acercado = pid.AvanzarHasta(int(z_filtrado/2),calcular_tiempo(int(z_filtrado/2),10))


            # coordenadas = [x, y, z, yaw] #guardamos en coordanadas los datos en una lista
            
            # inicio = False
            # if inicio == False:
                # print("CONECTANDO A BLUETOOTH")
                # pid.Conectar_Blu_Auto() #ejecutamos la funcion para conectar el bluetooth

            # #Extraemos las velocidades, ingresando las distancias en Z, X y angulo
            # velocidad_Z = pid.compute_Z(int(coordenadas[2]))
            # velocidad_X = pid.compute_X(int(coordenadas[0]))
            # angulo = pid.compute_Angulo(int(coordenadas[3]))
            # print("Datos sensados: ",velocidad_X, velocidad_Z, angulo) #Bandera para verificacion de datos computados

            #NO OLVIDES ESTO
            # pid.Matriz_transformacion_Motores(velocidad_X, velocidad_Z,angulo) #Envio para transformar las velocidades delantera, frontal y angular a la velocidad para cada rueda

            # tvec_str = "P x=%2.0f y=%2.0f  z=%2.0f p =%2.0f r =%2.0f y =%2.0f" % (realworld_tvec[0], realworld_tvec[1],realworld_tvec[2], math.degrees(pitch), math.degrees(roll),math.degrees(yaw))

            

        # Crear una ventana con el nombre 'DETECCION DE ARUCO'
        cv2.namedWindow('DETECCION DE ARUCO')

        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        # Mostrar el fotograma con los marcadores y las líneas guía
        height, width = resized_frame.shape[:2]

        # Dibujar la línea vertical central
        cv2.line(resized_frame, (width // 2, 0), (width // 2, height), (0, 255, 0), 2)

        # Dibujar la línea horizontal central
        cv2.line(resized_frame, (0, height // 2), (width, height // 2), (0, 255, 0), 2)

        # Dibujar las 4 líneas guía horizontales equidistantes
        for i in range(1, 3):
            offset = height // 4 * i
            cv2.line(resized_frame, (0, offset), (width, offset), (255, 0, 0), 1)  # Azul
            cv2.line(resized_frame, (0, height - offset), (width, height - offset), (255, 0, 0), 1)

        # Mostrar el resultado
        cv2.imshow('DETECCION DE ARUCO', resized_frame)


        key = cv2.waitKey(3) & 0xFF  # Esperar 1 milisegundo para la entrada del teclado

        
        ###### Regla para que salga de la funcion cuando se cumpla que la distancia es Z es menor a 200 y en X menor a 10 
        # if coordenadas[2] <= 200 and abs(coordenadas[0]) <= 10:
        #     print("Entro en el if")
        #     pid.Matriz_transformacion_Motores(0,0,0) #Envio de velocidades en 0 para que se detenga el robot
        #     time.sleep(1) #Tiempo para la escritura
        #     cap.release()  # Liberar la captura de vídeo
        #     cv2.destroyAllWindows()  # Cerrar todas las ventanas
        #     bandera_FIN_PID = 1 # Envio 1 en fin bandera para saber que se termino este proceso y publicar en thingsboard
        #     return bandera_FIN_PID

        if Posicionado == True and Posicionado == True:
            Posicionado = False
            Acercado = False
            print("MOVIMIENTO DE POSICION FINALIZADO")
            cap.release()  # Liberar la captura de vídeo
            cv2.destroyAllWindows()  # Cerrar todas las ventanas
            bandera_FIN_PID = 1 # Envio 1 en fin bandera para saber que se termino este proceso y publicar en thingsboard
            return 
            break


    cap.release()  # Liberar la captura de vídeo
    cv2.destroyAllWindows()  # Cerrar todas las ventanas

def calcular_tiempo(distancia_cm: float, velocidad_cm_s: float = 100.0) -> float:
    """
    Calcula el tiempo en segundos para recorrer una distancia dada.

    Parámetros:
    - distancia_cm (float): La distancia en centímetros.
    - velocidad_cm_s (float): La velocidad en centímetros por segundo (por defecto 100 cm/s).

    Retorna:
    - float: El tiempo en segundos.
    """
    if velocidad_cm_s <= 0:
        raise ValueError("La velocidad debe ser mayor a 0")
    
    tiempo_s = distancia_cm / velocidad_cm_s
    print(f"Tiempo requerido: {tiempo_s} segundos")
    return tiempo_s

def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
    '''
    This will estimate the rvec and tvec for each of the marker corners detected by:
       corners, ids, rejectedImgPoints = detector.detectMarkers(image)
    corners - is an array of detected corners for each detected marker in the image
    marker_size - is the size of the detected markers
    mtx - is the camera matrix
    distortion - is the camera distortion matrix
    RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
    '''
    marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
    trash = []
    rvecs = []
    tvecs = []
    
    for c in corners:
        nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
        rvecs.append(R)
        tvecs.append(t)
        trash.append(nada)
    return rvecs, tvecs, trash


def calcular_offset(distancia_z: float, puntos: list) -> float:
    """
    Calcula el offset en el eje X en función de la distancia z utilizando tres puntos de referencia.

    Parámetros:
    - distancia_z (float): La distancia en el eje z.
    - puntos (list): Una lista de tuplas con tres puntos de referencia (distancia_z, error_x).

    Retorna:
    - float: El offset calculado.
    """
    # Asegurarse de que hay tres puntos
    if len(puntos) != 3:
        raise ValueError("Se deben proporcionar exactamente tres puntos de referencia.")

    # Ordenar los puntos por distancia_z
    puntos = sorted(puntos, key=lambda p: p[0])

    # Descomponer los puntos
    z1, x1 = puntos[0]
    z2, x2 = puntos[1]
    z3, x3 = puntos[2]

    # Comprobar en qué tramo de la distancia está el valor de z
    if distancia_z < z1:
        # Si z está antes del primer punto, hacer una extrapolación usando el primer tramo
        offset = x1 + (distancia_z - z1) * (x2 - x1) / (z2 - z1)
    elif distancia_z > z3:
        # Si z está después del último punto, hacer una extrapolación usando el último tramo
        offset = x2 + (distancia_z - z2) * (x3 - x2) / (z3 - z2)
    else:
        # Si z está entre los dos puntos, usar interpolación lineal
        if distancia_z <= z2:
            offset = x1 + (distancia_z - z1) * (x2 - x1) / (z2 - z1)
        else:
            offset = x2 + (distancia_z - z2) * (x3 - x2) / (z3 - z2)

    return offset

# class FiltroDatos:
#     def __init__(self, umbral=500):
#         self.historico_x = []
#         self.historico_y = []
#         self.historico_z = []
#         self.historico_theta = []
#         self.umbral = umbral

#     def agregar_dato(self, dato, historico):
#         dato = round(dato)  # Redondear el dato a un entero sin decimales
        
#         if len(historico) > 0:
#             penultimo_dato = historico[-1]
#             if abs(dato - penultimo_dato) > self.umbral:
#                 return np.mean(historico)  # Retornar la media de los datos sin agregar el nuevo dato si supera el umbral
        
#         if len(historico) >= 5:
#             historico.pop(0)
        
#         historico.append(dato)
#         return np.mean(historico)  # Retornar la media de los datos

#     def filtrar_datos(self, x, y, z, theta):
#         x_filtrado = self.agregar_dato(x, self.historico_x)
#         y_filtrado = self.agregar_dato(y, self.historico_y)
#         z_filtrado = self.agregar_dato(z, self.historico_z)
#         theta_filtrado = self.agregar_dato(theta, self.historico_theta)
#         return round(x_filtrado), round(y_filtrado), round(z_filtrado), round(theta_filtrado)



class FiltroDatos:
    def __init__(self, umbral=500, peso_reciente=0.7, tamano_ventana=5, umbral_recuperacion=1000):
        self.historico_x = []
        self.historico_y = []
        self.historico_z = []
        self.historico_theta = []
        self.umbral = umbral
        self.peso_reciente = peso_reciente
        self.tamano_ventana = tamano_ventana
        self.umbral_recuperacion = umbral_recuperacion  # Umbral para permitir una rápida recuperación

    def agregar_dato(self, dato, historico, umbral_recuperacion=False):
        dato = round(dato)  # Redondear el dato a un entero

        # Si está activado el modo de recuperación, permitimos un umbral mayor
        umbral_a_usar = self.umbral_recuperacion if umbral_recuperacion else self.umbral

        # Descartar datos extremos respecto al último valor si el histórico existe
        if historico and abs(dato - historico[-1]) > umbral_a_usar:
            # Si se detecta un gran cambio, utilizamos el valor actual directamente y activamos la recuperación
            return dato

        # Mantener la ventana del histórico acotada
        if len(historico) >= self.tamano_ventana:
            historico.pop(0)

        historico.append(dato)
        
        # Media móvil ponderada para suavizar
        return np.average(historico, weights=self.calcular_pesos(len(historico)))

    def calcular_pesos(self, n):
        # Calcular pesos para media móvil ponderada
        pesos = [(self.peso_reciente ** (n - i - 1)) for i in range(n)]
        return pesos

    def filtrar_datos(self, x, y, z, theta):
        # Detectar si hay un cambio grande en los datos
        x_filtrado = self.agregar_dato(x, self.historico_x, abs(x - self.historico_x[-1]) > self.umbral if self.historico_x else False)
        y_filtrado = self.agregar_dato(y, self.historico_y, abs(y - self.historico_y[-1]) > self.umbral if self.historico_y else False)
        z_filtrado = self.agregar_dato(z, self.historico_z, abs(z - self.historico_z[-1]) > self.umbral if self.historico_z else False)
        theta_filtrado = self.agregar_dato(theta, self.historico_theta, abs(theta - self.historico_theta[-1]) > self.umbral if self.historico_theta else False)
        
        # Si hubo un cambio grande, se retorna el valor actual para que el filtro se recupere rápidamente
        return round(x_filtrado), round(y_filtrado), round(z_filtrado), round(theta_filtrado)

class PIDController:
    def __init__(self, kp , ki , kd , setpoint_Z, setpoint_X ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint_Z = setpoint_Z
        self.setpoint_X = setpoint_X
        self.prev_error_Z = self.prev_error_X = 0
        self.integral_Z = self.integral_X = 0
        self.serialArduino = None  # El objeto serial será configurado después.


        # self.mqtt_broker = "192.168.11.34"  # Cambiar a la dirección IP del servidor de thingsboard
        # self.mqtt_port = 1883 #puerto de envio de datos para Mqtt
        # self.mqtt_topic = "v1/devices/me/telemetry"  


    #Funcion para detemerinar la velocidad del angulo en funcion de que tan lejos esta del angulo que buscamos (PID Rustico)
    def compute_Angulo(self, Angulo):
        print("Esto es angulo: ", Angulo )

        if Angulo > 0:
            if Angulo > 50:
                return 10
            elif Angulo > 20:
                return 6
            elif Angulo > 2:
                return 4
            else :
                return 0
        if Angulo <= 0:
            Angulo = abs(Angulo)
            if Angulo > 50:
                return -10
            elif Angulo > 20:
                return -6
            elif Angulo > 2:
                return -4
            else :
                return 0

    ##Funcion para detemerinar la velocidad frontal en funcion de que tan lejos esta del aruco en Z (PID Rustico)
    def compute_Z(self, DisranciaZ):
        print("Esto es distancia z: ", DisranciaZ )

        if DisranciaZ > 1500:
            return 1.5
        elif DisranciaZ > 600:
            return 1
        elif DisranciaZ > 200:
            return 0.5
        else :
            return 0

    ##Funcion para detemerinar la velocidad Lateral en funcion de que tan lejos esta del aruco en X (PID Rustico)
    def compute_X(self, DistanciaX):

        print("Esto es distancia x: ", DistanciaX )

        if DistanciaX >0:
            if DistanciaX > 1000:
                return 1.5
            elif DistanciaX > 400:
                return 1
            elif DistanciaX >10:
                return 0.5
            else:
                return 0
            
        elif DistanciaX <= 0:

            DistanciaX = abs(DistanciaX)
            if DistanciaX > 1000:
                return -1.5
            elif DistanciaX > 400:
                return -1
            elif DistanciaX >10:
                return -0.5
            else:
                return 0


    #Funcion para determinar la velocidad de cada rueda usando la ecuacion cinametica del ROBOT mecanum de 4 ruedas (Esta ecuacion toma la velocidad: Frontal, lateral y angular)
    def Matriz_transformacion_Motores(self, velocidad_X, velocidad_Z, angulo):
        # Definir las matrices
        matriz1 = np.array([[1, -1, -0.33], #el valor de -0.33 corresponde a una constante la cual es la suma de la distancia entre los ejes de la rueda y la distancia entre la mitad de la rueda y la mitad del robot
                            [1, 1, 0.33],
                            [1, 1, -0.33],
                            [1, -1, 0.33]])

        matriz2 = np.array([[velocidad_Z],
                            [velocidad_X],
                            [angulo]])

        # Multiplicación de matrices
        resultado = np.dot(matriz1, matriz2)
        resultado_multiplicado = resultado * (1/ 0.05) #el 0.05 corresponde al radio de la rueda mecanum
       
        self.W1 = max(min(resultado_multiplicado[0][0], 255), -255) #Limitamos la salida a 255 y -255
        self.W2 = max(min(resultado_multiplicado[1][0], 255), -255)
        self.W3 = max(min(resultado_multiplicado[2][0], 255), -255)
        self.W4 = max(min(resultado_multiplicado[3][0], 255), -255)


        # Imprimir los valores de las variables individuales para ver que sale xd
        print("Valor de W1:", self.W1)
        print("Valor de W2:", self.W2)
        print("Valor de W3:", self.W3)
        print("Valor de W4:", self.W4)
        print("AQUI SE ENVIA A LOS MOTORES")

        # NO OLVIDES DESCOMENTAR ESTO
        self.Enviar_arduino_blu() #funcion para envio de datos por bluetooth
    
    #Funcion para concetar bluetooth mediante el puerto bluetooth de mi pc
    def Conectar_Blu_Auto(self, PortserialBT):
        """
        Configura el puerto Bluetooth usando el objeto pasado desde el Main.
        :param PortserialBT: Objeto serial ya inicializado desde el Main.
        """
        if not PortserialBT or not PortserialBT.is_open:
            raise ValueError("El puerto Bluetooth proporcionado no es válido o está cerrado.")
        
        # Asigna el puerto ya configurado
        self.serialArduino = PortserialBT
        print(f"Conexión configurada correctamente con el puerto: {self.serialArduino.port}")
        
    
    def Posicionamiento(self, x_, yaw_a):
        # Validación y ajuste de yaw
        if abs(yaw_a) > 15:
            direccion_giro = "Horario" if x_ > 0 else "AntiHorario"
            print(f"Girando {direccion_giro} para ajustar yaw")
            self.GiroRobot(direccion_giro, yaw_a)
            time.sleep(5)
            return False  # Yaw no está ajustado aún

        # Solo validar y ajustar x si la validación de yaw es positiva
        if abs(yaw_a) <= 15:
            if abs(x_) > 200:
                direccion_desplazar = "Izquierda" if x_ > 0 else "Derecha"
                ajuste_x = int(x_ / 10)  # Ajuste del desplazamiento en pasos
                print(f"Desplazando {ajuste_x}, hacia {direccion_desplazar} para ajustar posición en x")
                self.Desplazar(direccion_desplazar, ajuste_x)
                time.sleep(10)  # Pausa para completar el desplazamiento
                return False  # Indica que la posición aún no está ajustada


        # Si ambas condiciones se cumplen, regresar True
        print("Posicionamiento completado correctamente")
        return True

    def GiroRobot(self, Direccion, yaw_a):
            if Direccion == "Horario":
                print("GIRO HORARIO")
                data = f"19,{yaw_a}"
               
               
            elif Direccion == "AntiHorario":

                print("GIRO ANTIHORARIO")
                data = f"19,-{yaw_a}"
                
                
            else:
                print("Dirección no válida. Use 'Horario' o 'AntiHorario'.")
                return

            print("AQUI SE ENVIA A LOS MOTORES")
            
            # Aquí debes agregar la lógica para enviar los valores a los motores
            self.Enviar_arduino_blu(data) #funcion para envio de datos por bluetooth

    # def GiroRobotTime(self, Direccion, Velocidad, Duracion):
    #         if Direccion == "Horario":
    #             print("GIRO HORARIO")
    #             self.W1 = Velocidad
    #             self.W2 = -Velocidad
    #             self.W3 = Velocidad
    #             self.W4 = -Velocidad
    #         elif Direccion == "AntiHorario":

    #             print("GIRO ANTIHORARIO")
    #             self.W1 = -Velocidad
    #             self.W2 = Velocidad
    #             self.W3 = -Velocidad
    #             self.W4 = Velocidad
    #         else:
    #             print("Dirección no válida. Use 'Horario' o 'AntiHorario'.")
    #             return

    #         # Imprimir los valores de las variables individuales para ver qué sale
    #         print("Valor de W1:", self.W1)
    #         print("Valor de W2:", self.W2)
    #         print("Valor de W3:", self.W3)
    #         print("Valor de W4:", self.W4)
    #         print("AQUI SE ENVIA A LOS MOTORES")

    #         time.sleep(Duracion)
            
    #         # Aquí debes agregar la lógica para enviar los valores a los motores
    #         self.Enviar_arduino_blu() #funcion para envio de datos por bluetooth

    #         self.W1 = 0
    #         self.W2 = 0
    #         self.W3 = 0
    #         self.W4 = 0


    #         # Imprimir los valores de las variables individuales para ver qué sale
    #         print("Valor de W1:", self.W1)
    #         print("Valor de W2:", self.W2)
    #         print("Valor de W3:", self.W3)
    #         print("Valor de W4:", self.W4)
    #         print("AQUI SE ENVIA A LOS MOTORES")
            
    #         # Aquí debes agregar la lógica para enviar los valores a los motores
    #         self.Enviar_arduino_blu() #funcion para envio de datos por bluetooth


    def Avanzar(self, Direccion, Velocidad):
            if Direccion == "Adelante":
                print("AVANZAR ADELANTE")
                self.W1 = Velocidad
                self.W2 = Velocidad
                self.W3 = Velocidad
                self.W4 = Velocidad
            elif Direccion == "Atras":
                print("AVANZAR ATRAS")
                self.W1 = -Velocidad
                self.W2 = -Velocidad
                self.W3 = -Velocidad
                self.W4 = -Velocidad
            else:
                print("Dirección no válida. Use 'Adelante' o 'Atras'.")
                return

            # Imprimir los valores de las variables individuales para ver qué sale
            print("Valor de W1:", self.W1)
            print("Valor de W2:", self.W2)
            print("Valor de W3:", self.W3)
            print("Valor de W4:", self.W4)
            print("AQUI SE ENVIA A LOS MOTORES")
            
            # Aquí debes agregar la lógica para enviar los valores a los motores
            self.Enviar_arduino_blu() #funcion para envio de datos por bluetooth

    def Desplazar(self, Direccion, Distancia):
            if Direccion == "Derecha":
                print("DESPLAZAR DERECHA")
                data = f"18,{Distancia}"
            elif Direccion == "Izquierda":
                print("DESPLAZAR IZQUIERDA")
                data = f"18,-{Distancia}"
                
            else:
                print("Dirección no válida. Use 'Derecha' o 'Izquierda'.")
                return
            
            print("AQUI SE ENVIA A LOS MOTORES")
            
            # Aquí debes agregar la lógica para enviar los valores a los motores
            self.Enviar_arduino_blu(data) #funcion para envio de datos por bluetooth


        
    def AvanzarHasta(self, posicion, time):
            # Imprime la posición a la que se debe avanzar
            print(f"Avanzar hasta la posición {posicion}")
            
            # Llama a la función para enviar datos por Bluetooth
            # Suponiendo que 'Funcion' es 'Avanzar' y 'Param' es 'posicion'
            # self.Enviar_arduino_blu2(Funcion='Avanzar', Param=posicion)
            data = f"17,{posicion}"
            self.Enviar_arduino_blu(data)
            # Si ambas condiciones se cumplen, regresar True    
            print("Posicionamiento completado correctamente")
            time.sleep(time)
            return True
        

        
        #Funcion de envio de datos mediante bluetooth
    def Enviar_arduino_blu(self, datos):
        """
        Envía datos al Arduino conectado por Bluetooth.
        :param datos: Datos a enviar (string).
        """
        if not self.serialArduino or not self.serialArduino.is_open:
            raise ConnectionError("No hay conexión activa con el dispositivo Bluetooth.")

        try:
            mensaje_con_salto = datos + "\n"  # Agrega el salto de línea
            self.serialArduino.write(mensaje_con_salto.encode())  # Envía datos al puerto
            print(f"Datos enviados: {datos}")
        except Exception as e:
            print(f"Error al enviar datos: {e}")
            raise


