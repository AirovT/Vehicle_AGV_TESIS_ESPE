import math
import cv2
import numpy as np
from typing import List, Tuple, Any
import paho.mqtt.client as mqtt
from tb_device_mqtt import TBDeviceMqttClient, TBPublishInfo
import time
import logging
import socket 
import json
import serial
import matplotlib.pyplot as plt

logging.basicConfig(level=logging.INFO)

counter = 0


class MovementController:
    
    def __init__(self, BTPort: serial.Serial, bandera_init: list, bandera_fin: list):
        self.BTPort = BTPort
        self.bandera_init = bandera_init  # Shared list reference
        self.bandera_fin = bandera_fin    # Shared list reference
        self.serialArduino = None

        # Parámetros configurables
        self.UMBRAL_ROTACION = 3    # Grados
        self.UMBRAL_POSICION = 5    # Centímetros
        self.DISTANCIA_META = 40    # Centímetros
        self.TIMEOUT_DETECCION = 2  # Segundos
        self.MAX_INTENTOS = 3       # Intentos de recuperación

        # Parámetros de calibración y configuración de ArUco
        self.marker_size = 180  # Tamaño del marcador ArUco
        self.camera_matrix = np.array([[1.17390158e+03, 0.00000000e+00, 9.70673620e+02],
                                      [0.00000000e+00, 1.17020056e+03, 4.92174250e+02],
                                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]) 
        self.camera_distortion = np.array([[ -2.97179698e-01,
                                           6.95602029e-02,
                                           2.42497888e-03,
                                           1.90931850e-04,
                                           2.42043840e-03]]) 
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        
        self.Conectar_Blu_Auto(self.BTPort)

    @staticmethod
    def isRotationMatrix(R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    @staticmethod
    def rotationMatrixToEulerAngles(R):
        assert MovementController.isRotationMatrix(R)
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

    @staticmethod
    def my_estimatePoseSingleMarkers(corners, marker_size, mtx, distortion):
        marker_points = np.array([
            [-marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, marker_size / 2, 0],
            [marker_size / 2, -marker_size / 2, 0],
            [-marker_size / 2, -marker_size / 2, 0]
        ], dtype=np.float32)
        trash = []
        rvecs = []
        tvecs = []
        for c in corners:
            nada, R, t = cv2.solvePnP(marker_points, c, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
            trash.append(nada)
        return rvecs, tvecs, trash

    def LocationAruco(self, frame, ID_Objetivo):
        print("🔍 Buscando ArUco...")
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray_frame)
        
        if ids is not None and ID_Objetivo in ids:
            print("✅ ArUco encontrado!")
            indice = np.where(ids == ID_Objetivo)[0][0]
            target_corners = [corners[indice]]
            
            rvec_list_all, tvec_list_all, _ = self.my_estimatePoseSingleMarkers(
                target_corners, self.marker_size, self.camera_matrix, self.camera_distortion
            )
            rvec = rvec_list_all[0]
            tvec = tvec_list_all[0]
            
            rvec_flipped = rvec * -1
            tvec_flipped = tvec * -1
            rotation_matrix, _ = cv2.Rodrigues(rvec_flipped)
            realworld_tvec = np.dot(rotation_matrix, tvec_flipped)
            pitch, roll, yaw = self.rotationMatrixToEulerAngles(rotation_matrix)
            
            x = realworld_tvec[0, 0]
            y = realworld_tvec[1, 0]
            z = realworld_tvec[2, 0]
            roll_deg = math.degrees(roll)
            
            return x, y, z, roll_deg, target_corners, rvec, tvec, True
        else:
            print("❌ ArUco no encontrado")
            return None, None, None, None, None, None, None, False

    def Estimation(self, ID_Objetivo):
        print("🚀 Iniciando proceso de estimación...")
        self.Conectar_Blu_Auto(self.BTPort)
        
        self.filtro = FiltroDatos()
        estado = {
            'rotacion_corregida': False,
            'posicion_corregida': False,
            'distancia_ok': False
        }

        frame = self.capturar_frame()
        x_raw, y_raw, z_raw, roll_raw, corners, rvec, tvec, found = self.LocationAruco(frame, ID_Objetivo)
        
        if not found:
            raise Exception("⚠️ ArUco no encontrado inicialmente")

        iteracion = 1
        while not all(estado.values()):
            print(f"🔄 Iteración {iteracion}...")
            frame = self.capturar_frame()
            
            x, y, z, roll, corners, rvec, tvec, found = self.LocationAruco(frame,ID_Objetivo)
            if not found:
                print("🔄 Intentando recuperar ArUco...")
                self._recuperar_aruco(ID_Objetivo, frame)
                continue
            
            x_filtrado, y_filtrado, z_filtrado, roll_filtrado = self.filtro.filtrar_datos(x, y, z, roll)
            z_cm = z_filtrado / 10
            
            print(f"📍 Posición detectada -> X: {x_filtrado:.1f}, Y: {y_filtrado:.1f}, Z: {z_cm:.1f} cm, Roll: {roll_filtrado:.1f}°")

            if not estado['rotacion_corregida']:
                if abs(roll_filtrado) > self.UMBRAL_ROTACION:
                    print("🔄 Corrigiendo rotación...")
                    self._corregir_rotacion(roll_filtrado, ID_Objetivo, frame)
                else:
                    print("✅ Rotación corregida!")
                    estado['rotacion_corregida'] = True
            
            if estado['rotacion_corregida'] and not estado['posicion_corregida']:
                x_cm = x_filtrado/10
                if abs(x_cm) > self.UMBRAL_POSICION:
                    print("➡️ Desplazando lateralmente...")
                    self.Desplazar(x_cm)
                else:
                    print("✅ Posición corregida!")
                    estado['posicion_corregida'] = True
            
            if all([estado['rotacion_corregida'], estado['posicion_corregida']]) and not estado['distancia_ok']:
                if z_cm > self.DISTANCIA_OBJETIVO:
                    print("📏 Ajustando distancia...")
                    self.AvanzarHasta(z_cm / 2)
                    estado.update({'posicion_corregida': False, 'rotacion_corregida': False})
                else:
                    print("✅ Distancia corregida! Alineación completada")
                    self.AvanzarHasta(z_cm)
                    estado['distancia_ok'] = True
            
            iteracion += 1
        print("🎯 Alineación completada exitosamente")

    def _corregir_rotacion(self, roll, ID_Objetivo, frame):
        print("🔄 Corrigiendo rotación del robot...")
        self.GiroRobot(roll)
        start_time = time.time()
        
        while (time.time() - start_time) < self.TIMEOUT_DETECCION:
            frame = self.capturar_frame()
            _, _, _, _, _, _, _, found = self.LocationAruco(frame, ID_Objetivo)
            
            if not found:
                print("⚠️ ArUco perdido, intentando recuperación...")
                direccion = -15 if roll > 0 else 15
                self.Desplazar(direccion)
            else:
                print("✅ ArUco recuperado después de rotación")
                return
        raise Exception("❌ No se pudo recuperar el ArUco después de rotación")


    def _visualizar_deteccion(self, frame, corners, rvec, tvec, x, y, z, roll):
        """Muestra información de detección en el frame"""
        cv2.aruco.drawDetectedMarkers(frame, corners)
        cv2.drawFrameAxes(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 100)
        
        texto = f"X: {x:.1f} cm | Y: {y:.1f} cm | Z: {z:.1f} mm | Roll: {roll:.1f}°"
        cv2.putText(frame, texto, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        scale_percent = 50
        width = int(frame.shape[1] * scale_percent / 100)
        height = int(frame.shape[0] * scale_percent / 100)
        resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

        cv2.namedWindow('DETECCION DE ARUCO')
        h, w = resized_frame.shape[:2]
        cv2.line(resized_frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
        cv2.line(resized_frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
        for i in range(1, 3):
            offset = h//4 * i
            cv2.line(resized_frame, (0, offset), (w, offset), (255, 0, 0), 1)
            cv2.line(resized_frame, (0, h-offset), (w, h-offset), (255, 0, 0), 1)

        # Mostrar frame (opcional)
        cv2.imshow('ArUco Detection', resized_frame)
        cv2.waitKey(1)

    def _recuperar_aruco(self, ID_Objetivo, frame):
        """Estrategia para recuperar el marcador perdido"""
        print("Iniciando protocolo de recuperación de ArUco...")
        # Implementar movimiento en espiral o patrón de búsqueda
        # Ejemplo simple: movimiento lateral alternado
        for _ in range(3):
            self.Desplazar(15)
            # frame = self.capturar_frame()
            if self.LocationAruco(frame, ID_Objetivo)[7]:  # Índice 7 = 'found'
                return
        raise Exception("ArUco críticamente perdido")

    def capturar_frame(self):
        """Captura un frame desde la cámara RTSP"""
        rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
        cap = cv2.VideoCapture(rtsp_url)
        
        if not cap.isOpened():
            raise Exception("No se pudo conectar a la cámara RTSP")
        
        ret, frame = cap.read()
        cap.release()  # Liberar la cámara
        
        if not ret or frame is None:
            raise Exception("Error al capturar el frame desde la cámara")
        
        return frame


    # def Estimation(self, ID_Objetivo):
    #     self.Conectar_Blu_Auto(self.BTPort)
    #     rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
    #     cap = cv2.VideoCapture(rtsp_url)
    #     cap.set(3, 640)
    #     cap.set(4, 480)
        
    #     counter = 0
    #     Posicionado = False
    #     Acercado = False
    #     filtro = FiltroDatos()
    #     iterator = 0

    #     print("\n\nEmpezando iteracion aruco while")
    #     while True:
    #         time.sleep(2)
    #         print(f"\nEstamos en la iteracion: {iterator}")
    #         iterator += 1
    #         ret, frame = cap.read()
    #         if not ret:
    #             break

    #         x, y, z, roll, corners, rvec, tvec, found = self.LocationAruco(frame, ID_Objetivo)
    #         if found:
    #             cv2.aruco.drawDetectedMarkers(frame, corners)
    #             cv2.drawFrameAxes(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 100)
    #             x_filtrado, y_filtrado, z_filtrado, roll_filtrado = filtro.filtrar_datos(x, y, z, roll)
    #             print(f"Filtrado: x={x_filtrado}, y={y_filtrado}, z={z_filtrado}, roll={roll_filtrado}")

    #             tvec_str = f"P x={x_filtrado:2.0f} y={y_filtrado:2.0f} z={z_filtrado:2.0f} y={roll_filtrado:2.0f}"
    #             cv2.putText(frame, tvec_str, (20, 460), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2, cv2.LINE_AA)

    #             if not Posicionado:    
    #                 Posicionado = self.Posicionamiento(int(x_filtrado), roll_filtrado)
    #                 continue

    #             if not Acercado:
    #                 Acercado = self.AvanzarHasta(int(z_filtrado*1/3))
    #                 continue

    #         scale_percent = 50
    #         width = int(frame.shape[1] * scale_percent / 100)
    #         height = int(frame.shape[0] * scale_percent / 100)
    #         resized_frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)

    #         cv2.namedWindow('DETECCION DE ARUCO')
    #         h, w = resized_frame.shape[:2]
    #         cv2.line(resized_frame, (w//2, 0), (w//2, h), (0, 255, 0), 2)
    #         cv2.line(resized_frame, (0, h//2), (w, h//2), (0, 255, 0), 2)
            
    #         for i in range(1, 3):
    #             offset = h//4 * i
    #             cv2.line(resized_frame, (0, offset), (w, offset), (255, 0, 0), 1)
    #             cv2.line(resized_frame, (0, h-offset), (w, h-offset), (255, 0, 0), 1)

    #         cv2.imshow('DETECCION DE ARUCO', resized_frame)
    #         key = cv2.waitKey(3) & 0xFF

    #         if Posicionado and Acercado:
    #             Posicionado = False
    #             Acercado = False
    #             print("MOVIMIENTO DE POSICION FINALIZADO")
    #             break

    #     cap.release()
    #     cv2.destroyAllWindows()
    #     return 1



    
    # def Posicionamiento(self, x_, roll_a):
    #     """Handle positioning with roll correction and lateral movement"""
    #     print("\n📍 INICIANDO POSICIONAMIENTO")
    #     success = True
        
    #     # Roll adjustment
    #     if abs(roll_a) > 2:
    #         print(f"🔄 Ajuste de inclinación requerido: {roll_a:.1f}°")
    #         if not self.GiroRobot(roll_a):
    #             print("❌ Fallo en ajuste de inclinación")
    #             success = False
        
    #     # Lateral movement
    #     if abs(x_) > 5 and success:
    #         print(f"📏 Desplazamiento lateral requerido: {x_:.1f} mm")
    #         displacement = int(x_/10)
    #         try:
    #             self.Desplazar(displacement)
    #             print(f"✅ Desplazamiento completo: {displacement} pasos")
    #         except Exception as e:
    #             print(f"❌ Error en desplazamiento: {str(e)}")
    #             success = False
        
    #     print("🏁 Posicionamiento finalizado\n")
    #     return success

    # def GiroRobot(self, angle):
    #     """Handle rotation with full status tracking"""
    #     print(f"\n🔄 INICIANDO GIRO DE {angle:.1f}°")
        
    #     try:
    #         # Send rotation command
    #         command = f"19,{abs(int(angle))}"
    #         print(f"📡 Enviando comando: {command}")
    #         self._send_command(command)
            
    #         # Wait for movement confirmation
    #         print("⏳ Esperando confirmación de inicio...")
    #         if not self._wait_for_init():
    #             print("⌛ Timeout en inicio de giro")
    #             return False
                
    #         print("⏳ Giro en progreso...")
    #         if not self._wait_for_completion():
    #             print("⌛ Timeout en finalización de giro")
    #             return False
                
    #         print(f"✅ Giro completado: {angle:.1f}°")
    #         return True
            
    #     except Exception as e:
    #         print(f"❌ Error crítico durante el giro: {str(e)}")
    #         return False

    # # Helper methods for flag handling
    # def _wait_for_init(self, timeout=10):
    #     start = time.time()
    #     while not self.bandera_init[0]:
    #         if time.time() - start > timeout:
    #             return False
    #         time.sleep(0.1)
    #     self.bandera_init[0] = False
    #     return True

    # def _wait_for_completion(self, timeout=15):
    #     start = time.time()
    #     while not self.bandera_fin[0]:
    #         if time.time() - start > timeout:
    #             return False
    #         time.sleep(0.1)
    #     self.bandera_fin[0] = False
    #     return True


    def Posicionamiento(self, x_, roll_a):
        """Handle positioning with flag status tracking"""
        print(f"\n📍 INICIANDO POSICIONAMIENTO - Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
        
        # Roll adjustment
        if abs(roll_a) > 2:
            print(f"🔄 Ajuste de roll: {roll_a}°")
            print(f"   Pre-giro Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            if self.GiroRobot(roll_a):
                print(f"   Post-giro Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            else:
                print("❌ Fallo en ajuste de roll")
                return False

        # Lateral movement
        if abs(x_) > 5:
            direction = "Izquierda" if x_ > 0 else "Derecha"
            print(f"➡️ Desplazamiento {direction}")
            print(f"   Pre-desplazamiento Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            try:
                self.Desplazar(int(x_/10))
                print(f"   Post-desplazamiento Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            except Exception as e:
                print(f"❌ Error en desplazamiento: {str(e)}")
                return False

        print(f"🏁 Posicionamiento completado - Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]\n")
        return True

    def GiroRobot(self, angle):
        """Handle rotation with flag monitoring"""
        print(f"\n🔄 INICIANDO GIRO DE {angle}°")
        print(f"   Flags iniciales [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
        print("GIRANDO HORARIO" if angle > 0 else "GIRANDO ANTIHORARIO")

        try:
            self._send_command(f"19,{(int(angle))}")
            print("📡 Comando enviado, esperando INIT...")
            
            if not self._wait_for_init():
                print(f"⌛ Timeout INIT! Flags actuales [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
                return False
                
            print(f"✅ Movimiento iniciado - Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            print("⏳ Esperando FIN...")
            
            if not self._wait_for_completion():
                print(f"⌛ Timeout FIN! Flags actuales [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
                return False
                
            print(f"✅ Giro completado - Flags finales [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            return True
            
        except Exception as e:
            print(f"❌ Error en giro: {str(e)} - Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
            return False

    def _wait_for_init(self, timeout=10):
        """Wait for INIT flag with status printing"""
        start = time.time()
        while not self.bandera_init[0]:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        print(f"🏁 INIT recibido! Flag actual: {self.bandera_init[0]}")
        self.bandera_init[0] = False  # Reset flag
        return True

    def _wait_for_completion(self, timeout=15):
        """Wait for FIN flag with status printing"""
        start = time.time()
        while not self.bandera_fin[0]:
            if time.time() - start > timeout:
                return False
            time.sleep(0.1)
        print(f"🏁 FIN recibido! Flag actual: {self.bandera_fin[0]}")
        self.bandera_fin[0] = False  # Reset flag
        return True

    def _send_command(self, command):
        """Send command with INIT flag reset"""
        print(f"\n📤 Enviando comando: {command}")
        print(f"   Pre-envio Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
        self.bandera_init[0] = False
        self.serialArduino.write(f"{command}\n".encode())
        print(f"   Post-envio Flags [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")

    def Desplazar(self, offset: int):
        """
        Move left/right using signed offset
        - Positive values: Right (Derecha)
        - Negative values: Left (Izquierda)
        """
        print(f"\n🔄 DESPLAZAMIENTO LATERAL: {'Derecha' if offset > 0 else 'Izquierda'} {abs(offset)}u")
        
        # Send command with signed value
        command = f"18,{offset}"
        self._send_command(command)
        
        # Wait for confirmation
        if self._wait_for_movement():
            print(f"✅ Desplazamiento completado: {abs(offset)}u")
            return True
        print("❌ Fallo en desplazamiento lateral")
        return False
    
    def AvanzarHasta(self, distance: int):
        """
        Move forward/backward using signed distance
        - Positive values: Forward (Adelante)
        - Negative values: Backward (Atrás)
        """
        print(f"\n🚀 AVANCE LONGITUDINAL: {'Adelante' if distance > 0 else 'Atrás'} {abs(distance)}u")
        
        # Send command with signed value
        command = f"17,{distance}"
        self._send_command(command)
        
        # Wait for confirmation
        if self._wait_for_movement():
            print(f"✅ Avance completado: {abs(distance)}u")
            return True
        print("❌ Fallo en avance longitudinal")
        return False

    def _wait_for_movement(self, timeout=15):
        """Wait for movement confirmation flags"""
        start_time = time.time()
        
        # Wait for INIT
        while not self.bandera_init[0]:
            if time.time() - start_time > timeout:
                print(f"⌛ Timeout INIT! Flags: [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
                return False
            time.sleep(0.1)
        print(f"🏁 Movimiento iniciado - Flags: [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
        
        # Wait for FIN
        start_time = time.time()
        while not self.bandera_fin[0]:
            if time.time() - start_time > timeout:
                print(f"⌛ Timeout FIN! Flags: [INIT: {self.bandera_init[0]}, FIN: {self.bandera_fin[0]}]")
                return False
            time.sleep(0.1)
        
        # Reset flags
        self.bandera_init[0] = False
        self.bandera_fin[0] = False
        return True

    def Conectar_Blu_Auto(self, PortserialBT):
        if not PortserialBT or not PortserialBT.is_open:
            raise ValueError("El puerto Bluetooth proporcionado no es válido o está cerrado.")
        self.serialArduino = PortserialBT
        print(f"Conexión configurada correctamente con el puerto: {self.serialArduino.port}")
    
    def Enviar_arduino_blu(self, datos):
        if not self.serialArduino or not self.serialArduino.is_open:
            raise ConnectionError("No hay conexión activa con el dispositivo Bluetooth.")
        try:
            mensaje_con_salto = datos + "\n"
            self.serialArduino.write(mensaje_con_salto.encode())
            print(f"Datos enviados: {datos}")
            time.sleep(1)
        except Exception as e:
            print(f"Error al enviar datos: {e}")
            raise

class FiltroDatos:
    def __init__(self, umbral=500, peso_reciente=0.7, tamano_ventana=5, umbral_recuperacion=1000):
        self.historico_x = []
        self.historico_y = []
        self.historico_z = []
        self.historico_theta = []
        self.umbral = umbral
        self.peso_reciente = peso_reciente
        self.tamano_ventana = tamano_ventana
        self.umbral_recuperacion = umbral_recuperacion

    def agregar_dato(self, dato, historico, umbral_recuperacion=False):
        dato = round(dato)
        umbral_a_usar = self.umbral_recuperacion if umbral_recuperacion else self.umbral

        if historico and abs(dato - historico[-1]) > umbral_a_usar:
            return dato

        if len(historico) >= self.tamano_ventana:
            historico.pop(0)

        historico.append(dato)
        return np.average(historico, weights=self.calcular_pesos(len(historico)))

    def calcular_pesos(self, n):
        return [(self.peso_reciente ** (n - i - 1)) for i in range(n)]

    def filtrar_datos(self, x, y, z, theta):
        x_filtrado = self.agregar_dato(x, self.historico_x, abs(x - (self.historico_x[-1] if self.historico_x else 0)) > self.umbral)
        y_filtrado = self.agregar_dato(y, self.historico_y, abs(y - (self.historico_y[-1] if self.historico_y else 0)) > self.umbral)
        z_filtrado = self.agregar_dato(z, self.historico_z, abs(z - (self.historico_z[-1] if self.historico_z else 0)) > self.umbral)
        theta_filtrado = self.agregar_dato(theta, self.historico_theta, abs(theta - (self.historico_theta[-1] if self.historico_theta else 0)) > self.umbral)
        return round(x_filtrado), round(y_filtrado), round(z_filtrado), round(theta_filtrado)