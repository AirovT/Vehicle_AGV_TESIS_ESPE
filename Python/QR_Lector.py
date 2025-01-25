import requests
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from io import BytesIO
import paho.mqtt.client as mqtt
import json
import time
import os

# Diccionarios para almacenar datos
diccionarioProductos = {
    "MT01": "Martillo Metalico", 
    "MT02": "Martillo Goma", 
    "ST01": "Estilete Plastico", 
    "DX01": "Destornillador Estrella", 
    "DI01": "Destornillador Plano", 
    "DI02": "Dest-Plano Madera"
}

class QRLector:
    def __init__(self):
        # Configuración MQTT
        mqtt_server_ip = "192.168.1.5"
        self.mqtt_server_ip = mqtt_server_ip
        self.client = mqtt.Client()
        self.client.connect(self.mqtt_server_ip)
        self.client.loop_start()  # Iniciar el bucle de red para manejar las publicaciones

    def procesar_codigo_qr(self, video_source):
        print("Iniciando el procesamiento de código QR...")

        # Verificar si el video_source es una imagen PNG o un flujo de video
        if os.path.isfile(video_source) and video_source.endswith(".png"):
            self.procesar_imagen(video_source)
        else:
            self.procesar_video(video_source)

    def procesar_video(self, video_source):
        while True:
            try:
                response = requests.get(video_source, stream=True)
                response.raise_for_status()  # Verificar si la solicitud fue exitosa
                bytes_data = BytesIO(response.content)
                byte_frame = bytes_data.getvalue()

                # Convertir los bytes a una imagen de OpenCV
                nparr = np.frombuffer(byte_frame, np.uint8)
                frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if frame is None:
                    print("No se pudo decodificar el frame")
                    continue

                # Mostrar la imagen en una ventana de OpenCV
                cv2.imshow('Cámara en Vivo', frame)

                # Detección de códigos QR en el cuadro
                decoded_objects = decode(frame)
                if decoded_objects:
                    for obj in decoded_objects:
                        qr_data = obj.data.decode('utf-8')
                        datos = self.separacion(qr_data)
                        if datos:
                            self.publicar_datos(datos)  # Publicar los datos en MQTT
                            time.sleep(5)
                            cv2.destroyAllWindows()  # Cerrar la ventana cuando se detecta un código QR válido
                            return datos  # Retornar los datos y detener la función

                # Pausa para permitir la actualización de la ventana
                cv2.waitKey(100)

            except requests.RequestException as req_err:
                print(f"Error en la solicitud HTTP: {req_err}")
            except cv2.error as cv_err:
                print(f"Error en OpenCV: {cv_err}")
            except Exception as e:
                print(f"Error inesperado: {e}")

        cv2.destroyAllWindows()  # Asegurarse de cerrar la ventana en caso de error o salida

    def procesar_imagen(self, imagen_path):
        try:
            qr_imagen = cv2.imread(imagen_path)
            if qr_imagen is None:
                print("No se pudo cargar la imagen")
                return

            decoded_objects = decode(qr_imagen)
            if decoded_objects:
                for obj in decoded_objects:
                    qr_data = obj.data.decode('utf-8')
                    datos = self.separacion(qr_data)
                    if datos:
                        self.publicar_datos(datos)  # Publicar los datos en MQTT
                        return datos  # Retornar los datos procesados
            else:
                print("No se detectó ningún código QR en la imagen.")

        except Exception as e:
            print(f"Error al procesar la imagen: {e}")

    def separacion(self, qr_data):
        if len(qr_data) != 14:
            print("Error: El código QR no tiene el tamaño esperado de 14 caracteres.")
            return None

        ID = qr_data[:3]
        Separador1 = qr_data[3:5]
        ProductoCodigo = qr_data[5:9]
        Separador2 = qr_data[9:11]
        Celda = qr_data[11:14]

        # Validación de separadores
        if Separador1 != "AA" or Separador2 != "BB":
            print("Error: Los separadores no coinciden con los valores esperados.")
            return None

        Producto = diccionarioProductos.get(ProductoCodigo, "Producto desconocido")

        datos = {
            "ID": ID,
            "Producto": Producto,
            "Celda": Celda
        }

        print("DATOS PROCESADOS:", datos)
        return datos

    def publicar_datos(self, datos):
        for clave, valor in datos.items():
            topic = f"InformacionBox/{clave}"
            try:
                self.client.publish(topic, valor)
                print(f"Publicado en {topic}: {valor}")
            except Exception as e:
                print(f"Error al publicar en {topic}: {e}")
            time.sleep(0.1)  # Añadir una pequeña pausa para evitar problemas de publicación

# Ejemplo de uso
lector = QRLector()
# lector.procesar_codigo_qr('http://192.168.1.7/640x480.jpg')  # Desde cámara IP
# lector.procesar_codigo_qr('codigo_qr.png')  # Desde un archivo PNG
# lector.procesar_codigo_qr('http://192.168.1.11:4747/video')  # Desde cámara IP/
