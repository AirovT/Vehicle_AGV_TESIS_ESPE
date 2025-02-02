## Version 3 
#OBJETIVO: Se realizara una interfaz simple, en modo automatico, solo tendra 3 botones y cada que se inicie un proceso de se mostrara
#en una ventana diferente y solo por ese tiempo luego se cerrara para continuar
# NOTA 
# 
from tkinter import messagebox
import tkinter as tk
from tkinter import ttk
from tkinter import Label
from tkinter import Scale
from tkinter import Text
from tkinter import HORIZONTAL
from tkinter import NW
from PIL import Image, ImageTk
import requests
from io import BytesIO
import threading
import time
import os
import numpy as np  # Asegúrate de importar numpy
from QR_Lector import QRLector

import Tesis_Funciones_Version
import IdentificadorIdAruco


import cv2
import Aruco_Medicion_Distancia
from Aruco_Medicion_Distancia import PIDController


import serial

direccionCamera = "http://192.168.1.7/640x480.mjpeg"


##########################################  Variables #########################################z   
valoranteriorcajamesa1 = 3
cajavalormesa1 = 4
errorescajamesa1=1

########################################################## MODO AUTOMATICO ################################################################################

mqtt_server = "192.168.1.9"
mqtt_port = 1883
 
puertoBT = 'COM13'


class VentanaAutomatico:
    def __init__(self, master):

        # Abrir el puerto serial
        print("Abriendo puerto serial...")
        self.serialArduino = serial.Serial(puertoBT, 115200)  # Reemplaza 'COM1' por el puerto serie correspondiente
        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)  # Reemplaza 'COM1' por el puerto serie correspondiente
      
        
        self.master = master
        master.title("Ventana de Modo Automaticó Iniciada")
        self.master.geometry("850x650")
        # Obtener el ancho y alto de la pantalla
        ancho_pantalla = master.winfo_screenwidth()
        alto_pantalla = master.winfo_screenheight()
        
        # Crear la instancia de Tk
        
         
        self.velocidad = 100
        self.BANDERA = False

        # Crear la instancia de la clase VentanaAutomatico y pasarle la instancia de Tk
        # Iniciar el bucle de eventos de Tkinter
        
        # Calcular las coordenadas para centrar la ventana en la parte superior
        x = (ancho_pantalla - 600) // 2  # 600 es el ancho de la ventana
        y = 0  # La ventana estará en la parte superior
        
        # Establecer la posición de la ventana
        master.geometry(f"850x650+{x}+{y}")
        
        # Construir las rutas relativas a las imágenes
        path_logo_U = os.path.join(dir, 'HMI', 'Logo_U.png')
        path_logo_C = os.path.join(dir, 'HMI', 'Logo_M.png')

        # Cargar la imagen del logo de la universidad
        imagen_U = leer_imagen(path_logo_U,(120,100))
        # Cargar la imagen del logo de la carrera
        imagen_C = leer_imagen(path_logo_C,(120,100))

        # Crear un frame para contener las imágenes y el texto
        frame_contenedor = tk.Frame(master)
        frame_contenedor.pack(fill='x')

        # Mostrar el logo de la universidad
        label_logo_universidad = tk.Label(frame_contenedor,image=imagen_U)
        label_logo_universidad.image = imagen_U
        label_logo_universidad.pack(side=tk.LEFT, padx=(20, 0), pady=10, anchor='w')

        # Mostrar el logo de la carrera
        label_logo_carrera = tk.Label(frame_contenedor,image=imagen_C)
        label_logo_carrera.image = imagen_C
        label_logo_carrera.pack(side=tk.RIGHT, padx=(0, 20), pady=10, anchor='e')

        # Mostrar el texto
        texto = "UNIVERSIDAD DE LAS FUERZAS ARMADAS ESPE-L \n\nSISTEMA DE NAVEGACION AUTONOMA \n\nCONTROL DE MANDO: AUTOMÁTICO"
        label_texto = tk.Label(frame_contenedor, text=texto, font=("Arial", 14, "bold"))
        label_texto.pack(side=tk.LEFT, expand=True, fill="both", padx=(10, 0), pady=10, anchor='center')

        ########################## Frame de indicadores ###################
        # Crear marco principal para dividir en dos áreas
        self.contenedor_principal = tk.Frame(master)
        self.contenedor_principal.pack(fill=tk.BOTH, expand=True)

        # Crear columna izquierda
        self.lado_izquierdo = tk.Frame(self.contenedor_principal, width=200)
        self.lado_izquierdo.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)

        # Crear los óvalos con las etiquetas (inicialmente en gris)
        self.estados_robot = [
            ("ROBOT EN MOVIMIENTO", "gray"),
            ("ROBOT ESPERANDO DATO", "gray"),
            ("ROBOT CONECTADO", "green")
        ]

        self.ovalo_widgets = []  # Lista para almacenar los widgets de los óvalos

        for texto, color in self.estados_robot:
            etiqueta = tk.Label(
                self.lado_izquierdo, 
                text=texto, 
                font=("Arial", 10, "bold"),  # Texto en negrillas
                anchor="center",  # Centramos el texto
                justify="center"  # Aseguramos alineación centrada
            )
            etiqueta.pack(anchor="center", pady=5)  # Centramos la etiqueta en el marco

            canvas = tk.Canvas(self.lado_izquierdo, width=80, height=80)
            canvas.pack(anchor="center", pady=5)  # Centramos el canvas
            ovalo = canvas.create_oval(10, 10, 70, 70, fill=color)  # Óvalos de 20px de diámetro
            self.ovalo_widgets.append((canvas, ovalo))  # Almacenar el canvas y el óvalo para cambiar colores


        ##################################################################################

        
        self.Area_botonesAuto = tk.Frame(master)  # Crear un frame para contener los botones
        self.Area_botonesAuto.pack(pady=10) 

        # Botón "Regresar"
        rutaAnterior = os.path.join(dir, 'HMI', 'Regresar.png')  # Ruta de la imagen
        self.ima13 = leer_imagen(rutaAnterior, (40, 30))  # Cargar y redimensionar la imagen
        self.boton_regresar = tk.Button(
            self.Area_botonesAuto,
            image=self.ima13,  # Usar la imagen cargada
            text="REGRESAR",  # Texto del botón
            font=("Arial", 10, "bold"),  # Fuente y estilo del texto
            command=abrir_ventana_principal_auto,  # Comando asociado al botón
            compound="left",  # Colocar la imagen a la izquierda del texto
            padx=5,  # Espaciado interno horizontal
            width=110  # Ancho del botón
        )
        self.boton_regresar.pack(side=tk.LEFT, padx=10)  # Empaquetar el botón con separación horizontal

        
        # Modificar el botón "Inicio" para que sea verde y diga "Start"
        self.botonInicioA = tk.Button(self.Area_botonesAuto, text="INICIO", command=self.Validaciones_Inicio, width=15, height=2, bg="green",  fg="white", font=("Arial", 10, "bold")  )
        self.botonInicioA.pack(side=tk.LEFT, padx=10)

        # Botón de emergencia "STOP"
        self.boton_reiniciarA = tk.Button(
            self.Area_botonesAuto,
            text="STOP",  # Texto del botón
            width=15,  # Ancho del botón
            height=2,  # Alto del botón
            bg="red",  # Color de fondo rojo (simula un botón de emergencia)
            fg="white",  # Color del texto blanco
            font=("Arial", 10, "bold")  # Reemplaza con la función correspondiente
        )
        self.boton_reiniciarA.pack(side=tk.LEFT, padx=10)  # Empaquetar el botón con separación horizontal

        # Boton Salir
        self.boton_salirA = tk.Button(self.Area_botonesAuto, text="SALIR", image=logo4, command=VenAuto.quit, 
        width=110,  # Ajustar ancho del botón
        height=30,  # Ajustar alto del botón
        compound="left",  # Colocar la imagen a la izquierda del texto
        font=("Arial", 10, "bold"),  # Fuente del texto
        padx=10  # Separación interna
        )

        # Empaquetar el botón en el frame
        self.boton_salirA.pack(side=tk.LEFT, padx=10)

        # Crear un Label para mostrar el estado
        self.label_estado = tk.Label(master, text="", bd=1, relief=tk.SUNKEN, anchor=tk.W)
        self.label_estado.pack(side=tk.BOTTOM, fill=tk.X)

        # Configurar el hilo para recibir datos del serial
        self.thread = threading.Thread(target=self.leer_datos_serial)
        self.thread.daemon = True  # El hilo se detendrá cuando se cierre la ventana principal
        self.thread.start()

##################################### FUNCIONES MODO AUTOMATICO ###################################################


    def leer_datos_serial(self):
        while True:
            if self.serialArduino.in_waiting > 0:
                mensaje = self.serialArduino.readline().decode().strip()
                print("Mensaje recibido:", mensaje)
                
                # Procesar el mensaje recibido
                self.procesar_mensaje(mensaje)
            else:
                # Puedes ajustar el tiempo de espera o implementar otra lógica según tu necesidad
                pass

    def procesar_mensaje(self, mensaje):
        # Simular el procesamiento del mensaje como lo harías en Arduino
        partes = mensaje.split(",")
        if partes[0] == "150" and len(partes) >= 2:
            if partes[1] == "1":  # Asegurar que partes[1] es una cadena si mensaje es cadena
                self.BANDERA = True
            elif partes[1] == "0":
                self.BANDERA = False
            
        else:
            # Mensaje no reconocido
            print("Mensaje no reconocido AUTO:", mensaje)

    #Funcion al procesar el boton de inicio
    def Validaciones_Inicio(self):
        
        #Cambio de color del primer circulo referencial como led
        self.master.after(0, lambda: self.cambiar_color(0, "red"))
        self.master.after(0, lambda: self.cambiar_color(2, "gray"))
        
        funciones_complementarias = Tesis_Funciones_Version.Funciones_Complementarias(mqtt_server, mqtt_port) #Inicializamos el fichero de tesis funciones version complementarias
        print("Inicia el proceso de validaciones_Inicio")
        
        
        self.Existe_caja = int(funciones_complementarias.Validacion_Caja_Mesa()) #Funcion para la validacion de la caja

        print("Existe Caja?")
        if self.Existe_caja == 0:
            print("No se encontró caja.")
        elif self.Existe_caja == 1:
            print("Sí se encontró caja.")
        else:
            print("Valor desconocido para Existe_caja.")

        # time.sleep(2)
        
        
        if self.Existe_caja == 1:
            # Mostrar el cuadro de advertencia de inicio de movimiento del robot
            message = " " * 10 + "PRECAUCIÓN\n\nROBOT EN MOVIMIENTO"
            messagebox.showwarning("Advertencia", message, icon="warning", parent=self.master)

            self.MoverDis(50,10,self.BANDERA)
            self.GirarGrados(90,10)

            print("Detectamos caja y pasamos a enviar atributos")
            funciones_complementarias.publish_message("robot/estado", "TRUE")
            self.label_estado.config(text="Iniciando proceso, tenga cuidado con el robot de MesaCaja")  # Actualiza el texto del Label
            Objetivo_Aruco = 1 #asignamos el aruco objetivo ya que el aruco ID 1 corresponde al la mesa donde estara la caja
            Verificador_Aruco = False #Inicializamos verificacion aruco en false
            while Verificador_Aruco == False:
                print("Entro en el while para buscar el aruco de MesaCaja")
                Verificador_Aruco , Objetivo_Aruco = self.Aruco(Objetivo_Aruco) #Llamamos a la funcion para la busqueda del aruco
            
            self.Bandera_Fin_PID = Aruco_Medicion_Distancia.Estimation(Objetivo_Aruco, self.serialArduino) #Llamamos a la funcion de estimacion para empezar con la navegacion  
            
            #LECTURA DE CODIGO QR

            print("Esperando detección de código QR...")   
            lector = QRLector()
            #LLama a la Funcion de Detección de QR
            datos_mensaje = lector.procesar_codigo_qr(direccionCamera)
            print(datos_mensaje)
            #VALIDACION DE REPISAS
            # Extraer la celda de los datos
            celda = datos_mensaje.get("Celda")

            # Determinar la columna a partir del código de celda
            columna = determinar_columna(celda)

            # Validar que la columna esté dentro del rango permitido (1, 2 o 3)
            if not isinstance(columna, int) or columna not in [1, 2, 3]:
                print("ERROR: Columna inválida en los datos. Debe ser un entero entre 1 y 3.")
                self.Existe_Casillero = 0  # Indica que no se encontró un casillero válido
            else:
                # Validación de repisas
                self.Existe_Casillero = int(funciones_complementarias.Validar_Casilleros_Disponibles(columna))
            
            print(f"SE VALIDO LA INFORMACION DE LA CAJA, Y HAY DISPONIBILIDAD EN LA COLUMNA:{columna}")

            # MOVIMIENTO DE ACCIONAMIENTO DE CAJA
            self.CogerCarga()
            time.sleep(20)

            #GIRO PROGRAMADO PARA ORIENTACION
            self.MoverDis(-50,5,self.BANDERA)
            self.GirarGrados(180,10)
            self.MoverDis(100,5,self.BANDERA)
            
            Objetivo_Aruco = columna #asignamos el aruco objetivo ya que el aruco ID 0 corresponde al la mesa donde estara la caja
            while Verificador_Aruco == False:
                print("Entro en el while para buscar el aruco de Repisas")
                Verificador_Aruco , Objetivo_Aruco = self.Aruco(Objetivo_Aruco) #Llamamos a la funcion para la busqueda del aruco
            print("Salio del while para buscar el aruco de Repisas")
              
            self.Bandera_Fin_PID = Aruco_Medicion_Distancia.Estimation(Objetivo_Aruco) #Llamamos a la funcion de estimacion para empezar con la navegacion  
            

            # IR A LA REPISA
            if self.Existe_Casillero != 0:
                print(f"Mover hasta casillero {self.Existe_Casillero}")

                # Determinar el piso basado en el número del casillero
                if self.Existe_Casillero in [1, 4, 7]:
                    print("Moviendo al piso A")
                    self.ir_piso(1)  # Piso A
                elif self.Existe_Casillero in [2, 5, 8]:
                    print("Moviendo al piso B")
                    self.ir_piso(2)  # Piso B
                elif self.Existe_Casillero in [3, 6, 9]:
                    print("Moviendo al piso C")
                    self.ir_piso(3)  # Piso C
                else:
                    print("Número de casillero no válido.")
            else:
                print("No hay casillero disponible.")

            print(f"Esperar 20 segundos hasta llegar al Piso")
            time.sleep(20)
            self.DejarCarga()
            print(f"Esperar 30 segundos hasta dejar la carga")
            time.sleep(30)

            self.MoverDis(-50,5,self.BANDERA)
            self.GirarGrados(180,20)

            self.MoverDis(50,5)
            self.GirarGrados(90,10,self.BANDERA)


            #IR AL HOME

            Objetivo_Aruco = 4  #asignamos el aruco objetivo ya que el aruco ID 0 corresponde al la mesa donde estara la caja
            while Verificador_Aruco == False:
                print("Entro en el while para buscar el aruco de Repisas")
                Verificador_Aruco , Objetivo_Aruco = self.Aruco(Objetivo_Aruco) #Llamamos a la funcion para la busqueda del aruco
            print("Salio del while para buscar el aruco de Repisas")
              
            self.Bandera_Fin_PID = Aruco_Medicion_Distancia.Estimation(Objetivo_Aruco) #Llamamos a la funcion de estimacion para empezar con la navegacion  
            
            funciones_complementarias.publish_message("robot/estado", "FALSE") #Al final del proceso enviamos False para el monitoreo y decir que se finalizo el proceso de clasificacion de la caja
        
        elif self.Existe_caja == 0: #Si no existe la caja se ejecuta esta funcion y se muestra un mensaje de alerta
            print("No existe Caja")
            self.label_estado.config(text="NO EXISTE CAJA PARA INICIAR EL PROCESO, PONGA UNA CAJA Y LUEGO PRESIONE INICIO")  # Actualiza el texto del Label
            funciones_complementarias.publish_message("robot/estado", "FALSE")
            messagebox.showwarning("Advertencia", "NO EXISTE CAJA PARA INICIAR EL PROCESO, PONGA UNA CAJA Y LUEGO PRESIONE INICIO", icon="warning", parent=self.master)

#Funcion  Determina la columna correspondiente en función del código de celda.

    def determinar_columna(celda):
        """
        Determina la columna correspondiente en función del código de celda.

        Args:
            celda (str): Código de la celda (ejemplo: 'C01', 'C05', 'C09').

        Returns:
            int: Número de columna (1, 2 o 3). Retorna 0 si el código no es válido.
        """
        try:
            # Validar el formato del código de celda
            if not celda.startswith("C") or len(celda) != 3:
                print("ERROR: Código de celda inválido. Debe comenzar con 'C' y tener 3 caracteres.")
                return 0

            # Extraer el número de la celda
            numero_celda = int(celda[1:])  # Convertir el segundo y tercer carácter a entero

            # Determinar la columna según el número de celda
            if numero_celda in [1, 4, 7]:
                return 1  # Columna 1
            elif numero_celda in [2, 5, 8]:
                return 2  # Columna 2
            elif numero_celda in [3, 6, 9]:
                return 3  # Columna 3
            else:
                print("ERROR: Número de celda fuera de rango. Debe estar entre 1 y 9.")
                return 0
        except ValueError:
            print("ERROR: Formato de celda inválido. Asegúrese de que el código contiene un número válido.")
            return 0

        
#Funcion para identificar los arucos y buscar el aruvo objetivo a ser buscado

    def Aruco(self, Buscar):
        marker_size = 100
        # Obtener el diccionario de marcadores ArUco y los parámetros del detector
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        # cap = cv2.VideoCapture(0) # Iniciar captura desde la cámara (cambiar el número si tienes varias cámaras)
        rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
        cap = cv2.VideoCapture(rtsp_url)
        detected_ids = set()  # Usamos un conjunto para evitar duplicados de IDs detectados
        

        while True:
            ret, frame = cap.read()  # Leer un fotograma de la cámara
            if not ret:
                break
            # Convertir el fotograma a escala de grises
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Detectar los marcadores ArUco en el fotograma
            # marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray_frame, aruco_dict, parameters=aruco_params)

            marker_corners, marker_ids, _ = aruco_detector.detectMarkers(gray_frame)
        
            # Verificar si se detectaron marcadores
            if marker_ids is not None:
                # Añadir los IDs de los marcadores detectados a la lista
                for id in marker_ids.flatten():
                    if id not in detected_ids: #Compara los arucos
                        detected_ids.add(id)
                        if Buscar != id: # si el aruco que se agrega no es el objetivo se ejecuta la funcion Girar la cual mantiene girando al robot hasta que encuentre el aruco objetivo
                            self.Girar() #Funcion de girar el robot
                        if Buscar == id: #Cuando el aruco objetivo es encontrado 

                            Verificador_Aruco = True #Asiganamos el valor de true para decir que si se encontro el aruco objetivo
                            datos = "20,0"
                    
                            mensaje_con_salto = datos + "\n"
                            self.serialArduino.write(mensaje_con_salto.encode())
                            time.sleep(1)
                            # self.serialArduino.close() # Cerramos el envio de datos bluetooth
                            time.sleep(0.5)
                            cap.release() 
                            cv2.destroyAllWindows()
                            return Verificador_Aruco , id 
                # Dibujar los marcadores en el fotograma
                cv2.aruco.drawDetectedMarkers(frame, marker_corners, marker_ids)
            
            # Crear una ventana con el nombre 'DETECCION DE ARUCO'
            cv2.namedWindow('DETECCION DE ARUCO')

            # Especificar las coordenadas (x, y) de la esquina superior izquierda de la ventana
            posicion_x = 20  # Cambia esto según la posición deseada en el eje x
            posicion_y = 200  # Cambia esto según la posición deseada en el eje y

            # Mover la ventana a la posición especificada
            cv2.moveWindow('DETECCION DE ARUCO', posicion_x, posicion_y)

            # Mostrar el fotograma con los marcadores
            cv2.imshow('DETECCION DE ARUCO', frame)

            # Salir del bucle si se presiona la tecla 'q'
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

    

    def GirarGrados(self, grados, tiempo):
        print(f"Girando {grados} grados por {tiempo} segundos...")
        datos = f"19,{grados},0,0"
        try:
    
            mensaje_con_salto = datos + "\n"
            self.serialArduino.write(mensaje_con_salto.encode())
            print("Datos enviados a Arduino:", datos)
        except Exception as e:
            print(f"Error al enviar datos a Arduino: {e}")
        pass
        time.sleep(tiempo)

    #Funcion para cambiar el color de los indicadores
    def cambiar_color(self, indice, color):
        # Cambiar el color de un óvalo específico
        canvas, ovalo = self.ovalo_widgets[indice]
        canvas.itemconfig(ovalo, fill=color)


    # def MoverDis(self, distancia, tiempo):
    #     print(f"Girando a  {distancia} cm por {tiempo} segundos")
    #     datos = f"17,{distancia},0,0,0"
    #     try:
    #         mensaje_con_salto = datos + "\n"
    #         self.serialArduino.write(mensaje_con_salto.encode())
    #         print("Datos enviados a Arduino:", datos)
    #     except Exception as e:
    #         print(f"Error al enviar datos a Arduino: {e}")
    #     pass
    #     time.sleep(tiempo)

    import time

import time

def MoverDis(self, distancia, tiempo, BANDERA):
    print(f"Girando a {distancia} cm por {tiempo} segundos")
    datos = f"17,{distancia},0,0,0"
    
    try:
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        print("Datos enviados a Arduino:", datos)
    except Exception as e:
        print(f"Error al enviar datos a Arduino: {e}")
        return

    # Esperar hasta que BANDERA sea True o el tiempo expire
    start_time = time.time()
    while not BANDERA:
        if time.time() - start_time >= tiempo:
            print("Tiempo expirado, el robot no completó la tarea.")
            break
        time.sleep(0.1)  # Espera breve para evitar bloquear el procesador

    if BANDERA:
        print("El robot completó la tarea exitosamente.")




    def Girar(self):
        print("Girando........")
        datos = "0,0,0,0"

        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def CogerCarga(self):
        datos = "14"
        print(f"Enviando datos a Arduino: {datos}")
        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)
        time.sleep(0.5)
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        time.sleep(0.5)
        pass

    def DejarCarga(self):
        datos = "15"
        print(f"Enviando datos a Arduino: {datos}")
        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)
        time.sleep(0.5)
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        time.sleep(0.5)
        pass

    def ir_piso(self, piso_valor):
        # Configurar los datos a enviar
        datos = "0,0,0,0"
        print(f"Enviando datos a Arduino: {datos}")
        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

########################################################## MODO MANUAL ################################################################################

class VentanaManual:
    def __init__(self, master):
        self.master = master
        #master.overrideredirect(True)
        self.master.geometry("1366x768")
        self.master.resizable(width=False, height=False)

        self.cap = None
        self.accion_en_proceso = False
        self.stream_thread = None
        self.stream_active = threading.Event()

        # Aquí colocas el código para la ventana de modo manual
        # self.serialArduino = serial.Serial("COM7", 115200)  # Reemplaza 'COM1' por el puerto serie correspondiente
        # self.serialArduino = serial.Serial("/dev/cu.HC-05", 115200)  # Reemplaza 'COM1' por el puerto serie correspondiente

        self.serialArduino = serial.Serial(puertoBT, 115200)  # Reemplaza 'COM1' por el puerto serie correspondiente
        

        print("Ventana de Modo Manual Iniciada")
        master.title("Ventana de Modo Manual Iniciada")
        
        # Obtener el ancho y alto de la pantalla
        ancho_pantalla = master.winfo_screenwidth()
        alto_pantalla = master.winfo_screenheight() 

        # Calcular las coordenadas para centrar la ventana
        x = (ancho_pantalla - 1366) // 2  # 900 es el ancho de la ventana
        y = (alto_pantalla - 768) // 2   # 700 es el alto de la ventana

        # Establecer la posición de la ventana
        master.geometry(f"1366x768+{x}+{y}")
        
        self.velocidad = 50

        # Construir las rutas relativas a las imágenes
        path_logo_U = os.path.join(dir, 'HMI', 'Logo_U.png')
        path_logo_C = os.path.join(dir, 'HMI', 'Logo_M.png')

        # Cargar la imagen del logo de la universidad
        imagen_U = leer_imagen(path_logo_U,(120,100))
        # Cargar la imagen del logo de la carrera
        imagen_C = leer_imagen(path_logo_C,(120,100))

        # Crear un frame para contener las imágenes y el texto
        frame_contenedor = tk.Frame(master)
        frame_contenedor.pack(fill='x')

        # Mostrar el logo de la universidad
        label_logo_universidad = tk.Label(frame_contenedor,image=imagen_U)
        label_logo_universidad.image = imagen_U
        label_logo_universidad.pack(side=tk.LEFT, padx=(20, 0), pady=10, anchor='w')

        # Mostrar el logo de la carrera
        label_logo_carrera = tk.Label(frame_contenedor,image=imagen_C)
        label_logo_carrera.image = imagen_C
        label_logo_carrera.pack(side=tk.RIGHT, padx=(0, 20), pady=10, anchor='e')

        # Mostrar el texto
        texto = "UNIVERSIDAD DE LAS FUERZAS ARMADAS ESPE-L \n\nSISTEMA DE NAVEGACION AUTONOMA \n\nCONTROL DE MANDO: MANUAL"
        label_texto = tk.Label(frame_contenedor, text=texto, font=("Arial", 14, "bold"))
        label_texto.pack(side=tk.LEFT, expand=True, fill="both", padx=(10, 0), pady=10, anchor='center')

        ####################################################  FRAME TOTAL  #####################################################
        self.Total = tk.Frame(master)
        self.Total.pack(padx=5) 
        ####################################################  SUBFRAME CENTRAL  #####################################################

        #Crear subframe central izquierdo 
        self.centro_izquierdo = tk.Frame(self.Total)
        self.centro_izquierdo.grid(row=0,column=0,padx=20,sticky='nsew')  # Se adhiere a la izquierda, con espacio exterior
        #Crear el subframe 1
        self.Posicion1 = tk.Frame(self.centro_izquierdo)
        self.Posicion1.grid(row=0,column=0,padx=5,sticky='nsew')
        Label(self.Posicion1, text = "CONTROL DE MOVIMIENTO", font=("Arial", 12, "bold") ).grid(row=0, column=0,columnspan=3,pady=10,sticky='w')
        Valto = 1
        Vancho = 5
        #Crear botones para el control de movimiento manual 
        im2 = os.path.join(dir,'HMI','Diagonal_Superior_Izquierda.png')
        self.ima2 = leer_imagen(im2,(60,60))
        self.boton1 = tk.Button(self.Posicion1,image=self.ima2,width=3)
        self.boton1.grid(row=1,column=0,padx=5,pady=5,sticky='nsew')
        self.boton1.bind("<Button-1>",self.Diagonal_Superior_IZQ)
        self.boton1.bind("<ButtonRelease-1>", self.soltar_boton1)

        im3 = os.path.join(dir,'HMI','Subir.png')
        self.ima3 = leer_imagen(im3,(60,60))
        self.boton2 = tk.Button(self.Posicion1,image=self.ima3,width=3)
        self.boton2.grid(row=1,column=1,padx=5,pady=5,sticky='nsew') 
        self.boton2.bind("<Button-1>", self.Adelante)
        self.boton2.bind("<ButtonRelease-1>", self.soltar_boton1)

        im4 = os.path.join(dir,'HMI','Diagonal_Superior_Derecha.png')
        self.ima4 = leer_imagen(im4,(60,60))
        self.boton3 = tk.Button(self.Posicion1,image=self.ima4,width=3)
        self.boton3.grid(row=1,column=2,padx=5,pady=5,sticky='nsew') 
        self.boton3.bind("<Button-1>", self.Diagonal_Superior_DER)
        self.boton3.bind("<ButtonRelease-1>", self.soltar_boton1)

        im5 = os.path.join(dir,'HMI','Derecha.png')
        self.ima5 = leer_imagen(im5,(60,60))
        self.boton4 = tk.Button(self.Posicion1,image=self.ima5,width=3)
        self.boton4.grid(row=2,column=0,padx=5,pady=5,sticky='nsew')
        self.boton4.bind("<Button-1>", self.Izquierda)
        self.boton4.bind("<ButtonRelease-1>", self.soltar_boton1)

        im6 = os.path.join(dir,'HMI','Inicio.png')
        self.ima6 = leer_imagen(im6,(60,60))
        self.boton5 = tk.Button(self.Posicion1,image=self.ima6,width=3)
        self.boton5.grid(row=2,column=1,padx=5,pady=5,sticky='nsew')

        im7 = os.path.join(dir,'HMI','Izquierda.png')
        self.ima7 = leer_imagen(im7,(60,60))
        self.boton6 = tk.Button(self.Posicion1,image=self.ima7,width=3)
        self.boton6.grid(row=2,column=2,padx=5,pady=5,sticky='nsew')  
        self.boton6.bind("<Button-1>", self.Derecha)
        self.boton6.bind("<ButtonRelease-1>", self.soltar_boton1)

        im8 = os.path.join(dir,'HMI','Diagonal_Inferior_Izquierda.png')
        self.ima8 = leer_imagen(im8,(60,60))
        self.boton7 = tk.Button(self.Posicion1,image=self.ima8,width=3)
        self.boton7.grid(row=3,column=0,padx=5,pady=5,sticky='nsew')
        self.boton7.bind("<Button-1>",self.Diagonal_Inferior_IZQ)
        self.boton7.bind("<ButtonRelease-1>",self.soltar_boton1)

        im9 = os.path.join(dir,'HMI','Bajar.png')
        self.ima9 = leer_imagen(im9,(60,60))
        self.boton8 = tk.Button(self.Posicion1,image=self.ima9,width=3)
        self.boton8.grid(row=3,column=1,padx=5,pady=5,sticky='nsew') 
        self.boton8.bind("<Button-1>", self.Atras)
        self.boton8.bind("<ButtonRelease-1>", self.soltar_boton1)

        im10 = os.path.join(dir,'HMI','Diagonal_Inferior_Derecha.png')
        self.ima10 = leer_imagen(im10,(60,60))
        self.boton9 = tk.Button(self.Posicion1,image=self.ima10,width=3)
        self.boton9.grid(row=3,column=2,padx=5,pady=5,sticky='nsew')  
        self.boton9.bind("<Button-1>",self.Diagonal_Inferior_DER)
        self.boton9.bind("<ButtonRelease-1>",self.soltar_boton1)

        im11 = os.path.join(dir,'HMI','Giro_Antihorario.png')
        self.ima11 = leer_imagen(im11,(60,60))
        self.boton10 = tk.Button(self.Posicion1,image=self.ima11,width=3)
        self.boton10.grid(row=4,column=0,padx=5,pady=5,sticky='nsew')  
        self.boton10.bind("<Button-1>", self.Giro_Antihorario)
        self.boton10.bind("<ButtonRelease-1>", self.soltar_boton1)

        im12 = os.path.join(dir,'HMI','Giro_Horario.png')
        self.ima12 = leer_imagen(im12,(60,60))
        self.boton11 = tk.Button(self.Posicion1,image=self.ima12,width=3)
        self.boton11.grid(row=4,column=2,padx=5,pady=5,sticky='nsew')  
        self.boton11.bind("<Button-1>", self.Giro_Horario)
        self.boton11.bind("<ButtonRelease-1>", self.soltar_boton1)
        
        #Crear el subframe 2
        self.Posicion2 = tk.Frame(self.centro_izquierdo)
        self.Posicion2.grid(row=1,column=0,padx=5,pady=10,sticky='nsew')
        Label(self.Posicion2, text = "LECTURA DE SENSORES", font=("Arial", 12, "bold") ).grid(row=0,column=0,columnspan=3,pady=10,sticky='w')

        #Crear boton para medir la distancia con los sensores ultrasonicos
        self.boton22 = tk.Button(self.Posicion2, text="MEDIR",font=("Arial", 10, "bold"), width=10,height=Valto, command=self.Leer_Datos)
        self.boton22.grid(row=1,column=0,sticky='w')

        Label(self.Posicion2, text = "DISTANCIA 1:", font=("Arial",10, "bold")).grid(row=2,column=0,sticky='w')

        # Crear una caja de texto
        self.caja1 = Text(self.Posicion2, wrap=tk.WORD, width=7, height=1, font=("Arial", 12))
        self.caja1.grid(row=2,column=1,pady=10)

        Label(self.Posicion2,text = "DISTANCIA 2:", font=("Arial", 10, "bold")).grid(row=3,column=0,sticky='w')

        # Crear una caja de texto
        self.caja2 = Text(self.Posicion2, wrap=tk.WORD, width=7, height=1, font=("Arial", 12))
        self.caja2.grid(row=3,column=1,pady=10)

        # Configurar el hilo para recibir datos del serial
        self.thread = threading.Thread(target=self.leer_datos_serial)
        self.thread.daemon = True  # El hilo se detendrá cuando se cierre la ventana principal
        self.thread.start()


        ############################################################################################################################################
        
        #Crear subframe central centro
        self.centro_centro = tk.Frame(self.Total)
        self.centro_centro.grid(row=0,column=1,sticky='nsew') 
        Label(self.centro_centro, text = "VIDEO EN TIEMPO REAL", font=("Arial", 12, "bold") ).grid(row=0, column=0,columnspan=3,padx=5,pady=10,sticky='w')
        
        #Crear un suframe para mostrar el video de la cámara
        self.Posicion3 = tk.Frame(self.centro_centro)
        self.Posicion3.grid(row=1,column=0,columnspan=4,padx=10,sticky='nsew')

        #Crear el label dentro del subframe donde se mostrara la camra
        self.canvas_camara = tk.Canvas(self.Posicion3,bg="white",relief="flat",width=640,height=400)
        self.canvas_camara.pack(fill="both",expand=True)

        #Crear el frame para los botones de abrir y cerrar la cámara
        self.Posicion4 = tk.Frame(self.centro_centro)
        self.Posicion4.grid(row=2,column=0,pady=10)

        #Aqui se crea el label para seleccionar la camara 
        Label(self.Posicion4, text = "CAMARA:",font=("Arial", 10, "bold") ).grid(row=0,column=0,pady=10,sticky='w')

        #Crear el menu desplegable
        self.camaras = ["NINGUNA","SUPERIOR","INFERIOR"]
        self.valor0 = tk.StringVar()
        self.valor0.set(self.camaras[0]) 
        self.drop=tk.OptionMenu(self.Posicion4,self.valor0,*self.camaras)
        self.drop.config(width=10)
        self.drop.grid(row=0,column=1)
        self.drop['font'] = ('Arial', 10, 'bold')

        # Añadir un botón para abrir y cerrar la cámara en el frame de botones
        self.boton14 = tk.Button(self.Posicion4,text="ABRIR", font=("Arial", 10, "bold"),width=10, height=1, command=self.abrir_camara)
        self.boton14.grid(row=0,column=2,padx=10,sticky='w')

        self.boton15 = tk.Button(self.Posicion4,text="CERRAR", font=("Arial", 10, "bold"),width=10, height=1,command=self.cerrar_camara)
        self.boton15.grid(row=1,column=2,padx=10,sticky='w')

        self.cap = None

        #Label para el control del microservo
        Label(self.Posicion4, text = "CONTROL DEL MICROSERVO",font=("Arial",12,"bold")).grid(row=0,column=3,columnspan=3,padx=20,pady=10,sticky='w')

        #Crear un slider para controlar la rotacion de un microservo
        self.slider = Scale(self.Posicion4, from_=90, to=0, orient=HORIZONTAL, length=300, showvalue=True, tickinterval=10)
        self.slider.grid(row=1, column=3, columnspan=3, padx=20)
        
        # Botón para setear el ángulo
        self.set_angle_button =  tk.Button(self.Posicion4, text="SETEAR ANGULO", font=("Arial", 10, "bold"),width=10, height=1, command=self.angle)
        self.set_angle_button.grid(row=2,column=4,padx=20,sticky='w')
        #########################################################################################################################################################

        #Crear subframe central derecho 
        self.centro_derecho = tk.Frame(self.Total)
        self.centro_derecho.grid(row=0,column=2,sticky='nsew') 

        # Crear un frame para el control de la posicion vertical
        self.Posicion6 = tk.Frame(self.centro_derecho)
        self.Posicion6.grid(row=1,column=0,pady=10,sticky='nsew')
        Label(self.Posicion6,text = "CONTROL DE POSICION VERTICAL",font=("Arial", 12, "bold") ).grid(row=0,column=0,columnspan=2,padx=20,sticky='w')

        #Crear un frame para el menu desplegable
        self.Posicion9 = tk.Frame(self.Posicion6)
        self.Posicion9.grid(row=1,column=0,pady=10,sticky='nsew')

        #Crear un label para seleccionar el nivel al que tiene que elevarce 
        label1 = Label(self.Posicion9,text = "SELECCIONE EL NIVEL", font=("Arial",10,"bold"))
        label1.grid(row=0,column=0,columnspan=2,padx=20,sticky='w')
        self.niveles = ["PISO 0","PISO A1","PISO A2","PISO A3"]
        self.valor = tk.StringVar()
        self.valor.set(self.niveles[0]) 
        self.drop=tk.OptionMenu(self.Posicion9,self.valor,*self.niveles)
        self.drop.config(width=10)
        self.drop.grid(row=1,column=0,columnspan=2)
        self.drop['font'] = ('Arial', 10, 'bold')


        self.btn_ir_piso = tk.Button(self.Posicion9, text="Ir al Piso", command=self.ir_piso)
        self.btn_ir_piso.grid(row=2, column=0, columnspan=2, pady=10)

        self.Posicion10 = tk.Frame(self.Posicion6)
        self.Posicion10.grid(row=1,column=1,pady=10,sticky='nsew')

        

        #Crear boton para subir las tijeretas
        self.boton16 = tk.Button(self.Posicion10, text="SUBIR", font=("Arial", 10, "bold"), width=10,height=Valto, command=self.Subir_Tijereta)
        self.boton16.grid(row=0,column=0,padx=10,pady=10)
        self.canvas1= tk.Canvas(self.Posicion10,width=50,height=50) #Crear canvas para el LED 
        self.canvas1.grid(row=0,column=1)
        self.led_subir = self.canvas1.create_oval(10,10,40,40,fill="gray")

        #Crear boton para bajar las tijeretas
        self.boton17 = tk.Button(self.Posicion10, text="BAJAR", font=("Arial", 10, "bold"), width=10, height=Valto, command=self.Bajar_Tijereta)
        self.boton17.grid(row=1,column=0,padx=10,pady=10)
        self.canvas2= tk.Canvas(self.Posicion10,width=50,height=50)
        self.canvas2.grid(row=1,column=1)
        self.led_bajar = self.canvas2.create_oval(10,10,40,40,fill="gray")

        #Crear boton para detener las tijeretas
        self.boton18 = tk.Button(self.Posicion10,text="DETENER", font=("Arial", 10, "bold"), width=10, height=Valto, command=self.Detener_Tijereta)
        self.boton18.grid(row=2,column=0,padx=10,pady=10)
        self.canvas3= tk.Canvas(self.Posicion10, width=50,height=50)
        self.canvas3.grid(row=2,column=1)
        self.led_detener = self.canvas3.create_oval(10,10,40,40,fill="gray")

        #Crear frame para el movimiento lineal de la corredera
        self.Posicion7 = tk.Frame(self.centro_derecho)
        self.Posicion7.grid(row=2,column=0,pady=0,sticky='nsew')
        Label(self.Posicion7,text = "CONTROL DEL SISTEMA LINEAL", font=("Arial", 12, "bold") ).grid(row=0,column=0,columnspan=2,padx=20,sticky='w')

        #Crear boton para hacer avanzar el sistema lineal  
        self.boton19 = tk.Button(self.Posicion7, text="INICIO", font=("Arial", 10, "bold"), width=10, height=1, command=self.Inicio)
        self.boton19.grid(row=1,column=0,padx=(0,10),pady=10,sticky='e')
        self.canvas4 = tk.Canvas(self.Posicion7,width=50,height=50)
        self.canvas4.grid(row=1,column=1,sticky='w')
        self.led_1 = self.canvas4.create_oval(10,10,40,40,fill="gray")
    
        #Crear boton para hacer detener el sistema lineal  
        self.boton20 = tk.Button(self.Posicion7, text="HOME", font=("Arial", 10, "bold"), width=10,height=1,command=self.Home)
        self.boton20.grid(row=3,column=0,padx=(0,10),pady=10,sticky='e')
        self.canvas5= tk.Canvas(self.Posicion7,width=50, height=50)
        self.canvas5.grid(row=3,column=1,sticky='w')
        self.led_2 = self.canvas5.create_oval(10,10,40,40,fill="gray")

        #Crear boton para hacer regresar el sistema de movimiento
        self.boton21 = tk.Button(self.Posicion7,text="FINAL", font=("Arial", 10, "bold"), width=10,height=1,command=self.Final)
        self.boton21.grid(row=2,column=0,padx=(0,10),pady=10,sticky='e')
        self.canvas6= tk.Canvas(self.Posicion7,width=50,height=50)
        self.canvas6.grid(row=2,column=1,sticky='w')
        self.led_3 = self.canvas6.create_oval(10,10,40,40,fill="gray")

        #Crear el Frame para el control de el electroiman 
        self.Posicion8 = tk.Frame(self.centro_derecho)
        self.Posicion8.grid(row=3,column=0,pady=10,sticky='nsew')
        Label(self.Posicion8,text = "CONTROL DEL ELECTROIMAN", font=("Arial", 12,"bold") ).grid(row=0,column=0,columnspan=2,padx=20,sticky='w')
        
        #Crear boton para encender el electroiman 
        self.boton22 = tk.Button(self.Posicion8, text="ENCENDER", font=("Arial", 10, "bold"), width=10, height=1, command=self.Encender)
        self.boton22.grid(row=1,column=0,padx=(10,10),pady=10,sticky='e')

        #Crear boton para apagar el electroiman 
        self.boton23 = tk.Button(self.Posicion8, text="APAGAR", font=("Arial", 10, "bold"), width=10, height=1, command=self.Apagar)
        self.boton23.grid(row=2,column=0,padx=(10,10),pady=10,sticky='e')

        #Crear Canvas para el LED para encender
        self.canvas7= tk.Canvas(self.Posicion8, width=50, height=50)
        self.canvas7.grid(row=1,column=1,sticky='w')
        self.led_encender = self.canvas7.create_oval(10,10,40,40, fill="gray")

        #Crear Canvas para el LED para apagar
        self.canvas8= tk.Canvas(self.Posicion8, width=50, height=50)
        self.canvas8.grid(row=2,column=1,sticky='w')
        self.led_apagar = self.canvas8.create_oval(10,10,40,40, fill="gray")
        ######################################################################################################

        #Crear un frame para regresar y salir del modo de manod manual 
        self.Menu_Inferior  = tk.Frame(self.Total)
        self.Menu_Inferior.grid(row=1,column=0,columnspan=3,sticky='ew')

        # Configurar las columnas del Frame para que se expandan
        self.Menu_Inferior.columnconfigure(0, weight=1)
        self.Menu_Inferior.columnconfigure(1, weight=1)
        self.Menu_Inferior.columnconfigure(2, weight=1)

        im13 = os.path.join(dir,'HMI','Regresar.png')
        self.ima13 = leer_imagen(im13,(40,30))
        self.boton12 = tk.Button(self.Menu_Inferior,image=self.ima13,text="REGRESAR",font=("Arial", 10, "bold"),command=abrir_ventana_principal_manual,compound="left",padx=5,width=110)
        self.boton12.grid(row=0,column=0,padx=20,sticky='nw')  # Colocar en la fila 4, columna 0, con espacio exterior
        self.boton12.bind("<ButtonRelease-1>",self.soltar_boton1)

        im14 = os.path.join(dir,'HMI','Salir.png')
        self.ima14 = leer_imagen(im14,(40,30))
        self.boton13 = tk.Button(self.Menu_Inferior,image=self.ima14,text="SALIR",font=("Arial", 10, "bold"),command=VenManual.quit,compound="left",padx=5,width=110)
        self.boton13.grid(row=0,column=2,padx=20,sticky='ne')  # Colocar en la fila 4, columna 1, con espacio exterior
        self.boton13.bind("<ButtonRelease-1>",self.soltar_boton1)


############################################ FUNCIONES DE MODO MANUAL ##################################################
    def leer_datos_serial(self):
        while True:
            if self.serialArduino.in_waiting > 0:
                mensaje = self.serialArduino.readline().decode().strip()
                print("Mensaje recibido:", mensaje)
                
                # Procesar el mensaje recibido
                self.procesar_mensaje(mensaje)
            else:
                # Puedes ajustar el tiempo de espera o implementar otra lógica según tu necesidad
                pass

    def procesar_mensaje(self, mensaje):
        # Simular el procesamiento del mensaje como lo harías en Arduino
        partes = mensaje.split(",")
        if partes[0] == "133" and len(partes) >= 3:
            # Mensaje de alturas
            distance1 = partes[1]
            distance2 = partes[2]
            
            # Actualizar las cajas de texto con los valores de Distance1 y Distance2
            self.caja1.delete("1.0", tk.END)
            self.caja1.insert(tk.END, distance1)
            
            self.caja2.delete("1.0", tk.END)
            self.caja2.insert(tk.END, distance2)
        
        elif partes[0] == "102" and len(partes) >= 2:
            # Otro tipo de mensaje (ejemplo: mensaje que comienza con "102")
            valor1 = partes[1]
            
            # Aquí podrías manejar el mensaje de tipo "102"
            print(f"Mensaje tipo 102 recibido, valor1: {valor1}")
            
        else:
            # Mensaje no reconocido
            print("Mensaje no reconocido:", mensaje)

    def ir_piso(self):
        piso_valor = self.niveles.index(self.valor.get()) # Obtener el índice numérico de la selección
        # Configurar los datos a enviar
        datos = "4," + str(piso_valor) 
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Giro_Horario(self, event):
        datos = "20,100,-100,-100,100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Giro_Antihorario(self, event):
        datos = "20,-100,100,100,-100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Derecha(self, event):
        datos = "20,100,-100,100,-100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Izquierda(self, event):
        datos = "20,-100,100,-100,100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Adelante(self, event):
        datos = "20,100,100,100,100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Atras(self, event):
        datos = "20,-100,-100,-100,-100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Diagonal_Superior_IZQ(self, event):
        datos = "20,0,100,0,100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Diagonal_Superior_DER(self, event):
        datos = "20,100,0,100,0"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Diagonal_Inferior_IZQ(self, event):
        datos = "20,-100,0,-100,0"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def Diagonal_Inferior_DER(self, event):
        datos = "20,0,-100,0,-100"
        print(f"Enviando datos a Arduino: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

    def soltar_boton1(self, event):
        datos = "20,0,0,0,0"
        
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass
    ############################################################################################################
    
    # def abrir_camara(self):
    #     # Inicialización de la bandera para controlar si una acción está en proceso
    #     self.accion_en_proceso = getattr(self, 'accion_en_proceso', False)
        
    #     if self.accion_en_proceso:
    #         return  # Si hay una acción en proceso, salir sin hacer nada
        
    #     if self.valor0.get() == self.camaras[1]:
    #         self.cap = cv2.VideoCapture(0)  # definir el puerto de apertura de la camara superior
    #     elif self.valor0.get() == self.camaras[2]:
    #         self.cap = cv2.VideoCapture(1)  # definir el puerto de apertura de la camara inferior

    #     if self.cap is not None and self.cap.isOpened():
    #         self.accion_en_proceso = True
    #         self.update_frame()

    def abrir_camara(self):
        if self.accion_en_proceso:
            return
        self.accion_en_proceso = True

        try:
            if self.valor0.get() == self.camaras[1]:
                ip_cam_url = direccionCamera
                self.stream_active.set()  # Marca el stream como activo
                self.stream_thread = threading.Thread(target=self.stream_video, args=(ip_cam_url,))
                self.stream_thread.start()

            elif self.valor0.get() == self.camaras[2]:
                rtsp_url = "rtsp://admin:L28E4E11@192.168.1.6:554/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif"
                self.cap = cv2.VideoCapture(rtsp_url)

                if self.cap.isOpened():
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
                    self.update_frame()
                else:
                    print("No se pudo abrir la cámara")
                    self.accion_en_proceso = False

            else:
                print("Valor de cámara no válido")
                self.accion_en_proceso = False

        except Exception as e:
            print(f"Ocurrió un error al intentar abrir la cámara: {e}")
            self.accion_en_proceso = False

    def cerrar_camara(self):
        self.accion_en_proceso = False

        if self.cap:
            self.cap.release()
            self.cap = None

        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_active.clear()  # Marca el stream como inactivo
            self.stream_thread.join()  # Espera a que el hilo termine
            self.stream_thread = None

        self.canvas_camara.delete('all')

    def update_frame(self):
        if not self.accion_en_proceso:
            return

        if hasattr(self, 'cap') and self.cap is not None:
            ret, frame = self.cap.read()
            if ret:
                # Redimensionar la imagen al tamaño del Canvas
                frame = self.resize_frame(frame)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = Image.fromarray(frame)
                imgtk = ImageTk.PhotoImage(image=img)
                self.canvas_camara.create_image(0, 0, anchor=tk.NW, image=imgtk)
                self.canvas_camara.imgtk = imgtk

        self.centro_centro.after(10, self.update_frame)

    def resize_frame(self, frame):

        canvas_width = self.canvas_camara.winfo_reqwidth()
        canvas_height = self.canvas_camara.winfo_reqheight()

        canvas_ratio = canvas_width / canvas_height
        frame_ratio = frame.shape[1] / frame.shape[0]

        if frame_ratio > canvas_ratio:
            new_width = canvas_width
            new_height = int(canvas_width / frame_ratio)
        else:
            new_height = canvas_height
            new_width = int(canvas_height * frame_ratio)

        resized_frame = cv2.resize(frame, (new_width, new_height))
        return resized_frame
    
    def stream_video(self, url):
        try:
            stream = requests.get(url, stream=True)
            if stream.status_code == 200:
                bytes_data = b''
                while self.stream_active.is_set():
                    chunk = stream.raw.read(1024)
                    bytes_data += chunk
                    a = bytes_data.find(b'\xff\xd8')
                    b = bytes_data.find(b'\xff\xd9')
                    if a != -1 and b != -1:
                        jpg = bytes_data[a:b + 2]
                        bytes_data = bytes_data[b + 2:]
                        img = Image.open(BytesIO(jpg))
                        frame = np.array(img)
                        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                        img = Image.fromarray(frame)
                        imgtk = ImageTk.PhotoImage(image=img)
                        self.canvas_camara.create_image(0, 0, anchor=tk.NW, image=imgtk)
                        self.canvas_camara.imgtk = imgtk
                        self.centro_centro.update_idletasks()
                        self.centro_centro.update()

                        if not self.centro_centro.winfo_exists():
                            break

                    time.sleep(0.01)  # Añade una pausa para evitar el consumo alto de CPU

        except requests.RequestException as e:
            print(f"Error al conectar con la cámara: {e}")
        except Exception as e:
            print(f"Error al procesar el video: {e}")
        finally:
            self.accion_en_proceso = False
            if stream:
                stream.close()

    def close(self):
        self.cerrar_camara()
        self.centro_centro.destroy
    ###################################################################################################################
    
    def Estado(self, comando, led_a_encender):
        if not hasattr(self, 'ultimo_comando'):
            self.ultimo_comando = None
        
        if not hasattr(self, 'accion'):
            self.accion = False
        
        if comando == self.ultimo_comando:
            print(f"Ignorando acción de {comando.lower()} porque ya está en proceso.")
            return  # Ignorar la acción si es la misma que la anterior
        
        if hasattr(self, 'accion_en_proceso') and self.accion and (
            (comando == "SUBIR" and self.ultimo_comando == "BAJAR") or 
            (comando == "BAJAR" and self.ultimo_comando == "SUBIR")
        ):
            print(f"Ignorando acción de {comando.lower()} porque se está en proceso de {self.ultimo_comando.lower()}.")
            return  # Ignorar la acción si está en proceso de una acción opuesta
        
        self.ultimo_comando = comando
        self.accion = comando != "DETENER"
        piso = self.valor.get()  # Asumiendo que tienes self.valor definido correctamente
        if piso != "NINGUNA":
            # Asumiendo que tienes definidos correctamente canvas1, canvas2, canvas3, led_subir, led_bajar, led_detener
            self.canvas1.itemconfig(self.led_subir, fill="#00FF00" if led_a_encender == "subir" else "gray")
            self.canvas2.itemconfig(self.led_bajar, fill="#00FF00" if led_a_encender == "bajar" else "gray")
            self.canvas3.itemconfig(self.led_detener, fill="#FFFF00" if led_a_encender == "detener" else "gray")
            self.enviar_datos(comando)
            self.master.after(5000, self.resetear_leds)
    
    def Subir_Tijereta(self):
        self.Estado("SUBIR", "subir")
        datos = "1"
        print(f"Enviando datos: {datos}")
        # Asumiendo que tienes self.serialArduino definido correctamente
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    
    def Detener_Tijereta(self):
        self.Estado("DETENER", "detener")
        datos = "3"
        print(f"Enviando datos: {datos}")
        # Asumiendo que tienes self.serialArduino definido correctamente
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    
    def Bajar_Tijereta(self):
        self.Estado("BAJAR", "bajar")
        datos = "2"
        print(f"Enviando datos: {datos}")
        # Asumiendo que tienes self.serialArduino definido correctamente
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    
    def enviar_datos(self, movimiento):
        piso = self.valor.get()  # Asumiendo que tienes self.valor definido correctamente
        if piso in self.niveles:  # Asumiendo que tienes self.niveles definido correctamente
            datos = "0,0,0,0"
            print(f"Enviando datos: {datos}")
            # Asumiendo que tienes self.serialArduino definido correctamente
            mensaje_con_salto = datos + "\n"
            self.serialArduino.write(mensaje_con_salto.encode())
    
    def resetear_leds(self):
        # Asumiendo que tienes definidos correctamente canvas1, canvas2, canvas3, led_subir, led_bajar, led_detener
        self.canvas1.itemconfig(self.led_subir, fill="gray")
        self.canvas2.itemconfig(self.led_bajar, fill="gray")
        self.canvas3.itemconfig(self.led_detener, fill="gray")
        self.accion_en_proceso = False
        self.ultimo_comando = None 
    #########################################################################################
    
    def control_led(self, comando, led_canvas, led_index):
        leds = [self.led_1, self.led_2, self.led_3]
        for i, led in enumerate(leds):
            canvas = getattr(self, f'canvas{i + 4}')
            color = "#00FF00" if i == led_index else "gray"
            canvas.itemconfig(led, fill=color)
        
        if not hasattr(self, 'ultimo_control'):  # Verificar si es la primera vez que se llama a control_led
            self.ultimo_control = None
        
        if comando != self.ultimo_control:  # Verificar si es un comando diferente al último enviado
            # self.enviar_datos2(comando)
            self.ultimo_control = comando  # Actualizar el último comando enviado

    def Inicio(self):
        self.control_led("INICIO", self.canvas4, 0)
        datos = "14"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())

    def Home(self):
        self.control_led("HOME", self.canvas5, 1)
        datos = "5"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())

    def Final(self):
        self.control_led("FIN", self.canvas6, 2)
        datos = "15"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())

    def enviar_datos2(self, control):
        datos = "0,0,0,0"
        print(f"Enviando datos: {datos}")  # Mensaje de depuración
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    ####################################################################################################

    def control_led1(self,comando1,led_canvas, led_index1):
        leds1 = [self.led_encender, self.led_apagar]
        for i, led1 in enumerate(leds1):
            canvas = getattr(self, f'canvas{i + 7}')
            color = "#00FF00" if i == led_index1 else "gray"
            canvas.itemconfig(led1, fill=color)
        
        if not hasattr(self, 'ultimo_control'):  # Verificar si es la primera vez que se llama a control_led
            self.ultimo_control = None
        
        if comando1 != self.ultimo_control:  # Verificar si es un comando diferente al último enviado
            # self.enviar_datos3(comando1)
            self.ultimo_control = comando1  # Actualizar el último comando enviado

    def Encender(self):
        self.control_led1("ENCENDER_IMAN", self.canvas7, 0)
        datos = "8"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    
    def Apagar(self):
        self.control_led1("APAGAR_IMAN", self.canvas8, 1)
        datos = "9"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
    
    def enviar_datos3(self,estado):
        datos = "0,0,0,0"
        print(f"Enviando datos: {datos}")
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass
    ####################################################################################################
    
    def angle(self):
        # Obtener el valor actual del slider y limpiar cualquier carácter de nueva línea
        angleData = str(self.slider.get()).strip()
        datos = "12,"+angleData+",0,0,0"
        print(f"Enviando datos: {datos}")
        # Suponiendo que `self.serialArduino` está inicializado correctamente
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())

       

    def Leer_Datos(self):
        datos = "13,0,0,0"
        mensaje_con_salto = datos + "\n"
        self.serialArduino.write(mensaje_con_salto.encode())
        pass

########################################################## Ventana Principal ################################################################################

def abrir_ventana_principal_auto():
    VenAuto.destroy()
    ventana.deiconify()  # Mostrar la ventana principal

def abrir_ventana_principal_manual():

    VenManual.destroy()
    ventana.deiconify()  # Mostrar la ventana principal

def ventana_automatico():
    global VenAuto
    ventana.withdraw()
    VenAuto = tk.Toplevel()
    app = VentanaAutomatico(VenAuto)

def ventana_manual():
    global VenManual
    ventana.withdraw()
    VenManual = tk.Toplevel()
    app = VentanaManual(VenManual)

def leer_imagen(path,size):
    return ImageTk.PhotoImage(Image.open(path).resize(size, Image.LANCZOS))

def centrar_ventana(ventana,ancho,largo):
    pantalla_ancho = ventana.winfo_screenwidth()
    pantalla_largo = ventana.winfo_screenheight()
    x = int((pantalla_ancho/2)-(ancho/2))
    y = int((pantalla_largo/2)-(largo/2))
    return ventana.geometry(f"{ancho}x{largo}+{x}+{y}")
# Crear la ventana principal
ventana = tk.Tk()
#ventana.overrideredirect(True)
ventana.title("Ventana de Inicio")
# Cambiar el color de fondo de la ventana
w,h = ventana.winfo_screenwidth(), ventana.winfo_screenheight()
ventana.geometry("%dx%d+0+0"%(w,h))
ventana.resizable(width=0,height=0)
centrar_ventana(ventana,700,500)
# Configurar la cuadrícula de la ventana principal
ventana.rowconfigure(0, weight=8)  # Fila superior
ventana.rowconfigure(1, weight=0)  # Fila inferior
ventana.columnconfigure(0, weight=1)  # Única columna

# Crear Frame para la fila superior
frame_upper = tk.Frame(ventana)
frame_upper.grid(row=0, column=0, sticky="nsew")  # Ocupa toda la fila superior

# Crear Frame para la fila inferior
frame_lower = tk.Frame(ventana)
frame_lower.grid(row=1, column=0, sticky="nsew")  # Ocupa toda la fila inferior
#Obtener la ruta del directorio actual
dir = os.path.dirname(__file__)
######################################## FRAME SUPERIOR ##############################################
ruta_logo = os.path.join(dir,'HMI','Presentacion.png')
logo1 = leer_imagen(ruta_logo,(700,400))
# Crear un widget Label y asignarle la imagen
label_left = tk.Label(frame_upper,image=logo1)
label_left.place(x=0,y=0,relwidth=1,relheight=1)
####################################### FRAME INFERIOR ##################################################
# Colocar la imagen de fondo en el frame inferior
ruta_fondo = os.path.join(dir, 'HMI', 'Presentacion 2.png')  # Ruta de la imagen de fondo para la fila inferior
fondo_imagen = leer_imagen(ruta_fondo, (700, 150))  # Ajusta el tamaño de la imagen de fondo
fondo = tk.Label(frame_lower, image=fondo_imagen)
fondo.place(x=0, y=0, relwidth=1, relheight=1)  # Colocar la imagen de fondo y ajustarla al tamaño del frame

# Configurar el layout del frame inferior para centrar los botones
frame_lower.columnconfigure(0, weight=1)  # Columna vacía a la izquierda
frame_lower.columnconfigure(1, weight=1)  # Columna para el botón AUTOMÁTICO
frame_lower.columnconfigure(2, weight=1)  # Columna para el botón MANUAL
frame_lower.columnconfigure(3, weight=1)  # Columna para el botón SALIR
frame_lower.columnconfigure(4, weight=1)  # Columna vacía a la derecha

# Crear un frame con borde superior e inferior
frame_border = tk.Frame(frame_lower, bd=0)
frame_border.grid(row=0, column=0, columnspan=5, sticky="nw", padx=20)

# Etiqueta de título con el texto
title4 = tk.Label(frame_border, text='CONTROL DE MANDO', font=("Arial", 14, "bold"))
title4.pack()  # Agregar margen superior e inferior al texto

# Crear el borde superior e inferior
border_top = tk.Frame(frame_border, height=1, bg="black")
border_top.pack(fill="x", side="top", expand=True)  # Expande a lo largo de toda la fila

border_bottom = tk.Frame(frame_border, height=1, bg="black")
border_bottom.pack(fill="x", side="bottom", expand=True) 
# Botón "Modo Automático"
ruta1 = os.path.join(dir, 'HMI', 'Automatico.png')
logo2 = leer_imagen(ruta1, (40, 30))
boton_auto = tk.Button(frame_lower, image=logo2, text="AUTOMÁTICO", font=("Arial", 10, "bold"), command=ventana_automatico, compound="left", padx=10, width=130)
boton_auto.grid(row=1, column=2, padx=20, pady=20)

# Botón "Modo Manual"
ruta2 = os.path.join(dir, 'HMI', 'Manual.png')
logo3 = leer_imagen(ruta2, (40, 30))
boton_manual = tk.Button(frame_lower, image=logo3, text="MANUAL", font=("Arial", 10, "bold"), command=ventana_manual, compound="left", padx=10, width=130)
boton_manual.grid(row=1, column=1, padx=20, pady=20)

# Botón "Salir"
ruta3 = os.path.join(dir, 'HMI', 'Salir.png')
logo4 = leer_imagen(ruta3, (40, 30))
boton_salir = tk.Button(frame_lower, image=logo4, text="SALIR", font=("Arial", 10, "bold"), command=ventana.quit, compound="left", padx=10, width=110)
boton_salir.grid(row=1, column=3, padx=20, pady=20)


ventana.mainloop()





