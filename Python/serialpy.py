import serial
import time

# Configurar el puerto y la velocidad de baudios
port = 'COM3'
baud_rate = 115200

# Inicializar la conexión serial
try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"Conectado al puerto {port} a {baud_rate} baudios")
except Exception as e:
    print(f"Error al abrir el puerto: {e}")
    exit()

# Mensaje a enviar
message = "1"

try:
    while True:
        try:
            # Enviar el mensaje           
            ser.write(message.encode('utf-8'))
            print(f"Mensaje enviado: {message}")

            # Esperar 5 segundos antes de enviar el próximo mensaje
            time.sleep(5)

        except Exception as e:
            print(f"Error al enviar el mensaje: {e}")
            break

except KeyboardInterrupt:
    print("Interrupción del usuario, cerrando el programa.")

finally:
    # Cerrar la conexión serial
    ser.close()
    print(f"Puerto {port} cerrado")
