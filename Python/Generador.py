import qrcode
from pyzbar.pyzbar import decode
from PIL import Image

# Diccionarios para los datos
diccionarioProductos = {
    "MT01": "Martillo Metalico", 
    "MT02": "Martillo Goma", 
    "ST01": "Estilete Plastico", 
    "DX01": "Destornillador Estrella", 
    "DP01": "Destornillador Plano", 
    "DP02": "Dest-Plano Madera"
}

# Crear el código QR siguiendo la estructura esperada
ID = "ID3"         # 3 caracteres
Separador1 = "AA"  # 2 caracteres
Producto = "DX01"  # 4 caracteres
Separador2 = "BB"  # 2 caracteres
Celda = "C05"      # 3 caracteres

# Generar la cadena de texto siguiendo el formato esperado
data = f"{ID}{Separador1}{Producto}{Separador2}{Celda}"
print("Datos codificados en el QR:", data)

# Crear el código QR
qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_L, box_size=10, border=4)
qr.add_data(data)
qr.make(fit=True)
img = qr.make_image(fill_color="black", back_color="white")

# Guardar y mostrar el código QR
direccion_archivo = "codigo_qr2.png"
img.save(direccion_archivo)
print(f"El código QR se ha guardado como {direccion_archivo}.")
img.show()

# Verificación: Leer el PNG creado y convertirlo en cadena de texto
qr_imagen = Image.open(direccion_archivo)
decoded_objects = decode(qr_imagen)
if decoded_objects:
    for obj in decoded_objects:
        qr_data = obj.data.decode('utf-8')
        print("Datos leídos del QR:", qr_data)
else:
    print("No se pudo leer el código QR del archivo.")
