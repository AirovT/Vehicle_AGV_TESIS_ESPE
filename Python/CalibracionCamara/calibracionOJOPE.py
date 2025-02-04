import pickle

# Especifica la dirección completa del archivo .pkl con el nombre correcto
file_path = r'C:\Users\jairo\Documents\U_ESPE\Tesis\Nueva tesis ESPE\GitHub Tesis\Vehicle_AGV_TESIS_ESPE\Python\Calibracion\calibration.pkl'

# Abre el archivo en modo de lectura binaria
with open(file_path, 'rb') as file:
    data = pickle.load(file)

# Muestra los contenidos de 'dist' y 'CameraMatrix'
print(data['dist'])  # Si 'dist' está en el archivo
print(data['cameraMatrix'])  # Si 'CameraMatrix' está en el archivo
