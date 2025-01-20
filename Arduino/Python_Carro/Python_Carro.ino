#include <ArduinoJson.h>
#include <Servo.h>
#include "AccelStepper.h"

Servo myServo;  // Crea un objeto Servo

StaticJsonDocument<200> doc;

///////// SERIAL /////////////

String strT = "";
const char separatorT = ',';
const int dataLengthT = 4;
int datoT[dataLengthT];


//////////////////////////////

// ========= MOTORES MOVIMIENTO =============//
#define M1_Dir 2
#define M1_Vel 3

#define M2_Dir 4
#define M2_Vel 5

#define M3_Dir 6
#define M3_Vel 7

#define M4_Dir 8
#define M4_Vel 9

String mensaje, m1_vel, m2_vel, m3_vel, m4_vel;
String Dato_movimiento, Dato_velocidad;


// ========= ASCENSOR =============//
#define ENA 10
#define L_PWM 11
#define R_PWM 12
// #define A_ANAG_PIN A0
int potPin = A0; // Pin del potenciómetro
int Distance;

unsigned long previousMillis = 0; // Almacena el último tiempo de actualización
const long interval = 750; // Intervalo de 0.5 segundos (500 milisegundos)

int Pisos[] = {
  70, // Piso 1, altura aproximada 100
  250, // Piso 1, altura aproximada 100
  550, // Piso 2, altura aproximada 200
  1020, // Piso 3, altura aproximada 300
  // Agrega más pisos según sea necesario
};

// ========= STEP =============//

#define DIR 24
#define PUL 26
#define motorInterfaceType 1

#define IMAN 38

#define SERVO_PIN 13
#define ENDSTOP 52

// Define height and maximum height
int Position = 0;
int PositionMax = 17000;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, PUL, DIR);

int ServoDegree;
bool Iman;
// bool EndStop;

//========= ULTRASONIC ========//
#define TRIG1 30
#define ECHO1 32

#define TRIG2 44
#define ECHO2 46

int Distance1;
int Distance2;

//// ============= OTROS ===========///

#define IND 20

//====================== SETUP ======================//

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(115200);

  pinMode(M1_Dir, OUTPUT);
  pinMode(M1_Vel, OUTPUT);

  pinMode(M2_Dir, OUTPUT);
  pinMode(M2_Vel, OUTPUT);

  pinMode(M3_Dir, OUTPUT);
  pinMode(M3_Vel, OUTPUT);

  pinMode(M4_Dir, OUTPUT);
  pinMode(M4_Vel, OUTPUT);

  //====================

   // Configurar el pin del endstop como entrada
  pinMode(ENDSTOP, INPUT_PULLUP); // Utiliza un pull-up interno para evitar ruido

    myServo.attach(SERVO_PIN);  // Conecta el servo al pin 9
    setServoAngle(90);  // Establece el ángulo del servo a 0 grados

 

   pinMode(ENA, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);

    // Activa la salida
    digitalWrite(ENA, HIGH);


  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);

  pinMode(IMAN, OUTPUT);  // Configura el pin del relé como salida
  digitalWrite(IMAN, LOW);  // Asegúrate de que el relé esté inicialmente apagado

  pinMode(IND, OUTPUT);  // Configura el pin del relé como salida
  digitalWrite(IND, LOW);  // Asegúrate de que el relé esté inicialmente apagado




    pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);
  
  // Inicializar pines del segundo sensor ultrasónico
  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);


} 

//====================== BUCLE PRINCIPAL ======================//

void loop() {

  //////////////////////////// SERIAL USB //////////////////////////////////////
      
  strT = "";
    if (Serial.available())
    {
      strT = Serial.readStringUntil('\n');
      Serial.println(strT);
      for (int i = 0; i < dataLengthT; i++)
      {
        int index = strT.indexOf(separatorT);
        datoT[i] = strT.substring(0, index).toInt();
        strT = strT.substring(index + 1);
      }
      for (int i = 0; i < dataLengthT; i++)
      {
        Serial.print("Dato ");
        Serial.print(i);
        Serial.print("  =  ");
        Serial.print(datoT[i]);
        Serial.print("  -  ");
      }
      Serial.println(" ");
    }

    int velocidad = 150;

    switch ((int)datoT[0])
    {
      case 0:
      // printAltura();
      datoT[0] = 0;
      break;

      case 1:
      Subir(velocidad);
      datoT[0] = 0;
      break;

      case 2:
      Bajar(velocidad);
      datoT[0] = 0;
      break;

      case 3:
      Detener();
      datoT[0] = 0;
      break;

      case 4:
      // IrPiso(numeroPisoDeseado);
      IrPiso(datoT[1]);
      datoT[0] = 0;
      break;

      case 5:
      delay(10);
      Serial.println("Home Init");
      delay(10);
      home();
      datoT[0] = 0;
      break;

      case 6:

      moveToPosition(datoT[1]);
      datoT[0] = 0;
      break;

      case 7:
      printCurrentPosition();
      datoT[0] = 0;
      break;
          
      case 8:
      delay(10);
      ImanStatus(true);
      ImanStatus(true);
      datoT[0] = 0;
      break;

      case 9:
      delay(10);
      ImanStatus(false);
      ImanStatus(false);
      datoT[0] = 0;
      break;
      
      case 10:
      delay(10);
      LedStatus(true);
      LedStatus(true);
      datoT[0] = 0;
      break;
      
      case 11:
      delay(10);
      LedStatus(false);
      LedStatus(false);
      datoT[0] = 0;
      break;

      case 12:
      setServoAngle(datoT[1]);
      datoT[0] = 0;
      break;

      default:
      // Handle unexpected cases
      Serial.println("Unknown command");
      datoT[0] = 0;
      break;

    }

  ////////////////////////////////////////////////////////////

  ////////////////////// BLUETOOTH SERIAL ////////////////////
    if (Serial1.available()) { // Si hay datos disponibles en el puerto serie del Arduino IDE
      //mensaje = Serial.readString();  // Leer la velocidad de giro enviada desde Python
    // Serial.println(mensaje);

      deserializeJson(doc, Serial1);

      String Modo = doc["Modo"];
      
      /*
      int pos = mensaje.indexOf(',');
      int pos1 = mensaje.indexOf(',', pos + 1); // Buscar la segunda coma
      int pos2 = mensaje.indexOf(',', pos1 + 1); // Buscar la tercera coma
      int pos3 = mensaje.indexOf(',', pos2 + 1); // Buscar la cuarta coma
      int pos4 = mensaje.indexOf(',', pos3 + 1); // Buscar la quinta coma

      m1_vel = mensaje.substring(0, pos);
      m2_vel = mensaje.substring(pos + 1, pos1); // Extraer la subcadena entre la primera y segunda coma
      m3_vel = mensaje.substring(pos1 + 1, pos2); // Extraer la subcadena entre la segunda y tercera coma
      m4_vel = mensaje.substring(pos2 + 1, pos3); // Extraer la subcadena entre la tercera y cuarta coma
      Dato_movimiento = mensaje.substring(pos3 + 1, pos4); // Extraer la subcadena entre la cuarta y quinta coma
      Dato_velocidad = mensaje.substring(pos4 + 1); // La subcadena restante es para la velocidad

      int vel_m1 = m1_vel.toInt();
      int vel_m2 = m2_vel.toInt();
      int vel_m3 = m3_vel.toInt();
      int vel_m4 = m4_vel.toInt();
      int Valor_velocidad = Dato_velocidad.toInt();
      */

      if (Modo == "Auto"){

        int vel_m1 = doc["m1_vel"];
        int vel_m2 = doc["m2_vel"];
        int vel_m3 = doc["m3_vel"];
        int vel_m4 = doc["m4_vel"];

      // Controlar la velocidad del motor utilizando la señal PWM
        if (vel_m1 > 0) {
          Serial.println("Positivo M1");
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, vel_m1);
        }
        
        if (vel_m1 <= 0) {
          int Nvel_m1 = abs(vel_m1);
          Serial.println("Negativo M1");
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, Nvel_m1);
        }

        if (vel_m2 > 0) {
          Serial.println("Positivo M2");
          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, vel_m2);
        }
        
        if (vel_m2 <= 0) {
          int Nvel_m2 = abs(vel_m2);
          Serial.println("Negativo M2");
          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, Nvel_m2);
        }

        if (vel_m3 > 0) {
          Serial.println("Positivo M3");
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, vel_m3);
        }
        
        if (vel_m3 <= 0) {
          int Nvel_m3 = abs(vel_m3);
          Serial.println("Negativo M3");
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, Nvel_m3);
        }

        if (vel_m4 > 0) {
          Serial.println("Positivo M4");
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, vel_m4);
        }
        
        if (vel_m4 <= 0) {
          int Nvel_m4 = abs(vel_m4);
          Serial.println("Negativo M4");
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, Nvel_m4);
        }
      }

      if (Modo == "Manual"){

        String Dato_movimiento = doc["Dato_movimiento"];
        int Valor_velocidad = doc["Dato_velocidad"];

        //nuevo giro horario
        if (Dato_movimiento == "Giro_Horario"){
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //nuevo giro antihorario
        if (Dato_movimiento == "Giro_Antihorario"){
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //Nuevoo movimiento hacia adelante
        if (Dato_movimiento == "Adelante"){
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //Nuevo movimiento hacia atras
        if (Dato_movimiento == "Atras"){
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //Nuevo movimiento hacia la derecha
        if (Dato_movimiento == "Derecha"){
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, Valor_velocidad);

        }

        //Nuevo movimiento hacia la izquierda
        if (Dato_movimiento == "Izquierda"){
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, Valor_velocidad);

        }

        //Nuevo movimiento diagonal inferior derecha
        if (Dato_movimiento == "Diagonal_Superior_IZQ"){
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, 0);

          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, 0);
        }

        //Nuevo movimiento diagonal superior derecha
        if (Dato_movimiento == "Diagonal_Superior_DER"){
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, 0);
          
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, 0);
          
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //Nuevo movimiento diagonal inferior derecha
        if (Dato_movimiento == "Diagonal_Inferior_IZQ"){
          digitalWrite(M1_Dir, HIGH);
          analogWrite(M1_Vel, Valor_velocidad);

          digitalWrite(M2_Dir, HIGH);
          analogWrite(M2_Vel, 0);
          
          digitalWrite(M3_Dir, LOW);
          analogWrite(M3_Vel, 0);
          
          digitalWrite(M4_Dir, LOW);
          analogWrite(M4_Vel, Valor_velocidad);
        }

        //Nuevo movimiento diagonal superior izquierda
        if (Dato_movimiento == "Diagonal_Inferior_DER"){
          digitalWrite(M1_Dir, LOW);
          analogWrite(M1_Vel, 0);

          digitalWrite(M2_Dir, LOW);
          analogWrite(M2_Vel, Valor_velocidad);
          
          digitalWrite(M3_Dir, HIGH);
          analogWrite(M3_Vel, Valor_velocidad);
          
          digitalWrite(M4_Dir, HIGH);
          analogWrite(M4_Vel, 0);
        }
      }

      if (Modo == "Quieto"){
        digitalWrite(M1_Dir, LOW);
        analogWrite(M1_Vel, 0);

        digitalWrite(M2_Dir, LOW);
        analogWrite(M2_Vel, 0);
        
        digitalWrite(M3_Dir, HIGH);
        analogWrite(M3_Vel, 0);
        
        digitalWrite(M4_Dir, HIGH);
        analogWrite(M4_Vel, 0);

      }

    }

}

//====================== FUNCIONES ======================//

// Función para establecer el ángulo del servo
void setServoAngle(int angle) {
  if (angle >= 0 && angle <= 180) {  // Verifica que el ángulo esté dentro del rango permitido
    myServo.write(angle);  // Ajusta el ángulo del servo
    Serial.print("Servo angle set to: ");  // Imprime el ángulo establecido
    Serial.println(angle);  // Imprime el ángulo establecido
  } else {
    Serial.println("Error: Angle out of range");  // Imprime un mensaje de error si el ángulo está fuera del rango
  }
}

// Función para controlar el electroimán
void ImanStatus(bool activate) {
  if (activate) {
    digitalWrite(IMAN, HIGH);  // Activa el relé (enciende el electroimán)
    Serial.println("Electroimán activado");  // Imprime el estado en el monitor serial
  } else {
    digitalWrite(IMAN, LOW);  // Desactiva el relé (apaga el electroimán)
    Serial.println("Electroimán desactivado");  // Imprime el estado en el monitor serial
  }
}

// Función para controlar el electroimán
void LedStatus(bool activate1) {
  if (activate1) {
    digitalWrite(IND, HIGH);  // Activa el relé (enciende el electroimán)
    Serial.println("Led activado");  // Imprime el estado en el monitor serial
  } else {
    digitalWrite(IND, LOW);  // Desactiva el relé (apaga el electroimán)
    Serial.println("Led desactivado");  // Imprime el estado en el monitor serial
  }
}



void IrPiso(int numeroPisoDeseado) {
    // Verifica que el número de piso deseado esté dentro del rango de la lista
    if (numeroPisoDeseado < 0 || numeroPisoDeseado >= sizeof(Pisos) / sizeof(Pisos[0])) {
        Serial.println("El número de piso deseado no es válido.");
        return;
    }

    // Obtiene la altura deseada del piso
    int alturaDeseada = Pisos[numeroPisoDeseado];

    // Define la histeresis para evitar oscilaciones
    int histeresisSuperior = 10; // Ajusta según sea necesario
    int histeresisInferior = 10; // Ajusta según sea necesario

    // Define las velocidades máxima y mínima
    int velocidadMaxima = 200;
    int velocidadMinima = 100;

    // Loop para ajustar la altura hasta llegar al piso deseado
    while (true) {
        // Lee la altura actual
        int alturaActual = analogRead(potPin);

        // Calcula la diferencia entre la altura actual y la deseada
        int diferenciaAltura = alturaDeseada - alturaActual;

        // Calcula la velocidad basada en la diferencia de altura
        int velocidad = map(abs(diferenciaAltura), 1023, 0, velocidadMaxima, velocidadMinima);
        velocidad = constrain(velocidad, velocidadMinima, velocidadMaxima); // Asegura que la velocidad esté dentro del rango

        // Compara la altura actual con la altura deseada
        if (alturaActual < alturaDeseada - histeresisInferior) {
            // Si la altura actual es menor que la deseada, sube
            Subir(velocidad);
            Serial.println("Subiendo hacia el piso " + String(numeroPisoDeseado));
        } else if (alturaActual > alturaDeseada + histeresisSuperior) {
            // Si la altura actual es mayor que la deseada, baja
            Bajar(velocidad);
            Serial.println("Bajando hacia el piso " + String(numeroPisoDeseado));
        } else {
            // Si está dentro de la histeresis, detiene el motor
            Detener();
            Serial.println("Ha llegado al piso " + String(numeroPisoDeseado));
            break; // Sale del bucle while
        }
        
        // Espera un poco antes de volver a comprobar la altura
        delay(100);
    }
}



int ObtenerNumeroPiso() {
    // Espera un breve tiempo para asegurarse de que cualquier carácter residual se descarte por completo
    delay(500);

    // Borra cualquier carácter residual del buffer del puerto serial
    while (Serial.available() > 0) {
        Serial.read(); // Lee y descarta los caracteres del buffer
    }

    // Espera a que el usuario ingrese el número del piso
    Serial.println("Ingresa el número del piso al que deseas ir (0, 1, 2, ...):");

    while (true) {
        // Espera hasta que haya datos disponibles en el puerto serial
        while (Serial.available() <= 0) {
            // Espera activa
        }

        // Lee el número del piso ingresado por el usuario
        int numeroPiso = Serial.parseInt();

        // Limpia el buffer del puerto serial
        while (Serial.available()) {
            Serial.read(); // Lee y descarta los caracteres adicionales
        }

        // Verifica si el número ingresado es válido
        if (numeroPiso >= 0 && numeroPiso < sizeof(Pisos) / sizeof(Pisos[0])) {
            // Imprime el mensaje de confirmación
            Serial.println("Has seleccionado el piso " + String(numeroPiso));
            return numeroPiso; // Retorna el número del piso seleccionado
        } else {
            Serial.println("Número de piso no válido. Inténtalo de nuevo:");
        }
    }
}



void printAltura() {
    unsigned long currentMillis = millis();

    // Imprime el valor del potenciómetro cada 0.5 segundos
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        // Lee el valor del potenciómetro
        int altura = analogRead(potPin);
        Serial.println("Altura: " + String(altura));
    }
}

void Subir(int velocidad) {
    // Activa la salida
    digitalWrite(ENA, HIGH);
    // Giro derecha
    analogWrite(R_PWM, velocidad);
    analogWrite(L_PWM, 0);
    Serial.println("Girando a la derecha a velocidad " + String(velocidad));
}

void Bajar(int velocidad) {
    // Activa la salida
    digitalWrite(ENA, HIGH);
    // Giro izquierda
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, velocidad);
    Serial.println("Girando a la izquierda a velocidad " + String(velocidad));
}

void Detener() {
    // Desactiva la salida y detiene el motor
    digitalWrite(ENA, LOW);
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
    Serial.println("Motor detenido y desactivado");
}


// Function to move the carriage to home position
void home() {
  
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  // bool first = true;

  delay(10);
  // Imprimir el valor del endstop al inicio
  Serial.print("Valor inicial del endstop: ");
  Serial.println(digitalRead(ENDSTOP));
  
  // Mover hacia la posición de inicio hasta que se active el endstop
  while (!digitalRead(ENDSTOP)) {
    stepper.moveTo(-10000); // Mover lentamente hacia el home
    stepper.runToPosition();
    Serial.println("Homing");
  }

  // Configurar la posición actual a cero y detener el motor
  stepper.setCurrentPosition(0);
  Position = 0;
  stepper.stop();
  
  // Restablecer la velocidad y aceleración a los valores originales
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
}


// Function to move to a specific height and update Position variable
void moveToPosition(int targetPosition) {
  Serial.print("Move to position: ");
  Serial.println(targetPosition);
  if (targetPosition >= 0 && targetPosition <= PositionMax) {
    stepper.moveTo(targetPosition);
    stepper.runToPosition();
    Position = targetPosition;
  }
}

void printCurrentPosition() {
  long currentPosition = stepper.currentPosition();
  Serial.print("Current Position: ");
  Serial.println(currentPosition);
}