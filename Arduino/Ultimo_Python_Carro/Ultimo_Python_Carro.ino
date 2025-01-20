#include <ArduinoJson.h>
#include <Servo.h>
#include "AccelStepper.h"
#include <NewPing.h>


Servo myServo;  // Crea un objeto Servo

// StaticJsonDocument<200> doc;
const size_t bufferSize = 4096; // 4 KB, ajusta según sea necesario
DynamicJsonDocument doc(bufferSize);

String buffer = "";  // Buffer para acumular los datos



//SoftwareSerial BTSerial(19, 18);

// int M1_Dir = 53;
// int M1_Vel = 2;

// int M2_Dir = 51;
// int M2_Vel = 3;

// int M3_Dir = 49;
// int M3_Vel = 4;

// int M4_Dir = 47;
// int M4_Vel = 5;

///////// SERIAL /////////////

String strT = "";
const char separatorT = ',';
const int dataLengthT = 5;
int datoT[dataLengthT];


//////////////////////////////

// ========= MOTORES MOVIMIENTO =============//
#define M3_Dir 2
#define M3_Vel 3

#define M4_Dir 4
#define M4_Vel 5

#define M1_Dir 8
#define M1_Vel 9

#define M2_Dir 6
#define M2_Vel 7

String mensaje, m1_vel, m2_vel, m3_vel, m4_vel;
String Dato_movimiento, Dato_velocidad;


// ========= ASCENSOR =============//
#define ENA 10
#define L_PWM 11
#define R_PWM 12
// #define A_ANAG_PIN A0
int potPin = A0; // Pin del potenciómetro
int Distance;
int maxPosition = 16900;

unsigned long previousMillis = 0; // Almacena el último tiempo de actualización
const long interval = 500; // Intervalo de 0.5 segundos (500 milisegundos)

int Pisos[] = {
  80, // Piso 1, altura aproximada 100
  250, // Piso 1, altura aproximada 100
  550, // Piso 2, altura aproximada 200
  950, // Piso 3, altura aproximada 300
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
#define MAX_DISTANCE 200

NewPing sonar1(TRIG1, ECHO1, MAX_DISTANCE);
NewPing sonar2(TRIG2, ECHO2, MAX_DISTANCE);


int Distance1;
int Distance2;

//// ============= OTROS ===========///

#define LED_PIN 20 


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

    myServo.attach(SERVO_PIN);  // Conecta el servo al pin 9
    setServoAngle(90);  // Establece el ángulo del servo a 0 grados

  pinMode(IMAN, OUTPUT);  // Configura el pin del relé como salida
  digitalWrite(IMAN, LOW);  // Asegúrate de que el relé esté inicialmente apagado

   pinMode(ENA, OUTPUT);
    pinMode(L_PWM, OUTPUT);
    pinMode(R_PWM, OUTPUT);

    // Activa la salida
    digitalWrite(ENA, HIGH);

  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);

  pinMode(LED_PIN, OUTPUT);  // Configura el pin del relé como salida
  digitalWrite(LED_PIN, LOW);  // Asegúrate de que el relé esté inicialmente apagado


   ImanStatus(false);
   LedStatus(false);

    home();

} 

//====================== BUCLE PRINCIPAL ======================//

void loop() {
    ////////////////////////////////SERIAL USB//////////////////////////////////////

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

      moveToPosition(maxPosition);
      datoT[0] = 0;
      break;

      case 7:
      moveToPosition(datoT[1]);
      printCurrentPosition();
      printAltura();
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

       case 13:
      ReadAllDistances();
      datoT[0] = 0;
      break;

      case 14:
      CogerCarga();
      datoT[0] = 0;
      break;

      case 15:
      DejarCarga();
      datoT[0] = 0;
      break;

      case 17:
      AvanzarHasta(100, datoT[1]); // Mueve los motores hasta que la distancia sea <= 20 cm
      datoT[0] = 0;
      break;
      

      case 18:
      girar("Horario", 100 , datoT[1]*1000);
      datoT[0] = 0;
      break;

      case 19:
      girar("Antihorario", 100 , datoT[1]*1000);
      datoT[0] = 0;
      break;

      case 20:
      controlarTodosLosMotores(datoT[1], datoT[2], datoT[3], datoT[4]);
      datoT[0] = 0;
      break;

      default:
      // Handle unexpected cases
      Serial.println("Unknown command");
      datoT[0] = 0;
      break;

    }


    /////////////////////////////While Serial1////////////////////////////


        // Leer datos del HC-05
        while (Serial1.available()) {
          char c = Serial1.read();
          buffer += c;

          // Verificar si hemos recibido un terminador de línea
          if (c == '\n') {
            // Parpadear el LED indicando la recepción de datos
           

            // Intentar deserializar el JSON desde el buffer
            DeserializationError error = deserializeJson(doc, buffer);

            if (error) {
              Serial.print(F("Error al deserializar JSON: "));
              Serial.println(error.c_str());
              buffer = "";  // Limpiar el buffer en caso de error
              return;
            }

            // Extraer datos del JSON
            String Modo = doc["Modo"];
            String Dato_movimiento = doc["Dato_movimiento"];
            int Dato_velocidad = doc["Dato_velocidad"];

            // Validar que los datos sean correctos
            if (!Modo || !Dato_movimiento || Dato_velocidad < 0) {
              Serial.println("Datos no válidos recibidos.");
              buffer = "";  // Limpiar el buffer en caso de datos no válidos
              return;
            }

            // Imprimir los datos recibidos en el monitor serie
            Serial.print("Modo: ");
            Serial.println(Modo);
            Serial.print("Movimiento: ");
            Serial.println(Dato_movimiento);
            Serial.print("Velocidad: ");
            Serial.println(Dato_velocidad);


            //////////////////////////////////////////////////////////

            if (Modo == "Auto"){

                int vel_m1 = doc["m1_vel"];
                int vel_m2 = doc["m2_vel"];
                int vel_m3 = doc["m3_vel"];
                int vel_m4 = doc["m4_vel"];
                controlarTodosLosMotores(vel_m1,  vel_m2,  vel_m3,  vel_m4);

                String Dato_movimiento = doc["Dato_movimiento"];
                int Valor_velocidad = doc["Dato_velocidad"];
                int Valor_tiempo = doc["Dato_tiempo"];


                if (Dato_movimiento == "Avanzar"){
                  Serial.println("Avanzar Hasta");
                  AvanzarHasta(100, Valor_velocidad);
                }

                if (Dato_movimiento == "Giro_Horario"){
                  Serial.println("Avanzar Hasta");
                  girar("Horario", 100 , Valor_velocidad*1000);
                }

                if (Dato_movimiento == "Giro_Antihorario"){
                  Serial.println("Avanzar Hasta");
                  girar("Antihorario", 100 , Valor_velocidad*1000);
                }

                if (Dato_movimiento == "MoverPor"){
                  Serial.println("Mover Por");
                  MoverPor(Valor_velocidad, Valor_tiempo);
                }


              // Controlar la velocidad del motor utilizando la señal PWM
                


        
      }

      if (Modo == "Manual"){
        
        Serial.println("DENTRO MODO MANUAL");

        String Dato_movimiento = doc["Dato_movimiento"];
        int Valor_velocidad = doc["Dato_velocidad"];


        if (Dato_movimiento == "Giro_Time"){

          Serial.println("GIRO Tiempo");
          girar(Dato_movimiento, 100 , Valor_velocidad);
        }

        //nuevo giro horario
        if (Dato_movimiento == "Giro_Horario"){

          Serial.println("GIRO HORARIO SET");
          controlarTodosLosMotores(Valor_velocidad,  -Valor_velocidad,  Valor_velocidad,  -Valor_velocidad);
        }

        //nuevo giro antihorario
        if (Dato_movimiento == "Giro_Antihorario"){

          Serial.println("GIRO ANTI-HORARIO SET");
          controlarTodosLosMotores(-Valor_velocidad,  Valor_velocidad,  -Valor_velocidad,  Valor_velocidad);
        }

        //Nuevoo movimiento hacia adelante
        if (Dato_movimiento == "Adelante"){
          Serial.println("GIRO ADELANTE SET");
          controlarTodosLosMotores(Valor_velocidad,  Valor_velocidad,  Valor_velocidad,  Valor_velocidad);
        }

        //Nuevo movimiento hacia atras
        if (Dato_movimiento == "Atras"){
          Serial.println("GIRO ATRAS SET");
          controlarTodosLosMotores(-Valor_velocidad,  -Valor_velocidad,  -Valor_velocidad,  -Valor_velocidad);
        }

        //Nuevo movimiento hacia la derecha
        if (Dato_movimiento == "Derecha"){
          Serial.println("GIRO DERECHA SET");
          controlarTodosLosMotores(-Valor_velocidad,  Valor_velocidad,  Valor_velocidad,  -Valor_velocidad);

        }

        //Nuevo movimiento hacia la izquierda
        if (Dato_movimiento == "Izquierda"){
          Serial.println("GIRO IZQUIERDA SET");
          controlarTodosLosMotores(-Valor_velocidad,  Valor_velocidad,  -Valor_velocidad,  Valor_velocidad);

        }

        //Nuevo movimiento diagonal inferior derecha
        if (Dato_movimiento == "Diagonal_Superior_IZQ"){
          Serial.println("GIRO DIAG-SUP-IZQ SET");
          controlarTodosLosMotores(Valor_velocidad,  0,  0,  Valor_velocidad);
        }

        //Nuevo movimiento diagonal superior derecha
        if (Dato_movimiento == "Diagonal_Superior_DER"){
          Serial.println("GIRO DIAG-SUP-IZQ SET");
          controlarTodosLosMotores(0,  Valor_velocidad,  Valor_velocidad,  0);
        }

        //Nuevo movimiento diagonal inferior derecha
        if (Dato_movimiento == "Diagonal_Inferior_IZQ"){
          Serial.println("GIRO DIAG-INF-IZQ SET");
          controlarTodosLosMotores(0,  -Valor_velocidad,  -Valor_velocidad,  0);
        }

        //Nuevo movimiento diagonal superior izquierda
        if (Dato_movimiento == "Diagonal_Inferior_DER"){
          controlarTodosLosMotores(-Valor_velocidad,  0,  0,  -Valor_velocidad);
        }

        ////////////////////////////////////////////
        
        //Nuevo movimiento diagonal superior izquierda
        if (Dato_movimiento == "Leer"){
           ReadAllDistances();
            Serial1.print("100, ");
            Serial1.print(Distance1);
            Serial1.print(", ");
            Serial1.println(Distance2);
        }

        if (Dato_movimiento == "Servo"){
           setServoAngle(Valor_velocidad);
        }

        if (Dato_movimiento == "SUBIR"){
            Subir(velocidad);
        }

        if (Dato_movimiento == "BAJAR"){
           Bajar(velocidad);
        }

        if (Dato_movimiento == "DETENER"){
          Detener();
        }

        if (Dato_movimiento == "Ir_Piso"){
          IrPiso(Valor_velocidad);
        }

        if (Dato_movimiento == "HOME"){
           home();
        }

        if (Dato_movimiento == "INICIO"){
          moveToPosition(0);
        }

        if (Dato_movimiento == "FIN"){
          moveToPosition(PositionMax);
        }

         if (Dato_movimiento == "ENCENDER_LED"){
          LedStatus(true);
          LedStatus(true);
        }

         if (Dato_movimiento == "APAGAR_LED"){
          LedStatus(false);
          LedStatus(false);
        }

         if (Dato_movimiento == "ENCENDER_IMAN"){
          ImanStatus(true);
          ImanStatus(true);
        }

         if (Dato_movimiento == "APAGAR_IMAN"){
          ImanStatus(false);
          ImanStatus(false);
        }

        if (Dato_movimiento == "Coger_Carga"){
          CogerCarga();
        }

        if (Dato_movimiento == "Dejar_Carga"){
          DejarCarga();
        }

      }




      if (Modo == "Quieto"){
        Serial.println("QUIETO SET");
        digitalWrite(M1_Dir, LOW);
        analogWrite(M1_Vel, 0);

        digitalWrite(M2_Dir, LOW);
        analogWrite(M2_Vel, 0);
        
        digitalWrite(M3_Dir, HIGH);
        analogWrite(M3_Vel, 0);
        
        digitalWrite(M4_Dir, HIGH);
        analogWrite(M4_Vel, 0);

      }


            
            //////////////////////////////////////////////////////////

            // Limpiar el buffer después de procesar el mensaje
            buffer = "";
          }
        }




    //////////////////////////////////////////////////////////
}
    

  ////////////////////////////////////////////////////////////


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
    digitalWrite(IMAN, LOW);  // Activa el relé (enciende el electroimán)
    Serial.println("Electroimán activado");  // Imprime el estado en el monitor serial
  } else {
    digitalWrite(IMAN, HIGH);  // Desactiva el relé (apaga el electroimán)
    Serial.println("Electroimán desactivado");  // Imprime el estado en el monitor serial
  }
}

// Función para controlar el electroimán
void LedStatus(bool activate) {
  if (activate) {
    digitalWrite(LED_PIN, LOW);  // Activa el relé (enciende el electroimán)
    Serial.println("Led activado");  // Imprime el estado en el monitor serial
  } else {
    digitalWrite(LED_PIN, HIGH);  // Desactiva el relé (apaga el electroimán)
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
    int velocidadMaxima = 250;
    int velocidadMinima = 150;

    // Loop para ajustar la altura hasta llegar al piso deseado
    while (true) {
        // Lee la altura actual
        int alturaActual = analogRead(potPin);

        // Calcula la diferencia entre la altura actual y la deseada
        int diferenciaAltura = alturaDeseada - alturaActual;

        // Calcula la velocidad basada en la diferencia de altura
        int velocidad = map(abs(diferenciaAltura), 1023, 0, velocidadMaxima, velocidadMinima);
        velocidad = constrain(velocidad, velocidadMinima, velocidadMaxima); // Asegura que la velocidad esté dentro del rango
        // int velocidad = 150;
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
  // Configurar velocidad y aceleración para el proceso de homing
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  
  // Mover el motor continuamente hacia el home
  stepper.move(20000); // Configurar el movimiento inicial

  // Bucle para mover el motor hacia la posición de home
  while (digitalRead(ENDSTOP) == HIGH) {
    stepper.run(); // Mantener el motor en movimiento

    // Imprimir mensaje de homing en el monitor seria
    // Verificar si el endstop se ha activado
    if (digitalRead(ENDSTOP) == LOW) {
      // Detener el motor inmediatamente
      stepper.stop();
      Serial.println("ENDSTOP activado, motor detenido.");
      break; // Salir del bucle while
    }
  }

  // Establecer la posición actual a cero y detener el motor
  // stepper.setCurrentPosition(0); 
  // maxPosition = stepper.currentPosition();
  stepper.setCurrentPosition(maxPosition);
  
  // Configurar la velocidad y aceleración a los valores originales
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
  Serial.println("Home Done");
  moveToPosition(0);
  
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


// Función para leer la distancia utilizando NewPing
int ReadDistance(int trigPin, int echoPin, int maxDistance) {
  NewPing sonar(trigPin, echoPin, maxDistance);
  int distance = sonar.ping_cm();
  return (distance == 0) ? maxDistance : distance;
}

// Función para leer y actualizar las distancias globales de los sensores
void ReadAllDistances() {
  Distance1 = ReadDistance(TRIG1, ECHO1, MAX_DISTANCE);
  Distance2 = ReadDistance(TRIG2, ECHO2, MAX_DISTANCE);

  // Imprime las distancias en la consola serie
  Serial.print("Distance1: ");
  Serial.print(Distance1);
  Serial.println(" cm");

  Serial.print("Distance2: ");
  Serial.print(Distance2);
  Serial.println(" cm");

  // Espera 500 ms antes de la siguiente lectura
  delay(500);
}


void controlarMotor(int vel, int dirPin, int velPin, const char* motorLabel) {
  if (vel > 0) {
    Serial.print("Positivo ");
    Serial.println(motorLabel);
    digitalWrite(dirPin, LOW);
    analogWrite(velPin, vel);
  } else {
    int Nvel = abs(vel);
    Serial.print("Negativo ");
    Serial.println(motorLabel);
    digitalWrite(dirPin, HIGH);
    analogWrite(velPin, Nvel);
  }
}

void controlarTodosLosMotores(int vel1, int vel2, int vel3, int vel4) {
  controlarMotor(vel1, M1_Dir, M1_Vel, "M1");
  controlarMotor(-vel2, M2_Dir, M2_Vel, "M2");
  controlarMotor(vel3, M3_Dir, M3_Vel, "M3");
  controlarMotor(-vel4, M4_Dir, M4_Vel, "M4");
}

void girar(String direccion, int velocidad, unsigned long tiempo) {
    // Activar motores según la dirección
    if (direccion == "Horario") {
        Serial.println("GIRO HORARIO SET");
        controlarTodosLosMotores(velocidad, -velocidad, velocidad, -velocidad);
    } else if (direccion == "Antihorario") {
        Serial.println("GIRO ANTI-HORARIO SET");
        controlarTodosLosMotores(-velocidad, velocidad, -velocidad, velocidad);
    } else {
        Serial.println("DIRECCIÓN NO RECONOCIDA");
        return; // Salir si la dirección no es válida
    }

    // Esperar el tiempo especificado
    delay(tiempo);

    // Detener los motores
    Serial.println("DETENCIÓN DE MOTORES");
    controlarTodosLosMotores(0, 0, 0, 0);
}

// Función para avanzar hasta que la distancia sea menor o igual al límite
void AvanzarHasta(int velocidad, int limite) {
  // Activa los motores a la velocidad deseada
  controlarTodosLosMotores(velocidad, velocidad, velocidad , velocidad);
  
  long distancia;
  
  do {
    distancia = ReadDistance(TRIG2, ECHO2, MAX_DISTANCE); // Lee la distancia del sensor
    Serial.print("Distancia actual: ");
    Serial.print(distancia);
    Serial.println(" cm");
    
    delay(100); // Espera 100 ms antes de leer nuevamente
  } while (distancia > limite);
  
  // Detén los motores cuando se alcance el límite
  controlarTodosLosMotores(0,0,0,0);
}

void MoverPor(int velocidad, unsigned long tiempo) {
  // Activa todos los motores a la velocidad deseada
  controlarTodosLosMotores(velocidad, velocidad, velocidad, velocidad);
  
  unsigned long tiempoInicio = millis(); // Guarda el tiempo actual
  
  // Mantiene el movimiento durante el tiempo especificado
  while (millis() - tiempoInicio < tiempo) {
    // Puedes incluir aquí código adicional si es necesario
    delay(10); // Espera para evitar sobrecargar la CPU
  }
  
  // Detén los motores cuando se haya transcurrido el tiempo especificado
  controlarTodosLosMotores(0, 0, 0, 0);
}


void CogerCarga() {
    // Mover el accionamiento lineal hasta la posición máxima
    moveToPosition(PositionMax);
    delay(100); // Esperar 100 ms

    // Activar el electroimán
    LedStatus(true);
    delay(100); // Esperar 100 ms

    // Mover el accionamiento lineal de vuelta a la posición 0
    moveToPosition(0);
    delay(100); // Esperar 100 ms

    // Desactivar el electroimán
    LedStatus(false);
    delay(100); // Esperar 100 ms
}

void DejarCarga() {
    // Activar el electroimán
    LedStatus(true);
    delay(100); // Esperar 100 ms
    // Mover el accionamiento lineal hasta la posición máxima
    moveToPosition(PositionMax);
    delay(100); // Esperar 100 ms
    // Mover el accionamiento lineal de vuelta a la posición 0
    LedStatus(false);
    delay(100); // Esperar 100 ms
    moveToPosition(0);
    delay(100); // Esperar 100 ms
    // Desactivar el electroimán
    
}
