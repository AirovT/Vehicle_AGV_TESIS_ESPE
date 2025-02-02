#include <Servo.h>
#include "AccelStepper.h"
#include <NewPing.h>
#include "motorControl.h"

Servo myServo;  // Crea un objeto Servo

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
String strTot = "";
const char separatorT = ',';
const int dataLengthT = 5;
int datoT[dataLengthT];


//////////////////////////////

// ========= MOTORES MOVIMIENTO =============//

#define M1_Dir 4
#define M1_Vel 5

#define M2_Dir 2
#define M2_Vel 3

#define M3_Dir 6
#define M3_Vel 7

#define M4_Dir 8
#define M4_Vel 9



String mensaje, m1_vel, m2_vel, m3_vel, m4_vel;
String Dato_movimiento, Dato_velocidad;


// ========= ASCENSOR =============//
#define ENA 10
#define L_PWM 12
#define R_PWM 11
// #define A_ANAG_PIN A0
int potPin = A0; // Pin del potenciómetro
int Distance;
int maxPosition = 16900;
int minSpeed = 35;

unsigned long previousMillis = 0; // Almacena el último tiempo de actualización
const long interval = 500; // Intervalo de 0.5 segundos (500 milisegundos)

int Pisos[] = {
  200, // Piso 1, altura aproximada 100
  260, // Piso 1, altura aproximada 100
  550, // Piso 2, altura aproximada 200
  950, // Piso 3, altura aproximada 300
  // Agrega más pisos según sea necesario
};

// ========= STEP =============//

#define DIR 24
#define PUL 26
#define motorInterfaceType 1

#define IMAN 20
#define LED_PIN 38 

#define SERVO_PIN 13
#define ENDSTOP 48

// Define height and maximum height
int Position = 0;
int PositionMax = 17000;

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, PUL, DIR);

int ServoDegree;
bool Iman;

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



// speed
 
float rwheel = 9.5;
float Lx=18.15;
float Ly=19.63;
float speedA;
float speedB;
float speedC;
float speedD;
float DefspeedA=0;
float DefspeedB=0;
float DefspeedC=0;
float DefspeedD=0;


unsigned long sampleTimeW = 250;    // Para calcular


  motorControl motor1(sampleTimeW); 
  motorControl motor2(sampleTimeW); 
  motorControl motor3(sampleTimeW); 
  motorControl motor4(sampleTimeW);



unsigned long lastTime_CT = 0;

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

  Bajar(100);
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
  

  pinMode(LED_PIN, OUTPUT);  // Configura el pin del relé como salida
  digitalWrite(LED_PIN, LOW);  // Asegúrate de que el relé esté inicialmente apagado


   ImanStatus(false);
   LedStatus(false);

    home();


  motor1.setCvLimits(200,0); 
  motor1.setPvLimits(800,0);
  motor1.setGains(0.4, 0.5, 0.02); // (Kc,Ti,Td)
  
  motor2.setCvLimits(200,0); 
  motor2.setPvLimits(800,0);
  motor2.setGains(0.4, 0.5, 0.02); // (Kc,Ti,Td)

  motor3.setCvLimits(200,0); 
  motor3.setPvLimits(800,0);
  motor3.setGains(0.4, 0.5, 0.02); // (Kc,Ti,Td)

  motor4.setCvLimits(200,0); 
  motor4.setPvLimits(800,0);
  motor4.setGains(0.4, 0.5, 0.02); // (Kc,Ti,Td)

setServoAngle(175);  
Detener();
ReadAllDistances();

Serial1.println(datoT[1]);
} 

//====================== BUCLE PRINCIPAL ======================//

void loop() {
    ////////////////////////////////SERIAL USB//////////////////////////////////////

    strT = "";
    strTot = "";
    if (Serial.available())
    {
      strT = Serial.readStringUntil('\n');
      Serial.println(strT);
      strTot = strT;
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

    if (Serial1.available())
    {
      strT = Serial1.readStringUntil('\n');
      strTot = strT;
      // Serial.println(strT);
      for (int i = 0; i < dataLengthT; i++)
      {
        int index = strT.indexOf(separatorT);
        datoT[i] = strT.substring(0, index).toInt();
        strT = strT.substring(index + 1);
      }
      // for (int i = 0; i < dataLengthT; i++)
      // {
      //   Serial.print("Dato ");
      //   Serial.print(i);
      //   Serial.print("  =  ");
      //   Serial.print(datoT[i]);
      //   Serial.print("  -  ");
      // }
      // Serial.println(" ");
    }

    /////////////////////////////While Serial1////////////////////////////

    int velocidad = 200;

    switch ((int)datoT[0])
    {
      case 0:
      // printAltura();
      datoT[0] = 0;
      break;

      case 1:
      Subir(200);
      datoT[0] = 0;
      break;

      case 2:
      Bajar(100);
      datoT[0] = 0;
      break;

      case 3:
      Detener();
      datoT[0] = 0;
      break;

      case 4:
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
     
      datoT[0] = 0;
      break;
      
      case 11:
      delay(10);
      
      datoT[0] = 0;
      break;

      case 12:
      setServoAngle(datoT[1]);
      datoT[0] = 0;
      break;

       case 13:
      ReadAllDistances();
      PrintDistances();
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
      FrontBack(datoT[1],150);
      

      datoT[0] = 0;
      break;
      

      case 18:
      LeftRight(datoT[1],150);

      datoT[0] = 0;
      break;

      case 19:
      Rotate(datoT[1] , 150);

      datoT[0] = 0;
      break;

      case 20:
      // controlarTodosLosMotores(datoT[1], datoT[2], datoT[3], datoT[4]);
      DefspeedA= datoT[1];
      DefspeedB= datoT[2];
      DefspeedC= datoT[3];
      DefspeedD= datoT[4];


      datoT[0] = 0;
      break;

      case 21:
      printAltura();
      Endstop_Status();
      datoT[0] = 0;
      break;

      case 30:
      Serial1.println(strTot);
      Serial.println("Mensaje enviado: ");
      Serial.println(strTot);
      datoT[0] = 0;
      break;

      case 100:
      speedA= datoT[1];
      speedB= datoT[2];
      speedC= datoT[3];
      speedD= datoT[4];
      break;

      default:
      // Handle unexpected cases
      Serial.println("Unknown command");
      datoT[0] = 0;
      break;

    }

    if(millis()-lastTime_CT >= sampleTimeW || lastTime_CT == 0)
    {
      controlarTodosLosMotores(DefspeedA, DefspeedB, DefspeedC, DefspeedD);
       lastTime_CT = millis();
    }

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
    int velocidadMinima = 175;

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
  stepper.move(-20000); // Configurar el movimiento inicial

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
  // stepper.setCurrentPosition(maxPosition);
  stepper.setCurrentPosition(0);
  
  // Configurar la velocidad y aceleración a los valores originales
  stepper.setMaxSpeed(3000);
  stepper.setAcceleration(1000);
  Serial.println("Home Done");
  moveToPosition(maxPosition);
  
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
  // delay(500);
}

void PrintDistances() {
 
  ReadAllDistances();
  String message = "133," + String(int(Distance1)) + "," + String(int(Distance2));

  // Enviar el mensaje por Serial1
  Serial1.println(message);

  // También imprimir el mensaje en el monitor serial
  Serial.println(message);
}



void controlarMotor(int vel, int dirPin, int velPin, const char* motorLabel) {
  if (vel > 0) {
    // Serial.print("Positivo ");
    // Serial.println(motorLabel);
    digitalWrite(dirPin, LOW);
    analogWrite(velPin, vel);
  } else {
    int Nvel = abs(vel);
    // Serial.print("Negativo ");
    // Serial.println(motorLabel);
    digitalWrite(dirPin, HIGH);
    analogWrite(velPin, Nvel);
  }
}


// Variables
int cvM1 = 0;
double wM1 = 0;
double wM1ref = 0;

// Variables
int cvM2 = 0;
double wM2 = 0;
double wM2ref = 0;

// Variables
int cvM3 = 0;
double wM3 = 0;
double wM3ref = 0;

// Variables
int cvM4 = 0;
double wM4 = 0;
double wM4ref = 0;


void ControlPID(int velocidad, int pol1, int pol2, int pol3, int pol4) {
  int cvM = 0;

  // Calcular el valor del control PID usando solo el motor 1
  cvM = motor1.compute(velocidad, speedA); // Control PID para el motor 1

  // Controlar los motores utilizando el valor calculado y las polaridades
  controlarMotor(cvM * pol1, M1_Dir, M1_Vel, "M1");
  controlarMotor(cvM * pol2, M2_Dir, M2_Vel, "M2");
  controlarMotor(cvM * pol3, M3_Dir, M3_Vel, "M3");
  controlarMotor(cvM * pol4, M4_Dir, M4_Vel, "M4");
}


void controlarTodosLosMotores(int vel1, int vel2, int vel3, int vel4) {
 
  // Controlar motores con los valores calculados
  controlarMotor(vel1, M1_Dir, M1_Vel, "M1");
  controlarMotor(vel2, M2_Dir, M2_Vel, "M2");
  controlarMotor(vel3, M3_Dir, M3_Vel, "M3");
  controlarMotor(vel4, M4_Dir, M4_Vel, "M4");
}



// void girar(String direccion, int velocidad, unsigned long tiempo) {
//     // Activar motores según la dirección
//     if (direccion == "Horario") {
//         Serial.println("GIRO HORARIO SET");
//         controlarTodosLosMotores(velocidad, -velocidad, velocidad, -velocidad);
//     } else if (direccion == "Antihorario") {
//         Serial.println("GIRO ANTI-HORARIO SET");
//         controlarTodosLosMotores(-velocidad, velocidad, -velocidad, velocidad);
//     } else {
//         Serial.println("DIRECCIÓN NO RECONOCIDA");
//         return; // Salir si la dirección no es válida
//     }

//     // Esperar el tiempo especificado
//     delay(tiempo);

//     // Detener los motores
//     Serial.println("DETENCIÓN DE MOTORES");
//     controlarTodosLosMotores(0, 0, 0, 0);
// }

// // Función para avanzar hasta que la distancia sea menor o igual al límite
// void AvanzarHasta(int velocidad, int limite) {
//   // Activa los motores a la velocidad deseada
//   controlarTodosLosMotores(velocidad, velocidad, velocidad , velocidad);
  
//   long distancia;
  
//   do {
//     distancia = ReadDistance(TRIG2, ECHO2, MAX_DISTANCE); // Lee la distancia del sensor
//     Serial.print("Distancia actual: ");
//     Serial.print(distancia);
//     Serial.println(" cm");
    
//     delay(100); // Espera 100 ms antes de leer nuevamente
//   } while (distancia > limite);
  
//   // Detén los motores cuando se alcance el límite
//   controlarTodosLosMotores(0,0,0,0);
// }

// void MoverPor(int velocidad, unsigned long tiempo) {
//   // Activa todos los motores a la velocidad deseada
//   controlarTodosLosMotores(velocidad, velocidad, velocidad, velocidad);
  
//   unsigned long tiempoInicio = millis(); // Guarda el tiempo actual
  
//   // Mantiene el movimiento durante el tiempo especificado
//   while (millis() - tiempoInicio < tiempo) {
//     // Puedes incluir aquí código adicional si es necesario
//     delay(10); // Espera para evitar sobrecargar la CPU
//   }
  
//   // Detén los motores cuando se haya transcurrido el tiempo especificado
//   controlarTodosLosMotores(0, 0, 0, 0);
// }



void CogerCarga() {
    // Mover el accionamiento lineal hasta la posición máxima
    moveToPosition(0);
    delay(100); // Esperar 100 ms

    // Activar el electroimán
    ImanStatus(true);
    delay(100); // Esperar 100 ms

    // Mover el accionamiento lineal de vuelta a la posición 0
    moveToPosition(PositionMax);
    delay(100); // Esperar 100 ms

    // Desactivar el electroimán
    ImanStatus(false);
    delay(100); // Esperar 100 ms
}

void DejarCarga() {
    // Activar el electroimán
    ImanStatus(true);
    delay(100); // Esperar 100 ms
    // Mover el accionamiento lineal hasta la posición máxima
    moveToPosition(0);
    delay(100); // Esperar 100 ms
    // Mover el accionamiento lineal de vuelta a la posición 0
    ImanStatus(false);
    delay(100); // Esperar 100 ms
    moveToPosition(PositionMax);
    delay(100); // Esperar 100 ms
    // Desactivar el electroimán
    
}

// void FrontBack(int position, int Speed)
// {   
//     // Inicializa la posición acumulada y la matriz de datos
//     float PositionTraslation = 0;
//     int datos[5];

//     // Ajusta la velocidad según la dirección (posición positiva o negativa)
//     int adjustedSpeed = (position < 0) ? -Speed : Speed;

//     // Controla los motores con la velocidad ajustada
//     controlarTodosLosMotores(adjustedSpeed, adjustedSpeed, adjustedSpeed, adjustedSpeed);
//     // ControlPID(adjustedSpeed, 1,1,1,1);

//     // Bucle de control para avanzar o retroceder
//     while ((position > 0 && PositionTraslation < position) || 
//            (position < 0 && PositionTraslation > position))
//     {
//         if (Serial1.available())
//         {
//             strT = Serial1.readStringUntil('\n');
//             Serial.println(strT);
//             for (int i = 0; i < dataLengthT; i++)
//             {
//                 int index = strT.indexOf(separatorT);
//                 datos[i] = strT.substring(0, index).toInt();
//                 strT = strT.substring(index + 1);
//             }
//         }

//         if (datos[0] == 100)
//         {
//             unsigned long currentMillis = millis();

//             if (currentMillis - previousMillis > sampleTimeW)
//             {
//                 previousMillis = currentMillis;

//                 float dVx = rwheel * (datos[1] / 100);
//                 float dPx = dVx * sampleTimeW / 1000;

//                 // Ajusta la posición acumulada según la dirección
//                 PositionTraslation += dPx;

//                 Serial.println(PositionTraslation);
//             }
//             datos[0] = 0;
//         }
//     }
    
//     // Detiene los motores y reinicia la posición
//     if ((position > 0 && PositionTraslation >= position) || 
//         (position < 0 && PositionTraslation <= position))
//     {
//         controlarTodosLosMotores(0, 0, 0, 0);
//         PositionTraslation = 0;
//     }
// }

void FrontBack(int position, int Speed)
{   
        String message = "150,0";

        // Enviar el mensaje por Serial1
        Serial1.println(message);
        Serial1.println(message);

        // También imprimir el mensaje en el monitor serial
        Serial.println("BANDERA INICIO");

    // Inicializa la posición acumulada y la matriz de datos
    float PositionTraslation = 0;
    int datos[5];

    // Ajusta la dirección de la velocidad según la posición deseada
    int direction = (position < 0) ? -1 : 1;
    Speed = abs(Speed); // Aseguramos que Speed es positivo
    position = abs(position); // Trabajamos con valores absolutos de posición
    // Bucle de control para avanzar o retroceder
      while ((direction > 0 && PositionTraslation < position) || 
            (direction < 0 && PositionTraslation > -position))
      {
          // Leer datos del puerto serie
          if (Serial1.available())
          {
              strT = Serial1.readStringUntil('\n');
              Serial.println(strT);
              for (int i = 0; i < dataLengthT; i++)
              {
                  int index = strT.indexOf(separatorT);
                  datos[i] = strT.substring(0, index).toInt();
                  strT = strT.substring(index + 1);
              }
          }

           if (datos[0] == 20)
        {
          DefspeedA= 0;
          DefspeedB= 0;
          DefspeedC= 0;
          DefspeedD= 0;

          break;
        }

          if (datos[0] == 100)
          {
              unsigned long currentMillis = millis();

              if (currentMillis - previousMillis > sampleTimeW)
              {
                  previousMillis = currentMillis;

                  // Calcula la velocidad y desplazamiento absolutos
                  float dVx = rwheel * abs(datos[2] / 100); // Toma el valor absoluto de datos[1]
                  float dPx = dVx * sampleTimeW / 1000;

                  // Ajusta la posición acumulada según la dirección del movimiento
                  PositionTraslation += dPx * direction;

                  // Calcula la velocidad relativa al progreso total
                  float absPosition = abs(PositionTraslation); // Usa posición absoluta para las verificaciones
                  float speedFactor = 0;

                  if (absPosition <= position * 0.5) // Primera mitad del movimiento
                  {
                      speedFactor = absPosition / (position * 0.5); // Incremento lineal
                  }
                  else // Segunda mitad del movimiento
                  {
                      speedFactor = (position - absPosition) / (position * 0.5); // Decremento lineal
                  }

                  // Asegura que la velocidad se encuentra entre minSpeed y Speed
                  int adjustedSpeed = minSpeed + (Speed - minSpeed) * speedFactor;
                  adjustedSpeed *= direction; // Aplica la dirección para el control del motor

                  // Controla los motores con la velocidad ajustada
                  controlarTodosLosMotores(adjustedSpeed, adjustedSpeed, adjustedSpeed, adjustedSpeed);

                  // Mensajes de depuración
                  Serial.print("Position: ");
                  Serial.print(PositionTraslation);
                  Serial.print(" Speed: ");
                  Serial.println(adjustedSpeed);
              }

              // Resetea el indicador de datos leídos
              datos[0] = 0;
          }
      }

      // Detener los motores y reiniciar la posición
      controlarTodosLosMotores(0, 0, 0, 0);
      PositionTraslation = 0;

       message = "150,1";

        // Enviar el mensaje por Serial1
        Serial1.println(message);
        Serial1.println(message);

        // También imprimir el mensaje en el monitor serial
        Serial.println("BANDERA FIN");

}


void LeftRight(int position, int Speed)
{   
    // Inicializa la posición acumulada y la matriz de datos
    float PositionTraslation = 0;
    int datos[5];

    // Ajusta la dirección de la velocidad según la posición deseada
    int direction = (position < 0) ? -1 : 1;
    Speed = abs(Speed); // Aseguramos que Speed es positivo
    position = abs(position); // Trabajamos con valores absolutos de posición
    // Bucle de control para avanzar o retroceder
      while ((direction > 0 && PositionTraslation < position) || 
            (direction < 0 && PositionTraslation > -position))
      {
          // Leer datos del puerto serie
          if (Serial1.available())
          {
              strT = Serial1.readStringUntil('\n');
              Serial.println(strT);
              for (int i = 0; i < dataLengthT; i++)
              {
                  int index = strT.indexOf(separatorT);
                  datos[i] = strT.substring(0, index).toInt();
                  strT = strT.substring(index + 1);
              }
          }

        

          if (datos[0] == 20)
        {
          DefspeedA= 0;
          DefspeedB= 0;
          DefspeedC= 0;
          DefspeedD= 0;
           break;
        }

        if (Distance1 <=10)
        {
          DefspeedA= 0;
          DefspeedB= 0;
          DefspeedC= 0;
          DefspeedD= 0;
           break;
        }

          if (datos[0] == 100)
          {
              unsigned long currentMillis = millis();

              if (currentMillis - previousMillis > sampleTimeW)
              {
                  previousMillis = currentMillis;

                  // Calcula la velocidad y desplazamiento absolutos
                  float dVx = rwheel * (datos[2] / 100); // Toma el valor absoluto de datos[1]
                  float dPx = dVx * sampleTimeW / 1000;

                  // Ajusta la posición acumulada según la dirección del movimiento
                  PositionTraslation += dPx;

                  // Calcula la velocidad relativa al progreso total
                  float absPosition = abs(PositionTraslation); // Usa posición absoluta para las verificaciones
                  float speedFactor = 0;

                  if (absPosition <= position * 0.5) // Primera mitad del movimiento
                  {
                      speedFactor = absPosition / (position * 0.5); // Incremento lineal
                  }
                  else // Segunda mitad del movimiento
                  {
                      speedFactor = (position - absPosition) / (position * 0.5); // Decremento lineal
                  }

                  // Asegura que la velocidad se encuentra entre minSpeed y Speed
                  int adjustedSpeed = minSpeed + (Speed - minSpeed) * speedFactor;
                  adjustedSpeed *= direction; // Aplica la dirección para el control del motor

                  // Controla los motores con la velocidad ajustada
                  controlarTodosLosMotores(adjustedSpeed, -adjustedSpeed, adjustedSpeed, -adjustedSpeed);

                  // Mensajes de depuración
                  Serial.print("Position: ");
                  Serial.print(PositionTraslation);
                  Serial.print(" Speed: ");
                  Serial.println(adjustedSpeed);
                  ReadAllDistances();
              }

              // Resetea el indicador de datos leídos
              datos[0] = 0;
          }
      }

      // Detener los motores y reiniciar la posición
      controlarTodosLosMotores(0, 0, 0, 0);
      PositionTraslation = 0;


        String message = "150,1";

        // Enviar el mensaje por Serial1
        Serial1.println(message);
        Serial1.println(message);
        // También imprimir el mensaje en el monitor serial
        Serial.println("BANDERA FIN");      

}

// void LeftRight(int position, int Speed)
// {   
//     // Inicializa la posición acumulada y la matriz de datos
//     float PositionTraslation = 0;
//     int datos[5];

//     // Ajusta la velocidad según la dirección (posición positiva o negativa)
//     int adjustedSpeed = (position < 0) ? -Speed : Speed;

//     // Controla los motores con la velocidad ajustada
//     // controlarTodosLosMotores(adjustedSpeed, -adjustedSpeed, adjustedSpeed, -adjustedSpeed);
//     ControlPID(adjustedSpeed, 1,-1,1,-1);

//     // Bucle de control para avanzar o retroceder
//     while ((position > 0 && PositionTraslation < position) || 
//            (position < 0 && PositionTraslation > position))
//     {
//         if (Serial1.available())
//         {
//             strT = Serial1.readStringUntil('\n');
//             Serial.println(strT);
//             for (int i = 0; i < dataLengthT; i++)
//             {
//                 int index = strT.indexOf(separatorT);
//                 datos[i] = strT.substring(0, index).toInt();
//                 strT = strT.substring(index + 1);
//             }
//         }

//         if (datos[0] == 100)
//         {
//             unsigned long currentMillis = millis();

//             if (currentMillis - previousMillis > sampleTimeW)
//             {
//                 previousMillis = currentMillis;

//                 float dVx = rwheel * (datos[1] / 100);
//                 float dPx = dVx * sampleTimeW / 1000;

//                 // Ajusta la posición acumulada según la dirección
//                 PositionTraslation += dPx;

//                 Serial.println(PositionTraslation);
//             }
//             datos[0] = 0;
//         }
//     }
    
//     // Detiene los motores y reinicia la posición
//     if ((position > 0 && PositionTraslation >= position) || 
//         (position < 0 && PositionTraslation <= position))
//     {
//         controlarTodosLosMotores(0, 0, 0, 0);
//         PositionTraslation = 0;
//     }
// }

// void Rotate(int angle, int Speed)
// {
//     float rotationAngle = 0; // Ángulo acumulado
//     int datos[5];            // Matriz para almacenar los datos recibidos

//     // Ajusta la velocidad según la dirección (rotación horaria o antihoraria)
//     int adjustedSpeed = (angle < 0) ? Speed : -Speed;

//     // Controla los motores para rotar (direcciones opuestas en pares de motores)
//     // controlarTodosLosMotores(-adjustedSpeed, adjustedSpeed, adjustedSpeed, -adjustedSpeed);
//     ControlPID(adjustedSpeed, -1,1,1,-1);

//     // Bucle de control para realizar la rotación
//     while ((angle > 0 && rotationAngle < angle) || 
//            (angle < 0 && rotationAngle > angle))
//     {
//         if (Serial1.available())
//         {
//             strT = Serial1.readStringUntil('\n');
//             Serial.println(strT);
//             for (int i = 0; i < dataLengthT; i++)
//             {
//                 int index = strT.indexOf(separatorT);
//                 datos[i] = strT.substring(0, index).toInt();
//                 strT = strT.substring(index + 1);
//             }
//         }

//         if (datos[0] == 100)
//         {
//             unsigned long currentMillis = millis();

//             if (currentMillis - previousMillis > sampleTimeW)
//             {
//                 previousMillis = currentMillis;

//                 // Calcula la variación de ángulo (dW)
//                 float dWp = rwheel*(((datos[1] - datos[2] - datos[3] + datos[4]) / 100.0) / (4*(Lx + Ly))) ;
//                 // float dWp = rwheel*(((-datos[1] + datos[2] + datos[3] - datos[4]) / 100.0) / (4*(Lx + Ly))) ;
//                 float dW = dWp* sampleTimeW / 1000;
//                 float degree = dW*(180/3.1416);

//                 // Acumula el ángulo calculado
//                 rotationAngle += degree;

//                 Serial.println(rotationAngle);
//             }
//             datos[0] = 0;
//         }
//     }

//     // Detiene los motores y reinicia el ángulo
//     if ((angle > 0 && rotationAngle >= angle) || 
//         (angle < 0 && rotationAngle <= angle))
//     {
//         controlarTodosLosMotores(0, 0, 0, 0);
//         rotationAngle = 0;
//     }
// }

void Rotate(int angle, int Speed)
{
    float rotationAngle = 0; // Ángulo acumulado
    int datos[5];            // Matriz para almacenar los datos recibidos

    // Determina la dirección de la rotación
    int direction = (angle < 0) ? -1 : 1;
    Speed = abs(Speed); // Aseguramos que Speed sea positivo
    angle = abs(angle); // Trabajamos con el valor absoluto del ángulo

    // Bucle de control para realizar la rotación
    while ((direction > 0 && rotationAngle < angle) || 
           (direction < 0 && rotationAngle > -angle))
    {
        // Leer datos del puerto serie
        if (Serial1.available())
        {
            strT = Serial1.readStringUntil('\n');
            Serial.println(strT);
            for (int i = 0; i < dataLengthT; i++)
            {
                int index = strT.indexOf(separatorT);
                datos[i] = strT.substring(0, index).toInt();
                strT = strT.substring(index + 1);
            }
        }

        if (datos[0] == 20)
        {
          DefspeedA= 0;
          DefspeedB= 0;
          DefspeedC= 0;
          DefspeedD= 0;
           break;
        }

        if (datos[0] == 100)
        {
            unsigned long currentMillis = millis();

            if (currentMillis - previousMillis > sampleTimeW)
            {
                previousMillis = currentMillis;

                // Calcula la variación de ángulo (dW)
                float dWp = rwheel * (((datos[2]) / 100.0) / ((Lx + Ly)));
                float dW = dWp * sampleTimeW / 1000;
                float degree = dW * (180 / 3.1416);

                // Acumula el ángulo calculado
                rotationAngle += degree;

                // Calcula la velocidad relativa al progreso total
                float absRotation = abs(rotationAngle); // Usa ángulo absoluto para las verificaciones
                float speedFactor = 0;

                if (absRotation <= angle * 0.5) // Primera mitad de la rotación
                {
                    speedFactor = absRotation / (angle * 0.5); // Incremento lineal
                }
                else // Segunda mitad de la rotación
                {
                    speedFactor = (angle - absRotation) / (angle * 0.5); // Decremento lineal
                }

                // Asegura que la velocidad se encuentra entre minSpeed y Speed
                int adjustedSpeed = minSpeed + (Speed - minSpeed) * speedFactor;
                adjustedSpeed *= direction; // Aplica la dirección para el control del motor

                // Controla los motores para realizar la rotación
                controlarTodosLosMotores(adjustedSpeed, -adjustedSpeed, -adjustedSpeed, adjustedSpeed);

                // Mensajes de depuración
                Serial.print("Rotation Angle: ");
                Serial.print(rotationAngle);
                Serial.print(" Speed: ");
                Serial.println(adjustedSpeed);
            }
            datos[0] = 0;
        }
    }

    // Detiene los motores y reinicia el ángulo
    controlarTodosLosMotores(0, 0, 0, 0);
    rotationAngle = 0;

    String message = "150,1";

        // Enviar el mensaje por Serial1
        Serial1.println(message);
        Serial1.println(message);

        // También imprimir el mensaje en el monitor serial
        Serial.println("BANDERA FIN");    
}


void Endstop_Status() {
  int estadoEndstop = digitalRead(ENDSTOP); // Lee el valor del endstop

  if (estadoEndstop == HIGH) {
    Serial.println("ENDSTOP ACTIVADO");
  } else {
    Serial.println("ENDSTOP DESACTIVADO");
  }
  }

