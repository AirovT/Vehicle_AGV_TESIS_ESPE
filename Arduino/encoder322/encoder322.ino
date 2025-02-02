#include <BluetoothSerial.h>

#define RXD2 16
#define TXD2 17

BluetoothSerial SerialBT;

volatile long encoder_value_A;
volatile long encoder_value_B;
volatile long encoder_value_C;
volatile long encoder_value_D;

int antR;
int actR;

int antL;
int actL;

int antR1;
int actR1;

int antL1;
int actL1;

//VARIABLES MOTOR A 
// Variable for RPM measuerment
float rpm_A = 0;

//VARIABLES MOTOR B 
// Variable for RPM measuerment
float rpm_B = 0;

////MOTOR A
int DT_pin_A = 39;
int CLK_pin_A = 34;
 
////MOTOR B
int DT_pin_B = 35;
int CLK_pin_B = 32;


////MOTOR C
int DT_pin_C = 33;
int CLK_pin_C = 25;

////MOTOR D
int DT_pin_D = 26;
int CLK_pin_D = 27;


unsigned long lastTimeVelocidad = 0;  // Último tiempo para cálculo de velocidades
unsigned long lastTimeImpresion = 0;  // Último tiempo para impresión de velocidades
unsigned long lastTimeImpresion2 = 0;  // Último tiempo para impresión de velocidades

const unsigned long sampleTimeVelocidad = 100; // Tiempo de muestreo para velocidades (ms)
const unsigned long sampleTimeImpresion = 1000; // Tiempo de muestreo para impresión (ms)
const unsigned long sampleTimeEnvio = 250  ; // Tiempo de muestreo para impresión (ms)


const double R = 1440;             // Resolución del encoder
const double P = 4.19;                // Constante para convertir a rad/s


float velocidad[4] = {0, 0, 0, 0};    // Velocidades en rad/s

volatile int n[4] = {0, 0, 0, 0};     // Contadores para los pulsos de los encoders

void setup(){

   // LECTURA DEL ENCODER
////LECTURA ENCODER MOTOR A
  pinMode(DT_pin_A , INPUT);
  pinMode(CLK_pin_A , INPUT);
  attachInterrupt(DT_pin_A, interrupt_on_pulse_A, CHANGE);// RISING DE 0 A 1
  attachInterrupt(CLK_pin_A, interrupt_on_pulse_A, CHANGE);// RISING DE 0 A 1

////LECTURA ENCODER MOTOR B
  pinMode(DT_pin_B , INPUT);
  pinMode(CLK_pin_B , INPUT);
  attachInterrupt(DT_pin_B, interrupt_on_pulse_B, CHANGE);// RISING DE 0 A 1
  attachInterrupt(CLK_pin_B, interrupt_on_pulse_B, CHANGE);// RISING DE 0 A 1

  ////LECTURA ENCODER MOTOR C
  pinMode(DT_pin_C , INPUT);
  pinMode(CLK_pin_C , INPUT);
  attachInterrupt(DT_pin_C, interrupt_on_pulse_C, CHANGE);// RISING DE 0 A 1
  attachInterrupt(CLK_pin_C, interrupt_on_pulse_C, CHANGE);// RISING DE 0 A 1

  ////LECTURA ENCODER MOTOR D
  pinMode(DT_pin_D , INPUT);
  pinMode(CLK_pin_D , INPUT);
  attachInterrupt(DT_pin_D, interrupt_on_pulse_D, CHANGE);// RISING DE 0 A 1
  attachInterrupt(CLK_pin_D, interrupt_on_pulse_D, CHANGE);// RISING DE 0 A 1

  //PARA COMUNICACION SERIAL

   Serial.begin(115200); // Debug
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // Comunicación con Mega
  SerialBT.begin("ROBOTBT"); // Nombre del dispositivo Bluetooth
}

void loop(){


  static char btBuffer[256];  // Buffer para datos Bluetooth
  static int btIndex = 0;     // Índice del buffer Bluetooth
  static char serial2Buffer[256];  // Buffer para datos de Serial2
  static int serial2Index = 0;     // Índice del buffer Serial2

  // Procesar datos desde Bluetooth al Mega
  while (SerialBT.available()) {
    char c = SerialBT.read();
    if (c == '\n') { // Mensaje completo recibido
      btBuffer[btIndex] = '\0'; // Finaliza el mensaje
      Serial2.write(btBuffer, btIndex); // Envía todo el buffer al Mega
      Serial2.write('\n'); // Agrega salto de línea al Mega
      Serial.println(btBuffer); // Depuración (opcional)
      btIndex = 0; // Reinicia el índice
    } else if (btIndex < sizeof(btBuffer) - 1) {
      btBuffer[btIndex++] = c; // Almacena el carácter en el buffer
    }
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { // Mensaje completo recibido
      btBuffer[btIndex] = '\0'; // Finaliza el mensaje
      Serial2.write(btBuffer, btIndex); // Envía todo el buffer al Mega
      Serial2.write('\n'); // Agrega salto de línea al Mega
      // Serial.println(btBuffer); // Depuración (opcional)
      btIndex = 0; // Reinicia el índice
    } else if (btIndex < sizeof(btBuffer) - 1) {
      btBuffer[btIndex++] = c; // Almacena el carácter en el buffer
    }
  }

    // Procesar datos desde Mega (Serial2) y enviarlos por Bluetooth
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') { // Mensaje completo recibido desde Mega
      btBuffer[btIndex] = '\0'; // Finaliza el mensaje
      SerialBT.write((const uint8_t*)btBuffer, btIndex); // Envía todo el buffer por Bluetooth
      SerialBT.write('\n'); // Agrega salto de línea por Bluetooth
      Serial.println(btBuffer); // Imprime mensaje en el serial
      btIndex = 0; // Reinicia el índice
    } else if (btIndex < sizeof(btBuffer) - 1) {
      btBuffer[btIndex++] = c; // Almacena el carácter en el buffer
    }
  }

  

  imprimirVelocidades(sampleTimeImpresion);
  EnviarVelocidad(sampleTimeEnvio);
  
  unsigned long currentTime = millis();
  if (currentTime - lastTimeVelocidad >= sampleTimeVelocidad) {
    for (int i = 0; i < 4; i++) {
      velocidad[i] = 100*2*3.1416*(n[i]/R) / (sampleTimeVelocidad / 1000.0); // Calcular velocidad en rev/s
      // velocidad[i] = n[i]; // Calcular velocidad en rad/s

      n[i] = 0;  // Resetear contador de pulsos
      switch (i){
        case 0:
            encoder_value_A=0;
            break;
        case 1:
            encoder_value_B=0;
            break;
        case 2:
            encoder_value_C=0;
            break;
        case 3:
            encoder_value_D=0;
            break;
        

      }
    }
    lastTimeVelocidad = currentTime; // Actualizar último tiempo
  }
}

void interrupt_on_pulse_A() {
   
  // Read the value for the encoder for the right wheel

  antR=actR;
               
  if(digitalRead(CLK_pin_A)) bitSet(actR,0); else bitClear(actR,0);
  if(digitalRead(DT_pin_A)) bitSet(actR,1); else bitClear(actR,1);
  
  
  if(antR == 2 && actR ==0) encoder_value_A--;
  if(antR == 0 && actR ==1) encoder_value_A--;
  if(antR == 3 && actR ==2) encoder_value_A--;
  if(antR == 1 && actR ==3) encoder_value_A--;
  
  if(antR == 1 && actR ==0) encoder_value_A++;
  if(antR == 3 && actR ==1) encoder_value_A++;
  if(antR == 0 && actR ==2) encoder_value_A++;
  if(antR == 2 && actR ==3) encoder_value_A++;
  n[0]=encoder_value_A;
  // Serial.print("Contador A = ");
  // Serial.println(encoder_value_A);     
  
}

void interrupt_on_pulse_B() {
   
  // Read the value for the encoder for the right wheel
  
  antL=actL;
               
  if(digitalRead(CLK_pin_B)) bitSet(actL,0); else bitClear(actL,0);
  if(digitalRead(DT_pin_B)) bitSet(actL,1); else bitClear(actL,1);
  
  if(antL == 2 && actL ==0) encoder_value_B++;
  if(antL == 0 && actL ==1) encoder_value_B++;
  if(antL == 3 && actL ==2) encoder_value_B++;
  if(antL == 1 && actL ==3) encoder_value_B++;
  
  if(antL == 1 && actL ==0) encoder_value_B--;
  if(antL == 3 && actL ==1) encoder_value_B--;
  if(antL == 0 && actL ==2) encoder_value_B--;
  if(antL == 2 && actL ==3) encoder_value_B--;
  n[1]=encoder_value_B;

}

void interrupt_on_pulse_C() {
   
  // Read the value for the encoder for the right wheel
  
  antR1=actR1;
               
  if(digitalRead(CLK_pin_C)) bitSet(actR1,0); else bitClear(actR1,0);
  if(digitalRead(DT_pin_C)) bitSet(actR1,1); else bitClear(actR1,1);
  
  
  if(antR1 == 2 && actR1 ==0) encoder_value_C++;
  if(antR1 == 0 && actR1 ==1) encoder_value_C++;
  if(antR1 == 3 && actR1 ==2) encoder_value_C++;
  if(antR1 == 1 && actR1 ==3) encoder_value_C++;
  
  if(antR1 == 1 && actR1 ==0) encoder_value_C--;
  if(antR1 == 3 && actR1 ==1) encoder_value_C--;
  if(antR1 == 0 && actR1 ==2) encoder_value_C--;
  if(antR1 == 2 && actR1 ==3) encoder_value_C--;
  n[2]=encoder_value_C;       
}

void interrupt_on_pulse_D() {
   
  // Read the value for the encoder for the right wheel
  
  antL1=actL1;
               
  if(digitalRead(CLK_pin_D)) bitSet(actL1,0); else bitClear(actL1,0);
  if(digitalRead(DT_pin_D)) bitSet(actL1,1); else bitClear(actL1,1);
  
  
  if(antL1 == 2 && actL1 ==0) encoder_value_D++;
  if(antL1 == 0 && actL1 ==1) encoder_value_D++;
  if(antL1 == 3 && actL1 ==2) encoder_value_D++;
  if(antL1 == 1 && actL1 ==3) encoder_value_D++;
  
  if(antL1 == 1 && actL1 ==0) encoder_value_D--;
  if(antL1 == 3 && actL1 ==1) encoder_value_D--;
  if(antL1 == 0 && actL1 ==2) encoder_value_D--;
  if(antL1 == 2 && actL1 ==3) encoder_value_D--;
  n[3]=encoder_value_D;       
}

void imprimirVelocidades(unsigned long periodoMuestreo) {
  unsigned long currentTime = millis();
  if (currentTime - lastTimeImpresion >= periodoMuestreo) {
    Serial.print("100, ");
    for (int i = 0; i < 4; i++) {
      Serial.print(int(velocidad[i]));
      // Serial.print(int(n[i]));
      Serial.print(",");
      
    }
    Serial.println();
    lastTimeImpresion = currentTime; // Actualizar último tiempo
  }
}

void EnviarVelocidad(unsigned long periodoMuestreo) {
  unsigned long currentTime2 = millis();
  if (currentTime2 - lastTimeImpresion2 >= periodoMuestreo) {
    Serial2.print("100,");
    for (int i = 0; i < 4; i++) {
      Serial2.print(int(velocidad[i]));
      Serial2.print(",");
    }
    Serial2.println();
    lastTimeImpresion2 = currentTime2; // Actualizar último tiempo
  }
}