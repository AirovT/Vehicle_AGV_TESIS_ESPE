#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// Configuración de la red WiFi
const char* ssid = "CarBox";
const char* password = "Espe2024*";

// Configuración del broker MQTT
const char* mqtt_server = "192.168.1.9"; // Dirección IP del broker MQTT

WiFiClient espClient;
PubSubClient client(espClient);

// Pines de los sensores de presencia
const int sensorPins = 5; // Ejemplo de pines D1, D2, D3
const int numSensors = 1;
int lastSensorStates = -1; // Estados anteriores de los sensores, inicializados en valores imposibles

void setup() {
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  // Configuración de pines de sensores como entradas
  for (int i = 0; i < numSensors; ++i) {
    pinMode(sensorPins, INPUT);
  }
}

void setup_wifi() {
  delay(10);
  // Conexión a la red WiFi
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Manejar mensajes recibidos (opcional para este ejemplo)
}

void reconnect() {
  // Loop hasta que estemos conectados
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP8266Client")) {
      Serial.println("conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando de nuevo en 5 segundos");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer el estado de todos los sensores

    int sensorState = digitalRead(sensorPins);

    // Invertir el estado del sensor
    sensorState = !sensorState;

    // Publicar y imprimir el estado del sensor solo si ha cambiado
    if (sensorState != lastSensorStates) {
      lastSensorStates = sensorState;
      char msg[50];
      snprintf(msg, 50, "%d", sensorState);

      Serial.print("Estado del sensor ");
      Serial.print(1);
      Serial.print(": ");
      Serial.println(msg);

      // Generar el tema para cada sensor (ejemplo: /sensores/presencia1, /sensores/presencia2, ...)
      char topic[50];
      snprintf(topic, 50, "/Caja/presencia%d", 1);

      // Publicar con QoS 1
      client.publish(topic, msg, 1);
    }
  

  delay(200); // Pequeño retraso para evitar demasiadas lecturas rápidas
}
