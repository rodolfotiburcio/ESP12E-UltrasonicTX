/**
 * Programa para medir la distancia y transmitirla a internet
 * utilizando mesajes MQTT, ademas de visualizarlo en un led
 * utilizando pwm.
 */
#include "Arduino.h"    //Libreria de arduino
#include <ESP8266WiFi.h>    //Libreria para la conexion wifi
#include <PubSubClient.h>   //Libreria para publicar mensajes MQTT
#include <Ultrasonic.h>   //Libreria para el utilizar el modulo ultrasonico
#include <math.h>

const char* ssid = "Inamex-fp";    //Nombre de la red wifi
const char* password = "Inxfluidpower";    //password de la red
const char* mqtt_server = "broker.mqtt-dashboard.com";    //Dirección del broker para los mensajes MQTT
const int PIN_LED = 5;   //Es el pin GPIO5 usado para PWM
//int EntAna=50;    //declaración e inicialización de la variable para el PWM
int EntTim=0;     //
int RecValue=0;

WiFiClient espClient;
PubSubClient client(espClient);
Ultrasonic ultrasonic(4,14);     //Pin 4 es trig; el 14 es echo
long lastMsg = 0;
char msg[50];
int value = 0;

void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);

void setup(){
  pinMode(PIN_LED, OUTPUT);   // initialize LED digital pin as an output.
//  pinMode(4, OUTPUT);
//  pinMode(14, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);  // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
//  analogWriteRange(320);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  RecValue=0;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
//  for (int i = 0; i < length; i++) {
//    RecValue=RecValue+(payload[i]-48)*(pow(10,length-(i+1)));
//  }

  Serial.println();

  // Switch on the LED if an 1 was received as first character
  if ((char)payload[0] == '1') {
    digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is acive low on the ESP-01)
  } else {
    digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TibuESP8266Client1")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("TibuInTopic", "hello world");
      // ... and resubscribe
      client.subscribe("TibuInTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
//  EntAna=analogRead(A0);
  EntTim=(ultrasonic.Timing()/3)-150;
  delay(3);
//  analogWrite(PIN_LED, RecValue);

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 2000) {
    lastMsg = now;
    ++value;
    snprintf (msg, 75, "%ld", EntTim);
    Serial.print("Publish Value: ");
    Serial.println(msg);
    client.publish("TibuOutTopic", msg);
  }
}
