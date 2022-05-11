// Librerias usadas para levantar el Web Server
#include <LittleFS.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>

//Led 
const int led = 13;

//Librerias usadas para trabajar con el MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//Clave y id de la red WiFi
//Nota: La red debe de estár libre de firewall 
//(no funciona la de iteso)
const char* ssid = "IZZI-0AEA";
const char* password = "ffXfhYcD";

//Creamos handlers para el server
AsyncWebServer server(80);
AsyncEventSource events("/events");
JSONVar readings;

//Creamos handlers para los 2 sensores
MPU6050 accelgyro1(0x68);
MPU6050 accelgyro2(0x69); // <-- 2nd MPU6050 con AD0 en high (3.3V)

//Inicialización de variables de lectura SENSOR 1
int16_t ax1, ay1, az1;   //Acelerometro
int16_t gx1, gy1, gz1;   //Giroscopio
int16_t temperature;
float accel_ang_x1, accel_ang_y1;
float girosc_ang_x1, girosc_ang_y1;
float girosc_ang_x_prev1, girosc_ang_y_prev1;
long tiempo_prev1, dt1;
float ang_x1, ang_y1;
float ang_x_prev1, ang_y_prev1;

//Inicialización de variables de lectura SENSOR 2
int16_t ax2, ay2, az2;   //Acelerometro
int16_t gx2, gy2, gz2;   //Giroscopio
float accel_ang_x2, accel_ang_y2;
float girosc_ang_x2, girosc_ang_y2;
float girosc_ang_x_2prev, girosc_ang_y_prev2;
long tiempo_prev2, dt2;
float ang_x2, ang_y2;
float ang_x_prev2, ang_y_prev2;

// Factores de conversion
const float accScale = 2.0 * 9.81 / 32768.0;
const float gyroScale = 250.0 / 32768.0;

// Timer variables
unsigned long lastTime = 0; 
unsigned long gyroDelay = 500;


#define BUFFER_LEN 256
long lastMsg = 0;
char msg[BUFFER_LEN];
int value = 0;
byte mac[6];
char mac_Id[18];
const char* AWS_endpoint = "XXXXXXXXXXX-ats.iot.us-east-1.amazonaws.com"; //MQTT broker ip

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
WiFiClientSecure espClient;

/////////////////// FUNCIONES ///////////////////

String getGyroscopeReadings(){   
  readings["rotationX_1"] = String(ang_x1);
  readings["rotationY_1"] = String(ang_y1);
  readings["rotationX_2"] = String(ang_x2);
  readings["rotationY_2"] = String(ang_y2);
  String jsonString = JSON.stringify(readings);
  return jsonString;
}

void getInclination(){
  //SENSOR 1
  accel_ang_x1 = atan(ax1 / sqrt(pow(ay1, 2) + pow(az1, 2)))*(180.0 / 3.14);
  accel_ang_y1 = atan(ay1 / sqrt(pow(ax1, 2) + pow(az1, 2)))*(180.0 / 3.14); 
  //SENSOR 2
  accel_ang_x2 = atan(ax2 / sqrt(pow(ay2, 2) + pow(az2, 2)))*(180.0 / 3.14);
  accel_ang_y2 = atan(ay2 / sqrt(pow(ax2, 2) + pow(az2, 2)))*(180.0 / 3.14); 
}

void updateGiro(){
  dt1 = (millis() - tiempo_prev1) / 1000.0;
  tiempo_prev1 = millis();
  dt2 = (millis() - tiempo_prev2) / 1000.0;
  tiempo_prev2 = millis();

  //Calcular los ángulos con acelerometro
  getInclination();
 
  //Calcular angulo de rotación con giroscopio y filtro complementario
  //SENSOR 1
  if (isnan(ang_x_prev1)) { ang_x_prev1 = 0; }
  if (isnan(ang_y_prev1)) { ang_y_prev1 = 0; }
  ang_x1 = 0.98*(ang_x_prev1 + (gx1 / 131)*dt1) + 0.02*accel_ang_x1;
  ang_y1 = 0.98*(ang_y_prev1 + (gy1 / 131)*dt1) + 0.02*accel_ang_y1;
  //SENSOR 2
  if (isnan(ang_x_prev2)) { ang_x_prev2 = 0; }
  if (isnan(ang_y_prev2)) { ang_y_prev2 = 0; }
  ang_x2 = 0.98*(ang_x_prev2 + (gx2 / 131)*dt2) + 0.02*accel_ang_x2;
  ang_y2 = 0.98*(ang_y_prev2 + (gy2 / 131)*dt2) + 0.02*accel_ang_y2;
  
  //SENSOR 1
  ang_x_prev1 = ang_x1;
  ang_y_prev1 = ang_y1;
  //SENSOR 2
  ang_x_prev2 = ang_x2;
  ang_y_prev2 = ang_y2;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}
PubSubClient client(AWS_endpoint, 8883, callback, espClient); //set MQTT port number to 8883 as per //standard

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  espClient.setBufferSizes(512, 512);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  timeClient.begin();
  while(!timeClient.update()){
    timeClient.forceUpdate();
  }
  espClient.setX509Time(timeClient.getEpochTime());
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESPthing")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      char buf[256];
      espClient.getLastSSLError(buf,256);
      Serial.print("WiFiClientSecure SSL error: ");
      Serial.println(buf);
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/////////////////// SETUP ///////////////////

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  setup_wifi();
  delay(1000);
  if (!LittleFS.begin()) {
    Serial.println("Failed to mount file system");
    return;
  }
  Serial.println("Mounted LittleFS");
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  
  // Load certificate file
  File cert = LittleFS.open("/cert.der", "r"); //replace cert.crt eith your uploaded file name
  if (!cert) {
    Serial.println("Failed to open cert file");
  }
  else  Serial.println("Success to open cert file");
  delay(1000);
  if (espClient.loadCertificate(cert))
    Serial.println("cert loaded");
  else Serial.println("cert not loaded");
  
  // Load private key file
  File private_key = LittleFS.open("/private.der", "r"); //replace private eith your uploaded file name
  if (!private_key) {
    Serial.println("Failed to open private cert file");
  }  else  Serial.println("Success to open private cert file");
  delay(1000);
  if (espClient.loadPrivateKey(private_key))
    Serial.println("private key loaded");
  else   Serial.println("private key not loaded");
  
  // Load CA file
  File ca = LittleFS.open("/ca.der", "r"); //replace ca eith your uploaded file name
  if (!ca) {
    Serial.println("Failed to open ca ");
  }  else  Serial.println("Success to open ca");
  delay(1000);
  if(espClient.loadCACert(ca))
    Serial.println("ca loaded");
  else  Serial.println("ca failed");
  Serial.print("Heap: "); Serial.println(ESP.getFreeHeap());
  
  WiFi.macAddress(mac);
  snprintf(mac_Id, sizeof(mac_Id), "%02x:%02x:%02x:%02x:%02x:%02x",
  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print(mac_Id);

  delay(10000);
  Serial.end();
  Wire.begin(1,3);
  
  //Serial.println("Initializing I2C device 1...");
  accelgyro1.initialize();
  //Serial.println("Initializing I2C device 2...");
  accelgyro2.initialize();

  // verify connection
  while (!accelgyro1.testConnection()) {
    //Serial.println("MPU6050 0x69 is not properly connected. Check circuit!");
    digitalWrite(led, LOW);
    delay(1000);
    digitalWrite(led, HIGH);
    delay(1000);
  }
  /*
  digitalWrite(led, HIGH);
  while (!accelgyro2.testConnection()) {
    //Serial.println("MPU6050 0x68 is not properly connected. Check circuit!");
    digitalWrite(led, LOW);
    delay(1000);
    digitalWrite(led, HIGH);
    delay(1000);
  }
  digitalWrite(led, HIGH);*/

  //Add calibration values 
  //  -60 9 58 -843 111 1521
  //To get this values run IMU_ZERO for 5 min (0x69)
  accelgyro1.setXGyroOffset(-51);
  accelgyro1.setYGyroOffset(16);
  accelgyro1.setZGyroOffset(56);
  accelgyro1.setXAccelOffset(-767);
  accelgyro1.setYAccelOffset(112);
  accelgyro1.setZAccelOffset(1525);    
  //  -60 9 58 -843 111 1521
  //To get this values run IMU_ZERO for 5 min (0x68)
  accelgyro2.setXGyroOffset(-19);
  accelgyro2.setYGyroOffset(9);
  accelgyro2.setZGyroOffset(18);
  accelgyro2.setXAccelOffset(-4036);
  accelgyro2.setYAccelOffset(-295);
  accelgyro2.setZAccelOffset(773);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.serveStatic("/",LittleFS, "/");

  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      //Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
      digitalWrite(led, LOW);
    }
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
  
  digitalWrite(led, LOW);
  
}

/////////////////// MAIN ///////////////////

void loop() {
  if (!client.connected()) { reconnect(); }
  client.loop();
  long now = millis();
  
  // read measurements from device
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  updateGiro();

  // Send data
  if ((now - lastTime) > gyroDelay) {
    events.send(getGyroscopeReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  
  if ((now - lastMsg) > 2000) {
    lastMsg = now;
    String macIdStr = mac_Id;
    snprintf (msg, BUFFER_LEN, "{\"mac_Id\" : \"%s\", \"rotationX_1\" : \"%f\", \"rotationY_1\" : \"%f\", \"rotationX_2\" : \"%f\", \"rotationY_2\" : \"%f\"}", 
                macIdStr.c_str(), temperature, ang_x1, ang_y1, ang_x2, ang_y2);
    client.publish("outTopic", msg);
  }
  digitalWrite(LED_BUILTIN, HIGH); // turn the LED on (HIGH is the voltage level)
  delay(100); // wait for a second
  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  delay(100); // wait for a second
}
