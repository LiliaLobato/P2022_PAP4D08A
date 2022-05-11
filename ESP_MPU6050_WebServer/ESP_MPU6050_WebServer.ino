// Librerias usadas para levantar el Web Server
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Arduino_JSON.h>
#include <LittleFS.h>

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
MPU6050 accelgyro1(0x69);
MPU6050 accelgyro2(0x68); // <-- 2nd MPU6050 con AD0 en high (3.3V)

//Inicialización de variables de lectura SENSOR 1
int16_t ax1, ay1, az1;   //Acelerometro
int16_t gx1, gy1, gz1;   //Giroscopio
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

/////////////////// SETUP ///////////////////

void setup() {
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());

  // Initialize LittleFS
  if(!LittleFS.begin()){
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  Serial.println("Mounted LittleFS");

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
  digitalWrite(led, HIGH);
  while (!accelgyro2.testConnection()) {
    //Serial.println("MPU6050 0x68 is not properly connected. Check circuit!");
    digitalWrite(led, LOW);
    delay(1000);
    digitalWrite(led, HIGH);
    delay(1000);
  }
  digitalWrite(led, HIGH);

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

  //Serial.println("MPU6050 Found");

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

  // read raw accel/gyro measurements from device
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  updateGiro();

  // Send data
  if ((millis() - lastTime) > gyroDelay) {
    events.send(getGyroscopeReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  
}
