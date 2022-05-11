#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
    #include "Wire.h"


MPU6050 accelgyro1(0x69);
MPU6050 accelgyro2(0x68); // <-- use for second MPU6050 device where AD0 must be high (3.3V)

const char* ssid = "IZZI-0AEA";
const char* password = "ffXfhYcD";

ESP8266WebServer server(80);

const int led = 13;


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

String status = "NULL";


void handleRoot() {
  //digitalWrite(led, 1);
  server.send(200, "text/plain",  
   " X1:  " + String(ang_x1) +
   "\t Y1: " + String(ang_y1) +
   "\t X2:  " +  String(ang_x2) +
   "\t Y2: " + String(ang_y2) +
   "\n");
  //digitalWrite(led, 0);
}

void handleNotFound(){
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET)?"GET":"POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i=0; i<server.args(); i++){
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
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
  ang_x1 = 0.98*(ang_x_prev1 + (gx1 / 131)*dt1) + 0.02*accel_ang_x1;
  ang_y1 = 0.98*(ang_y_prev1 + (gy1 / 131)*dt1) + 0.02*accel_ang_y1;
  //SENSOR 2
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

void setup(void){
  pinMode(led, OUTPUT);
  digitalWrite(led, 0);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.on("/inline", [](){
    server.send(200, "text/plain", "this works as well");
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");

    delay(10000);
  Serial.end();
        Wire.begin(1,3);
  
    //Serial.println("Initializing I2C device 1...");
    accelgyro1.initialize();
    //Serial.println("Initializing I2C device 2...");
    accelgyro2.initialize();
    
    // verify connection
    accelgyro1.testConnection() ? digitalWrite(led, LOW) : digitalWrite(led, HIGH);
    accelgyro2.testConnection() ? digitalWrite(led, LOW) : digitalWrite(led, HIGH);

        
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
    
  server.on("/", handleRoot);
}

/////////////////// MAIN ///////////////////

void loop(void){
   server.handleClient();   
  // read raw accel/gyro measurements from device
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  updateGiro();
}
