#include "I2Cdev.h"
#include "MPU6050.h"
#include <SoftwareSerial.h>

//Terminal Serial
SoftwareSerial mySerial(2, 3); // RX, TX

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Inicialización de la variable del giroscopio
MPU6050 accelgyro1(0x69);
MPU6050 accelgyro2(0x68); // <-- use for second MPU6050 device where AD0 must be high (3.3V)

//Lest try to make a quick calibration
bool first_time=true; 

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

void setup() {//pin de vida
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    //mySerial.begin(19200);

    // initialize device
    Serial.println("Initializing I2C device 1...");
    accelgyro1.initialize();
    Serial.println("Initializing I2C device 2...");
    accelgyro2.initialize();

    // verify connection
    accelgyro1.testConnection() ? digitalWrite(LED_BUILTIN, LOW) : digitalWrite(LED_BUILTIN, HIGH);
    accelgyro2.testConnection() ? digitalWrite(LED_BUILTIN, LOW) : digitalWrite(LED_BUILTIN, HIGH);
    
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
    digitalWrite(LED_BUILTIN, LOW);
    Serial.flush();
}

void printMotion6_IS(){
  // display tab-separated accel/gyro x/y/z values
  Serial.print("S1:\t");
  Serial.print(ax1*accScale); Serial.print(" (m/s2)\t");
  Serial.print(ay1*accScale); Serial.print(" (m/s2)\t");
  Serial.print(az1*accScale); Serial.print(" (m/s2)\t");
  Serial.print(gx1*gyroScale); Serial.print(" (deg/s)\t");
  Serial.print(gy1*gyroScale); Serial.print(" (deg/s)\t");
  Serial.print(gz1*gyroScale); Serial.print(" (deg/s)\n");
  Serial.print("S2:\t");
  Serial.print(ax2*accScale); Serial.print(" (m/s2)\t");
  Serial.print(ay2*accScale); Serial.print(" (m/s2)\t");
  Serial.print(az2*accScale); Serial.print(" (m/s2)\t");
  Serial.print(gx2*gyroScale); Serial.print(" (deg/s)\t");
  Serial.print(gy2*gyroScale); Serial.print(" (deg/s)\t");
  Serial.print(gz2*gyroScale); Serial.print(" (deg/s)\n");
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

void printIncl(){
  Serial.print(F("Inclinacion en Sensor 1 X: "));
  Serial.print(accel_ang_x1);
  Serial.print(F("\t Y:"));
  Serial.println(accel_ang_y1);
  Serial.print(F("Inclinacion en Sensor 2 X: "));
  Serial.print(accel_ang_x2);
  Serial.print(F("\t Y:"));
  Serial.println(accel_ang_y2);
  Serial.print(F("\n"));
}

void printRot(){
  Serial.print(F("X1:  "));
  Serial.print(ang_x1);
  Serial.print(F("\t Y1: "));
  Serial.println(ang_y1);
  Serial.print(F("\t X2:  "));
  Serial.print(ang_x2);
  Serial.print(F("\t Y2: "));
  Serial.println(ang_y2);
  Serial.print(F("\n"));
}


void graficaRaw_X(){
  Serial.print(F("X1:  "));
  Serial.print(ang_x1);
  Serial.print(F("\t Y1: "));
  Serial.println(accel_ang_x1);
  Serial.print(F("\n"));
}

void graficaRaw_Y(){
  Serial.print(F("X1:  "));
  Serial.print(ang_y1);
  Serial.print(F("\t Y1: "));
  Serial.println(accel_ang_y1);
  Serial.print(F("\n"));
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  accelgyro2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  updateGiro();
    
  //Imprimir valores
  //printMotion6_IS();
  //printIncl();
  //printRot();
  graficaRaw_X();
  //graficaRaw_Y();
  
}
