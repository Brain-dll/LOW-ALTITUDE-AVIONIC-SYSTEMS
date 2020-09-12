#define buzzer 38
#define Fire 37
#define delaytime 250
#define tolerance_heigh -2

/*
  connections (LORA E-32 868T20D)
  
  Module      Teensy
  M0          4
  M1          3
  Rx          2 (MCU Tx line)
  Tx          1 (MCU Rx line)
  Aux         0
  Vcc         3V3
  Gnd         Gnd
*/

// LORA E-32 868T20D *******************************
#include "EBYTE.h"
#include <SoftwareSerial.h>

SoftwareSerial ESerial(0,1); // RX, TX

#define PIN_M0 2
#define PIN_M1 3
#define PIN_AX 4

struct DATA {
  float LAT;
  float LONG;
  bool CONN;
  bool Pay_load;
  float Altitude;
  float Z_eksen;
};

//int Chan;
DATA myDATA;

EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX);

/*
  connections (BME280 MODULE) (SPI KABLOLAMA)
  
  SCK  23 (A9) PIN  
  SDO  22 (A8) PIN
  SDI  21 (A7) PIN
  CS   20 (A6) PIN
*/
#include "Seeed_BME280.h"
#include <Wire.h>

BME280 bme280;

float Alt, Alt1,Alt2,Alt3,Alt4;

unsigned long delayTime;

/*
  connections from sensor to board (ADXL345 ACCELEROMETER)
  
  SCL = SCL0 pin A5(19)
  SCA = SDA0 pin A4(18)
*/

#include "Adafruit_ADXL345_U.h"
#include "Adafruit_Sensor.h"

Adafruit_ADXL345_Unified erisim = Adafruit_ADXL345_Unified();

/*
  connections from GPS to board (GY-GPSV3-NEO)

  RX =  TX2  (PIN 10)
  TX =  RX2  (PIN 9)
*/

#include <TinyGPS++.h>

TinyGPSPlus gps; 

static void smartdelay(unsigned long ms);

SoftwareSerial GPS(9, 10); // RX, TX

void setup() {

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  pinMode(Fire, OUTPUT);

  GPS.begin(9600); //for GPS

  ESerial.begin(9600); //for LORA

  erisim.begin(); //for ADXL345

  

  if (!erisim.begin())
  {
    delay(500);
    if (!erisim.begin())
    {
      while (true)
        {
          digitalWrite(buzzer, HIGH);
          delay(1000);
        }
    }
  }
  if (!bme280.init()) 
  {
    delay(500);
    if (!bme280.init())
    {
      while (true)
      {
        digitalWrite(buzzer, HIGH);
        delay(1000);
        digitalWrite(buzzer, LOW);
        delay(1000);
      }
    }
  }

  Transceiver.init();
  
  delay(100);
  Alt = bme280.calcAltitude(bme280.getPressure());
  delay(100);
  myDATA.Pay_load = false;
  myDATA.CONN = false;

  for (byte i = 0; i < 3 ; i++)
  {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    delay(1000);
  }

}

void loop() 
{
  Alt1 = Zero_Alt(bme280.calcAltitude(bme280.getPressure()));
  delay(delaytime);
  if (myDATA.Pay_load == false)
  {
    Alt2 = Zero_Alt(bme280.calcAltitude(bme280.getPressure()));
    delay(delaytime);
  
    if (((Alt2 - Alt1) < tolerance_heigh)/* && (Accelemeter() < 0)*/)
    {
      Alt3 = Zero_Alt(bme280.calcAltitude(bme280.getPressure()));
      delay(delaytime);
      if (((Alt3 - Alt2) < tolerance_heigh)/* && (Accelemeter() < 0)*/)
        {
          Alt4 = Zero_Alt(bme280.calcAltitude(bme280.getPressure()));
          delay(delaytime);
          if (((Alt4 - Alt3) < tolerance_heigh)/* && (Accelemeter() < 0)*/)
            {
              digitalWrite(Fire, HIGH);
              delay(3000);
              digitalWrite(Fire, LOW);
              myDATA.Pay_load = true;
            }
        }
    }
  }
  
  else
  {
    digitalWrite(buzzer, HIGH);
    delay(1000);
    digitalWrite(buzzer, LOW);
    delay(500);
  }

  MYDATA_preprocessing(Alt1);
    
  Transceiver.SendStruct(&myDATA, sizeof(myDATA));
  
  delay(500);

}

void MYDATA_preprocessing(float Alt1)
{  
  while (GPS.available() > 0)
    gps.encode(GPS.read());

  myDATA.LAT = gps.location.lat();
  myDATA.LONG = gps.location.lng();
  myDATA.Altitude = Alt1;
  myDATA.Z_eksen = Accelemeter();
  if (gps.satellites.value() > 0)
    myDATA.CONN = true;
  else
    myDATA.CONN = false;
  
  smartdelay(1000);
}

static void smartdelay(unsigned long ms)
{
  unsigned long start = millis(); 
  do
  {
  while (GPS.available())gps.encode(GPS.read());
  }
  while (millis() - start < ms); 
}

float Zero_Alt(float Alt_new)
{
  return Alt_new - Alt;
}  

float Accelemeter()
{
  sensors_event_t sensor;
  erisim.getEvent(&sensor);

  return sensor.acceleration.z;
}
