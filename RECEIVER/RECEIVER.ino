
/*

  This example shows how to connect to an EBYTE transceiver
  using a Teensy 3.2

  This code for for the receiver


  connections
  Module      Teensy
  M0          2
  M1          3
  Rx          1 (MCU Tx line)
  Tx          0 (MCU Rx line)
  Aux         4
  Vcc         3V3
  Gnd         Gnd

*/

#include "EBYTE.h"
#define ESerial Serial1

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

DATA myDATA;
unsigned long Last;

EBYTE Transceiver(&ESerial, PIN_M0, PIN_M1, PIN_AX);

void setup() {

  Serial.begin(9600);

  while (!Serial) {}

  ESerial.begin(9600);

  Serial.println("Starting Reader");
  
   Transceiver.init();
   Transceiver.PrintParameters();
}

void loop() {

  if (Transceiver.available()) 
  {
    Transceiver.GetStruct(&myDATA, sizeof(myDATA));
    Serial.print("LAT = ");  Serial.print(myDATA.LAT, 6);
    Serial.print("  LONG = "); Serial.print(myDATA.LONG, 6);
    Serial.print("  GPS =  ");  Serial.print(myDATA.CONN);
    Serial.print("  Alt =  ");  Serial.print(myDATA.Altitude);
    Serial.print("  Z =  ");  Serial.print(myDATA.Z_eksen);
    Serial.print("  PL =  ");  Serial.println(myDATA.Pay_load);

    Last = millis();
  }
  else 
  {
    if ((millis() - Last) > 1500) 
    {
      Serial.println("Searching: ");
      Last = millis();
    }

  }

}
