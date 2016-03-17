#include <OSCMessage.h>

/*
Make an OSC message and send it over serial
 */

#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial( thisBoardsSerialUSB );
#else
#include <SLIPEncodedSerial.h>
 SLIPEncodedSerial SLIPSerial(Serial);
#endif


/* Knock Sensor

created 29 Feb 2016
by Diego Di Carlo and Jorge Madrid Portillo
modified 29 Feb 2016
by Diego Di Carlo
*/

// these constants won't change:
const int knockSensorA1 = A1;
const int knockSensorA2 = A2;
const int knockSensorA3 = A3;

const int softPotSensor = A0;

// these variables will change:
int sensorReadingA1 = 0;
int sensorReadingA2 = 0;
int sensorReadingA3 = 0;
int sensorReadingA0 = 0;
int thr = 500;

void setup() {
   //begin SLIPSerial just like Serial
  SLIPSerial.begin(9600);   // set this as high as you can reliably run on your platform
  #if ARDUINO >= 100
    while(!Serial)
      ; //Leonardo "feature"
  #endif
}


void loop() {
    // read the sensor and store it in the variable sensorReading:
    sensorReadingA1 = analogRead(knockSensorA1); 
    if (sensorReadingA1 >= thr){
      Serial.write("/piezo/01 ");

    }

    sensorReadingA2 = analogRead(knockSensorA2);
    if (sensorReadingA2 >= thr){
      //SLIPSerial.println("SensorA2 " + String(sensorReadingA2));
    }

    sensorReadingA3 = analogRead(knockSensorA3);
    if (sensorReadingA3 >= thr){
      //SLIPSerial.println("SensorA3 " + String(sensorReadingA3));
    }

    sensorReadingA0 = analogRead(softPotSensor);
    Serial.write(sensorReadingA0);
    
    delay(100);
}
