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
const int piezoPinA1 = A1;
const int piezoPinA2 = A2;
const int piezoPinA3 = A3;

const int softPotPin = A0;

// these variables will change:
int sensorValueA0 = 0;
int sensorValueA1 = 0;
int sensorValueA2 = 0;
int sensorValueA3 = 0;

int piezoThr = 500;
int softPotThr = 813;

void setup() {

    //enable pullup resistor
    digitalWrite(softPotPin, HIGH); // to avoid open-circuit interference
    
    //begin SLIPSerial just like Serial
    SLIPSerial.begin(9600);   // set this as high as you can reliably run on your platform
    #if ARDUINO >= 100
      while(!Serial)
      ; //Leonardo "feature"
    #endif
}


void loop() {
  
    // read the sensor and store it in the variable sensorReading:
    sensorValueA0 = analogRead(softPotPin);
    if (sensorValueA0 <= softPotThr){
      Serial.println(sensorValueA0);
    }
    
    sensorValueA1 = analogRead(piezoPinA1); 
    if (sensorValueA1 >= piezoThr){
      Serial.write("/piezo/01 \n");
    }

    sensorValueA2 = analogRead(piezoPinA2);
    if (sensorValueA2 >= piezoThr){
      SLIPSerial.println("Piezo 2 " + String(sensorValueA2));
    }

    sensorValueA3 = analogRead(piezoPinA3);
    if (sensorValueA3 >= piezoThr){
      //SLIPSerial.println("Piezo 3 " + String(sensorValueA3));
    }
    
    delay(50);
}
