/*
  OddString
  ---------
  by Diego Di Carlo and Jorge Madrid Portillo
  created 29 Feb 2016
*/

#include <stdio.h>
#include <stdlib.h>
#include <EEPROM.h>
#include <math.h>
#include "PiezoData.h"
#define PADDING 3

//** NOTE CLASS **//
/*
  A note class that stores some info about each note played is necessary
  to ensure that open strings are held for the specified amount of time.
  That is a problem with using the piezos as triggers instead of FSRs, they
  only register momentary impact or vibration, creating a problem for open strings.
*/

class Note {
    int _number;
    int _velocity;
    int _startTime;
    int _fretted;

  public:
    void init(int number, int velocity, int startTime, int fretted) {
      _number = number;
      _velocity = velocity;
      _startTime = startTime;
      _fretted = fretted;
    }

    int number() {
      return _number;
    }

    int velocity() {
      return _velocity;
    }

    int fretted() {
      return _fretted;
    }

    int timeActive() {
      return millis() - _startTime;
    }
};

/** CONSTANT **/
// sensors pins
const int PIN_SOFTPOT = 5;
const int PIN_PIEZO_INDEX = 1;
const int PIN_PIEZO_THUMB = 2;
const int PIN_PIEZO_MODE = 0;

const byte PIN_ACCEL_Z = 6;

const int PIN_LED_BLUE = 6;
const int PIN_LED_GREEN = 7;
const int PIN_LED_RED = 8;

// sensors proprieties
int PIEZO_THRESHOLD_ON = 250;
int PIEZO_SAMPLES = 400;
const int CALIBRATE_PIEZO_THRESHOLD = 450;

int MIDI_CHANNEL = 0x94;
int SOFTPOT_THRESHOLD_ON = 7;

/** GLOBAL VARIABLES **/

// sensors proprieties
const int nPiezos = 2;
int piezoPins[nPiezos] = {PIN_PIEZO_THUMB, PIN_PIEZO_INDEX};
int piezoCalibration[nPiezos];
int piezoVal;       // x[n] state of the pad 1 for touched, 0 for not
int oldPiezoVal;    // x[n-1] used for onset detection
int oldOldPiezoVal; // x[n-2] used for onset detection
int rawPiezoVal = 0;
int piezoVelocity[nPiezos];      // state of the pad as it is processed:
// 0 = read for new touch
// 1 = have touch, waiting for volume
// 2 = have volume, waiting to be played (note on)
// 3 = played, waiting to be turned off (note off)
// 4 = disable pad
long int padLastTime[nPiezos]; // last time pad was triggered
byte padLastChannel[nPiezos]; // last channel held to turn right note off after key changes
byte padLastNote[nPiezos]; // last note held to turn right note off after key change
byte padVolume[nPiezos]; // current note volume
byte padIndex = 0;  // index for pads throught each loop
boolean softpotActive = true;
boolean doesSoftpotActAsString = true;
int piezoMinVelocity = 0;

int indexEnergy = 0;
int thumbEnergy = 0;
PiezoData indexPiezo(PIN_PIEZO_INDEX);

/* FRETS AND CALIBRATION VARIABLES */
int softpotVal = 0;
int softpotValOld = 0;
int fretTouched;
int lastFretTouched = 0;
int noteFretted;
boolean isSoftpotActived = false;
boolean isSoftpotPlucked = false;
boolean legato = false;
int calibrationMin = 0;
int calibrationMax = 1023;
int calibrationZ = 0;
int refPiezoMode = 0;
int capacitivePiezoValue = 0;

int numFrets = 13;
int fretDefs[13];
int F0 = 220;

/* DEBUG */
boolean debugFrets = false;
boolean debugSoftPot = false;
boolean isCalibrating = false;
boolean debugPickNotes = false;
boolean debugTime = false;

// the played note (usefull for open-string note)
Note *activeNote;
boolean stringActive = false;
boolean stringPlucked = false;

// midi proprieties
byte channel = 0x94; // arbitrary MIDI channel -- change as desired

/** MAIN LOOP **/
int meanPiezoMode;
const int maxPiezoCounter = 60;
int piezoCounter = maxPiezoCounter;

int currentLED = PIN_LED_RED;


/** SET UP **/
void setup() {
  //begin at MIDI spec baud rate
  Serial.begin(31250);
  while (!Serial) ;

  /* Pin set up */
  // led
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  // mode
  pinMode(PIN_PIEZO_MODE, INPUT);
  digitalWrite(PIN_PIEZO_MODE, HIGH);
  // piezos
  pinMode(PIN_PIEZO_INDEX, INPUT);
  digitalWrite(PIN_PIEZO_INDEX, HIGH);
  pinMode(PIN_PIEZO_THUMB, INPUT);
  digitalWrite(PIN_PIEZO_THUMB, HIGH);

  // softpot
  pinMode(PIN_SOFTPOT, INPUT);
  

  /* DSP */


  /* load setup data */
  //read fret definition from EEPROM
  fretDefs[0] = F0;
  for (int j = 1; j < numFrets; j++) {
    fretDefs[j] = EEPROM.read(j);
    Serial.println(String(fretDefs[j]));
  }
  fretDefs[numFrets] = 0;
  calibrationMin = EEPROM.read(numFrets);

  /* calibration */
  if (isCalibrating) {
    calibrate();
  } else {
    digitalWrite(currentLED, HIGH);
    
    int aux = 0;
    for(int i = 0; i < 20; i++) {
      aux += analogRead(PIN_PIEZO_MODE);
    }
  
    refPiezoMode = aux / 20;
  }

}

void loop() {
  if (isCalibrating) {
    return;
  }

  /* RESET */
  stringPlucked = false;
  piezoVal = false;
  //rawPiezoVal = 0;

  readSensors();
  if(piezoCounter == 0) {
    int mean = meanPiezoMode / maxPiezoCounter;
    Serial.println(mean);
    if(mean > 250) {
      doesSoftpotActAsString = !doesSoftpotActAsString;
      digitalWrite(currentLED, LOW);
      if(currentLED == PIN_LED_BLUE) {
        currentLED = PIN_LED_RED;
      }else {
        currentLED = PIN_LED_BLUE;
      }
      digitalWrite(currentLED, HIGH);
    }
    piezoCounter = maxPiezoCounter;
    meanPiezoMode = 0;
  }else {
    meanPiezoMode += capacitivePiezoValue;
    piezoCounter--;
  }

  if (doesSoftpotActAsString) {
    determineFrets();
    if (fretTouched == numFrets) {
      fretTouched = 0;
    }
    if (debugFrets) {
      Serial.println("Fret touched: " + String(fretTouched));
    }

    pickNotes();

    cleanUp();
  } else {
    int val = map(softpotVal, 0, 255, 0, 16383);
    pitchBend(val);

  }

  /* check for control changes */
  // TO DO
}


/** FUNCTIONS **/
void readSensors() {
  /* Piezo 0 */
  capacitivePiezoValue = analogRead(PIN_PIEZO_MODE);
  //Serial.println(capacitivePiezoValue);

  /* Piezo */
  indexEnergy = indexPiezo.detectOnset();

  thumbEnergy = analogRead(PIN_PIEZO_THUMB);

  if (thumbEnergy > CALIBRATE_PIEZO_THRESHOLD) {
    Serial.println("thumb");
  }

  /* Softpot */
  int val = readSoftpotVal();

  //if the string is touched
  if (val > 0) {
    softpotActive = true;
    softpotVal = map(val, calibrationMin, calibrationMax, 0, 255);
    softpotVal = constrain(softpotVal, 0, 255);
    if (debugSoftPot) {
      Serial.println("SoftPot Val:  " + String(softpotVal));
    }
  } else {
    softpotActive = false;
    softpotVal = 0;
  }
}

int readSoftpotVal() {
  int total = 0;
  int n = 10;
  int measures[n];
  int _softpotVal = 0;
  for (int i = 0; i < n; i++) {
    _softpotVal = analogRead(PIN_SOFTPOT) - 10; //Remove some noise....
    measures[i] = _softpotVal;
  }

  qsort(measures, n, sizeof(int), cmpfunc);

  int offset = 2;
  int limit = n - 2;
  for (int j = 0; j < limit; j++) {
    total += measures[j + offset];
  }
  int mean = total / (n - offset);

  return mean;
}

int cmpfunc (const void * a, const void * b)
{
  return ( *(int*)a - * (int*)b );
}


void calibrate() {
  Serial.println("Activating leds..");
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(100);
  digitalWrite(PIN_LED_GREEN, LOW);
  delay(100);
  digitalWrite(PIN_LED_GREEN, HIGH);
  delay(100);
  digitalWrite(PIN_LED_GREEN, LOW);
  delay(100);
  digitalWrite(PIN_LED_GREEN, HIGH);

  int sensorMax = 1000;
  int sensorMin = 0;
  int val;

  //loop through the array of fret definitions
  Serial.println("Entering calibration...");
  for (int j = numFrets; j > 0; j--) {

    int response = false;

    //wait for response

    while (!response) {
      //read piezo val
      int piezoVal = analogRead(PIN_PIEZO_MODE);

      //get the sensor min value (highest fret) on the first round
      if (j == numFrets) {
        //        int fretVal = analogRead(PIN_SOFTPOT);
        //        if (fretVal > sensorMax) (sensorMax = fretVal);

        //if the piezo is hit, register this as the definition for this fret
        if (piezoVal > CALIBRATE_PIEZO_THRESHOLD) {
          int fretVal = readSoftpotVal();
          sensorMin = fretVal;
          val = fretVal;
          response = true;
          Serial.println("Calibrated: " + String(j) + " Val: " + String(val));
        }
      }

      else {
        if (piezoVal > CALIBRATE_PIEZO_THRESHOLD) {
          int fretVal = readSoftpotVal();
          fretVal = map(fretVal, sensorMin, sensorMax, 0, 255);
          fretVal = constrain(fretVal, 0, 255);
          val = fretVal;
          response = true;
          Serial.println("Calibrated: " + String(j) + " Val: " + String(val));
        }
      }
    }

    //write to memory
    digitalWrite(PIN_LED_GREEN, LOW);
    EEPROM.write(j, val);

    delay(100);
    digitalWrite(PIN_LED_GREEN, HIGH);
  }

  //update global definitions
  calibrationMin = EEPROM.read(numFrets);

  for (int j = 1; j < numFrets; j++) {
    fretDefs[j] = EEPROM.read(j);
  }

  digitalWrite(PIN_LED_GREEN, LOW);
  Serial.println("Calibration completed!");
}

void determineFrets() {
  lastFretTouched = fretTouched;
  //check for open strings
  //  if (softpotVal <) {
  //    softpotValOld = softpotVal;
  //    fretTouched = 0;
  //  }

  //loop through the array of fret definitions
  for (int j = 1; j < numFrets; j++) {

    int k = j - 1;
    if (softpotVal <= fretDefs[k] &&
        softpotVal > fretDefs[j] &&
        abs(softpotVal - softpotValOld) > PADDING) {

      softpotValOld = softpotVal;
      fretTouched = j;
    }
  }

  if (softpotVal <= fretDefs[numFrets - 1]) {
    softpotValOld = softpotVal;
    fretTouched = numFrets;
  }
}

int velocity;

void pickNotes() {
  if (indexEnergy > 0) {
    noteFretted = fretTouched + 52;
    if (stringActive) {
      if (debugPickNotes) {
        Serial.println("Deactivating fret: " + String(activeNote->number()));
      }
      noteOff(activeNote->number(), 0);
      free(activeNote);
    } else {
      stringActive = true;
    }
    //register with active notes
    activeNote = (Note *) malloc(sizeof(Note));
    velocity = map(indexEnergy, 20, 650, 65, 127);
    velocity = constrain(velocity, 65, 127);

    activeNote->init(noteFretted, velocity, millis(), fretTouched > 0);
    if (debugPickNotes) {
      Serial.println("Activating fret: " + String(activeNote->number()));
    }

    noteOn(activeNote->number(), activeNote->velocity());

    stringPlucked = true;
  } else {
    int previousNote = activeNote->number();
    if ((lastFretTouched != fretTouched) && stringActive && legato) {
      free(activeNote);
      if (fretTouched > 0) {
        noteFretted = fretTouched + 52;
        activeNote = (Note *) malloc(sizeof(Note));

        activeNote->init(noteFretted, velocity, millis(), fretTouched > 0);
        noteOn(activeNote->number(), activeNote->velocity());
        

        if (debugPickNotes) {
          Serial.println("Legato fret: " + String(activeNote->number()));
        }
      }else {
        stringActive = false;
      }
      noteOff(previousNote, 0);
    }
  }
}


void cleanUp() {
  if (!fretTouched && stringActive) {

    //if open string
    if (!activeNote->fretted()) {
      //      if (activeNotes->timeActive() > minDurationOpen) {
      //        //turn off the active note
      //        noteOff(0x80, activeNotes->number(), 0);
      //
      //        //mark as inactive
      //        stringActive = false;
      //        free(activeNotes);
      //      }
    }
    else {
      //turn off the active note
      Serial.println("turning off");
      noteOff(activeNote->number(), 0);
      if (debugPickNotes) {
        Serial.println("Deactivating fret: " + String(activeNote->number()));
      }
      //mark as inactive
      stringActive = false;
      free(activeNote);
    }
  }
}

/* MIDI functions */
void noteOn(int pitch, int velocity) {
  usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
  //Serial.println(">>>ON!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

void noteOff(int pitch, int velocity) {
  usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
  //Serial.println(">>>OFF!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

void pitchBend(int value) {
  usbMIDI.sendPitchBend(value, MIDI_CHANNEL);
}
