/*
* OddString
* ---------
* by Diego Di Carlo and Jorge Madrid Portillo
* created 29 Feb 2016
*/

#include <stdio.h>
#include <stdlib.h>
#include <EEPROM.h>

#define PADDING 3
//** NOTE CLASS **//
/*
* A note class that stores some info about each note played is necessary
* to ensure that open strings are held for the specified amount of time.
* That is a problem with using the piezos as triggers instead of FSRs, they
* only register momentary impact or vibration, creating a problem for open strings.
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

int MIDI_CHANNEL = 0x94;
int SOFTPOT_THRESHOLD_ON = 7;

/** GLOBAL VARIABLES **/

// sensors proprieties
const int nPiezos = 2;
int piezoPins[nPiezos] = {PIN_PIEZO_THUMB, PIN_PIEZO_INDEX};
int piezoCalibration[nPiezos];
int piezoVal; // state of the pad 1 for touched, 0 for not
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

/* FRETS AND CALIBRATION VARIABLES */
int softpotVal = 0;
int softpotValOld = 0;
int fretTouched;
int noteFretted;
boolean isSoftpotActived = false;
boolean isSoftpotPlucked = false;
int calibrationMin = 0;
int calibrationMax = 1023;
int calibrationZ = 0;

int numFrets = 12;
int fretDefs[13];
int F0 = 220;

/* DEBUG */
boolean debugFrets = false;
boolean debugSoftPot = false;
boolean debugPiezo = false;
boolean debugPiezo2 = false;
boolean isCalibrating = false;
boolean debugPickNotes = false;

// the played note (usefull for open-string note)
Note *activeNote;
boolean stringActive = false;
boolean stringPlucked = false;

// midi proprieties
byte channel = 0x94; // arbitrary MIDI channel -- change as desired


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
  if(isCalibrating) {
    calibrate();
  }

  for (int x=0; x<2; x++) {
    calibrationZ = analogRead(PIN_ACCEL_Z);
  }
}

/** MAIN LOOP **/
void loop() {
  if(isCalibrating) {
    return;
  }

  /* RESET */
  stringPlucked = false;
  piezoVal = false;
  //rawPiezoVal = 0;

  readSensors();

  if (doesSoftpotActAsString) {
    determineFrets();
    if (debugFrets) {
      Serial.println("Fret touched: " + String(fretTouched));
    }

    pickNotes();

    cleanUp();
  } else {
    int val = map(softpotVal, 0, 255, 8192, 16383);
    pitchBend(val);

  }

  /* check for control changes */
  // TO DO
}


/** FUNCTIONS **/
void readSensors() {
  /* Accelerometer */
  int accelVal = analogRead(PIN_ACCEL_Z) - calibrationZ;
  //Serial.println("acceleration z:  " + String(accelVal));
  
  /* Piezo */
  int m = micros();
  piezoVal = analogRead(PIN_PIEZO_INDEX);

  //if the value breaks the threshold read for max amplitude
  if (piezoVal > PIEZO_THRESHOLD_ON) {
    int highestPiezoVal = piezoVal;
    for (int sample = 0; sample < PIEZO_SAMPLES; sample++) {
      piezoVal = analogRead(PIN_PIEZO_INDEX);
      if (piezoVal > highestPiezoVal) {
        highestPiezoVal = piezoVal;
      }
      if(debugPiezo2) {
        Serial.println(String(piezoVal) + ", " + rawPiezoVal);
      }
      
    }
    piezoVal = highestPiezoVal;
    rawPiezoVal = piezoVal;
    if(debugPiezo) {
      Serial.println(String(piezoVal));
    }
  }
  Serial.println(micros() - m);

  piezoVal = map(piezoVal, PIEZO_THRESHOLD_ON, 1023, 0, 127);         // adapt the analog value to the midi range
  piezoVal = constrain(piezoVal, piezoMinVelocity, 127); // adapt the value to the empirical range

  /* Softpot */
  int total = 0;
  int n = 20;
  int measures[n];
  for (int i = 0; i < n; i++) {
    softpotVal = analogRead(PIN_SOFTPOT) - 10; //Remove some noise....
    measures[i] = softpotVal;
  }

  qsort(measures, n, sizeof(int), cmpfunc);

  int offset = 5;
  int limit = n - 5;
  for (int j = 0; j < limit; j++) {
    total += measures[j + offset];
  }
  int mean = total / (n - offset);

  //if the string is touched
  if (mean > 0) {
    if (debugSoftPot) {
      Serial.println("SoftPot Val:  " + String(mean));
    }
    softpotActive = true;
    softpotVal = map(softpotVal, calibrationMin, calibrationMax, 0, 255);
    softpotVal = constrain(softpotVal, 0, 255);
  } else {
    softpotActive = false;
    softpotVal = 0;
    return;
  }
}

int cmpfunc (const void * a, const void * b)
{
  return ( *(int*)a - * (int*)b );
}

void sendNote() {
  // TO DO: if the piezo was hit, play the note
  if (softpotActive) {
    noteOn(softpotVal, 100);
    softpotActive = false;
  }
  else return;
}

const int CALIBRATE_PIEZO_THRESHOLD = 800;
void calibrate() {
  Serial.println("Activating leds..");
  digitalWrite(PIN_LED_BLUE, HIGH);
  delay(100);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(100);
  digitalWrite(PIN_LED_BLUE, HIGH);
  delay(100);
  digitalWrite(PIN_LED_BLUE, LOW);
  delay(100);
  digitalWrite(PIN_LED_BLUE, HIGH);

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
      int piezoVal = analogRead(piezoPins[0]);

      //get the sensor min value (highest fret) on the first round
      if (j == numFrets) {
        //        int fretVal = analogRead(PIN_SOFTPOT);
        //        if (fretVal > sensorMax) (sensorMax = fretVal);

        //if the piezo is hit, register this as the definition for this fret
        if (piezoVal > CALIBRATE_PIEZO_THRESHOLD) {
          int fretVal = analogRead(PIN_SOFTPOT);
          sensorMin = fretVal;
          val = fretVal;
          response = true;
          Serial.println("Calibrated: " + String(j) + " Val: " + String(val));
        }
      }

      else {
        if (piezoVal > CALIBRATE_PIEZO_THRESHOLD) {
          int fretVal = analogRead(PIN_SOFTPOT);
          fretVal = map(fretVal, sensorMin, sensorMax, 0, 255);
          fretVal = constrain(fretVal, 0, 255);
          val = fretVal;
          response = true;
          Serial.println("Calibrated: " + String(j) + " Val: " + String(val));
        }
      }
    }

    //write to memory
    digitalWrite(PIN_LED_BLUE, LOW);
    EEPROM.write(j, val);

    delay(100);
    digitalWrite(PIN_LED_BLUE, HIGH);
  }

  //update global definitions
  calibrationMin = EEPROM.read(numFrets);

  for (int j = 1; j < numFrets; j++) {
    fretDefs[j] = EEPROM.read(j);
  }

  digitalWrite(PIN_LED_BLUE, LOW);
}

void determineFrets() {
  //check for open strings
  if (softpotVal >= F0) {
    softpotValOld = softpotVal;
    fretTouched = 0;
  }

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

void pickNotes() {
  //if the piezo was hit, play the fretted note
  if (rawPiezoVal > 1000){
    noteFretted = fretTouched;
    if (stringActive){
      if(debugPickNotes) {
        Serial.println("Deactivating String: " + String(activeNote->number()));
      }
      noteOff(activeNote->number(), 0);
      free(activeNote);
    }else {
      stringActive = true;
    }
    //register with active notes
    activeNote = (Note *) malloc(sizeof(Note));
    activeNote->init(noteFretted, piezoVal, millis(), fretTouched > 0);
    if(debugPickNotes) {
      Serial.println("Activating String: " + String(activeNote->number()));
    }

    noteOn(activeNote->number(), activeNote->velocity());
  
    stringPlucked = true; 
  }
}


void cleanUp() {
  if (!fretTouched && stringActive){
    
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
    noteOff(activeNote->number(), 0);
    
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


