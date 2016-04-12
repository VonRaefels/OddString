/* 
* OddString
* ---------
* by Diego Di Carlo and Jorge Madrid Portillo
* created 29 Feb 2016
*/

/** CONSTANT **/
// sensors
const int channel = 1;

const int PIN_SOFTPOT = A6;
const int PIN_PIEZO_STRING = A0;

int PIEZO_THRESHOLD_ON = 0;
int PIEZO_SAMPLES = 400;
int SOFTPOT_THRESHOLD_ON = 0;

// midi
const int MIDI_CHANNEL = 0;

/** GLOBAL VARIABLES **/
int piezoVal = 0;
int piezoMinVelocity = 10;

int softpotVal = 0;
boolean softpotActived = false;
int calibrationMin = 0;
int calibrationMax = 1023;
int stringPlucked = false;

/** SET UP **/
void setup() {

    /* load setup data */
    //read fret definition from EEPROM
    //TO DO

    /* pin function declaration */
    //begin at MIDI spec baud rate
    Serial.begin(31250);
    pinMode(PIN_SOFTPOT, INPUT);
    pinMode(PIN_PIEZO_STRING, INPUT);
//    digitalWrite(PIN_SOFTPOT, HIGH);

    /* calibration */
    //TO DO
}

/** MAIN LOOP **/
void loop() {

    /* reset */
    piezoVal = false;
    stringPlucked = false;

    /* read values of all sensors */
    readSensors();
    
    /* continuos or discrete string */
    // TO DO
    // using the softpot as a fretted strin

    /* send note on */
//    sendNote();

    /* send note off and reset necessary things */
//    cleanUp();
    
    /* check for control changes */
    //TO DO

    //wait??
    delay(500);
}


/** FUNCTIONS **/
void readSensors() {

  /* Piezo */
  int piezoVal = analogRead(PIN_PIEZO_STRING);

  //if the value breaks the threshold read for max amplitude
  if (piezoVal > PIEZO_THRESHOLD_ON) {
    int highestPiezoVal = piezoVal;
    for (int sample=0; sample < PIEZO_SAMPLES; sample++) {
      piezoVal = analogRead(PIN_PIEZO_STRING);
      if (piezoVal > highestPiezoVal) {
        highestPiezoVal = piezoVal;
      }
    }
    piezoVal = highestPiezoVal;
  }
  Serial.println("Piezo Val:  " + String(piezoVal));
//  piezoVal = map(piezoVal, 0, 500, 0, 127);         // adapt the analog value to the midi range
//  piezoVal = constrain(piezoVal, piezoMinVelocity, 127); // adapt the value to the empirical range
  
  /* Softpot */
  softpotVal = analogRead(PIN_SOFTPOT);

  //if the string is touched
  if (softpotVal > SOFTPOT_THRESHOLD_ON) {
    softpotActived = true;      
//    softpotVal = map(softpotVal, calibrationMin, calibrationMax, 0, 255);
//    softpotVal = constrain(softpotVal, 0, 255);
      Serial.println("SoftPot Val:  " + String(softpotVal));
  } else {
    softpotActived = false;
    return;
  }
}

void sendNote() {
  // TO DO: if the piezo was hit, play the note
  if (softpotActived) {
    noteOn(softpotVal, 100);
    softpotActived = false;
  }
  else return;
}

void  cleanUp() {
  //turn off the active note
  if (!softpotActived){ 
    noteOff(softpotVal, 0);
  } else return;
}

/* MIDI functions */
void noteOn(int pitch, int velocity) {
    usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
    Serial.println(">>>ON!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

void noteOff(int pitch, int velocity) {
    usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
    Serial.println(">>>OFF!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

