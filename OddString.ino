/* 
* OddString
* ---------
* by Diego Di Carlo and Jorge Madrid Portillo
* created 29 Feb 2016
*/


/** CONSTANT **/
// sensors pins
const int PIN_SOFTPOT = A5;
const int PIN_PIEZO_INDEX = A1;
const int PIN_PIEZO_THUMB = A2;
const int PIN_PIEZO_MODE = A0;

const int PIN_ACCEL_Z = A6;

const int PIN_LED_RED = 6;
const int PIN_LED_BLUE = 7;
const int PIN_LED_GREEN = 8;

// sensors proprieties
int PIEZO_THRESHOLD_ON = 0;
int PIEZO_SAMPLES = 400;

int SOFTPOT_THRESHOLD_ON = 0;

/** GLOBAL VARIABLES **/

// sensors proprieties
int nPiezos = 2
int piezoPins[nPiezos] = {PIN_PIEZO_THUMB, PIN_PIEZO_INDEX};
int piezoCalibration[nPiezos];
int piezoVals[nPiezos] = {0, 0};  // state of the pad 1 for touched, 0 for not
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


int softpotVal = 0;
int softpotValOld = 0;
int fretTouched;
int noteFretted;
boolean isSoftpotActived = false;
boolean isSoftpotPlucked = false;
int calibrationMin = 0;
int calibrationMax = 0;

// the played note (usefull for open-string note)
Note *activeNote;

// midi proprieties
byte channel = 0x94; // arbitrary MIDI channel -- change as desired



/** SET UP **/
void setup() {

  /* Pin set up */
  // led
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  // mode
  pinMOde(PIN_PIEZO_MODE, OUTPUT);
  // piezos
  pinMode(PIN_PIEZO_INDEX, INPUT);
  pinMode(PIN_PIEZO_THUMB, INPUT);
  // softpot
  pinMode(PIN_SOFTPOT, INPUT);

  // turn off pullup resistors -- should not be needed
  

  /* load setup data */
  //read fret definition from EEPROM
  //TO DO

  /* pin function declaration */
  //begin at MIDI spec baud rate
  Serial.begin(31250);
  pinMode(PIN_SOFTPOT, INPUT);
  
  pinMode(PIN_PIEZO_STRING, INPUT);

  /* calibration */
  //TO DO
}

/** MAIN LOOP **/
void loop() {

  /* reset */
  // is it necessary??

  /* read values of all sensors */
  readSensors();

  /* act as a string or a efx controller? */
  if (!doesSoftpotActAsString) {
      // read only the softpot value
  } else {
      // TO DO
      // using the softpot as a fretted string
  }
  
  /* send note on */
  sendNote();

  /* send note off and reset necessary things */
  cleanUp();
   
  /* check for control changes */
  // TO DO 
   
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
  piezoVal = map(piezoVal, 0, 500, 0, 127);         // adapt the analog value to the midi range
  piezoVal = constrain(piezoVal, piezoMinVelocity, 127); // adapt the value to the empirical range

  
  /* Softpot */
  softpotVal = analogRead(PIN_SOFTPOT);
  Serial.println("SoftPot Val:  " + String(softpotVal));

  //if the string is touched
  if (softpotVal > SOFTPOT_THRESHOLD_ON) {
    softpotActived = true;      
    softpotVal = map(softpotVal, calibrationMin, calibrationMax, 0, 255);
    softpotVal = constrain(softpotVal, 0, 255);
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

