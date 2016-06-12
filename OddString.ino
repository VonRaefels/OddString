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

struct piezoData {
  boolean on;
  int energy;
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
struct piezoData piezoDataIndex, piezoDataThumb;

// DSP VARIABLE

const int FS = 10000;
const uint16_t nFFT = 128;
const uint16_t nWindow = nFFT/2;
double frame[2*nWindow];
double vFFT[nFFT];
double xEnergy;
double xAvarage;
double xAvaragePrev;
double xAvaragePrevPrev;
double xDetected;
const double FIXED_THR_DELTA = 300;
const double ADAPT_THR_LAMBDA = 1;
double detectedPrev;
double detectedPrevPrev;
double xThresh;
double xLocMax;
double xLocMaxPrev;
double xLocMaxPrevPrev;



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

int numFrets = 13;
int fretDefs[13];
int F0 = 220;

/* DEBUG */
boolean debugFrets = true;
boolean debugSoftPot = true;
boolean debugPiezo = false;
boolean debugPiezo2 = false;
boolean isCalibrating = false;
boolean debugPickNotes = false;
boolean debugTime = false;

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
  }

  for (int x = 0; x < 2; x++) {
    calibrationZ = analogRead(PIN_ACCEL_Z);
  }
}

/** MAIN LOOP **/
void loop() {
  if (isCalibrating) {
    return;
  }

  /* RESET */
  stringPlucked = false;
  piezoVal = false;
  //rawPiezoVal = 0;

  readSensors();

  if(piezoDataThumb.on) {
    doesSoftpotActAsString = !doesSoftpotActAsString;
  }
  
  if (doesSoftpotActAsString) {
    determineFrets();
    if(fretTouched == numFrets) {
      fretTouched = 0;
    }
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

  /* Piezo */
  piezoDataIndex = detectPiezoOnset(PIN_PIEZO_INDEX);
  piezoDataThumb = detectPiezoOnset(PIN_PIEZO_THUMB);

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
  int n = 20;
  int measures[n];
  int _softpotVal = 0;
  for (int i = 0; i < n; i++) {
    _softpotVal = analogRead(PIN_SOFTPOT) - 10; //Remove some noise....
    measures[i] = _softpotVal;
  }

  qsort(measures, n, sizeof(int), cmpfunc);

  int offset = 5;
  int limit = n - 5;
  for (int j = 0; j < limit; j++) {
    total += measures[j + offset];
  }
  int mean = total / (n - offset);
  
  return mean;
}

struct piezoData detectPiezoOnset(int PIEZO) {
  struct piezoData piezoData_instance;
    // Collect the raw data and perform zeropadding
  for (int i = 0; i < nWindow*2; i++) {
    if(i > nWindow) {
      frame[i] = 0;
    }else {
      frame[i] = analogRead(PIEZO);
    }
  }
  
  fft(frame, nWindow);
    
  // ENERGY
  double xEnergy = 0;
  for (int i = 0; i < 2*nWindow; i += 2) {
    xEnergy += (pow(frame[i], 2) + pow(frame[i+1], 2));
  }

  // MOVING AVARAGE
  xAvarage = (xEnergy + xAvaragePrev + xAvaragePrevPrev)/3;
  xAvaragePrevPrev = xAvaragePrev;
  xAvaragePrev = xAvarage;

  // ADAPTIVE THRESHOLDING
  xDetected = FIXED_THR_DELTA + ADAPT_THR_LAMBDA*(xAvarage + detectedPrev + detectedPrevPrev)/3;
  detectedPrevPrev = detectedPrev;
  detectedPrev = xDetected;

  xThresh = xAvarage - xDetected;

  if (xThresh < 0){
    xThresh = 0;
  }
  if(debugPiezo) {
    Serial.println(String(piezoVal) + ", " + String(xEnergy) + ", " + String(xAvarage) + ", " + String(xDetected));
  }
  
  xLocMax = xThresh;

  if ((xLocMax-xLocMaxPrev)<0 && (xLocMaxPrev-xLocMaxPrevPrev)>0) {
    piezoVal = xLocMaxPrev;
  }
  //TO DO set output...
  piezoData_instance.on = false;
  piezoData_instance.energy = 0;
  return piezoData_instance;
}

int cmpfunc (const void * a, const void * b)
{
  return ( *(int*)a - * (int*)b );
}

void sendNote() {
  if (softpotActive) {
    noteOn(softpotVal, 100);
    softpotActive = false;
  }
  else return;
}

const int CALIBRATE_PIEZO_THRESHOLD = 700;
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
  Serial.println("Calibration completed!");
}

void determineFrets() {
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

void pickNotes() {
  if (piezoDataIndex.on) {
    noteFretted = fretTouched + 52;
    if (stringActive) {
      if (debugPickNotes) {
        Serial.println("Deactivating String: " + String(activeNote->number()));
      }
      noteOff(activeNote->number(), 0);
      free(activeNote);
    } else {
      stringActive = true;
    }
    //register with active notes
    activeNote = (Note *) malloc(sizeof(Note));
    activeNote->init(noteFretted, piezoVal, millis(), fretTouched > 0);
    if (debugPickNotes) {
      Serial.println("Activating String: " + String(activeNote->number()));
    }

    noteOn(activeNote->number(), activeNote->velocity());

    stringPlucked = true;
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
      noteOff(activeNote->number(), 0);

      //mark as inactive
      stringActive = false;
      free(activeNote);
    }
  }
}

/* MIDI functions */
void noteOn(int pitch, int velocity) {
  //  usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
  //Serial.println(">>>ON!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

void noteOff(int pitch, int velocity) {
  //  usbMIDI.sendNoteOn(pitch, velocity, MIDI_CHANNEL);
  //Serial.println(">>>OFF!  pitch: " + String(pitch) + ", " + " velocity: " + String(velocity));
}

void pitchBend(int value) {
  //  usbMIDI.sendPitchBend(value, MIDI_CHANNEL);
}


// FFT ROUTINE

#define SWAP(a,b) tempr = (a); (a) = (b); (b) = tempr

// Input: nn is the number of points in the data and in the FFT, 
//           nn must be a power of 2
// Input: data is sampled voltage v(0),0,v(1),0,v(2),...v(nn-1),0 versus time
// Output: data is complex FFT Re[V(0)],Im[V(0)], Re[V(1)],Im[V(1)],...
// data is an array of 2*nn elements
void fft(double data[], unsigned long nn){
unsigned long n,mmax,m,j,istep,i;
double wtemp,wr,wpr,wpi,wi,theta;
double tempr,tempi;
  n = nn<<1;  // n is the size of data array (2*nn)
  j = 1;
  for(i=1; i<n; i+=2){
    if(j > i){        // bit reversal section
      SWAP(data[j-1],data[i-1]);
      SWAP(data[j],data[i]);
    }
    m = n>>1;
    while((m >= 2)&&(j > m)){
      j = j-m;
      m = m>>1;
    }
    j = j+m;
  }
  mmax = 2;             // Danielson-Lanczos section
  while( n > mmax){     // executed log2(nn) times
    istep = mmax<<1;
    theta = -6.283185307179586476925286766559/mmax;
    // the above line should be + for inverse FFT
    wtemp = sin(0.5*theta);
    wpr = -2.0*wtemp*wtemp;  // real part
    wpi = sin(theta);        // imaginary part
    wr = 1.0;
    wi = 0.0;
    for(m=1; m<mmax; m+=2){
      for(i=m; i<=n; i=i+istep){
        j = i+mmax;
        tempr     = wr*data[j-1]-wi*data[j]; // Danielson-Lanczos formula
        tempi     = wr*data[j]+wi*data[j-1];
        data[j-1] = data[i-1]-tempr;
        data[j]   = data[i]-tempi;
        data[i-1] = data[i-1]+tempr;
        data[i]   = data[i]+tempi;
      }
      wtemp = wr;
      wr = wr*wpr-wi*wpi+wr;
      wi = wi*wpr+wtemp*wpi+wi;
    }
    mmax = istep;
  }
}

//-----------------------------------------------------------
// Calculates the FFT magnitude at a given frequency index. 
// Input: data is complex FFT Re[V(0)],Im[V(0)], Re[V(1)],Im[V(1)],...
// Input: nn is the number of points in the data and in the FFT, 
//           nn must be a power of 2
// Input: k is frequency index 0 to nn/2-1
//        E.g., if nn=16384, then k can be 0 to 8191
// Output: Magnitude in volts at this frequency (volts)
// data is an array of 2*nn elements
// returns 0 if k >= nn/2
double fftMagnitude(double data[], unsigned long nn, unsigned long k){
  double nr, realPart, imagPart;

  nr = (double) nn;
  if (k >= nn/2){
    return 0.0; // out of range
  }
  if (k == 0){
    return sqrt(data[0] * data[0] + data[1] * data[1]) / nr;
  }
  realPart = fabs(data[2*k])   + fabs(data[2*nn-2*k]);
  imagPart = fabs(data[2*k+1]) + fabs(data[2*nn-2*k+1]);
  return  sqrt(realPart * realPart + imagPart * imagPart) / nr;
}

//-----------------------------------------------------------
// Calculates the FFT magnitude in db full scale at a given frequency index. 
// Input: data is complex FFT Re[V(0)],Im[V(0)], Re[V(1)],Im[V(1)],...
// Input: nn is the number of points in the data and in the FFT, 
//           nn must be a power of 2
// Input: k is frequency index 0 to nn/2-1
//        E.g., if nn=16384, then k can be 0 to 8191
// Input: fullScale is the largest possible component in volts
// Output: Magnitude in db full scale at this frequency
// data is an array of 2*nn elements
// returns -200 if k >= nn/2
double fftMagdB(double data[], unsigned long nn, unsigned long k, double fullScale){
  double magnitude = fftMagnitude(data, nn, k);
  if (magnitude<0.0000000001){ // less than 0.1 nV
    return -200; // out of range
  }
  return 20.0*log10(magnitude/fullScale);
}

//-----------------------------------------------------------
// Calculates the FFT phase at a given frequency index. 
// Input: data is complex FFT Re[V(0)],Im[V(0)], Re[V(1)],Im[V(1)],...
// Input: nn is the number of points in the data and in the FFT, 
//           nn must be a power of 2
// Input: k is frequency index 0 to nn/2-1
//        E.g., if nn=16384, then k can be 0 to 8191
// Output: Phase at this frequency
// data is an array of 2*nn elements
// returns 0 if k >= nn/2
double fftPhase(double data[], unsigned long nn, unsigned long k){
  if (k >= nn/2){
    return 0.0;     // out of range
  }
  if (data[2*k+1]==0.0){
    return 0.0;     // imaginary part is zero
  }
  if (data[2*k]==0.0){ // real part is zero
    if (data[2*k+1]>0.0){ // real part is zero
      return 1.5707963267948966192313216916398;    // 90 degrees
    }
    else{
      return -1.5707963267948966192313216916398;   // -90 degrees
    }
  }
  return atan2 (data[2*k+1], data[2*k]);  // imaginary part over real part
}

//-----------------------------------------------------------
// Calculates equivalent frequency in Hz at a given frequency index. 
// Input: fs is sampling rate in Hz
// Input: nn is the number of points in the data and in the FFT, 
//           nn must be a power of 2
// Input: k is frequency index 0 to nn-1
//        E.g., if nn=16384, then k can be 0 to 16383
// Output: Equivalent frequency in Hz
// returns 0 if k >= nn
double fftFrequency (unsigned long nn, unsigned long k, double fs){
  if (k >= nn){
    return 0.0;     // out of range
  }

  if (k <= nn/2){
    return fs * (double)k / (double)nn;
  }
  return -fs * (double)(nn-k)/ (double)nn;
}

