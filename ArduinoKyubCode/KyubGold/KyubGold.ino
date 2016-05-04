//************************************
//************************************
//**           Gold Kyub            **
//**           6/1/2015             **
//** Low latency, chord selection   **
//**         with RGB LED           **
//************************************
//************************************


//************************************
//************************************
//**      Global Variables          **
//************************************
//************************************

#include <EEPROM.h>

boolean onlyonce=LOW;

//Debugging settings
int consolemidimode=1; //consolemode=0 midimode=1  capacitive mode = 2 accelerometer=3 

//Teensy pin assignments
//int profilepin=18;  //used for latency experiments
const byte modebutton=22;
boolean chord1=0;  //determined by pads 9 and 10
boolean chord2=0;
byte chordselect=0;  //chord selection
byte chordoverridenote=0;
byte chordoverridetype=0;
byte lastchordselect=0;
byte chordpallet=0;  //chord pallet selection
const byte Rled = 12; //tricolor LED pins
const byte Gled = 14;
const byte Bled = 15;
const byte driverpin = 13; //common pad drive pin
const byte zpin=2; //accelerometer axes inputs
const byte ypin=1;
const byte xpin=0;

byte padnote[9] = {
  60,60,60,60,60,60,60,60,60};  //arbitrary initial pad note values

int incomingByte = 0;   // for incoming serial data
unsigned long keepalive=0;
int bytecounter=0;
byte palletcounter=0;
byte rootnumber=0;
int octave=0;
//byte quantize=1;
unsigned long drumpulse=0;  //for quantization

byte majortemplatev1[9]={
  0,12,4,16,7,24,19,28,36};
byte minortemplatev1[9]={
  0,12,3,15,19,7,24,27,31};
byte majortemplatev2[9]={
  0,12,4,24,19,16,7,28,36};
byte minortemplatev2[9]={
  0,12,3,15,7,19,24,27,31};
byte maj6template[9]={
  0,4,7,12,24,9,21,19,16};
byte min6template[9]={
  0,3,7,9,24,15,21,12,19};
byte maj7template[9]={
  0,11,4,12,24,7,23,16,19};
byte min7template[9]={
  0,11,7,12,24,3,23,19,16};
byte dimtemplate[9]={
  0,12,3,15,6,24,18,15,30};
byte dim7template[9]={
  0,9,3,15,6,21,18,15,30};
byte sus2template[9]={
  0,12,2,14,7,24,19,26,30};
byte sus4template[9]={
  0,12,5,17,7,24,19,29,30};
byte augtemplate[9]={
  0,4,8,24,36,20,16,32,28};
byte d1template[9]={
  47,59,60,49,49,50,52,55,56};
byte d2template[9]={
  36,37,38,39,40,41,42,43,44};
byte d3template[9]={
  36,38,40,41,43,45,37,42,39};// most reason drums start with c1=36
byte powertemplate[9]={
  0,12,7,19,24,31,36,48,43};


//default values
//first chord pallet 1 3m 4 5
byte chordA0[9]={
  52,64,55,67,71,59,76,79,83};//Em
byte chordA1[9]={
  48,60,52,64,55,72,67,76,84};//C
byte chordA2[9]={
  53,65,57,69,72,60,77,81,84};//F
byte chordA3[9]={
  55,67,59,71,74,62,79,83,86};//G
byte chordAchannel =0x94;
byte chordAchannel_1 =0x94;
byte chordAchannel_2 =0x94;
byte chordATrack=1;
byte chordAPassChan=1;
byte chordAQuant=1;
byte chordAtype[4]={
  0,0,0,0};

//second chord pallet 1 5 6m 4 sfc 
byte chordB0[9]={
  48,60,52,64,55,72,67,76,84};//C
byte chordB1[9]={
  55,67,59,71,74,62,79,83,86};//G
byte chordB2[9]={
  57,69,60,72,76,64,81,84,87};//Am
byte chordB3[9]={
  53,65,57,69,72,60,77,81,84};//F
byte chordBchannel =0x94;
byte chordBchannel_1 =0x94;
byte chordBchannel_2 =0x94;
byte chordBTrack=1;
byte chordBPassChan=1;
byte chordBQuant=1;
byte chordBtype[4]={
  0,0,9,0};

//third chord pallet 1 3 6m 1
byte chordC0[9]={
  67,55,71,55,62,74,79,86,83};//G
byte chordC1[9]={
  55,67,59,71,74,62,79,83,86};//G
byte chordC2[9]={
  47,59,51,63,54,71,66,75,83};//B
byte chordC3[9]={
  52,64,55,76,71,59,76,67,83}; //Em
byte chordCchannel = 0x94;
byte chordCchannel_1 =0x94;
byte chordCchannel_2 =0x94;
byte chordCTrack=1;
byte chordCPassChan=1;
byte chordCQuant=1;
byte chordCtype[4]={
  0,0,0,0};

//misc varibles
byte channel_0=0x94;  //arbitrary MIDI channel--change as desired
byte channel_1=0x94;
byte channel_2=0x94;
int quantSelect=0;
byte trackSelect=1;
byte trackOctave=1;
byte passchannel=3;
byte trackRoot=1;
byte miditrigger=0;
byte pad[11]={
  0,1,2,3,4,5,6,7,8,9,10};
boolean padstate[11];  //state of pad as touched (HIGH) or not (LOW)
byte padmode[11];  //state of note as it is processed
//   0 = ready for new pad touch
//   1 = have touch, waiting for volume
//   2 = have volume, waiting to be played (note on)
//   3 = played, waiting to be turned off (note off)
//   4=disable pad
long int padlasttime[11];  //last time pad was triggered
byte padlastchannel[11];  //last channel held to turn right note off after key changes
byte padlastnote[11];  //last note held to turn right note off after key change
byte padvolume[11];  //current note volume
byte pnum=0; //index for pads through each loop

//capactive sensing variables
int firsttime=0;  //trigger for capacitive calibration
int cap_calibration[11];  //calibration value for each pad
long int chargetime[11];  //sensed charge time for each pad
int overflow=0;
unsigned long starttime=0;
unsigned long grabtime;
int hysteresishigh=25; //turn on threshold for touch
int hysteresislow=20; //turn off threshold for touch

//MIDI variables
int notevolume=0;
int volume=0;
int pitch=0;
long int min_note_duration=100000;
long int holdoff[11]; //??

//debug printout delay variable
long int next=0;

//accelerometer variables
int once=0; //controls sample acquisitions
int circularaccbufferx[21]; // !!!was 100holds samples of A/D taken before and after pad hit (about +/- 5 ms
int circularaccbuffery[21];
int circularaccbufferz[21];
int circbuffpointer=0;
int triggerpoint=0; //time of pad hit
long acc_calibrationx=0; //A/D calibration values (may not be needed)
long acc_calibrationy=0;
long acc_calibrationz=0;
boolean hithappened=LOW;
int xaxispeak=0; //peaks and valleys of acceleration waveforms
int xaxisvalley=0;
int yaxispeak=0;
int yaxisvalley=0;
int zaxispeak=0;
int zaxisvalley=0;

///RGB LED variables
byte Rledinput=0;
byte Gledinput=0;
byte Bledinput=0;

unsigned long RGBledtimer=0;

//************************************
//************************************
//**          Set-Up                **
//************************************
//************************************

void setup() 
{                

  Serial.begin(9600);

  //used in tracking mode
  usbMIDI.setHandleNoteOff(OnNoteOff);  //call backs for automatic MIDI data receipt
  usbMIDI.setHandleNoteOn(OnNoteOn); 

  //3-color LED pin assignments
  pinMode(Rled, OUTPUT); 
  pinMode(Bled, OUTPUT); 
  pinMode(Gled, OUTPUT); 

  // capacitive sensor common pad driver
  pinMode(driverpin, OUTPUT);

  pinMode(modebutton, INPUT);
  digitalWrite(modebutton, HIGH);

  pinMode(pad[0], INPUT);
  pinMode(pad[1], INPUT);
  pinMode(pad[2], INPUT);
  pinMode(pad[3], INPUT);
  pinMode(pad[4], INPUT);
  pinMode(pad[5], INPUT);
  pinMode(pad[6], INPUT);
  pinMode(pad[7], INPUT);
  pinMode(pad[8], INPUT);
  pinMode(pad[9], INPUT);
  pinMode(pad[10], INPUT);

  //turn off pullup resistors--should not be needed
  digitalWrite(pad[0], LOW);
  digitalWrite(pad[1], LOW); 
  digitalWrite(pad[2], LOW);
  digitalWrite(pad[3], LOW);
  digitalWrite(pad[4], LOW);
  digitalWrite(pad[5], LOW);
  digitalWrite(pad[6], LOW);
  digitalWrite(pad[7], LOW);
  digitalWrite(pad[8], LOW);
  digitalWrite(pad[9], LOW);
  digitalWrite(pad[10], LOW);    
}//end setup

//************************************
//************************************
//**      Main Loop                **
//************************************
//************************************

// the main loop routine runs over and over again forever:
void loop() 
{

  usbMIDI.read();  //read midi input from external sequencer, if any

  if (onlyonce==LOW) 
  {
    loadEEPROM(); //load chords from eeprom once
    loadchords();
  }

  //receive personality data from visual basic program
  if (millis()-keepalive>3000)  //reset byte counter for serial decoding
  {
    bytecounter=0;  //reset bytecounter if timeout
    palletcounter=0;
  }
  //Serial.println("displaying recieved data");
  if (Serial.available() > 0) 
  {
    keepalive=millis();
    // read the incoming byte:
    incomingByte = Serial.read();
    if (bytecounter<18) palletcounter=0;
    else if (bytecounter<36) palletcounter=1;
    else palletcounter=2;
    decodeChord(incomingByte);
    //Serial.write(incomingByte);//to send ASCAII

    // say what you got:
    Serial.print(bytecounter);
    Serial.print("  received-");
    Serial.print(incomingByte,HEX); //to send ASCAII
    Serial.println("");
    bytecounter++;
    //flash red led to show loading
    analogWrite(Rled, 0);
    delay(100);
    analogWrite(Rled, 255);
    if (bytecounter==54) 
    {
      Serial.println("reading and building chords");
      buildChords();  //unpack root data into actual chords
      loadchords();
    }
  }

  //set chord pallet according to presses of mode button
  if (digitalRead(modebutton)==LOW)
  {
    if (chordpallet==0) //
    {
      analogWrite(Gled, 0); 
      delay (1000);
      analogWrite(Gled, 255);
      chordpallet=1;
      trackSelect=chordATrack;
    }

    else if (chordpallet==1) 
    {
      analogWrite(Bled, 0); 
      delay (1000);
      analogWrite(Bled, 255);
      chordpallet=2;
      trackSelect=chordBTrack;
    }
    else if (chordpallet==2) 
    {
      analogWrite(Rled, 0); 
      delay (1000);
      analogWrite(Rled, 255);
      chordpallet=0;
      trackSelect=chordCTrack;
    } 
  }

  //pad calibration--early after boot
  if (firsttime<20) firsttime++; 
  if (firsttime==19) 
  {
    for (int x=0; x<12; x++)  cap_calibration[x]=chargetime[x];
  } 

  //****************************************************************
  //loop through each of 11 pads according to pnum
  //****************************************************************
  if (pnum<10) pnum++; 
  else pnum=0;   
  overflow=0;

  //*****************************************************************
  //*****************************************************************
  //***********************start cap sensing and accel sensing ******
  //for high speed, read A/D for x, y, and z interleaved at times of necessary delay

  if (circbuffpointer<19) circbuffpointer++; //!!!set to 20 tops
  else
  {
    circbuffpointer=0; //accel. axis circular buffer pointer
    if (hithappened==LOW)  //if one buffer cycle without hit, use data for calibrations
    {
      //running calibration of accelerometer
      acc_calibrationx=0; //zero out
      acc_calibrationy=0;
      acc_calibrationz=0;

      for (int x=0; x<20; x++)  
      {
        acc_calibrationx+=circularaccbufferx[x]; 
        acc_calibrationy+=circularaccbuffery[x];
        acc_calibrationz+=circularaccbufferz[x]; 
      } 
      acc_calibrationx=int(acc_calibrationx/20);
      acc_calibrationy=int(acc_calibrationy/20);
      acc_calibrationz=int(acc_calibrationz/20);
    }
    hithappened=LOW;
  }

  //first measure charge up time, then measure fall time to cut sensitivity to gate threshold level
  //CHARGEUP
  //set driver pin high and measure rise time of selected pad
  digitalWrite(driverpin, HIGH);   // common driver pin high
  once=0;
  starttime = micros();
  while ((digitalRead(pad[pnum])==LOW) && (overflow==0))  //digital read is pretty slow it seems
  { //charge while loop
    if (micros()-starttime>1000) 
    {
      overflow=1;
      if (consolemidimode==0) //debug output to console
      {
        Serial.print ("overflow up on pin:"); 
        Serial.print(pnum);
        Serial.println(""); 
      } 
    }
  } //end charge while loop

  grabtime= micros()-starttime;

  //*********************interleaved x-axis accelerometer read **************************************

  //get x axis accel
  circularaccbufferx[circbuffpointer]= analogRead(xpin);
  //delayMicroseconds(30);  //a/d conversion time about 26us?

  //*********************interleave x-axis end****************************************************** 

  //finish charging of input pin to full voltage
  digitalWrite(pad[pnum],HIGH);  //set pullup resistor to on
  delayMicroseconds(100);//not needed if have delay from A/D
  digitalWrite(pad[pnum],LOW); //turn off pull up resistor

  //*********************interleaved y-axis accelerometer read **************************************

  //get y axis accel
  once=1;
  circularaccbuffery[circbuffpointer]= analogRead(ypin);
  delayMicroseconds(30);  


  //*********************interleave y-axis end ****************************************************** 

  //set driver pin low and measure fall time of selected pad
  //CHARGE DOWN
  digitalWrite(driverpin, LOW);   
  starttime = micros();
  once=0;
  while ((digitalRead(pad[pnum])==HIGH) && (overflow==0))
  { //discharging while loop
    if (micros()-starttime>1000) 
    {
      overflow=1;
      if (consolemidimode==0) //debug mode console output
      {
        Serial.print ("overflow down on pin:"); 
        Serial.print(pnum);
        Serial.println("");
      } 
    }
  } //end discharging while loops

  grabtime= grabtime+ micros()-starttime;  //add rise and fall times together
  //*********************interleaved z-axis accelerometer read **************************************

  //get z axis accel
  circularaccbufferz[circbuffpointer] = analogRead(zpin);
  delayMicroseconds(30);


  //*********************interleave #3 ******************************************************  

  delayMicroseconds(100); //to obtain the benefit of rise and fall measurements, must hit zero volts here
  chargetime[pnum]=grabtime;
  //*************************end of cap and accell sensing *****************************
  //****************************************************************************************
  //****************************************************************************************

  //touch detected ****************************
  if (chargetime[pnum]-cap_calibration[pnum]>hysteresishigh)
  {
    padstate[pnum]=HIGH;
    if (pnum<9)hithappened=HIGH; //exclude chord keys
  }
  else if (chargetime[pnum]-cap_calibration[pnum]<hysteresislow) padstate[pnum]=LOW;

  //special non playing pads--chord selectors
  if (padstate[9]==1) chord1=HIGH;
  else chord1=LOW;
  if (padstate[10]==1) chord2=HIGH; 
  else chord2=LOW;

  if ((chord1==LOW)&&(chord2==LOW)) chordselect=0;
  else if ((chord1==LOW)&&(chord2==HIGH)) chordselect=1;
  else if ((chord1==HIGH)&&(chord2==LOW)) chordselect=2;
  else if ((chord1==HIGH)&&(chord2==HIGH)) chordselect=3;

  if (trackSelect==2)  chordselect=0; //override chord buttons when tracking

  //load chord pallets *************************************************

  // if MIDI tracking override pad data for chord pads
  if ((trackSelect!=2) && (chordselect!=lastchordselect)) loadchords();

  //deactivate these pads for all other functions than chord select
  padmode[9]=4;
  padmode[10]=4;

  if ((padmode[pnum]==0) && (padstate[pnum]==HIGH)) //ready for new note
  {
    padlasttime[pnum]=micros();  //keep this low as long as pad is held
    triggerpoint=0; 
    padmode[pnum]= 1; //1 marks pending note before volume is determined
    //digitalWrite(profilepin, HIGH);
    holdoff[pnum]=micros();  //?? needed ??hold off stops rapid second trigger "bounce"
  }

  //check touch induced acceleration
  if (triggerpoint<11) triggerpoint++; //!!
  //let buffer run a bit then find max and load it into pending notes

  if (triggerpoint==10) //half of buffer==5.3 ms in this build
  {

    yaxispeak=acc_calibrationy;
    yaxisvalley=acc_calibrationy;
    xaxispeak=acc_calibrationx;
    xaxisvalley=acc_calibrationx;
    zaxispeak=acc_calibrationz;
    zaxisvalley=acc_calibrationz;

    for (int x=0; x<19; x++)  //!!grab peaks and valleys of 100 samples of accelerometer
    {
      if (circularaccbufferx[x]>xaxispeak)  xaxispeak=circularaccbufferx[x];
      if (circularaccbufferx[x]<xaxisvalley) xaxisvalley=circularaccbufferx[x];
      if (circularaccbuffery[x]>yaxispeak) yaxispeak=circularaccbuffery[x];
      if (circularaccbuffery[x]<yaxisvalley) yaxisvalley=circularaccbuffery[x];
      if (circularaccbufferz[x]>zaxispeak) zaxispeak=circularaccbufferz[x];
      if (circularaccbufferz[x]<zaxisvalley)  zaxisvalley=circularaccbufferz[x]; 
    } 

    xaxispeak=xaxispeak-int(acc_calibrationx);  //!!removed /100s
    yaxispeak=yaxispeak-int(acc_calibrationy);
    zaxispeak=zaxispeak-int(acc_calibrationz);

    xaxisvalley=xaxisvalley-int(acc_calibrationx);
    yaxisvalley=yaxisvalley-int(acc_calibrationy);
    zaxisvalley=zaxisvalley-int(acc_calibrationz);

    if (consolemidimode==3) acceleration_dump();  //debug console outputs
    if ((consolemidimode==2)&& (micros()/1000000>next)) chargedata_dump();

    //load up pending all notes with volume numbers
    for (int x=0; x<11; x++) 
    {
      if (padmode[x]==1) 
      {   
        if ((x==6) || (x==5)|| (x==2)|| (x==1)) //top of Kyub
        {
          padvolume[x]=-zaxisvalley;
          padmode[x]=2;    
        }

        if ((x==4)) //side of Kyub
        {
          padvolume[x]=-yaxisvalley; 
          padmode[x]=2;
        }

        if ((x==3) )
        {
          padvolume[x]=yaxispeak;  
          padmode[x]=2;
        }

        if ((x==0) )
        {
          padvolume[x]=-xaxisvalley;
          padmode[x]=2;
        }

        if ((x==7)||(x==8)||(x==9)||(x==10))
        {
          padvolume[x]=xaxispeak;
          padmode[x]=2;
        }
      }
    } 
  }//end of triggerpoint=50

  //play notes *****************************************************************

  //if quantification is selected, stall note playing until quant boundary
  int quantDelay=0;  //select quantization or none
  if (quantSelect==1) quantDelay=0;
  else if (quantSelect==2) quantDelay=300;
  else if (quantSelect==3) quantDelay=150; 

  if (trackSelect==2)  //tracking on
  {
    quantDelay=0; //override quantization
  }

  if ((millis()-drumpulse>quantDelay)) // && (miditrigger==1))  //150 is wild
  {
    drumpulse=millis();
    miditrigger=0;

    for (int x=0; x<11; x++)
    {
      if (padmode[x]==2)
      {
        //calculate volume
        notevolume=int((padvolume[x])*4);  //!!!!room for improvement--mapping of accel to volume
        if (notevolume>127) notevolume=127;
        if (notevolume<2) notevolume=2;//was 10

        RGBledtimer = millis();//used to fade out LED over time
        padmode[x]=3;  //3 is ready for note off

        pitch=padnote[x];
        //padlastchannel[x]=channel_0;
        padlastnote[x]=padnote[x];

        usbMIDI.sendNoteOn(pitch, notevolume, channel_0);
        colorcalculation(notevolume, x);
        analogWrite(Rled, Rledinput);
        analogWrite(Bled, Bledinput);
        analogWrite(Gled, Gledinput); 
        if (channel_1!=11) usbMIDI.sendNoteOn(pitch, notevolume, channel_1);
        if (channel_2!=11) usbMIDI.sendNoteOn(pitch, notevolume, channel_2);
      }
    }
  }

  //turn off notes **************************************************
  for (int x=0; x<11; x++)
  {
    if ((padstate[x]==LOW) && (padmode[x]==3)&& (micros()-padlasttime[x]>min_note_duration)) //need reset
    {
      padmode[x]=0;
      pitch=padlastnote[x];
      //channel_0=padlastchannel[x];
      usbMIDI.sendNoteOff(pitch, notevolume, channel_0);
      if (channel_1!=11) usbMIDI.sendNoteOff(pitch, notevolume, channel_1);
      if (channel_2!=11) usbMIDI.sendNoteOff(pitch, notevolume, channel_2);
    }
  }

  //dim RGB LED if on
  if (millis()-RGBledtimer>250)
  {
    if (Rledinput<255) Rledinput=Rledinput+1;
    if (Bledinput<255) Bledinput=Bledinput+1;
    if (Gledinput<255) Gledinput=Gledinput+1;
    analogWrite(Rled, Rledinput);
    analogWrite(Bled, Bledinput);
    analogWrite(Gled, Gledinput); 
  }


}//end main loop

//************************************
//************************************
//**          Functions             **
//************************************
//************************************


//FUNCTION TO SAVE EXTERNALLY LOADED CHORD PALLETS TO EEPROM
void savepallets(void)  //Save pallets --Teensy 2.0 has 1024 bytes
{

  Serial.print ("trackselect is equal to:");
  Serial.print(trackSelect);

  Serial.println("i'm saving to EEPROM!");
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader,chordA0[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+9,chordA1[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+18,chordA2[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+27,chordA3[loader]);
  EEPROM.write(36,chordAchannel);
  EEPROM.write(37,chordAchannel_1);
  EEPROM.write(38,chordAchannel_2);
  EEPROM.write(39,chordAQuant);
  EEPROM.write(40,chordATrack);
  EEPROM.write(41,chordAPassChan);


  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+42,chordB0[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+51,chordB1[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+60,chordB2[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+69,chordB3[loader]);

  EEPROM.write(78,chordBchannel);
  EEPROM.write(79,chordBchannel_1);
  EEPROM.write(80,chordBchannel_2);
  EEPROM.write(81,chordBQuant);
  EEPROM.write(82,chordBTrack);
  EEPROM.write(83,chordAPassChan);

  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+84,chordC0[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+93,chordC1[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+102,chordC2[loader]);
  for (byte loader=0; loader<9; loader++) EEPROM.write(loader+111,chordC3[loader]);

  EEPROM.write(120,chordCchannel);
  EEPROM.write(121,chordCchannel_1);
  EEPROM.write(122,chordCchannel_2);
  EEPROM.write(123,chordCQuant);
  EEPROM.write(124,chordCTrack);
  EEPROM.write(125,chordAPassChan);

  delay (1000);  //Help preven accidental EEPROM destruction 

  for (int rom=0; rom<150; rom++)
  {
    Serial.print(" printing rom");
    byte value = EEPROM.read(rom);
    Serial.print(rom);
    Serial.print("\t");
    Serial.print(value, DEC);
    Serial.println(); 
  }
}  

//FUNCTION TO LOAD  CHORD PALLETS FROM EEPROM (if any)

void loadEEPROM (void)
{
  delay(1000);
  //load chord pallets from EEprom
  if (EEPROM.read(0)!=255)
  {

    Serial.println("reading eeprom");
    for (byte loader=0; loader<9; loader++)chordA0[loader]=EEPROM.read(loader);
    for (byte loader=0; loader<9; loader++)chordA1[loader]=EEPROM.read(loader+9);
    for (byte loader=0; loader<9; loader++)chordA2[loader]=EEPROM.read(loader+18);
    for (byte loader=0; loader<9; loader++)chordA3[loader]=EEPROM.read(loader+27);
    chordAchannel=EEPROM.read(36);
    chordAchannel_1=EEPROM.read(37);
    chordAchannel_2=EEPROM.read(38);
    chordAQuant=EEPROM.read(39);
    chordATrack=EEPROM.read(40);
    chordAPassChan=EEPROM.read(41);


    for (byte loader=0; loader<9; loader++)chordB0[loader]=EEPROM.read(loader+42);
    for (byte loader=0; loader<9; loader++)chordB1[loader]=EEPROM.read(loader+51);
    for (byte loader=0; loader<9; loader++)chordB2[loader]=EEPROM.read(loader+60);
    for (byte loader=0; loader<9; loader++)chordB3[loader]=EEPROM.read(loader+69);
    chordBchannel=EEPROM.read(78);
    chordBchannel_1=EEPROM.read(79);
    chordBchannel_2=EEPROM.read(80);
    chordBQuant=EEPROM.read(81);
    chordBTrack=EEPROM.read(82);
    chordBPassChan=EEPROM.read(83);

    for (byte loader=0; loader<9; loader++)chordC0[loader]=EEPROM.read(loader+84);
    for (byte loader=0; loader<9; loader++)chordC1[loader]=EEPROM.read(loader+93);
    for (byte loader=0; loader<9; loader++)chordC2[loader]=EEPROM.read(loader+102);
    for (byte loader=0; loader<9; loader++)chordC3[loader]=EEPROM.read(loader+111);
    chordCchannel=EEPROM.read(120);
    chordCchannel_1=EEPROM.read(121);
    chordCchannel_2=EEPROM.read(122);
    chordCQuant=EEPROM.read(123);
    chordCTrack=EEPROM.read(124);
    chordCPassChan=EEPROM.read(125);
  } 
  onlyonce=HIGH; 

  for (int rom=0; rom<150; rom++)
  {
    Serial.print(" printing as read rom");
    byte value = EEPROM.read(rom);
    Serial.print(rom);
    Serial.print("\t");
    Serial.print(value, DEC);
    Serial.println(); 
  }
} 

//FUNCTION TO SET LED TO DIFFERENT COLORS
void colorcalculation(byte loudness, byte padhit) //calculates LED color by pad
{
  byte offbright=255;
  byte lowbright=226;
  byte midbright=198;
  byte fullbright=170; 

  if (notevolume>39)
  { 
    offbright=255;
    lowbright=197;
    midbright=141;
    fullbright=85;
  }
  else if (notevolume>84)
  { 
    offbright=255;
    lowbright=170;
    midbright=85;
    fullbright=0; 
  }

  if (padhit==0) //blue
  {
    Rledinput=offbright;
    Gledinput=offbright;
    Bledinput=fullbright;    
  }
  else if (padhit==1) //red
  {
    Rledinput=fullbright;
    Gledinput=offbright;
    Bledinput=offbright; 
  }
  else if (padhit==2) //aqua
  {
    Rledinput=offbright;
    Gledinput=fullbright;
    Bledinput=lowbright;  
  }
  else if (padhit==3)  //green
  {
    Rledinput=offbright;
    Gledinput=fullbright;
    Bledinput=offbright;  
  }
  else if (padhit==4) //violet
  {
    Rledinput=lowbright;
    Gledinput=offbright;
    Bledinput=midbright; 
  }
  else if (padhit==5) //white
  {
    Rledinput=lowbright;
    Gledinput=lowbright;
    Bledinput=lowbright; 
  }
  else if (padhit==6)  //lime
  {
    Rledinput=lowbright;
    Gledinput=midbright;
    Bledinput=offbright; 
  }
  else if (padhit==7) //plum
  {
    Rledinput=midbright;
    Gledinput=offbright;
    Bledinput=lowbright; 
  }
  else if (padhit==8) //orange
  {
    Rledinput=midbright;
    Gledinput=lowbright;
    Bledinput=offbright; 
  }
  else if (padhit==9) //light blue
  {
    Rledinput=offbright;
    Gledinput=lowbright;
    Bledinput=midbright; 
  }
  else if (padhit==10) //dark blue
  {
    Rledinput=offbright;
    Gledinput=offbright;
    Bledinput=fullbright;    
  }
  else //off
  {
    Rledinput=offbright;
    Gledinput=offbright;
    Bledinput=offbright;
  }
}

//FUNCTION TO UNPACK SERIAL DATA FROM EXTERNAL PROGRAM
void decodeChord(byte inbyte)
{
  rootnumber=bytecounter%18;
  switch (palletcounter) 
  {
  case 0:
    if (rootnumber==0) chordA0[0]=56+inbyte;  
    if (rootnumber==1) chordAtype[0]=inbyte; 
    if (rootnumber==2)
    {
      octave=(inbyte-4)*12;  //1=A
      chordA0[0]=chordA0[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==3) chordA1[0]=56+inbyte;  
    if (rootnumber==4) chordAtype[1]=inbyte; 
    if (rootnumber==5)
    {
      octave=(inbyte-4)*12;  //1=A
      chordA1[0]=chordA1[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==6) chordA2[0]=56+inbyte;  
    if (rootnumber==7) chordAtype[2]=inbyte; 
    if (rootnumber==8)
    {
      octave=(inbyte-4)*12;  //1=A
      chordA2[0]=chordA2[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 

    if (rootnumber==9) chordA3[0]=56+inbyte;  
    if (rootnumber==10) chordAtype[3]=inbyte; 
    if (rootnumber==11)
    {
      octave=(inbyte-4)*12;  //1=A
      chordA3[0]=chordA3[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 
    if (rootnumber==12) chordAchannel=inbyte; 
    if (rootnumber==13) chordAchannel_1=inbyte; 
    if (rootnumber==14) chordAchannel_2=inbyte;
    if (rootnumber==15) chordAQuant=inbyte;
    if (rootnumber==16) chordATrack=inbyte;
    if (rootnumber==17) chordAPassChan=inbyte;  
    break;

  case 1:
    if (rootnumber==0) chordB0[0]=56+inbyte;  
    if (rootnumber==1) chordBtype[0]=inbyte; 
    if (rootnumber==2)
    {
      octave=(inbyte-4)*12;  //1=A
      chordB0[0]=chordB0[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==3) chordB1[0]=56+inbyte;  
    if (rootnumber==4) chordBtype[1]=inbyte; 
    if (rootnumber==5)
    {
      octave=(inbyte-4)*12;  //1=A
      chordB1[0]=chordB1[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==6) chordB2[0]=56+inbyte;  
    if (rootnumber==7) chordBtype[2]=inbyte; 
    if (rootnumber==8)
    {
      octave=(inbyte-4)*12;  //1=A
      chordB2[0]=chordB2[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 

    if (rootnumber==9) chordB3[0]=56+inbyte;  
    if (rootnumber==10) chordBtype[3]=inbyte; 
    if (rootnumber==11)
    {
      octave=(inbyte-4)*12;  //1=A
      chordB3[0]=chordB3[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 
    if (rootnumber==12) chordBchannel=inbyte; 
    if (rootnumber==13) chordBchannel_1=inbyte; 
    if (rootnumber==14) chordBchannel_2=inbyte;
    if (rootnumber==15) chordBQuant=inbyte;
    if (rootnumber==16) chordBTrack=inbyte;
    if (rootnumber==17) chordBPassChan=inbyte;  
    break;

  case 2:
    if (rootnumber==0) chordC0[0]=56+inbyte;  
    if (rootnumber==1) chordCtype[0]=inbyte; 
    if (rootnumber==2)
    {
      octave=(inbyte-4)*12;  //1=A
      chordC0[0]=chordC0[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==3) chordC1[0]=56+inbyte;  
    if (rootnumber==4) chordCtype[1]=inbyte; 
    if (rootnumber==5)
    {
      octave=(inbyte-4)*12;  //1=A
      chordC1[0]=chordC1[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    }

    if (rootnumber==6) chordC2[0]=56+inbyte;  
    if (rootnumber==7) chordCtype[2]=inbyte; 
    if (rootnumber==8)
    {
      octave=(inbyte-4)*12;  //1=A
      chordC2[0]=chordC2[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 

    if (rootnumber==9) chordC3[0]=56+inbyte;  
    if (rootnumber==10) chordCtype[3]=inbyte; 
    if (rootnumber==11)
    {
      octave=(inbyte-4)*12;  //1=A
      chordC3[0]=chordC3[0]+octave;
      trackOctave=octave; //octave for tracking purposes
    } 
    if (rootnumber==12) chordCchannel=inbyte; 
    if (rootnumber==13) chordCchannel_1=inbyte; 
    if (rootnumber==14) chordCchannel_2=inbyte;
    if (rootnumber==15) chordCQuant=inbyte;
    if (rootnumber==16) chordCTrack=inbyte;
    if (rootnumber==17) chordCPassChan=inbyte;  
    break;
  default: 
    Serial.print(" out of range");
  }
}

/*void decodeChannel(byte info)
 {  
 
 if (palletcounter=0) chordAchannel=info;
 else if (palletcounter=1) chordBchannel=info;
 else if (palletcounter=2) chordCchannel=info;
 
 Serial.print("decodedchannel=");
 Serial.println(info);  
 }*/


//FUNCTION TO BUILD CHORDS FROM EXTERNALLY LOADED ROOT DATA
void buildChords(void)
{
  //Serial.println("im building chords");
  for(byte march=0; march<9; march++)
  {
    if (chordAtype[0]==1) chordA0[march]=chordA0[0]+majortemplatev1[march];
    if (chordAtype[0]==2) chordA0[march]=chordA0[0]+minortemplatev1[march];
    if (chordAtype[0]==3) chordA0[march]=chordA0[0]+majortemplatev2[march];
    if (chordAtype[0]==4) chordA0[march]=chordA0[0]+minortemplatev2[march];
    if (chordAtype[0]==5) chordA0[march]=chordA0[0]+maj6template[march];
    if (chordAtype[0]==6) chordA0[march]=chordA0[0]+min6template[march];
    if (chordAtype[0]==7) chordA0[march]=chordA0[0]+maj7template[march];
    if (chordAtype[0]==8) chordA0[march]=chordA0[0]+min7template[march];
    if (chordAtype[0]==9) chordA0[march]=chordA0[0]+dimtemplate[march];
    if (chordAtype[0]==10) chordA0[march]=chordA0[0]+dim7template[march];
    if (chordAtype[0]==11) chordA0[march]=chordA0[0]+sus2template[march];
    if (chordAtype[0]==12) chordA0[march]=chordA0[0]+sus4template[march];
    if (chordAtype[0]==13) chordA0[march]=chordA0[0]+augtemplate[march];
    if (chordAtype[0]==14) chordA0[march]=d1template[march];
    if (chordAtype[0]==15)chordA0[march]=d2template[march];
    if (chordAtype[0]==16) chordA0[march]=d3template[march];
    if (chordAtype[0]==17)chordA0[march]=chordA0[0]+powertemplate[march];      


    //Serial.print("a0 is");
    //Serial.println(chordA0[0]);

    //Serial.print("a0 notes");
    //Serial.println(chordA0[march]);


    if (chordAtype[1]==1) chordA1[march]=chordA1[0]+majortemplatev1[march];
    if (chordAtype[1]==2) chordA1[march]=chordA1[0]+minortemplatev1[march];
    if (chordAtype[1]==3) chordA1[march]=chordA1[0]+majortemplatev2[march];
    if (chordAtype[1]==4) chordA1[march]=chordA1[0]+minortemplatev2[march];
    if (chordAtype[1]==5) chordA1[march]=chordA1[0]+maj6template[march];
    if (chordAtype[1]==6) chordA1[march]=chordA1[0]+min6template[march];
    if (chordAtype[1]==7) chordA1[march]=chordA1[0]+maj7template[march];
    if (chordAtype[1]==8) chordA1[march]=chordA1[0]+min7template[march];
    if (chordAtype[1]==9) chordA1[march]=chordA1[0]+dimtemplate[march];
    if (chordAtype[1]==10) chordA1[march]=chordA1[0]+dim7template[march];
    if (chordAtype[1]==11) chordA1[march]=chordA1[0]+sus2template[march];
    if (chordAtype[1]==12) chordA1[march]=chordA1[0]+sus4template[march];
    if (chordAtype[1]==13) chordA1[march]=chordA1[0]+augtemplate[march];
    if (chordAtype[1]==14) chordA1[march]=d1template[march];
    if (chordAtype[1]==15) chordA1[march]=d2template[march];
    if (chordAtype[1]==16) chordA1[march]=d3template[march];
    if (chordAtype[1]==17) chordA1[march]=chordA1[0]+powertemplate[march];
    //Serial.print("a1 notes");
    //Serial.println(chordA1[march]);


    if (chordAtype[2]==1) chordA2[march]=chordA2[0]+majortemplatev1[march];
    if (chordAtype[2]==2) chordA2[march]=chordA2[0]+minortemplatev1[march];
    if (chordAtype[2]==3) chordA2[march]=chordA2[0]+majortemplatev2[march];
    if (chordAtype[2]==4) chordA2[march]=chordA2[0]+minortemplatev2[march];
    if (chordAtype[2]==5) chordA2[march]=chordA2[0]+maj6template[march];
    if (chordAtype[2]==6) chordA2[march]=chordA2[0]+min6template[march];
    if (chordAtype[2]==7) chordA2[march]=chordA2[0]+maj7template[march];
    if (chordAtype[2]==8) chordA2[march]=chordA2[0]+min7template[march];
    if (chordAtype[2]==9) chordA2[march]=chordA2[0]+dimtemplate[march];
    if (chordAtype[2]==10) chordA2[march]=chordA2[0]+dim7template[march];
    if (chordAtype[2]==11) chordA2[march]=chordA2[0]+sus2template[march];
    if (chordAtype[2]==12) chordA2[march]=chordA2[0]+sus4template[march];
    if (chordAtype[2]==13) chordA2[march]=chordA2[0]+augtemplate[march];
    if (chordAtype[2]==14) chordA2[march]=d1template[march];
    if (chordAtype[2]==15) chordA2[march]=d2template[march];
    if (chordAtype[2]==16) chordA2[march]=d3template[march];
    if (chordAtype[2]==17) chordA2[march]=chordA2[0]+powertemplate[march];

    if (chordAtype[3]==1) chordA3[march]=chordA3[0]+majortemplatev1[march];
    if (chordAtype[3]==2) chordA3[march]=chordA3[0]+minortemplatev1[march];
    if (chordAtype[3]==3) chordA3[march]=chordA3[0]+majortemplatev2[march];
    if (chordAtype[3]==4) chordA3[march]=chordA3[0]+minortemplatev2[march];
    if (chordAtype[3]==5) chordA3[march]=chordA3[0]+maj6template[march];
    if (chordAtype[3]==6) chordA3[march]=chordA3[0]+min6template[march];
    if (chordAtype[3]==7) chordA3[march]=chordA3[0]+maj7template[march];
    if (chordAtype[3]==8) chordA3[march]=chordA3[0]+min7template[march];
    if (chordAtype[3]==9) chordA3[march]=chordA3[0]+dimtemplate[march];
    if (chordAtype[3]==10) chordA3[march]=chordA3[0]+dim7template[march];
    if (chordAtype[3]==11) chordA3[march]=chordA3[0]+sus2template[march];
    if (chordAtype[3]==12) chordA3[march]=chordA3[0]+sus4template[march];
    if (chordAtype[3]==13) chordA3[march]=chordA3[0]+augtemplate[march];
    if (chordAtype[3]==14) chordA3[march]=d1template[march];
    if (chordAtype[3]==15) chordA3[march]=d2template[march];
    if (chordAtype[3]==16) chordA3[march]=d3template[march];
    if (chordAtype[3]==17) chordA3[march]=chordA3[0]+powertemplate[march];

    if (chordBtype[0]==1) chordB0[march]=chordB0[0]+majortemplatev1[march];
    if (chordBtype[0]==2) chordB0[march]=chordB0[0]+minortemplatev1[march];
    if (chordBtype[0]==3) chordB0[march]=chordB0[0]+majortemplatev2[march];
    if (chordBtype[0]==4) chordB0[march]=chordB0[0]+minortemplatev2[march];
    if (chordBtype[0]==5) chordB0[march]=chordB0[0]+maj6template[march];
    if (chordBtype[0]==6) chordB0[march]=chordB0[0]+min6template[march];
    if (chordBtype[0]==7) chordB0[march]=chordB0[0]+maj7template[march];
    if (chordBtype[0]==8) chordB0[march]=chordB0[0]+min7template[march];
    if (chordBtype[0]==9) chordB0[march]=chordB0[0]+dimtemplate[march];
    if (chordBtype[0]==10) chordB0[march]=chordB0[0]+dim7template[march];
    if (chordBtype[0]==11) chordB0[march]=chordB0[0]+sus2template[march];
    if (chordBtype[0]==12) chordB0[march]=chordB0[0]+sus4template[march];
    if (chordBtype[0]==13) chordB0[march]=chordB0[0]+augtemplate[march];
    if (chordBtype[0]==14) chordB0[march]=d1template[march];
    if (chordBtype[0]==15) chordB0[march]=d2template[march];
    if (chordBtype[0]==16) chordB0[march]=d3template[march];
    if (chordBtype[0]==17) chordB0[march]=chordB0[0]+powertemplate[march];

    if (chordBtype[1]==1) chordB1[march]=chordB1[0]+majortemplatev1[march];
    if (chordBtype[1]==2) chordB1[march]=chordB1[0]+minortemplatev1[march];
    if (chordBtype[1]==3) chordB1[march]=chordB1[0]+majortemplatev2[march];
    if (chordBtype[1]==4) chordB1[march]=chordB1[0]+minortemplatev2[march];
    if (chordBtype[1]==5) chordB1[march]=chordB1[0]+maj6template[march];
    if (chordBtype[1]==6) chordB1[march]=chordB1[0]+min6template[march];
    if (chordBtype[1]==7) chordB1[march]=chordB1[0]+maj7template[march];
    if (chordBtype[1]==8) chordB1[march]=chordB1[0]+min7template[march];
    if (chordBtype[1]==9) chordB1[march]=chordB1[0]+dimtemplate[march];
    if (chordBtype[1]==10) chordB1[march]=chordB1[0]+dim7template[march];
    if (chordBtype[1]==11) chordB1[march]=chordB1[0]+sus2template[march];
    if (chordBtype[1]==12) chordB1[march]=chordB1[0]+sus4template[march];
    if (chordBtype[1]==13) chordB1[march]=chordB1[0]+augtemplate[march];
    if (chordBtype[1]==14) chordB1[march]=d1template[march];
    if (chordBtype[1]==15) chordB1[march]=d2template[march];
    if (chordBtype[1]==16) chordB1[march]=d3template[march];
    if (chordBtype[1]==17) chordB1[march]=chordB1[0]+powertemplate[march];


    if (chordBtype[2]==1) chordB2[march]=chordB2[0]+majortemplatev1[march];
    if (chordBtype[2]==2) chordB2[march]=chordB2[0]+minortemplatev1[march];
    if (chordBtype[2]==3) chordB2[march]=chordB2[0]+majortemplatev2[march];
    if (chordBtype[2]==4) chordB2[march]=chordB2[0]+minortemplatev2[march];
    if (chordBtype[2]==5) chordB2[march]=chordB2[0]+maj6template[march];
    if (chordBtype[2]==6) chordB2[march]=chordB2[0]+min6template[march];
    if (chordBtype[2]==7) chordB2[march]=chordB2[0]+maj7template[march];
    if (chordBtype[2]==8) chordB2[march]=chordB2[0]+min7template[march];
    if (chordBtype[2]==9) chordB2[march]=chordB2[0]+dimtemplate[march];
    if (chordBtype[2]==10) chordB2[march]=chordB2[0]+dim7template[march];
    if (chordBtype[2]==11) chordB2[march]=chordB2[0]+sus2template[march];
    if (chordBtype[2]==12) chordB2[march]=chordB2[0]+sus4template[march];
    if (chordBtype[2]==13) chordB2[march]=chordB2[0]+augtemplate[march];
    if (chordBtype[2]==14) chordB2[march]=d1template[march];
    if (chordBtype[2]==15) chordB2[march]=d2template[march];
    if (chordBtype[2]==16) chordB2[march]=d3template[march];
    if (chordBtype[2]==17) chordB2[march]=chordB2[0]+powertemplate[march];

    if (chordBtype[3]==1) chordB3[march]=chordB3[0]+majortemplatev1[march];
    if (chordBtype[3]==2) chordB3[march]=chordB3[0]+minortemplatev1[march];
    if (chordBtype[3]==3) chordB3[march]=chordB3[0]+majortemplatev2[march];
    if (chordBtype[3]==4) chordB3[march]=chordB3[0]+minortemplatev2[march];
    if (chordBtype[3]==5) chordB3[march]=chordB3[0]+maj6template[march];
    if (chordBtype[3]==6) chordB3[march]=chordB3[0]+min6template[march];
    if (chordBtype[3]==7) chordB3[march]=chordB3[0]+maj7template[march];
    if (chordBtype[3]==8) chordB3[march]=chordB3[0]+min7template[march];
    if (chordBtype[3]==9) chordB3[march]=chordB3[0]+dimtemplate[march];
    if (chordBtype[3]==10) chordB3[march]=chordB3[0]+dim7template[march];
    if (chordBtype[3]==11) chordB3[march]=chordB3[0]+sus2template[march];
    if (chordBtype[3]==12) chordB3[march]=chordB3[0]+sus4template[march];
    if (chordBtype[3]==13) chordB3[march]=chordB3[0]+augtemplate[march];
    if (chordBtype[3]==14) chordB3[march]=d1template[march];
    if (chordBtype[3]==15) chordB3[march]=d2template[march];
    if (chordBtype[3]==16) chordB3[march]=d3template[march];
    if (chordBtype[3]==17) chordB3[march]=chordB3[0]+powertemplate[march];

    if (chordCtype[0]==1) chordC0[march]=chordC0[0]+majortemplatev1[march];
    if (chordCtype[0]==2) chordC0[march]=chordC0[0]+minortemplatev1[march];
    if (chordCtype[0]==3) chordC0[march]=chordC0[0]+majortemplatev2[march];
    if (chordCtype[0]==4) chordC0[march]=chordC0[0]+minortemplatev2[march];
    if (chordCtype[0]==5) chordC0[march]=chordC0[0]+maj6template[march];
    if (chordCtype[0]==6) chordC0[march]=chordC0[0]+min6template[march];
    if (chordCtype[0]==7) chordC0[march]=chordC0[0]+maj7template[march];
    if (chordCtype[0]==8) chordC0[march]=chordC0[0]+min7template[march];
    if (chordCtype[0]==9) chordC0[march]=chordC0[0]+dimtemplate[march];
    if (chordCtype[0]==10) chordC0[march]=chordC0[0]+dim7template[march];
    if (chordCtype[0]==11) chordC0[march]=chordC0[0]+sus2template[march];
    if (chordCtype[0]==12) chordC0[march]=chordC0[0]+sus4template[march];
    if (chordCtype[0]==13) chordC0[march]=chordC0[0]+augtemplate[march];
    if (chordCtype[0]==14) chordC0[march]=d1template[march];
    if (chordCtype[0]==15) chordC0[march]=d2template[march];
    if (chordCtype[0]==16) chordC0[march]=d3template[march];
    if (chordCtype[0]==17) chordC0[march]=chordC0[0]+powertemplate[march];

    if (chordCtype[1]==1) chordC1[march]=chordC1[0]+majortemplatev1[march];
    if (chordCtype[1]==2) chordC1[march]=chordC1[0]+minortemplatev1[march];
    if (chordCtype[1]==3) chordC1[march]=chordC1[0]+majortemplatev2[march];
    if (chordCtype[1]==4) chordC1[march]=chordC1[0]+minortemplatev2[march];
    if (chordCtype[1]==5) chordC1[march]=chordC1[0]+maj6template[march];
    if (chordCtype[1]==6) chordC1[march]=chordC1[0]+min6template[march];
    if (chordCtype[1]==7) chordC1[march]=chordC1[0]+maj7template[march];
    if (chordCtype[1]==8) chordC1[march]=chordC1[0]+min7template[march];
    if (chordCtype[1]==9) chordC1[march]=chordC1[0]+dimtemplate[march];
    if (chordCtype[1]==10) chordC1[march]=chordC1[0]+dim7template[march];
    if (chordCtype[1]==11) chordC1[march]=chordC1[0]+sus2template[march];
    if (chordCtype[1]==12) chordC1[march]=chordC1[0]+sus4template[march];
    if (chordCtype[1]==13) chordC1[march]=chordC1[0]+augtemplate[march];
    if (chordCtype[1]==14) chordC1[march]=d1template[march];
    if (chordCtype[1]==15) chordC1[march]=d2template[march];
    if (chordCtype[1]==16) chordC1[march]=d3template[march];
    if (chordCtype[1]==17) chordC1[march]=chordC1[0]+powertemplate[march];


    if (chordCtype[2]==1) chordC2[march]=chordC2[0]+majortemplatev1[march];
    if (chordCtype[2]==2) chordC2[march]=chordC2[0]+minortemplatev1[march];
    if (chordCtype[2]==3) chordC2[march]=chordC2[0]+majortemplatev2[march];
    if (chordCtype[2]==4) chordC2[march]=chordC2[0]+minortemplatev2[march];
    if (chordCtype[2]==5) chordC2[march]=chordC2[0]+maj6template[march];
    if (chordCtype[2]==6) chordC2[march]=chordC2[0]+min6template[march];
    if (chordCtype[2]==7) chordC2[march]=chordC2[0]+maj7template[march];
    if (chordCtype[2]==8) chordC2[march]=chordC2[0]+min7template[march];
    if (chordCtype[2]==9) chordC2[march]=chordC2[0]+dimtemplate[march];
    if (chordCtype[2]==10) chordC2[march]=chordC2[0]+dim7template[march];
    if (chordCtype[2]==11) chordC2[march]=chordC2[0]+sus2template[march];
    if (chordCtype[2]==12) chordC2[march]=chordC2[0]+sus4template[march];
    if (chordCtype[2]==13) chordC2[march]=chordC2[0]+augtemplate[march];
    if (chordCtype[2]==14) chordC2[march]=d1template[march];
    if (chordCtype[2]==15) chordC2[march]=d2template[march];
    if (chordCtype[2]==16) chordC2[march]=d3template[march];
    if (chordCtype[2]==17) chordC2[march]=chordC2[0]+powertemplate[march];

    if (chordCtype[3]==1) chordC3[march]=chordC3[0]+majortemplatev1[march];
    if (chordCtype[3]==2) chordC3[march]=chordC3[0]+minortemplatev1[march];
    if (chordCtype[3]==3) chordC3[march]=chordC3[0]+majortemplatev2[march];
    if (chordCtype[3]==4) chordC3[march]=chordC3[0]+minortemplatev2[march];
    if (chordCtype[3]==5) chordC3[march]=chordC3[0]+maj6template[march];
    if (chordCtype[3]==6) chordC3[march]=chordC3[0]+min6template[march];
    if (chordCtype[3]==7) chordC3[march]=chordC3[0]+maj7template[march];
    if (chordCtype[3]==8) chordC3[march]=chordC3[0]+min7template[march];
    if (chordCtype[3]==9) chordC3[march]=chordC3[0]+dimtemplate[march];
    if (chordCtype[3]==10) chordC3[march]=chordC3[0]+dim7template[march];
    if (chordCtype[3]==11) chordC3[march]=chordC3[0]+sus2template[march];
    if (chordCtype[3]==12) chordC3[march]=chordC3[0]+sus4template[march];
    if (chordCtype[3]==13) chordC3[march]=chordC3[0]+augtemplate[march];
    if (chordCtype[3]==14) chordC3[march]=d1template[march];
    if (chordCtype[3]==15) chordC3[march]=d2template[march];
    if (chordCtype[3]==16) chordC3[march]=d3template[march];
    if (chordCtype[3]==17) chordC3[march]=chordC3[0]+powertemplate[march];
  }
  loadchords();
  savepallets();
}

//FUNCTION TO LOAD CHORD PALLET PER PAD PRESSES
void loadchords(void)
{
  lastchordselect=chordselect;
  if (chordpallet==0)  
  {
    if (chordselect==0)for (int i=0; i<9; i++) 
    {
      padnote[i]=(chordA0[i]);
       Serial.print("padnote=");
       Serial.println(padnote[i]);
    }
    if (chordselect==1)for (int i=0; i<9; i++) padnote[i]=chordA1[i];
    if (chordselect==2)for (int i=0; i<9; i++) padnote[i]=chordA2[i];
    if (chordselect==3)for (int i=0; i<9; i++) padnote[i]=chordA3[i];
    channel_0=chordAchannel;
    channel_1=chordAchannel_1;
    channel_2=chordAchannel_2;
    quantSelect=chordAQuant;
    trackSelect=chordATrack;
    passchannel=chordAPassChan;

  }
  if (chordpallet==1)  
  {
    if (chordselect==0)for (int i=0; i<9; i++) padnote[i]=chordB0[i];
    if (chordselect==1)for (int i=0; i<9; i++) padnote[i]=chordB1[i];
    if (chordselect==2)for (int i=0; i<9; i++) padnote[i]=chordB2[i];
    if (chordselect==3)for (int i=0; i<9; i++) padnote[i]=chordB3[i];
    channel_0=chordBchannel;
    channel_1=chordBchannel_1;
    channel_2=chordBchannel_2;
    quantSelect=chordBQuant;
    trackSelect=chordBTrack;
    passchannel=chordBPassChan;
  }
  if (chordpallet==2)  
  {
    if (chordselect==0)for (int i=0; i<9; i++) padnote[i]=chordC0[i];
    if (chordselect==1)for (int i=0; i<9; i++) padnote[i]=chordC1[i];
    if (chordselect==2)for (int i=0; i<9; i++) padnote[i]=chordC2[i];
    if (chordselect==3)for (int i=0; i<9; i++) padnote[i]=chordC3[i];
    channel_0=chordCchannel;
    channel_1=chordCchannel_1;
    channel_2=chordCchannel_2;
    quantSelect=chordCQuant;
    trackSelect=chordCTrack;
    passchannel=chordCPassChan;
  }
} 


//DEBUG FUNCTION FOR ACCELERATION
void acceleration_dump(void)  //debuging routine
//useful for testing accelerometer
{
  int indexer=0; 
  for (int p=0; p<20; p++)
  {
    if (p==10) //!! 
    {
      Serial.println("");
      Serial.println("hit point");
    }
    Serial.println("");
    Serial.print(p);
    Serial.print(" x:");
    Serial.print(circularaccbufferx[circbuffpointer+indexer]-int(acc_calibrationx)); //!! remobed 100s
    Serial.print(" ~y:");
    Serial.print(circularaccbuffery[circbuffpointer+indexer]-int(acc_calibrationy));
    Serial.print(" z:");
    Serial.print(circularaccbufferz[circbuffpointer+indexer]-int(acc_calibrationz));
    indexer++;
    if (indexer+circbuffpointer>19) indexer=-circbuffpointer; ///!!!!  
  } 
  Serial.println ("");   
  Serial.print (" xaxis peak:");
  Serial.print (xaxispeak);
  Serial.print (" yaxis peak:");
  Serial.print (yaxispeak);
  Serial.print (" zaxis peak:");
  Serial.println (zaxispeak);

  Serial.print (" xaxis valley:");
  Serial.print (xaxisvalley);
  Serial.print ("  yaxis valley:");
  Serial.print (yaxisvalley);
  Serial.print ("  zaxis valley:");
  Serial.println (zaxisvalley);
  Serial.println ("  ");


  Serial.print (" x cal raw:");
  Serial.print (acc_calibrationx);
  Serial.print ("  y cal raw:");
  Serial.print (acc_calibrationy);
  Serial.print ("  z cal raw:");
  Serial.println (acc_calibrationz);

} 

void chargedata_dump(void) //debugging routine
//useful for texting capacitive sensing and pad wiring
{
  next++;
  for (int x=0; x<11; x++)  //prints out all charge times
  {
    Serial.print(" pin:");
    Serial.print(x);
    Serial.print("=");
    Serial.print(chargetime[x]-cap_calibration[x]);
  }
  Serial.println("");
  /*Serial.print ("padvol#1:  ");
   Serial.print (padmode[1]);
   Serial.print ("-xvalley: ");
   Serial.print(-xaxisvalley); 
   Serial.println(""); */
} 

//check for free RAM--thanks to jeelabs.org
int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

//Midi receive callback functions
//channel 1 provides chord roots for Kyub
//channel 2 provides chord types
// once channel passes through Kyub to play autmatically per chordNPassChah
// channel 11 can sync playing if desired.

void OnNoteOn(byte channel, byte note, byte velocity)
{
  //"trackselect" turns on tracking
  if (trackSelect==2) //use MIDI data if track button on
  {
    //grab root note from MIDI track, octave from curent mode
    trackRoot=chordoverridenote+47+trackOctave; 
    if (channel==1)
    {
      chordoverridenote=note%12;//use channel one to control chord note c=0 (so c#, d, e, f)
      Serial.print("chord chanel recieved= ");
      Serial.println(channel);
    }
    if (chordoverridetype==0)
    { 
      //Serial.print("chord override type= ");
      //Serial.println(chordoverridetype); 
      for (int march=0; march<9; march++) padnote[march]=trackRoot+majortemplatev1[march];
    } 
    if (chordoverridetype==1)
    { 
      for (int march=0; march<9; march++) chordA0[march]=trackRoot+minortemplatev1[march];
    } 

    if (channel==2) chordoverridetype=0; ///note%12;//use channel one to control chord type c=0 (so c#, d, e, f)
    if ((channel==passchannel) && (passchannel!=10)) usbMIDI.sendNoteOn(note, velocity, channel);  //play this channel
    //if (channel==10) usbMIDI.sendNoteOn(note, velocity, channel); //play this channel
    if (channel==11) miditrigger=1;
    analogWrite(Rled, 0);
  }
}

void OnNoteOff(byte channel, byte note, byte velocity)
{
  if (channel==10) usbMIDI.sendNoteOff(note, velocity, channel);
  if (channel==2) usbMIDI.sendNoteOff(note, velocity, channel);
  analogWrite(Rled, 255);
}




































































































