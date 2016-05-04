//************************************
//************************************
//**         Kyub Basic             **
//**           5/7/2014             **
//** Low latency, Scale selection   **
//**         with RGB LED           **
//************************************
//************************************


//************************************
//************************************
//**      Global Variables          **
//************************************
//************************************

//Debugging settings
int consolemidimode=1; //consolemode=0 midimode=1  capacitive mode = 2 accelerometer=3 

//Teensy pin assignments
//int profilepin=18;  //used for latency experiments
const byte modebutton=22;
byte scale=0;  //scale selection
const byte Rled = 12; //tricolor LED pins
const byte Gled = 14;
const byte Bled = 15;
const byte driverpin = 13; //common pad drive pin
const byte zpin=2; //accelerometer axes inputs
const byte xpin=1;
const byte ypin=0;

byte padnote[11] = {
  69,60,65,62,61,67,64,71,70,66,70};  //arbitrary initial pad note values

//misc varibles
byte channel=0x94;  //arbitrary MIDI channel--change as desired 
byte pad[11]={0,1,2,3,4,5,6,7,8,9,10};
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

//capactivie sensing variables
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
  //pinMode(LEDpin, OUTPUT);
  pinMode(Rled, OUTPUT); 
  pinMode(Bled, OUTPUT); 
  pinMode(Gled, OUTPUT); 

  // pinMode(profilepin, OUTPUT);
  pinMode(driverpin, OUTPUT);

  pinMode(modebutton, INPUT);

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

  digitalWrite(modebutton, HIGH);

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

  //set scale according to presses of mode button
  if (digitalRead(modebutton)==LOW)
  {
    if (scale==0) //C major--yellow
    {

      analogWrite(Rled, 0);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);  
      scale=1;
    }

    else if (scale==1) //C minor--red
    {
      analogWrite(Gled, 0);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);
      scale=2;
    }
    else if (scale==2) //C major Pentatonic--green
    {
      analogWrite(Bled, 0);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);
      scale=3;
    }

    else if (scale==3) //C blues--blue
    {
      analogWrite(Bled, 125);
      analogWrite(Gled, 125);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);
      scale=4;
    }
    else if (scale==4) //japanese--aqua
    {
      analogWrite(Bled, 125);
      analogWrite(Rled, 125);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);
      scale=5;
    }

    else if (scale==5) //kletzmer-purple
    {
      analogWrite(Gled, 125);
      analogWrite(Rled, 125);
      delay (1000);
      analogWrite(Rled, 255); 
      analogWrite(Gled, 255); 
      analogWrite(Bled, 255);
      scale=0;
    } 


    if (scale==0)  //*************************************************
    {//c major scale
      padnote[0]=60;
      padnote[1]=62;
      padnote[2]=64;
      padnote[3]=65;
      padnote[4]=67;
      padnote[5]=69;
      padnote[6]=71;
      padnote[7]=72;
      padnote[8]=74;
      padnote[9]=76;
      padnote[10]=77;
    }

    if (scale==1)//
    {//c minor scale
      padnote[0]=60;
      padnote[1]=62;
      padnote[2]=63;
      padnote[3]=65;
      padnote[4]=67;
      padnote[5]=68;
      padnote[6]=70;
      padnote[7]=72;
      padnote[8]=74;
      padnote[9]=75;
      padnote[10]=77;
    }
    if (scale==2)//
    {//c major pentatonic

      padnote[0]=60;
      padnote[1]=62;
      padnote[2]=64;
      padnote[3]=67;
      padnote[4]=69;
      padnote[5]=72;
      padnote[6]=74;
      padnote[7]=76;
      padnote[8]=79;
      padnote[9]=81;
      padnote[10]=84;
    }

    if (scale==3)//
    {//c major blues

      padnote[0]=60;
      padnote[1]=62;
      padnote[2]=63;
      padnote[3]=64;
      padnote[4]=67;
      padnote[5]=69;
      padnote[6]=72;
      padnote[7]=74;
      padnote[8]=75;
      padnote[9]=76;
      padnote[10]=79;
    }

    if (scale==4)//
    {//japanese

      padnote[0]=60;
      padnote[1]=61;
      padnote[2]=65;
      padnote[3]=67;
      padnote[4]=68;
      padnote[5]=72;
      padnote[6]=73;
      padnote[7]=77;
      padnote[8]=79;
      padnote[9]=80;
      padnote[10]=84;
    }
    if (scale==5)//
    {//kletzmer

      padnote[0]=60;
      padnote[1]=61;
      padnote[2]=64;
      padnote[3]=65;
      padnote[4]=67;
      padnote[5]=68;
      padnote[6]=70;
      padnote[7]=72;
      padnote[8]=73;
      padnote[9]=76;
      padnote[10]=77;
    }

  }

  //pad calibration--early after boot
  if (firsttime<20) firsttime++; 
  if (firsttime==19) 
  {
    for (int x=0; x<12; x++)  cap_calibration[x]=chargetime[x];  
    for (int x=0; x<2; x++)
    {
      acc_calibrationx=analogRead(xpin);  //!!!!added all this
      acc_calibrationy=analogRead(ypin);
      acc_calibrationz=analogRead(zpin);
    }
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

  if (circbuffpointer<20) circbuffpointer++; //!!!set to 20 tops
  else circbuffpointer=0; //acell. axis circular buffer pointer

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
  if (chargetime[pnum]-cap_calibration[pnum]>hysteresishigh)padstate[pnum]=HIGH;
  else if (chargetime[pnum]-cap_calibration[pnum]<hysteresislow) padstate[pnum]=LOW;

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

    for (int x=0; x<20; x++)  //!!grab peaks and valleys of 100 samples of accelerometer
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
          padvolume[x]=-xaxisvalley; 
          padmode[x]=2;
        }

        if ((x==3) )
        {
          padvolume[x]=xaxispeak;  
          padmode[x]=2;
        }

        if ((x==0) )
        {
          padvolume[x]=-yaxisvalley;
          padmode[x]=2;
        }

        if ((x==7)||(x==8)||(x==9)||(x==10))
        {
          padvolume[x]=yaxispeak;
          padmode[x]=2;
        }
      }
    } 
  }//end of triggerpoint=50

  //play notes *****************************************************************
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
      padlastchannel[x]=channel;
      padlastnote[x]=padnote[x];
      usbMIDI.sendNoteOn(pitch, notevolume, channel);
      colorcalculation(notevolume, x);
      analogWrite(Rled, Rledinput);
      analogWrite(Bled, Bledinput);
      analogWrite(Gled, Gledinput); 
    }
  }

  //turn off notes **************************************************
  for (int x=0; x<11; x++)
  {
    if ((padstate[x]==LOW) && (padmode[x]==3)&& (micros()-padlasttime[x]>min_note_duration)) //need reset
    {
      padmode[x]=0;
      pitch=padlastnote[x];
      channel=padlastchannel[x];
      usbMIDI.sendNoteOff(pitch, notevolume, channel);
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
  Serial.println(padhit); 
  Serial.print("Rledinput "); 
  Serial.println(Rledinput); 
  Serial.print("Gledinput "); 
  Serial.println(Gledinput); 
  Serial.print("Bledinput "); 
  Serial.println(Bledinput); 
}

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
    Serial.print(" raw:  ");
    Serial.print(circularaccbufferx[circbuffpointer+indexer]);
    Serial.print(" ~y:");
    Serial.print(circularaccbuffery[circbuffpointer+indexer]-int(acc_calibrationy));
    Serial.print(" z:");
    Serial.print(circularaccbufferz[circbuffpointer+indexer]-int(acc_calibrationz));
    indexer++;
    if (indexer+circbuffpointer>20) indexer=-circbuffpointer; ///!!!!  
  } 
  Serial.println ("");   
  Serial.print (" zaxis peak:");
  Serial.print (zaxispeak);
  Serial.print (" xaxis peak:");
  Serial.print (xaxispeak);
  Serial.print (" yaxis peak:");
  Serial.println (yaxispeak);

  Serial.print (" zaxis valley:");
  Serial.print (zaxisvalley);
  Serial.print (" xaxis valley:");
  Serial.print (xaxisvalley);
  Serial.print (" yaxis valley:");
  Serial.println (yaxisvalley);

  Serial.print (" x cal raw:");
  Serial.println (acc_calibrationx);
  Serial.print (" x cal:  ");
  Serial.println (int(acc_calibrationx)); //!!
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

































