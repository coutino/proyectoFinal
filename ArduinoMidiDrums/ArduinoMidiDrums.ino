/*
 * Copyright (c) 2015 Evan Kale
 * Email: EvanKale91@gmail.com
 * Website: www.ISeeDeadPixel.com
 *          www.evankale.blogspot.ca
 *
 * This file is part of ArduinoMidiDrums.
 *
 * ArduinoMidiDrums is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 
 /*
  * Modify by coutino@gmail.com
  * 2019/0705
  * https://www.koutino.com
  */


// ---------------------------------------LCD
// include the library code:
#include <LiquidCrystal.h>


// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
boolean showMessageLCD;


// ---------------------------------------PADS
//Piezo defines
#define NUM_PIEZOS 2
#define SNARE_THRESHOLD 30     //anything < TRIGGER_THRESHOLD is treated as 0
#define LTOM_THRESHOLD 30
#define RTOM_THRESHOLD 181
#define LCYM_THRESHOLD 100
#define RCYM_THRESHOLD 100
#define KICK_THRESHOLD 30
#define MAX_THRESHOLD 200 // must be grather than all up define threshold
#define START_SLOT A0     //first analog slot of piezos

//MIDI note defines for each trigger
#define SNARE_NOTE 37
#define LTOM_NOTE 40
#define RTOM_NOTE 72
#define LCYM_NOTE 73
#define RCYM_NOTE 74
#define KICK_NOTE 75

//MIDI defines
#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define MAX_MIDI_VELOCITY 127



//Program defines
//ALL TIME MEASURED IN MILLISECONDS
#define SIGNAL_BUFFER_SIZE 100
#define PEAK_BUFFER_SIZE 30
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50


// pad struct
struct padStruct{
  //map that holds the mux slots of the piezos
  byte slot;
  //map that holds the respective note to each piezo
  byte miniNote;
  //map that holds the respective threshold to each piezo
  byte threshold;
  
  //Ring buffers to store analog signal and peaks
  short currentSignalIndex;
  short currentPeakIndex;
  unsigned short signalBuffer[SIGNAL_BUFFER_SIZE];
  unsigned short peakBuffer[PEAK_BUFFER_SIZE];
  
  boolean noteReady;
  unsigned short noteReadyVelocity;
  boolean isLastPeakZeroed;
  
  unsigned long lastPeakTime;
  unsigned long lastNoteTime;
};

typedef struct padStruct Pad;
Pad padNote[NUM_PIEZOS];

byte currentMidiNoteSlot;
unsigned short currentMidiChannel;

byte resetAllValue;
boolean resetAllValueConfirm;

byte saveAllValue;
boolean saveAllValueConfirm;


boolean isChangeDataPadNote;





// ---------------------------------------CONFIG


#define NUMBER_OPTIONS 9
#define VALUE_INCREMENT 0
#define VALUE_DECREMENT 1

//MIDI baud rate for MIDI
#define RATE_MIDI 0
//MIDI baud rate for USM
#define RATE_USB 1


struct buttonStruct{
  byte id;
  byte state;
  byte lastState;
  byte value;
};

typedef struct buttonStruct Button;

Button buttonSet;
Button buttonPrev;
Button buttonNext;

// ------ Language in a future
//byte language;
/*enum{
  ES=0,
  EN
};*/

/*
enum{
M_WELCOME=0,
M_ENJOY_EDRUMKIT,
M_VERIFY_MEMORY,
M_GETTING_DATA,
M_NO_DATA_GET,
M_MIDI_NOTE_lowercase,
M_MIDI_NOTE_uppercase,
M_YES,
M_NO,
M_READY_PLAY,
M_MIDI_NOTE_CHANGES,
M_THRESHOLD,
M_MIDI_CHANNEL,
M_SAVE_NEW_DATA,
M_SAVING_WAIT,
M_RESET_ALL,
M_RESETING_WAIT

};
*/

byte currentSerialMode;

//String lanMessage[17];




// ---------------------------------------EEPROM and backup
#include <EEPROM.h>
#include <Arduino.h>

struct bckPadNoteStruct{
  //map that holds the mux slots of the piezos
  byte slot;
  //map that holds the respective note to each piezo
  byte miniNote;
  //map that holds the respective threshold to each piezo
  byte threshold;
  
};


typedef struct bckPadNoteStruct BckPadNote;


struct eepromStruct{
  boolean isDataSave;
  byte midiChannel;
  byte serialMode;
  BckPadNote bckPadNote[NUM_PIEZOS];
};
typedef struct eepromStruct EepromStruct;
EepromStruct eepromStructMemory;



int EEPROM_writeAnything(int ee, const struct eepromStruct & value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          EEPROM.write(ee++, *p++);
    return i;
}

int EEPROM_readAnything(int ee, struct eepromStruct& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
          *p++ = EEPROM.read(ee++);
    return i;
}






// --------------------------------------- SETUP
void setup()
{

  setupLCD();
  printWelcome();
  setupCONFIG();
  setupPADS();
  setupMEMORY();
  
  
}


void printWelcome(){
  // Print a message to the LCD.
  lcd.print("Bienvenido");
  delay(1000);
  lcd.clear();
  lcd.print("Disfruta E-DrumKit");
  delay(2000);
  
}

void setSerialMode(byte modeMIDIUSB){
  switch(modeMIDIUSB){
    case RATE_MIDI:
          Serial.begin(31250);
          break;
    case RATE_USB:
          Serial.begin(115200);
          break;
     default:
         Serial.begin(31250);
         break;
  }

}


void setupLCD(){
 /* 
  lanMessage[0] = "Bienvenido";
  lanMessage[1] = "Disfruta EDrumK";
  lanMessage[2] = "Memoria ...";
  lanMessage[3] = "Obteniendo datos";
  lanMessage[4] = "Sin datos";
  lanMessage[5] = "Nota midi";
  lanMessage[6] = "NOTA MIDI";
  lanMessage[7] = "si";
  lanMessage[8] = "no";
  lanMessage[9] = "LISTO PARA TOCA";
  lanMessage[10] = "CAMBIO NOTA MID";
  lanMessage[11] = "Limite sensor";
  lanMessage[12] = "CANAL MIDI";
  lanMessage[13] = "Salvar cambios?";
  lanMessage[14] = "SALVANDO, ESPERE";
  lanMessage[15] = "¿REINICIAR TODO?";
  lanMessage[16] = "REINICIANDO ...";


    "Welcome",
    "Enjoy E-DrumKit",
    "Verify memory...",
    "Getting data...",
    "No data to get",
    "Midi note",
    "MIDI NOTE",
    "yes",
    "no",
    "READY TO PLAY",
    "MIDI NOTE CHANGES",
    "Threshold",
    "MIDI CHANNEL",
    "SAVE NEW DATA",
    "SAVING, WAIT...",
    "RESET ALL?",
    "RESETING,W A0IT..."
*/
 
 
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  showMessageLCD = false;

}


void setupCONFIG(){

  buttonSet.id = 7;
  buttonPrev.id = 13;
  buttonNext.id = 8;
  pinMode( buttonSet.id, INPUT_PULLUP);
  pinMode( buttonPrev.id, INPUT_PULLUP);
  pinMode( buttonNext.id, INPUT_PULLUP);
  
  buttonPrev.state = 0;
  buttonPrev.lastState = 1;
  buttonNext.state = 0;
  buttonNext.lastState = 1;

}


void setupPADS()
{
  
  currentSerialMode = 0;
  setSerialMode(currentSerialMode);
  
  
  //initialize globals
  for(byte i=0; i<NUM_PIEZOS; ++i)
  {
    padNote[i].currentSignalIndex = 0;
    padNote[i].currentPeakIndex = 0;
    memset(padNote[i].signalBuffer,0,sizeof(padNote[i].signalBuffer));
    memset(padNote[i].peakBuffer,0,sizeof(padNote[i].peakBuffer));
    padNote[i].noteReady = false;
    padNote[i].noteReadyVelocity = 0;
    padNote[i].isLastPeakZeroed = true;
    padNote[i].lastPeakTime = 0;
    padNote[i].lastNoteTime = 0;    
    padNote[i].slot = START_SLOT + i;
  }
  
  padNote[0].threshold = KICK_THRESHOLD;
  padNote[1].threshold = RTOM_THRESHOLD;
  //padNote[2].threshold = RCYM_THRESHOLD;
  //padNote[3].threshold = LCYM_THRESHOLD;
  //padNote[4].threshold = SNARE_THRESHOLD;
  //padNote[5].threshold = LTOM_THRESHOLD;  
  
  padNote[0].miniNote = KICK_NOTE;
  padNote[1].miniNote = RTOM_NOTE;
  //padNote[2].miniNote = RCYM_NOTE;
  //padNote[3].miniNote = LCYM_NOTE;
  //padNote[4].miniNote = SNARE_NOTE;
  //padNote[5].miniNote = LTOM_NOTE;  
  currentMidiChannel = 10;
  resetAllValue = 0;
  resetAllValueConfirm = false;
  saveAllValue = 0;
  saveAllValueConfirm = false;
  isChangeDataPadNote = false;
  //language = 1;
}


void setupMEMORY(){
  
  lcd.clear();
  lcd.print("Memoria...");
  delay(1000);
  EEPROM_readAnything(0, eepromStructMemory);
  lcd.clear();
  if(eepromStructMemory.isDataSave){
    
    lcd.print("Obteniendo datos");
    copyDataFromEepronToPadNote();
    delay(1000);
  }else{
    lcd.print("Sin datos");
    copyDataFromPadNoteToEeprom();
    delay(1000);
  }
  

}


void copyDataFromEepronToPadNote(){
  currentMidiChannel = eepromStructMemory.midiChannel;
  currentSerialMode = eepromStructMemory.serialMode;
  for(byte i=0; i<NUM_PIEZOS; ++i){
    padNote[i].slot = eepromStructMemory.bckPadNote[i].slot;
    padNote[i].miniNote = eepromStructMemory.bckPadNote[i].miniNote;
    padNote[i].threshold = eepromStructMemory.bckPadNote[i].threshold;
  }
}

void copyDataFromPadNoteToEeprom(){
  eepromStructMemory.midiChannel = currentMidiChannel;
  eepromStructMemory.serialMode = currentSerialMode;
  for(byte i=0; i<NUM_PIEZOS; ++i){
    eepromStructMemory.bckPadNote[i].slot = padNote[i].slot;
    eepromStructMemory.bckPadNote[i].miniNote = padNote[i].miniNote;
    eepromStructMemory.bckPadNote[i].threshold = padNote[i].threshold;
  }
}



// MAIN
void loop()
{
  actionCONFIG();
  actionPADS();
}


void printMessageNoteLCD(String message){
  if(showMessageLCD){
    showMessageLCD = false;
    lcd.clear();
    lcd.print(message);
    lcd.setCursor(0, 1);
    String m = "Nota midi " + String(padNote[currentMidiNoteSlot].miniNote);
    lcd.print(m);
  }
}

void printMessageThresholdLCD(String message){
  if(showMessageLCD){
    showMessageLCD = false;
    String mn = "NOTA MIDI " + String(padNote[currentMidiNoteSlot].miniNote);
    lcd.clear();
    lcd.print(mn);
    lcd.setCursor(0, 1);
    message = message +" "+ String(padNote[currentMidiNoteSlot].threshold);
    lcd.print(message);
    
  }
}

void printMessageChannelLCD(String message){
  if(showMessageLCD){
    showMessageLCD = false;
    lcd.clear();
    lcd.print(message);
    lcd.setCursor(0, 1);
    message = " "+ String(currentMidiChannel+1);
    lcd.print(message);
    
  }
}

void printMessageResetAllValuesLCD(String message,byte numYesNo){
  if(showMessageLCD){
    showMessageLCD = false;
    lcd.clear();
    lcd.print(message);
    lcd.setCursor(0, 1);
    message = numYesNo==1 ? "si":"no";
    lcd.print(message);
  }
}

void printMessageSerialModeLCD(String message,byte midiUSB){
  if(showMessageLCD){
    showMessageLCD = false;
    lcd.clear();
    lcd.print(message);
    lcd.setCursor(0, 1);
    message = midiUSB==1 ? "puerto usb":"puerto midi";
    lcd.print(message);
  }
}

void printMessageLCD(String message){
  if(showMessageLCD){
    showMessageLCD = false;
    lcd.clear();
    //lcd.setCursor(0, 0);
    lcd.print(message);
  }
}

boolean isButtonPress1(struct buttonStruct *button, byte digitanButtonWhatStateToCheck, byte incrementDecrement, byte module ){
  
  button->state = digitalRead(button->id);
  if(button->state != button->lastState){
    if(button->state == digitanButtonWhatStateToCheck){
      //Serial.println("Cambio el estado del boton 1");
      
      
      switch(incrementDecrement){
        case VALUE_INCREMENT:
            button->value = ++button->value % module;
            break;
        case VALUE_DECREMENT:
            if(button->value == 0){
              button->value = module-1;
            }else{
              button->value = --button->value % module;
            }
            
            break;
      }
      //abs(button->value);
      //Serial.println(button->value);
    }
    button->lastState = button->state;
    showMessageLCD = true;
    return true;
  }else{
    return false;
  }
  
}


void tmpMessage(String m){
  lcd.clear();
            lcd.print(m);
            delay(3000);
}

void actionCONFIG(){
  
  byte valueAuxModify;
  String mm;
  String mmmm;

    if(isButtonPress1(&buttonSet, LOW, VALUE_INCREMENT, NUMBER_OPTIONS)){

        
        //Serial.println("Cambio el estado del boton");
        //Serial.println(digitalButonOPTION);
        
      
    }else{
      
      switch(buttonSet.value){
          case 0:
          
            // Ready to  play e-drum

            //printMessageNoteLCD("play");
            printMessageNoteLCD("LISTO PARA TOCAR");
            
            break;
          case 1:
            printMessageNoteLCD("NOTA MIDI?");
            
            
            valueAuxModify = padNote[currentMidiNoteSlot].miniNote;

            buttonPrev.value = valueAuxModify;
            if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, 127)){
              valueAuxModify = buttonPrev.value;
              isChangeDataPadNote = true;
              //tmpMessage("entry 1");
            }
            
            buttonNext.value = valueAuxModify;
            if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, 127)){
              valueAuxModify = buttonNext.value;
              isChangeDataPadNote = true;
              //tmpMessage("entry 2");
            }
            padNote[currentMidiNoteSlot].miniNote = valueAuxModify;
            
           
            
            break;
            
          case 2:
          
            printMessageThresholdLCD("Lim. sensor");
            
            
            valueAuxModify = padNote[currentMidiNoteSlot].threshold;
            
            buttonPrev.value = valueAuxModify;
            if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, MAX_THRESHOLD)){
              valueAuxModify = buttonPrev.value;
              isChangeDataPadNote = true;
              //tmpMessage("entry 3");
            }
            
            buttonNext.value = valueAuxModify;
            if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, MAX_THRESHOLD)){
              valueAuxModify = buttonNext.value;
              isChangeDataPadNote = true;
              //tmpMessage("entry 4");
            }
            padNote[currentMidiNoteSlot].threshold = valueAuxModify;
            
            
            break;
            
          
          
              
          
          case 3:
          // CHANNEL
            printMessageChannelLCD("CANAL MIDI?");
            
            
            valueAuxModify = currentMidiChannel;
            buttonPrev.value = valueAuxModify;
            if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, 16)){
              valueAuxModify = buttonPrev.value;
              isChangeDataPadNote = true;
            }
            
            buttonNext.value = valueAuxModify;
            if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, 16)){
              valueAuxModify = buttonNext.value;
              isChangeDataPadNote = true;
            }
            currentMidiChannel = valueAuxModify;
  
            break;
            
          case 4:
            // SERIAL PORT MODE
             mmmm = "ant" + String(buttonSet.value);
                //          tmpMessage(mmmm);
              valueAuxModify = currentSerialMode;
              printMessageSerialModeLCD("PUERTO?",currentSerialMode);
              //delay(3000);
              buttonPrev.value = valueAuxModify;
              if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, 2)){
                valueAuxModify = buttonPrev.value;
                setSerialMode(valueAuxModify);
                isChangeDataPadNote = true;
              }
              
              buttonNext.value = valueAuxModify;
              if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, 2)){
                valueAuxModify = buttonNext.value;
                setSerialMode(valueAuxModify);
                isChangeDataPadNote = true;
              }
              currentSerialMode = valueAuxModify;
              mmmm = "des" + String(buttonSet.value);
              //tmpMessage(mmmm);
              break;
            
          case 5:
            // SAVE ALL VALUES
            if(isChangeDataPadNote){
              
              valueAuxModify = saveAllValue;
              printMessageResetAllValuesLCD("SALVAR CAMBIOS?",saveAllValue);
              buttonPrev.value = valueAuxModify;
              if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, 2)){
                valueAuxModify = buttonPrev.value;
              }
              
              buttonNext.value = valueAuxModify;
              if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, 2)){
                valueAuxModify = buttonNext.value;
              }
              saveAllValue = valueAuxModify;
              saveAllValueConfirm = valueAuxModify == 1 ? true:false;
              
            }else{
              buttonSet.value = 7;
            }
            break;
            
          case 6:
            isChangeDataPadNote = false;
            if(saveAllValueConfirm){
              printMessageLCD("SALVANDO, ESPERE...");
              //setupPADS();
              copyDataFromPadNoteToEeprom();
              eepromStructMemory.isDataSave = true;
              EEPROM_writeAnything(0, eepromStructMemory);
  
              saveAllValueConfirm = false;
              saveAllValue = 0;
              delay(1000);
              buttonSet.value = 0;
            }else{
              copyDataFromEepronToPadNote();
              setSerialMode(currentSerialMode);
              buttonSet.value = 7;
            }
            break;
          
          case 7:
            isChangeDataPadNote = false;
            // RESET ALL VALUES
            valueAuxModify = resetAllValue;
            printMessageResetAllValuesLCD("REINICIAR TODO?",resetAllValue);
            buttonPrev.value = valueAuxModify;
            if(isButtonPress1(&buttonPrev, HIGH, VALUE_DECREMENT, 2)){
              valueAuxModify = buttonPrev.value;
            }
            
            buttonNext.value = valueAuxModify;
            if(isButtonPress1(&buttonNext, HIGH, VALUE_INCREMENT, 2)){
              valueAuxModify = buttonNext.value;
            }
            resetAllValue = valueAuxModify;
            resetAllValueConfirm = valueAuxModify == 1 ? true:false;
            break;
            
          
          case 8:
            // CONFIRM RESET ALL VALUES
            if(resetAllValueConfirm){
              printMessageLCD("REINICIANDO,ESPERE...");
              setupPADS();
              resetAllValueConfirm = false;
              resetAllValue = 0;
              delay(2000);
            }
            buttonSet.value = 0;
            break;
            
        }
    }


  
}


void actionPADS(){
  //Serial.println("--------PADS");
    unsigned long currentTime = millis();
  
  for(byte i=0; i<NUM_PIEZOS; ++i)
  {
    //get a new signal from analog read
    unsigned short newSignal = analogRead(padNote[i].slot);
    padNote[i].signalBuffer[padNote[i].currentSignalIndex] = newSignal;
    //Serial.println(newSignal);
    
    //if new signal is 0
    if(newSignal < padNote[i].threshold)
    {
      if(!padNote[i].isLastPeakZeroed && (currentTime - padNote[i].lastPeakTime) > MAX_TIME_BETWEEN_PEAKS)
      {
        recordNewPeak(i,0);
      }
      else
      {
        //get previous signal
        short prevSignalIndex = padNote[i].currentSignalIndex-1;
        if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;        
        unsigned short prevSignal = padNote[i].signalBuffer[prevSignalIndex];
        
        unsigned short newPeak = 0;
        
        //find the wave peak if previous signal was not 0 by going
        //through previous signal values until another 0 is reached
        while(prevSignal >= padNote[i].threshold)
        {
          if(padNote[i].signalBuffer[prevSignalIndex] > newPeak)
          {
            newPeak = padNote[i].signalBuffer[prevSignalIndex];        
          }
          
          //decrement previous signal index, and get previous signal
          prevSignalIndex--;
          if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
          prevSignal = padNote[i].signalBuffer[prevSignalIndex];
        }
        
        if(newPeak > 0)
        {
          recordNewPeak(i, newPeak);
        }
      }
  
    }
        
    padNote[i].currentSignalIndex++;
    if(padNote[i].currentSignalIndex == SIGNAL_BUFFER_SIZE) padNote[i].currentSignalIndex = 0;
  }
}
void recordNewPeak(byte slot, short newPeak)
{
  padNote[slot].isLastPeakZeroed = (newPeak == 0);
  
  unsigned long currentTime = millis();
  padNote[slot].lastPeakTime = currentTime;
  
  //new peak recorded (newPeak)
  padNote[slot].peakBuffer[padNote[slot].currentPeakIndex] = newPeak;
  
  //1 of 3 cases can happen:
  // 1) note ready - if new peak >= previous peak
  // 2) note fire - if new peak < previous peak and previous peak was a note ready
  // 3) no note - if new peak < previous peak and previous peak was NOT note ready
  
  //get previous peak
  short prevPeakIndex = padNote[slot].currentPeakIndex-1;
  if(prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE-1;        
  unsigned short prevPeak = padNote[slot].peakBuffer[prevPeakIndex];
   
  if(newPeak > prevPeak && (currentTime - padNote[slot].lastNoteTime)>MIN_TIME_BETWEEN_NOTES)
  {
    padNote[slot].noteReady = true;
    if(newPeak > padNote[slot].noteReadyVelocity)
      padNote[slot].noteReadyVelocity = newPeak;
  }
  else if(newPeak < prevPeak && padNote[slot].noteReady)
  {
    noteFire(slot, currentMidiChannel, padNote[slot].miniNote, padNote[slot].noteReadyVelocity);
    padNote[slot].noteReady = false;
    padNote[slot].noteReadyVelocity = 0;
    padNote[slot].lastNoteTime = currentTime;
  }
  
  padNote[slot].currentPeakIndex++;
  if(padNote[slot].currentPeakIndex == PEAK_BUFFER_SIZE) padNote[slot].currentPeakIndex = 0;  
}

void noteFire(byte slot, byte channel, byte note, unsigned short velocity)
{
  // time to shot MIDI NOTE
  if(velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;
  currentMidiNoteSlot = slot;
  showMessageLCD = true;
  midiNoteOn(channel, note, velocity);
  midiNoteOff(channel, note, velocity);
}

void midiNoteOn(byte channel, byte note, byte midiVelocity)
{
  channel =  NOTE_ON_CMD + channel;
  Serial.write(channel);
  Serial.write(note);
  Serial.write(midiVelocity);
  //Serial.println(note);
  //lcd.print(m);
}

void midiNoteOff(byte channel, byte note, byte midiVelocity)
{
  channel =  NOTE_OFF_CMD + channel;
  Serial.write(channel);
  Serial.write(note);
  Serial.write(midiVelocity);
  
}


