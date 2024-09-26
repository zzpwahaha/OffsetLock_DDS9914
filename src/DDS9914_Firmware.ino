#define DBG 0

#include <Adafruit_SSD1306.h>
#include "Menu.h"
#include "DisplayMenu.h"
#include <ClickButton.h>
//#include <Encoder_GA_E/Encoder.h>
#include <../lib/Encoder/Encoder.h>
#ifndef GRA_AND_AFCH_ENCODER_MOD3 
  #error The "Encoder" library v3 modified by GRA and AFCH must be used!
#endif
#include <SPI.h>
#include "AD9914.h"
#include <EEPROM.h>
#include <elapsedMillis.h>
#include <../lib/GParser/src/GParser.h>

#define FIRMWAREVERSION 0.84
//v0.84 22.06.2023 
//Добавленны комнады вклчения и отключения выходов
//v0.83 20.06.2023
//ускорена обработка комманд переданных через последвоательный порт 
//v0.82 06.06.2023 
//Добавление поддержки комманд через последовательный порт
//Обновлена библиотека энкодера до версии 3, для более стабильной работы
//v0.81 08.04.2021 Обновление библиотеки Encoder
//v.0.8 24.09.2020 Добавляем сохранение всех параметров в EEPROM
//v.0.7 23.09.2020 Внедряем управление DDS
//v.0.6 22.09.2020 Добавлены ограничение и проверка при установке внешней частоты тактирования
//v.0.5 21.09.2020 Почти додеално меню с настройками такитрования

//v0.84 06/22/2023
//Added commands to enable and disable outputs
//v0.83 06/20/2023
//speeded up processing of commands transmitted via the serial port
//v0.82 06/06/2023
//Added support for commands via the serial port
//Updated the encoder library to version 3, for more stable operation
//v0.81 04/08/2021 Encoder library update
//v.0.8 09/24/2020 Adding saving of all parameters to EEPROM
//v.0.7 09/23/2020 Implementing DDS control
//v.0.6 09/22/2020 Added limitation and check when setting an external frequency clocking
//v.0.5 09/21/2020 Almost perfect menu with clocking settings


//Declare the DDS object:
AD9914 DDS(CSPIN, RESETPIN, IO_UPDATEPIN, PS0PIN, PS1PIN, PS2PIN, OSKPIN);

uint32_t EXT_OSC_Freq=BASE_DDS_CORE_CLOCK;
uint32_t DDS_Core_Clock=BASE_DDS_CORE_CLOCK;
#define LOW_FREQ_LIMIT  100000 //Hz
//#define HIGH_FREQ_LIMIT  2250000000 //Hz HIGH_FREQ_LIMIT must be calcultaed as CoreClock/2
uint32_t ui32HIGH_FREQ_LIMIT=DDS_Core_Clock/2;
uint32_t ui32CurrentOutputFreq=0;

int32_t ramp0[40][6];  //ramp parameters for ch0 (Fs,Fe,R#,Rleng,dF,dt)
elapsedMicros rampDelay0;  //ramp delay timer0
uint32_t dt0;  //ch0 ramp step delay interval
uint32_t stepCount0;  //ramp step counter for ch0
uint32_t rampflg0;  //ramp start/stop flag
uint32_t rampCounter0;  //number of uploaded ramp in ch0

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire, -1, 800000UL, 800000UL);

bool isPWR_DWN = false;

#define MODE_PIN 2    //A0
//#define DOWN_PIN     A1
//#define UP_PIN       A2
#define Trig0 3  //ch0 ramp trigger pin

ClickButton modeButton (MODE_PIN, LOW, CLICKBTN_PULLUP);
//ClickButton downButton (DOWN_PIN, LOW, CLICKBTN_PULLUP);
//ClickButton upButton (UP_PIN, LOW, CLICKBTN_PULLUP);

Encoder myEnc(19, 18);

ScrollingText stTooHighFreq("Too HIGH Freq. > "+String(trunc((ui32HIGH_FREQ_LIMIT)/float(1E8))/10.0, 1)+" GHz", 10, 150);
ScrollingText stTooLowFreq("Too LOW Freq. < "+String((LOW_FREQ_LIMIT)/float(1E3), 0)+" kHz", 10, 150);

void setup() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.cp437(true);
  display.clearDisplay();

  DisplayHello();
  delay(2000);

  Serial.begin(115200);

  Serial.println(F("DDS AD9914 Arduino Shield by GRA & AFCH. (gra-afch.com)"));
  Serial.print(F("Firmware v.:"));
  Serial.println(FIRMWAREVERSION);

  modeButton.Update();

  if (modeButton.depressed == true) //if the MODE button was pressed when turning on, then erase the control flags in the EEPROM, which will restore the factory values ​​of all parameters
  {
    EEPROM.write(CLOCK_SETTINGS_FLAG_ADR, 255); //flag that force save default clock settings to EEPROM 
    EEPROM.write(MAIN_SETTINGS_FLAG_ADR, 255); //flag that force save default main settings to EEPROM 
    //EEPROM.write(MODULATION_SETTINGS_FLAG, 255); //flag that force save default modulation settings to EEPROM 
  }

  MenuLinking();
  InitMenuItems();
  
  LoadMainSettings();
  LoadClockSettings();

  ui32CurrentOutputFreq=GetFreq();

  curItem = &GHZ;

  //************ 4 del ***********
  //curItem = &ClockSrc; // 4 del
  //menuType = CORE_CLOCK_MENU; // 4 del
  //************ 4 del end ********

  modeButton.debounceTime   = 25;   // Debounce timer in ms
  modeButton.multiclickTime = 10;  // Time limit for multi clicks
  modeButton.longClickTime  = 1000; // time until "held-down clicks" register

  //********* DDS SETUP **************
  pinMode(F0, OUTPUT);
  pinMode(F1, OUTPUT);
  pinMode(F2, OUTPUT);
  pinMode(F3, OUTPUT);
  pinMode(EXT_PWR_DWN, OUTPUT);
  pinMode(SYNCIO_PIN, OUTPUT);

  pinMode(RESETPIN, OUTPUT);

  digitalWrite(F0, HIGH);
  digitalWrite(F1, LOW);
  digitalWrite(F2, LOW);
  digitalWrite(F3, LOW);

  digitalWrite(SYNCIO_PIN, LOW);
  
  digitalWrite(EXT_PWR_DWN, LOW);
  
  SPI.begin();
  SPI.setClockDivider(4);
  SPI.setDataMode(SPI_MODE0); //was 0
  SPI.setBitOrder(MSBFIRST);
  //*****************

  //DDS.initialize(2560000000); //Grisha
  DDS.initialize(DDSCoreClock.GetDDSCoreClock(), DDSCoreClock.value);
  DDS.enableProfileMode();
  DDS.enableOSK();

  //DDS.setFreq(100000000,0); 
  DDS.setFreq(GetFreq(),0);
  //DDS.setAmpdB(-10,0);
  DDS.setAmpdB(Amplitude.value, 0);
  DDS.selectProfile(0);

  pinMode(Trig0,INPUT);
  attachInterrupt(1,setRamp0,RISING);
  pinMode(OSKPIN, OUTPUT);
  digitalWrite(OSKPIN, HIGH);
}

bool MenuEditMode=false;

long oldPosition  = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()){
    processData();
  }

  if (rampflg0%2 && rampflg0/2 < rampCounter0){
    if (stepCount0 == 0){
      dt0 = ramp0[rampflg0/2][5];
      DDS.setFreq(ramp0[rampflg0/2][0], 0);
      stepCount0++;
    }
    if (rampDelay0 >= dt0 && stepCount0 <= ramp0[rampflg0/2][2]){
      rampDelay0 = rampDelay0 - dt0;
      //update ch0 DDS function goes here
      DDS.setFreq(ramp0[rampflg0/2][0]+(ramp0[rampflg0/2][4]*stepCount0), 0);
      stepCount0++;
    }
    else if (stepCount0 > ramp0[rampflg0/2][2]){
      stepCount0 = 0;
      rampflg0++;
    }
  }

  else if(rampflg0/2 >= rampCounter0){
    int curPos=0;
    curPos=myEnc.read();
    
    modeButton.Update();

    if (modeButton.clicks == 1) 
    {
      //MenuEditMode=!MenuEditMode;
      MenuEditMode=curItem->goToEditMode(MenuEditMode);
    }

    if ((modeButton.clicks == -1) && (modeButton.depressed == true)) 
    {
      curItem = &ClockSrc; 
      menuType = CORE_CLOCK_MENU; 
    }
    

    //long newPosition = curPos; //myEnc.read();///2;
    /*if (curPos != oldPosition) 
    {
      oldPosition = curPos;
      if (curPos!=0) Serial.println(curPos);
    }*/

    if (curPos>0)
    {
      //oldPosition = newPosition;
      switch(MenuEditMode)
      {
        case true: 
          curItem->incValue(curPos); 
          ui32CurrentOutputFreq=GetFreq(); 
          DDS.setFreq(ui32CurrentOutputFreq,0); 
          DDS.setAmpdB(Amplitude.value * -1, 0); 
          if (menuType == MAIN_MENU) SaveMainSettings();
        break;
        case false: curItem=curItem->moveToNextItem(); break;
      }
    }
    
    if (curPos<0)
    {
      switch(MenuEditMode)

      {
        case true: 
          curItem->decValue(curPos);
          ui32CurrentOutputFreq=GetFreq();
          DDS.setFreq(ui32CurrentOutputFreq,0);
          DDS.setAmpdB(Amplitude.value * -1, 0);
          if (menuType == MAIN_MENU) SaveMainSettings();
        break;
        case false: curItem=curItem->moveToPrevItem(); break;
      }
    }

    DisplayMenu(menuType); 
  }
}

uint32_t GetFreq()
{
  return GHZ.value * 1000000000UL + MHZ.value * 1000000UL + KHZ.value * 1000UL + HZ.value;
}
int8_t FreqInRange() //1 - Higher, -1 - lower, 0 - in range
{
  //ui32HIGH_FREQ_LIMIT=DDS_Core_Clock/2;
  if (ui32CurrentOutputFreq>ui32HIGH_FREQ_LIMIT) return 1;
  if (ui32CurrentOutputFreq<LOW_FREQ_LIMIT) return -1;
  return 0;
}

//process incoming serial data and check if data are valid and write default into EEPROM
//test data
//(0,100000000,200000000,5,10000)e
void processData(){
  float para[5];
  uint32_t paraint0[40][5];
  uint32_t rampCount0 = 0;
  uint32_t value = 0;
  char startMarker = '(';  //start marker for each set of data
  char endMarker = ')';  //end marker for each set of data
  char endendMarker = 'e';  //end marker for the whole frame of data
  char trigMarker = 't'; //software trigger marker type "te" to software trigger
  char rc = 0;
  bool error = true;   //error flag for input parameter
  bool triggered = false; //software triggered?
  elapsedMillis serialTimeout;  //timeout for serial receive in case of faliur
  
  //process all serial data until the end marker is received
  serialTimeout = 0;
  while (rc != endendMarker && serialTimeout != 10000){
    if (Serial.available()){
      rc = Serial.read();
    }
    if (rc == startMarker){
      //Take the first 5 float from serial command
      for (int i=0; i<5; i++){
        para[i] = Serial.parseFloat();
      }
      //convert float into int and transfer to parameter buffer also check the data validity
      if ((int)para[0] == 0){
        paraint0[rampCount0][0] = para[0];    //ch
        paraint0[rampCount0][1] = para[1]*1000000;    //freq start in Hz
        paraint0[rampCount0][2] = para[2]*1000000;    //freq stop in Hz
        paraint0[rampCount0][3] = para[3];    //step number
        paraint0[rampCount0][4] = para[4]*1000;    //step length time in us
        error = dataCheck(paraint0[rampCount0]);
        rampCount0++;
      }
    }
    else if (rc == trigMarker){
      triggered = true;
    }
  }
  //check if there are errors in the data or the serial communication timeout is reached
  if (error == false && serialTimeout < 10000 && triggered == false){
    //transfer local buffered parameter to global parameter in the main program
    for (int i=0; i<rampCount0; i++){
      for(int j=0; j<4; j++){
        ramp0[i][j] = paraint0[i][j+1];
      }
      ramp0[i][4] = (ramp0[i][1]-ramp0[i][0])/ramp0[i][2];
      ramp0[i][5] = ramp0[i][3]/ramp0[i][2];
    }
    rampCounter0 = rampCount0;
    rampflg0 = 0;
    //Write frequency to eeprom
    DDS.setFreq(ramp0[0][0], 0);

    Serial.print("Parameter programmed in :");
    Serial.print(serialTimeout);
    Serial.println(" ms");
    Serial.print("f");
  }
  else if (triggered == true){
    //trigger ch0 ramp
    rampDelay0 = 0;
    rampflg0++;
    stepCount0 = 0;
    Serial.println("Software triggered!!");
    Serial.print("f");
    triggered = false;
  }
  else{
    Serial.println("Error in paramter!! Nothing changed.");
    Serial.print("f");
  }
  while (Serial.available()){
    Serial.read();    
  }
}

//check if input data are in range and valid for ch0
bool dataCheck(uint32_t par[5]){
  bool errorflg = true;
  if (par[0] == 0 ){
    if (par[1] >= 100000 && par[1] <= 1000000000 && par[2] >= 100000 && par[2] <= 1000000000){
      if (par[3] > 0 && par[4]/par[3] >= 200){
        errorflg = false;
      }
      else{
        errorflg = true;
      }
    }
    else{
      errorflg = true;
    }
  }
  else{
    errorflg = true;
  }
  return errorflg;
}

//ISR for ch0 ramp when the trigger is detected
void setRamp0(){
  rampDelay0 = 0;
  rampflg0++;
  stepCount0 = 0;
}