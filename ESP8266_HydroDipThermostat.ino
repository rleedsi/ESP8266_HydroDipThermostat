// HydroDipThermostat.ino
// Copyright (C) 2016 Richard Lee
//
// Hydro Dip tank termostat using esp8266 (ESP-12E)/2.8" TFT Touch display
// 2016Feb07 Created -- RL
// 2016Feb07 Notes:
//           1. Need to be able to store setpoints (runarr, idle) in nv ram
//           2. Speed - need to speed display up
// 2016Feb13 Reorganizing buttons to optimize displays -- RL
// 2016May01 After building hardwired board and adding both temp sensors,
//           the tank sensor shows up as index 1; modifying to support this -- RL
// 2016May01 Adding ds2431 eeprom memory support -- RL


#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Ucglib.h>  // Required
#include <XPT2046.h>

#define PROGNAME "Hydro dip thermostat"
#define COPYRIGHT "Copyright (C) R.Lee"
#define VERSION "0.8"
#define VERDATE "2016May01"

#define OUTPUTPIN 9 // 2016Feb16 write to pin 9 apparently crashing - trying swap of 9&10 - nope, won't even run with 9&10 swapped
// 2016Feb19 Even ESP-12E won't support write to pins 9&10 without a hack - see
//   http://smarpl.com/content/esp8266-esp-201-module-freeing-gpio9-and-gpio10
//   (I just lifted pins 3 & 7 and wired them to pin 8, then edited 
//    ./Arduino/esp8266com/esp8266/boards.txt, updating
//    modemcu.build.flash_mode=qio to =dio which seemed to work just fine (also see
//    https://github.com/esp8266/Arduino/issues/292 -- RL)
//#define OUTPUTPIN 10 
#define TEMP_HYSTERESIS 0.1

// Screen positions and sizes
// Version
#define VERSIONX 10
#define VERSIONY 50

// Buttons
// Common
// Button size
#define BUTTONW 100
#define BUTTONH 50
// Main Screen
#define RUNX 5
#define RUNY 5
#define IDLEX 5
#define IDLEY 65
#define OFFX 5
#define OFFY 125
#define TIMEBX 5
#define TIMEBY 185
#define TEMPBX 5
#define TEMPBY 185

// Time Screen
#define ONFORX 5
#define ONFORY 5
#define ONATX 5
#define ONATY 65
//#define TIMETEMPX 5
//#define TIMETEMPY 125

// Status position
#define STATUSX 110
#define STATUSY 235

// Arrows
#define UPARROWX 145
#define UPARROWY 150
#define DNARROWX 220
#define DNARROWY 150
#define ARROWW 30
#define ARROWH 30

// Temp & Setpoint
#define TANKTEMPX 145
#define TANKTEMPY 10
#define AMBTEMPX 110
#define AMBTEMPY 205
#define SETPTX 145
#define SETPTY 80
#define TANKTEMPW 120
#define TANKTEMPH 60
#define AMBTEMPW 150
#define AMBTEMPH 50

// Globals
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 2 , /*cs=*/ 4, /*reset=*/ 5);
XPT2046 touch(/*cs=*/ 16, /*irq=*/ 0);

// Onewire support
OneWire oneWire(10); // Pin # // 2016Feb16 write to pin 9 apparently crashing - trying swap of 9&10
//OneWire oneWire(9); // Pin #
DallasTemperature ts(&oneWire);

float fTankTemp;
float fAmbientTemp;
float fOffSetpoint;
float fIdleSetpoint;
float fRunSetpoint;
char szTemperature[40];
byte tsTankAddr[8];     // address of 18b20 for tank
byte tsAmbientAddr[8];  // address of 18b20 for ambient
byte ds2431[8];         // address of DS2431 1024 bit EEPROM
uint16_t bSpUpd;

enum runStates
{
  Off,
  Idle,
  Run
};
runStates runMode;

// ftoa5()
// Convert float to string with arbitrary precision, leading space padded to 5 chars min
// (primarily for room temperatures)
// 2016Feb07 Created -- RL
char *ftoa5(char *dest, double f, int precision)
{
 long p[] = {0,10,100,1000,10000,100000,1000000,10000000,100000000};

 char clBuf[40];
 char *a = clBuf;
 long lInt = (long)f;
 itoa(lInt, a, 10);
 while (*a != '\0') a++;
 *a++ = '.';
 long lDecimal = abs((long)((f - lInt) * p[precision]));
 itoa(lDecimal, a, 10);
 int bLen=strlen(clBuf);
 int i=0;
 if(bLen < 5)
 {
  while(i < 5-bLen)
  {
    dest[i++] = ' '; // pad front of string to 5 characters
  }
 }
 strcpy(&dest[i], clBuf);
 return dest;
 
} // ftoa5()

// displayVersion()
// Display current version
void displayVersion()
{
  char msg[80];
  ucg.setColor(0, 255, 255);
  ucg.setPrintPos(VERSIONX, VERSIONY);
  ucg.print(PROGNAME);
  Serial.println(PROGNAME);
  ucg.setPrintPos(VERSIONX, VERSIONY+30);
  ucg.print(COPYRIGHT);
  Serial.println(COPYRIGHT);
  ucg.setPrintPos(VERSIONX, VERSIONY+60);
  strcpy(msg, "Version: ");
  strcat(msg, VERSION);
  ucg.print(msg);
  Serial.println(msg);
  ucg.setPrintPos(VERSIONX, VERSIONY+90);
  strcpy(msg, "Date: ");
  strcat(msg, VERDATE);
  ucg.print(msg);
  Serial.println(msg);

  delay(2000);

  ucg.clearScreen();

} // displayVersion()

// findDS2431()
// Input: byte *address: pointer to 8 byte array to contain found address
// Returns: int:
//          -1 if eeprom was found but CRC is invalid
//          1 if eeprom was found
//          0 if no eeprom was found
// History:
//  2016May01 Created -- RL
bool findDS2431(byte *address)
{
  int i;
  int iRVal=0;
  while(oneWire.search(address) && iRVal != 1)
  {
    Serial.print("ADDR= ");
    for(i = 0; i < 8; i++)
    {
      Serial.print(address[i], HEX);
      Serial.print(" ");
    }
  
    if ( OneWire::crc8( address, 7) != address[7])
    {
       Serial.print("CRC is not valid, address is corrupted\n");
       iRVal = -1;
    }
   
    if ( address[0] != 0x2D)
    {
       Serial.print("Device is not a 1-wire Eeprom.\n");
    }
    else
    {
      if(iRVal == 0)  // if bad CRC code is not set...
      {
        iRVal = 1;    // ...terminate the loop
      }

    }
    Serial.println();
 }
 return(iRVal);
 
} // findDS2431()

// =============================================
// DS2431 utilities
// =============================================
void WriteReadScratchPad(byte *addr, byte TA1, byte TA2, byte *data)
{
 int i;
 oneWire.reset();
 oneWire.select(addr);
 oneWire.write(0x0F,1);  // Write ScratchPad
 oneWire.write(TA1,1);
 oneWire.write(TA2,1);
 for ( i = 0; i < 8; i++)
   oneWire.write(data[i],1);  
 
 oneWire.reset();
 oneWire.select(addr);    
 oneWire.write(0xAA);         // Read Scratchpad
 
 for ( i = 0; i < 13; i++)    
   data[i] = oneWire.read();
   
} // WriteReadScratchPad()

void CopyScratchPad(byte* addr, byte *data)
{
 oneWire.reset();
 oneWire.select(addr);
 oneWire.write(0x55,1);  // Copy ScratchPad
 oneWire.write(data[0],1);
 oneWire.write(data[1],1);  // Send TA1 TA2 and ES for copy authorization
 oneWire.write(data[2],1);
 delay(25); // Waiting for copy completion
 //Serial.print("Copy done!\n");
 
} // CopyScratchPad()

void WriteRow(byte* addr, byte row, byte* buffer)
{
 int i;
 if (row < 0 || row > 15) // There are 16 rows of 8 bytes in the main memory
   return;                // The remaining are for the 64 bits register page
   
 WriteReadScratchPad(addr, row*8, 0x00, buffer);
 
 //  Print result of the ReadScratchPad
 for ( i = 0; i < 13; i++)
 {
   Serial.print(buffer[i], HEX);
   Serial.print(" ");
 }
 //
 CopyScratchPad(addr, buffer);
 
} // WriteRow()

void ReadRow(byte *addr, byte row, byte *buffer)
{
  int i;
  unsigned int uRow = row;
  uRow *= 8;
  oneWire.reset();
  oneWire.select(addr);
  oneWire.write(0xF0,1);        // Read Memory
  oneWire.write((byte)uRow,1);  // Read Offset 0000h
  oneWire.write((byte)uRow >> 8,1);
   
  for ( i = 0; i < 8; i++) //whole mem is 144
  {
    buffer[i] = oneWire.read();
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
 }
 Serial.println();

} // ReadRow()

// updateEEPromValues()
void updateEEPromValues()
{
  byte eeBuf[13];

  *(float*)(&eeBuf[0]) = fIdleSetpoint;
  *(float*)(&eeBuf[4]) = fRunSetpoint;
  WriteRow(ds2431, 0, eeBuf);
  
} // updateEEPromValues()


// readTemp()
// tsAddr: char[8] address of temperature to read
// Read temperature and return value as a float in degrees Fahrenheit
// Returns -255.0 on error
// 2016Feb07 Created -- RL
float readTemp(byte tsAddr[8])
{
  float fRetVal = -255.0;
  ts.requestTemperatures();
  fRetVal = ts.getTempF(tsAddr);
  return(fRetVal);
} // readTemp();

// Draw buttons
void drawRunButton()
{
  drawButton(/*x*/RUNX, /*y*/RUNY, /*r*/255, /*g*/0, /*b*/0, /*text*/"Run");
}
void drawIdleButton()
{
 drawButton(/*x*/IDLEX, /*y*/IDLEY, /*r*/0, /*g*/255, /*b*/0, /*text*/"Idle");
}
void drawOffButton()
{
  drawButton(/*x*/OFFX, /*y*/OFFY, /*r*/0, /*g*/0, /*b*/255, /*text*/"Off");
}
void drawTimeButton()
{
  drawButton(/*x*/TIMEBX, /*y*/TIMEBY, /*r*/0, /*g*/128, /*b*/255, /*text*/"Time");
}
void drawTempButton()
{
  drawButton(/*x*/TEMPBX, /*y*/TEMPBY, /*r*/0, /*g*/128, /*b*/255, /*text*/"Temp");
}

// drawButton()
// x,y = Start coordinates
// r,g,b = Color
// text = Text to place on button
void drawButton(uint16_t x, uint16_t y, uint16_t r, uint16_t g, uint16_t b, char *text)
{
  ucg.setColor(r, g, b);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, BUTTONW, BUTTONH);
  ucg.setPrintPos(x+15, y+30);
  ucg.print(text);
  
} // drawButton()

// Draw arrows
void drawUpArrow(uint16_t x, uint16_t y)
{
  ucg.setColor(255, 0, 0);
  ucg.drawFrame(x, y, ARROWW, ARROWH);
  ucg.drawTriangle(x+2,y+ARROWH-2, x+15,y+2, x+ARROWW-2,y+30-2);
  
} // drawUpArrow()

void drawDownArrow(uint16_t x, uint16_t y)
{
  ucg.setColor(0, 0, 255);
  ucg.drawFrame(x, y, ARROWW, ARROWH);
  ucg.drawTriangle(x+2,y+2, x+ARROWW-2,y+2, x+15,y+ARROWH-2);
  
} // drawDownArrow()

// displayTankTemp(x, y, temperature)
// uint16_t x,y: Upper left coordinates of value
// float temperature: the value to display
void displayTankTemp(uint16_t x, uint16_t y, float temperature)
{
  ftoa5(szTemperature, temperature, 1);
  ucg.setColor(255, 0, 0);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, TANKTEMPW, TANKTEMPH);
  ucg.setPrintPos(x+19, y+23);
  ucg.print("Temp:");
  ucg.setPrintPos(x+25, y+52);
  ucg.print(szTemperature);
  
} // displayTankTemp()

// displayAmbientTemp(x, y, temperature)
// uint16_t x,y: Upper left coordinates of value
// float temperature: the value to display
void displayAmbientTemp(uint16_t x, uint16_t y, float temperature)
{
  static int iDisplayCount=0;
  char szTemp[40];

  ftoa5(szTemp, temperature, 1);

  if(iDisplayCount == 0)          // Display everything on first pass
  {
    iDisplayCount++;
    ucg.setColor(0, 255, 255);
    ucg.setPrintPos(x, y);
    ucg.print("Ambient:");
    ucg.setPrintPos(x+125, y);
    ucg.print(szTemp);
  }
  else if(iDisplayCount++ == 10)  // then on every n cycles, display the heading as well
  {
    iDisplayCount = 0;
//    ucg.drawBox(x+125, y-20, 85, 25);
    ucg.setColor(0, 255, 255);
    ucg.setPrintPos(x, y);
    ucg.print("Ambient:");
    ucg.setPrintPos(x+125, y);
    ucg.print(szTemp);
  }
  else                            // Otherwise, just display the temp
  {
    ucg.setColor(0, 255, 255);
    ucg.setPrintPos(x+125, y);
    ucg.print(szTemp);
  }
  
} // displayAmbientTemp()

// displaySetpoint(x, y, temperature)
// uint16_t x,y: Upper left coordinates of value
// float temperature: the value to display
void displaySetpoint(uint16_t x, uint16_t y, float temperature)
{
  ftoa5(szTemperature, temperature, 1);
  // clear old values
  ucg.setColor(0, 0, 0);
  ucg.drawBox(x, y, TANKTEMPW, TANKTEMPH);
  
  ucg.setColor(0, 255, 255);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, TANKTEMPW, TANKTEMPH);
  ucg.setPrintPos(x+19, y+23);
  ucg.print("Setpt:");
  ucg.setPrintPos(x+25, y+52);
  ucg.print(temperature);
  
} // displaySetpoint()

// displayRunStatus(x, y)
// uint16_t x,y: Upper left coordinates of status
// Display current run status
void displayRunStatus(uint16_t x, uint16_t y)
{
  char msg[40];
  // clear old values
  ucg.setColor(0, 0, 0);
  ucg.drawBox(x, y-20, 200, 25);
  
  switch(runMode)
  {
    case Run:
      ucg.setColor(255, 0, 0);
      strcpy(msg, "Running");
      break;
    case Idle:
      ucg.setColor(0, 255, 0);
      strcpy(msg, "Idling");
      break;
    default:
      ucg.setColor(0, 0, 255); 
      strcpy(msg, "Off");
      break;
  }
  
  ucg.setPrintPos(STATUSX, STATUSY);
  ucg.print("Mode:");
  ucg.setPrintPos(x+82, y);
  ucg.print(msg);
  Serial.println(msg);
  
} // displayRunStatus()

// touchInRange() - Check if touch was in certain area
// x, y = Touch point
// sx, sy = Upper left bounding coordinates
// w, h = Width and height of boundary
int touchInRange(uint16_t x, uint16_t y, uint16_t sx, uint16_t sy, uint16_t w, uint16_t h)
{
  return(x >= sx && x <= (sx+w) && y >= sy && y <= (sy+h));

} // touchInRange()

//******************* setup() *********************
void setup() {
  byte dat[13];
  bool bEEPromRead=0;
  
  delay(1000);
  Serial.begin(115200);
  while(!Serial)
    ;

  pinMode(OUTPUTPIN, OUTPUT);
  digitalWrite(OUTPUTPIN, HIGH);
  
  //ucg.begin(UCG_FONT_MODE_TRANSPARENT);
  ucg.begin(UCG_FONT_MODE_SOLID);
  touch.begin(ucg.getWidth(), ucg.getHeight());  // Must be done before setting rotation
  ucg.setRotate270();
  touch.setRotation(touch.ROT270);
  ucg.clearScreen();

  // Replace these for your screen module
//  touch.setCalibration(209, 1759, 1775, 273);
  // After running XPTCalibrate on my module, it offered the following coordinates:
  touch.setCalibration(145,1816,1759,280);

//  ucg.setFont(ucg_font_freedoomr10_tr);
  ucg.setFont(ucg_font_ncenB18_tf);

  displayVersion();
  
//  SPI.setFrequency(32000000);
  // Initialize the temperature sensors
  ts.begin();
  if(!ts.getAddress(tsTankAddr, 0))
  {
    Serial.println("No tank sensor found!");
  }
  if(!ts.getAddress(tsAmbientAddr, 1))
  {
     Serial.println("No ambient sensor found!");
  }

  // Find DS2431 eeprom memory
  if(!findDS2431(ds2431))
  {
    Serial.println("No DS2431 EEPROM found!");
    fRunSetpoint = 70.0;
  }
  else
  {
    // read the setpoints from row zero of the eeprom
    ReadRow(ds2431, /* row */0, dat);
    fIdleSetpoint = *(float*)&dat[0];
    fRunSetpoint = *(float*)&dat[4];
    bEEPromRead = 1;
  }

  if(!bEEPromRead)
  {
    fRunSetpoint = 70.0;
    fIdleSetpoint = 45.0;
    fOffSetpoint = 0.0;
  }
  runMode = Off;
  bSpUpd = 0;

  drawTempScreen();
  
} // setup()

// drawTempScreen()
// Draws temperature screen
void drawTempScreen()
{
  ucg.clearScreen();
  switch(runMode)
  {
    case Run:
      displaySetpoint(SETPTX, SETPTY, fRunSetpoint);
      break;
    case Idle:
      displaySetpoint(SETPTX, SETPTY, fIdleSetpoint);
      break;
    default:
      displaySetpoint(SETPTX, SETPTY, fOffSetpoint);
      break;
  }
  drawRunButton();
  drawIdleButton();
  drawOffButton();
  drawTimeButton();
  drawUpArrow(UPARROWX, UPARROWY);
  drawDownArrow(DNARROWX, DNARROWY);

  displayRunStatus(STATUSX, STATUSY);
  
} // drawTempScreen()


// drawTimeScreen()
// Draws time screen
void drawTimeScreen()
{
  ucg.clearScreen();
  drawRunButton();
  drawIdleButton();
  drawOffButton();
  drawTempButton();
  drawUpArrow(UPARROWX, UPARROWY);
  drawDownArrow(DNARROWX, DNARROWY);

  displayRunStatus(STATUSX, STATUSY);
  
} // drawTimeScreen()

//********************* loop() *********************
void loop() {
  static float fTankTemp;
  static unsigned long ulMSBtnPress;
  uint16_t x, y;
  unsigned long ulMS;
  byte eeDat[13];
  
  if(touch.isTouching())
  {
    bSpUpd = 0;
    touch.getPosition(x, y);
    Serial.print("x,y=");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.printf("\n");
    if(touchInRange(x, y, RUNX, RUNY, BUTTONW, BUTTONH)) // Run
    {
      ulMSBtnPress = millis();        // set base timer to trigger subsequent write to eeprom 
      if(runMode != Run)
      {
//        digitalWrite(OUTPUTPIN, LOW);
        Serial.println("Shifting to run");
        runMode = Run;
        drawTempScreen();
      }
    }
    else if(touchInRange(x, y, IDLEX, IDLEY, BUTTONW, BUTTONH)) // Idle
    {
      if(runMode != Idle) 
      {
        runMode = Idle;
        drawTempScreen();
      }
    }
    else if(touchInRange(x, y, OFFX, OFFY, BUTTONW, BUTTONH)) // Off
    {
      digitalWrite(OUTPUTPIN, HIGH);
      runMode = Off;
      drawTempScreen();
    }
    else if(touchInRange(x, y, UPARROWY, UPARROWY, ARROWW, ARROWH)) // Up arrow
    {
      ulMSBtnPress = millis();        // set base timer to trigger subsequent write to eeprom 
      switch(runMode)
      {
        case Run:
          fRunSetpoint += 1.0;
          break;
        case Idle:
          fIdleSetpoint += 1.0;
          break;
        default:
          break;
      }
      bSpUpd = 1;
    }
    else if(touchInRange(x, y, DNARROWX, DNARROWY, ARROWW, ARROWH)) // Down arrow
    {
      ulMSBtnPress = millis();        // set base timer to trigger subsequent write to eeprom 
      switch(runMode)
      {
        case Run:     displaySetpoint(SETPTX, SETPTY, fOffSetpoint);
          fRunSetpoint -= 1.0;
          break;
        case Idle:
          fIdleSetpoint -= 1.0;
          break;
        default:
          break;
      }
      bSpUpd = 1;
    }
    
    displayRunStatus(STATUSX, STATUSY);
    
  } // end if touching

  // Process temperature
  // Tank temp
  fTankTemp = readTemp(tsTankAddr);
  displayTankTemp(TANKTEMPX, TANKTEMPY, fTankTemp);

  // Ambient temp
  fAmbientTemp = readTemp(tsAmbientAddr);
  displayAmbientTemp(AMBTEMPX, AMBTEMPY, fAmbientTemp);

  // If setpt changed, redisplay it
  if(bSpUpd)
  {
    if(runMode == Idle)
    {
      displaySetpoint(SETPTX, SETPTY, fIdleSetpoint);
      bSpUpd = 0;
      Serial.println(fIdleSetpoint);
    }
    else if(runMode == Run)
    {
      displaySetpoint(SETPTX, SETPTY, fRunSetpoint);
      bSpUpd = 0;
      Serial.println(fRunSetpoint);
    }
  }

  // Update controller
  if(runMode == Run)
  {
    if(fTankTemp < (fRunSetpoint - TEMP_HYSTERESIS))
    {
      digitalWrite(OUTPUTPIN, LOW);
    }
    else if(fTankTemp > (fRunSetpoint + TEMP_HYSTERESIS))
    {
      digitalWrite(OUTPUTPIN, HIGH);
    }
  }
  else if(runMode == Idle)
  {
    if(fTankTemp < (fIdleSetpoint - TEMP_HYSTERESIS))
    {
      digitalWrite(OUTPUTPIN, LOW);
    }
    else if(fTankTemp > (fIdleSetpoint + TEMP_HYSTERESIS))
    {
      digitalWrite(OUTPUTPIN, HIGH);
    }
  }
  else // only one other state: Off
  {
    digitalWrite(OUTPUTPIN, HIGH); // ensure controller stays off

  } // endupdate controller

  // Check and write eeprom if values have changed
  // Main loop runs at about 1/sec with overhead, so we're looking for the window
  // between 10S minus 2S (8000 mS) and 10S plus 2S (12000 mS) to write in
  ulMS = millis();
  // Serial.print(ulMSBtnPress); Serial.print(" "); Serial.println(ulMS);
  if(ulMS >= (ulMSBtnPress + 8000) && ulMS <= (ulMSBtnPress + 12000))
  {
    updateEEPromValues();
    Serial.println("Updating eeprom");
  }

  delay(20);
 
} // loop()
