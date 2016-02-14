// Hydro Dip tank termostat using esp8266/2.8" TFT Touch display
// 2016Feb07 Created -- RL
// 2016Feb07 Notes:
//           1. Need to be able to store setpoints (run, idle) in nv ram
//           2. Speed - need to speed display up

#include <Arduino.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <Ucglib.h>  // Required
#include <XPT2046.h>

// Globals
Ucglib_ILI9341_18x240x320_HWSPI ucg(/*cd=*/ 2 , /*cs=*/ 4, /*reset=*/ 5);
XPT2046 touch(/*cs=*/ 16, /*irq=*/ 0);

OneWire oneWire(10); // Pin #
DallasTemperature ts(&oneWire);
float fTemperature;
float fOffSetpoint;
float fIdleSetpoint;
float fRunSetpoint;
char szTemperature[40];
byte tsAddr[8]; // address of 18b20
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
} // ftoa

// readTemp()
// Read temperature and return value as a float in degrees Fahrenheit
// Returns -255.0 on error
// 2016Feb07 Created -- RL
float readTemp()
{
  float fRetVal = -255.0;
  ts.requestTemperatures();
  fRetVal = ts.getTempF(tsAddr);
  return(fRetVal);
} // readTemp();

void drawRunButton()
{
  drawButton(10, 10, 255, 0, 0, "Run");
}
void drawIdleButton()
{
 drawButton(10, 10, 0, 255, 0, "Idle");
}
void drawOffButton()
{
  drawButton(10, 80, 0, 0, 255, "Off");
}
// drawButton()
// x,y = Start coordinates
// r,g,b = Color
// text = Text to place on button
void drawButton(uint16_t x, uint16_t y, uint16_t r, uint16_t g, uint16_t b, char *text)
{
  ucg.setColor(r, g, b);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, 120, 60);
  ucg.setPrintPos(x+30, y+40);
  ucg.print(text);
}
void drawUpArrow(uint16_t x, uint16_t y)
{
  ucg.setColor(255, 0, 0);
  ucg.drawFrame(x, y, 30, 30);
  ucg.drawTriangle(x+2,y+30-2, x+15,y+2, x+30-2,y+30-2);
  
}
void drawDownArrow(uint16_t x, uint16_t y)
{
  ucg.setColor(0, 0, 255);
  ucg.drawFrame(280, 50, 30, 30);
  ucg.drawTriangle(x+2,y+2, x+30-2,y+2, x+15,y+30-2);
  
}

// displayTemperature(x, y, temperature)
// uint16_t x,y: upper left coordinates
// float temperature: the value to display
void displayTemperature(uint16_t x, uint16_t y, float temperature)
{
  ftoa5(szTemperature, temperature, 1);
  ucg.setColor(255, 0, 0);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, 120, 60);
//  ucg.drawFrame(145, 10, 120, 60);
  ucg.setPrintPos(x+19, y+23);
  ucg.print("Temp:");
  ucg.setPrintPos(x+25, y+52);
  ucg.print(temperature);
  
} // displayTemperature()

// displayTemperature(x, y, temperature)
// uint16_t x,y: upper left coordinates
// float temperature: the value to display
void displaySetpoint(uint16_t x, uint16_t y, float temperature)
{
  ftoa5(szTemperature, temperature, 1);
  // clear old values
  ucg.setColor(0, 0, 0);
  ucg.drawBox(x, y, 120, 60);
  
  ucg.setColor(0, 255, 255);
  // drawFrame(x, y, w, h)
  ucg.drawFrame(x, y, 120, 60);
//  ucg.drawFrame(145, 10, 120, 60);
  ucg.setPrintPos(x+19, y+23);
  ucg.print("Setpt:");
  ucg.setPrintPos(x+25, y+52);
  ucg.print(temperature);
  
} // displaySetpoint()

// displayRunStatus(x, y)
// uint16_t x,y: Position to display status at
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
  
  ucg.setPrintPos(x, y);
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

void setup() {
  delay(1000);
  Serial.begin(115200);
  
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

//  SPI.setFrequency(32000000);
  // Initialize the temperature sensor
  ts.begin();
  if(!ts.getAddress(tsAddr, 0))
  {
    Serial.println("No sensor found!");
  }

  drawRunButton();
  drawOffButton();
  drawUpArrow(280, 10);
  drawDownArrow(280, 50);

  fIdleSetpoint = 45.0;
  fRunSetpoint = 90.0;
  fOffSetpoint = 0.0;
  runMode = Off;
  switch(runMode)
  {
    case Run:
      displaySetpoint(145, 80, fRunSetpoint);
      break;
    case Idle:
      displaySetpoint(145, 80, fIdleSetpoint);
      break;
    default:
      displaySetpoint(145, 80, fOffSetpoint);
      break;
  }
  bSpUpd = 0;

  displayRunStatus(10, 210);
  
} // setup()

void loop() {
  static float fTemperature;
  uint16_t x, y;
  if(touch.isTouching())
  {
    bSpUpd = 0;
    touch.getPosition(x, y);
    Serial.print("x,y=");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.printf("\n");
    if(touchInRange(x, y, 10, 10, 120, 60)) // Run/Idle
    {
      if(runMode != Run)
      {
        runMode = Run;
        ucg.clearScreen();
        drawIdleButton();
        drawOffButton();
        drawUpArrow(280, 10);
        drawDownArrow(280, 50);
        displaySetpoint(145, 80, fRunSetpoint);

      }
      else 
      {
        runMode = Idle;
        ucg.clearScreen();
        drawRunButton();
        drawOffButton();
        drawUpArrow(280, 10);
        drawDownArrow(280, 50);
        displaySetpoint(145, 80, fIdleSetpoint);
      }
    }
    else if(touchInRange(x, y, 10, 80, 120, 60)) // Off
    {
      runMode = Off;
      drawRunButton();
      displaySetpoint(145, 80, fOffSetpoint);
    }
    else if(touchInRange(x, y, 280, 10, 30, 30)) // Up arrow
    {
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
    else if(touchInRange(x, y, 280, 50, 30, 30)) // Down arrow
    {
      switch(runMode)
      {
        case Run:
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
    
    displayRunStatus(10, 210);
    
  } // end if touching

  // Process temperature
  fTemperature = readTemp();
  displayTemperature(145, 10, fTemperature);

  // If setpt changed, redisplay it
  if(bSpUpd)
  {
    if(runMode == Idle)
    {
      displaySetpoint(145, 80, fIdleSetpoint);
      bSpUpd = 0;
      Serial.println(fIdleSetpoint);
    }
    else if(runMode == Run)
    {
      displaySetpoint(145, 80, fRunSetpoint);
      bSpUpd = 0;
      Serial.println(fRunSetpoint);
    }
  }
  
  delay(20);
 
} // end loop()
