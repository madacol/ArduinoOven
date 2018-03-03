// Display
#define  TOUCH_ORIENTATION  3
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>            //we are using UTFT display methods
UTFTGLUE myGLCD(0x9341, A2, A1, A3, A4, A0);

// Thermocouple
#include "max6675.h"
#define pinTopTempSensor1DO  20
#define pinTopTempSensor1CS  19
#define pinTopTempSensor1CLK  18
#define pinTopTempSensor1VCC  17
#define pinTopTempSensor1GND  16
#define pinBottomTempSensor1DO  31
#define pinBottomTempSensor1CS  29
#define pinBottomTempSensor1CLK  27
#define pinBottomTempSensor1VCC  25
#define pinBottomTempSensor1GND  23

// MCUFRIEND UNO shield shares pins with the TFT.   Due does NOT work
// TouchScreen
#include <TouchScreen.h>
#define YP A1   //A3 for ILI9320
#define YM 7    //9
#define XM A2
#define XP 6    //8  
TouchScreen myTouch(XP, YP, XM, YM, 300);
TSPoint tp;                      //Touchscreen_due branch uses Point

// Custom defines
#define GREEN 0, 255, 0
#define WHITE 255, 255, 255
#define BLACK 0, 0, 0
#define RED 255, 0, 0
#define BLUE 0, 0, 255
#define YELLOW 255, 255, 0

#define SET_CONTROL_TEXT_SIZE 5
#define PROFILE_ID_TEXT_SIZE 4
#define PROFILE_PARAM_TEXT_SIZE 1
#define SENSOR_TEXT_SIZE 2

int dispX, dispY;
byte controlButtonWidth, controlButtonHeight, blockWidth, blockHeight, profileEndY;
bool isOutline = false;

class Control {

  public:

    int x, y;
    int setControl;

    Control(int _x, int _y):
      x(_x),
      y(_y)
      {};

    void drawMinusButton(void)
    {
      int startX = x;
      int startY = y;
      int endX = startX+blockWidth-2;
      int endY = startY+blockHeight-2;
      myGLCD.setColor(BLUE);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    
    void drawSetControl(void)
    {
      int startX = x+blockWidth;
      int startY = y;
      int endX = startX+blockWidth*2-2;
      int endY = startY+blockHeight-2;
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
      myGLCD.print(String(setControl), startX+11, startY+49);
    };

    void drawPlusButton(void)
    {
      int startX = x+blockWidth*3;
      int startY = y;
      int endX = startX+blockWidth-2;
      int endY = startY+blockHeight-2;
      myGLCD.setColor(RED);
      myGLCD.fillRect(startX, startY, startX+62, endY);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX+27, startY+15, startX+34, startY+46);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };

    virtual void draw(void)
    {
      drawMinusButton();
      drawPlusButton();
      drawSetControl();
    };

} cookTimeControl(0, 112);

class TempControl : public Control {

  public:
    int sensor1;
    int sensor2;
    int sensor3;
    MAX6675 *Sensor1;

    TempControl(int _x, int _y, int pinSensor1DO, int pinSensor1CS, int pinSensor1CLK):
      Control(_x, _y),
      Sensor1(new MAX6675(pinSensor1DO, pinSensor1CS, pinSensor1CLK))
    {};

    void drawSensors(void)
    {
      int startX = x+blockWidth*4;
      int startY = y;
      int endX = startX+controlButtonWidth-1;
      int endY = startY+controlButtonHeight-1;
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(SENSOR_TEXT_SIZE);
      myGLCD.print(String(sensor1), startX+10, startY+7+10);
      myGLCD.print(String(sensor2), startX+20+10, startY+35+10);
      myGLCD.fillRect(startX+8, startY+35+18 , startX+8+13, startY+35+19);
      myGLCD.fillRect(startX+8, startY+35+7 , startX+8+13, startY+35+8);
      myGLCD.fillRect(startX+8+6, startY+35 , startX+8+7, startY+35+15);
    };

    void draw(void)
    {
      drawMinusButton();
      drawPlusButton();
      drawSetControl();
      drawSensors();
    };

    void updateSensors(void)
    {
      sensor1 = Sensor1->readCelsius();
    };

// TempControl(x, y, SensorCLK, SensorCS, SensorDO)
} topTempControl(0,48, pinTopTempSensor1CLK, pinTopTempSensor1CS, pinTopTempSensor1DO),
  bottomTempControl(0,176, pinBottomTempSensor1CLK, pinBottomTempSensor1CS, pinBottomTempSensor1DO);

byte activeProfile;
class Profile {

  public:
    int bottomTemp;
    int topTemp;
    int cookTime;
    bool isActive = false;
    byte id;
    int x, y;

    Profile(int _topTemp, int _cookTime, int _bottomTemp):
      topTemp(_topTemp),
      cookTime(_cookTime),
      bottomTemp(_bottomTemp)
    {};

    void draw (void)
    {
      int endX = x + blockWidth - 2;
      int endY = profileEndY;
      if (isActive)
      {
        myGLCD.setColor(GREEN);
        myGLCD.fillRect(x, y, endX, endY);
        myGLCD.setColor(BLACK);
      }
      else
      {
        myGLCD.setColor(BLACK);
        myGLCD.fillRect(x, y, endX, endY);
        myGLCD.setColor(WHITE);
      }
      myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
      myGLCD.print(String(id+1), x+9 , endY-12);
      myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
      myGLCD.print(String(topTemp), x+38, endY-34-10);
      myGLCD.print(String(cookTime), x+38, endY-20-10);
      myGLCD.print(String(bottomTemp), x+38, endY-6-10);
    };

    void load(void)
    {
      topTempControl.setControl = topTemp;
      cookTimeControl.setControl = cookTime;
      bottomTempControl.setControl = bottomTemp;
      isActive = true;
      draw();
    };

    void unload(void)
    {
      isActive = false;
      draw();
    };

     void save(void)
    {
      topTemp = topTempControl.setControl;
      cookTime = cookTimeControl.setControl;
      bottomTemp = bottomTempControl.setControl;
    };

} profiles[] = {
  // Profile(topTemp, cookTime, bottomTemp)
  Profile(110,120,130),
  Profile(210,220,230),
  Profile(310,320,330),
  Profile(410,420,430),
  Profile(510,520,530),
};
byte profilesSize = sizeof(profiles) / sizeof(Profile);


void readResistiveTouch(void)
{
  tp = myTouch.getPoint();
  pinMode(YP, OUTPUT);      //restore shared pins
  pinMode(XM, OUTPUT);
  digitalWrite(YP, HIGH);
  digitalWrite(XM, HIGH);
}

bool ISPRESSED(void)
{
  readResistiveTouch();
  return tp.z > 20 && tp.z < 1000;
}

void showpoint(void)
{
  Serial.print("\r\nx="); Serial.print(tp.x);
  Serial.print(" y="); Serial.print(tp.y);
  Serial.print(" z="); Serial.print(tp.z);
}

void loadProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].load();
  activeProfile = id;
}

void drawDivisions(void)
{
  myGLCD.setColor(WHITE);
  myGLCD.drawLine(blockWidth - 1 + isOutline, dispY - 1, blockWidth - 1 + isOutline, 0);
  myGLCD.drawLine(blockWidth * 2 - 1 + isOutline , 0 , blockWidth * 2 - 1 + isOutline , profileEndY);
  myGLCD.drawLine(blockWidth * 3 - 1 + isOutline , dispY-1 , blockWidth * 3 - 1 + isOutline , 0);
  myGLCD.drawLine(blockWidth * 4 - 1 + isOutline , dispY-1 , blockWidth * 4 - 1 + isOutline , 0);
  myGLCD.drawLine(0, topTempControl.y-1 , dispX-1 , topTempControl.y-1);
  myGLCD.drawLine(0, cookTimeControl.y-1 , dispX-1 , cookTimeControl.y-1);
  myGLCD.drawLine(0, bottomTempControl.y-1 , dispX-1 , bottomTempControl.y-1);
}

void calculateProfilesProperties (void)
{
  for (int i=0; i<profilesSize; i++)
  {
    profiles[i].id = i;
    profiles[i].x = blockWidth*i + isOutline;
    profiles[i].y = isOutline;
  }

}

void drawProfiles(void)
{
  for (int i=0; i<profilesSize; i++)
  { 
    profiles[i].draw();
  }
}

void draw(void)
{
  drawDivisions();
  drawProfiles();
  topTempControl.draw();
  cookTimeControl.draw();
  bottomTempControl.draw();
}


void setup()
{
  Serial.begin(9600);
  Serial.println("SpRvN");

  // Display init
  digitalWrite(A0, HIGH);
  pinMode(A0, OUTPUT);
  myGLCD.InitLCD(TOUCH_ORIENTATION);
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);

  // Set coordinates
  dispX = myGLCD.getDisplayXSize();
  dispY = myGLCD.getDisplayYSize();
  controlButtonWidth = (dispX - 4) / 5; // Substract because of 4 white line division with 1px width
  controlButtonHeight = controlButtonWidth;
  blockWidth = controlButtonWidth + 1;
  blockHeight = blockWidth;
  if ( (dispX - 4) % 5 >= 2 )  { isOutline = true; } // If there's more than 2 extra pixels, draw outline
  bottomTempControl.x = isOutline;
  bottomTempControl.y = dispY - isOutline - controlButtonHeight;
  cookTimeControl.x = isOutline;
  cookTimeControl.y = bottomTempControl.y - blockHeight;
  topTempControl.x = isOutline;
  topTempControl.y = cookTimeControl.y - blockHeight;
  profileEndY = topTempControl.y - 2;
  calculateProfilesProperties();

  // Thermocouple Init
  pinMode(pinTopTempSensor1VCC, OUTPUT); digitalWrite(pinTopTempSensor1VCC, HIGH);
  pinMode(pinTopTempSensor1GND, OUTPUT); digitalWrite(pinTopTempSensor1GND, LOW);
  pinMode(pinBottomTempSensor1VCC, OUTPUT); digitalWrite(pinBottomTempSensor1VCC, HIGH);
  pinMode(pinBottomTempSensor1GND, OUTPUT); digitalWrite(pinBottomTempSensor1GND, LOW);
  
  topTempControl.sensor1 = 45;
  topTempControl.sensor2 = 0;
  
  loadProfile(4);

  draw();
}


void loop()
{
  delay(1000);
  topTempControl.updateSensors();  topTempControl.drawSensors();
  bottomTempControl.updateSensors();  bottomTempControl.drawSensors();
  loadProfile(( (activeProfile >= 4) ? 0 : activeProfile+1 ));
}