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
byte gridWidth, gridHeight, gridInternalWidth, gridInternalHeight;
bool isOutline = false;

class Coordinates {
  public:
    int startX, startY, endX, endY;
};

class MinusButton : public Coordinates {
  public:
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridInternalWidth;
      endY = startY+gridInternalHeight;
    };
    void draw(void) {
      myGLCD.setColor(BLUE);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
};

class SetControl : public Coordinates {
  public:
    int value;
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridWidth+gridInternalWidth; // +gridWidth for 2 columns width
      endY = startY+gridInternalHeight;
    };
    void draw(void) {
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
      myGLCD.print(String(value), startX+11, startY+49);
    };
};

class PlusButton : public Coordinates {
  public:
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridInternalWidth;
      endY = startY+gridInternalHeight;
    };
    void draw(void) {
      myGLCD.setColor(RED);
      myGLCD.fillRect(startX, startY, startX+62, endY);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX+27, startY+15, startX+34, startY+46);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
};

class Sensors : public Coordinates {
  public:
    int value1;
    int value2;
    MAX6675 Sensor1;
    Sensors(byte pinSensor1DO, byte pinSensor1CS, byte pinSensor1CLK):
      Sensor1(pinSensor1DO, pinSensor1CS, pinSensor1CLK)
    {};
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridInternalWidth;
      endY = startY+gridInternalHeight;
    };
    void draw(void) {
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(SENSOR_TEXT_SIZE);
      myGLCD.print(String(value1), startX+10, startY+7+10);
      myGLCD.print(String(value2), startX+20+10, startY+35+10);
      myGLCD.fillRect(startX+8, startY+35+18 , startX+8+13, startY+35+19);
      myGLCD.fillRect(startX+8, startY+35+7 , startX+8+13, startY+35+8);
      myGLCD.fillRect(startX+8+6, startY+35 , startX+8+7, startY+35+15);
    };
    void update(void) {
      value1 = Sensor1.readCelsius();
    };
};

class Control {
  public:
    int x, y;
    MinusButton minusButton;
    SetControl setControl;
    PlusButton plusButton;
    virtual void setCoordinates(int _x, int _y) {
      x = _x;
      y = _y;
      minusButton.setCoordinates(x, y);
      setControl.setCoordinates(gridWidth, y);
      plusButton.setCoordinates(gridWidth*3, y);
    };
    virtual void draw(void) {
      minusButton.draw();
      plusButton.draw();
      setControl.draw();
    };
} cookTimeControl;

class TempControl : public Control {
  public:
    Sensors sensors;
    TempControl(byte pinSensor1DO, byte pinSensor1CS, byte pinSensor1CLK):
      sensors(pinSensor1DO, pinSensor1CS, pinSensor1CLK)
    {};
    void setCoordinates(int _x, int _y) {
      x = _x;
      y = _y;
      minusButton.setCoordinates(x, y);
      setControl.setCoordinates(minusButton.endX+2, y);
      plusButton.setCoordinates(setControl.endX+2, y);
      sensors.setCoordinates(plusButton.endX+2, y);
    };
    void draw(void) {
      minusButton.draw();
      setControl.draw();
      plusButton.draw();
      sensors.draw();
    };
// TempControl(SensorCLK, SensorCS, SensorDO)
} topTempControl(pinTopTempSensor1CLK, pinTopTempSensor1CS, pinTopTempSensor1DO),
  bottomTempControl(pinBottomTempSensor1CLK, pinBottomTempSensor1CS, pinBottomTempSensor1DO);

byte activeProfile;
class Profile : public Coordinates {

  public:
    int bottomTemp;
    int topTemp;
    int cookTime;
    bool isActive = false;
    byte id;

    Profile(int _topTemp, int _cookTime, int _bottomTemp):
      topTemp(_topTemp),
      cookTime(_cookTime),
      bottomTemp(_bottomTemp)
    {};

    void draw (void)
    {
      if (isActive)
      {
        myGLCD.setColor(GREEN);
        myGLCD.fillRect(startX, startY, endX, endY);
        myGLCD.setColor(BLACK);
      }
      else
      {
        myGLCD.setColor(BLACK);
        myGLCD.fillRect(startX, startY, endX, endY);
        myGLCD.setColor(WHITE);
      }
      myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
      myGLCD.print(String(id+1), startX+9 , endY-12);
      myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
      myGLCD.print(String(topTemp), startX+38, endY-34-10);
      myGLCD.print(String(cookTime), startX+38, endY-20-10);
      myGLCD.print(String(bottomTemp), startX+38, endY-6-10);
    };

    void load(void)
    {
      topTempControl.setControl.value = topTemp;
      cookTimeControl.setControl.value = cookTime;
      bottomTempControl.setControl.value = bottomTemp;
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
      topTemp = topTempControl.setControl.value;
      cookTime = cookTimeControl.setControl.value;
      bottomTemp = bottomTempControl.setControl.value;
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
  if (isOutline) {myGLCD.drawRect(0,0,dispX-1,dispY-1);}
  myGLCD.setColor(WHITE);
  myGLCD.drawLine(gridWidth-1+isOutline , dispY-1 , gridWidth-1+isOutline , 0);
  myGLCD.drawLine(gridWidth*2-1+isOutline , 0 , gridWidth*2-1+isOutline , dispY-gridHeight*3);
  myGLCD.drawLine(gridWidth*3-1+isOutline , dispY-1 , gridWidth*3-1+isOutline , 0);
  myGLCD.drawLine(gridWidth*4-1+isOutline , dispY-1 , gridWidth*4-1+isOutline , 0);
  myGLCD.drawLine(0 , topTempControl.y-1 , dispX-1 , topTempControl.y-1);
  myGLCD.drawLine(0 , cookTimeControl.y-1 , dispX-1 , cookTimeControl.y-1);
  myGLCD.drawLine(0 , bottomTempControl.y-1 , dispX-1 , bottomTempControl.y-1);
}

void calculateProfilesProperties (void)
{
  for (int i=0; i<profilesSize; i++)
  {
    profiles[i].id = i;
    profiles[i].startX = gridWidth*i + isOutline;
    profiles[i].startY = isOutline;
    profiles[i].endX = profiles[i].startX + gridInternalWidth;
    profiles[i].endY = topTempControl.y - 2;
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
  gridWidth = dispX / 5;
  gridHeight = gridWidth;
  gridInternalWidth = gridWidth - 2;
  gridInternalHeight = gridHeight - 2;
  if ( dispX % 5 >= 1 )  { isOutline = true; } // If there's at least 1 extra pixel, draw outline
  bottomTempControl.setCoordinates( isOutline, ( dispY - 1 ) - isOutline - gridInternalHeight );
  cookTimeControl.setCoordinates( isOutline, bottomTempControl.y - gridHeight );
  topTempControl.setCoordinates( isOutline, cookTimeControl.y - gridHeight );
  calculateProfilesProperties();

  // Thermocouple Init
  pinMode(pinTopTempSensor1VCC, OUTPUT); digitalWrite(pinTopTempSensor1VCC, HIGH);
  pinMode(pinTopTempSensor1GND, OUTPUT); digitalWrite(pinTopTempSensor1GND, LOW);
  pinMode(pinBottomTempSensor1VCC, OUTPUT); digitalWrite(pinBottomTempSensor1VCC, HIGH);
  pinMode(pinBottomTempSensor1GND, OUTPUT); digitalWrite(pinBottomTempSensor1GND, LOW);

  topTempControl.sensors.value1 = 100;
  topTempControl.sensors.value2 = 0;

  loadProfile(4);

  draw();
}


void loop()
{
  delay(1000);
  topTempControl.sensors.update();  topTempControl.sensors.draw();
  bottomTempControl.sensors.update();  bottomTempControl.sensors.draw();
  loadProfile(( (activeProfile >= 4) ? 0 : activeProfile+1 ));
}