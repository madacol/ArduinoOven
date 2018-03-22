// Display
#define  TOUCH_ORIENTATION  3
#include <Adafruit_GFX.h>
#include <UTFTGLUE.h>            //we are using UTFT display methods
UTFTGLUE myGLCD(0x9341, A2, A1, A3, A4, A0);

// Thermocouples
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

// TouchScreen
#include <TouchScreen.h>
#define YP A2   //A3 for ILI9320
#define YM 6    //9
#define XM A1
#define XP 7    //8
TouchScreen myTouch(XP, YP, XM, YM, 300);
TSPoint tp, last_tp;
#define CLICK_TIME      30              // minimum ms after initial touch to trigger CLICK_EVENT
#define HOLD_TIME       CLICK_TIME      // minimum ms after initial touch to trigger HOLD_EVENT
#define LONG_CLICK_TIME 600             // minimum ms after initial touch to trigger LONG_CLICK_EVENT
#define LONG_HOLD_TIME  LONG_CLICK_TIME // minimum ms after initial touch to trigger LONG_HOLD_EVENT
#define DEAD_ZONE 5 // outest pixels of objects where Touch-Events are disabled
// Touch events
#define CLICK_EVENT       0
#define LONG_CLICK_EVENT  1
#define HOLD_EVENT        2
#define LONG_HOLD_EVENT   3

// Colors
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define GREY    0x7BEF
#define CYAN    0x07FF
#define MAGENTA 0xF81F

// Text sizes
#define SET_CONTROL_TEXT_SIZE   6
#define PROFILE_ID_TEXT_SIZE    4
#define PROFILE_PARAM_TEXT_SIZE 1
#define SENSOR_TEXT_SIZE        3

void debug (String text) {
  Serial.println(text);
};

// ###############################################################
// #####################   GLOBAL VARIABLES   ####################
// ###############################################################


int dispX, dispY;
byte gridWidth, gridHeight, gridInternalWidth, gridInternalHeight;
byte activeProfile;
bool isOutline = false;

bool TouchStatus;       // the current value read from isPressed()
bool lastTouchStatus = false;
long timeTouchStarted, timeSinceTouchStarted, lastTimeSinceTouchStarted;


// ###############################################################
// #########################   CLASSES   #########################
// ###############################################################


class Coordinates {
  public:
    int startX, startY, endX, endY;
    virtual void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridInternalWidth;
      endY = startY+gridInternalHeight;
    };
};
class Block : public Coordinates {
  public:
    int backgroundColor = BLACK;
    int foregroundColor = WHITE;
};

class MinusButton : public Block {
  public:
    void draw(void) {
      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setColor(foregroundColor);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    MinusButton() { backgroundColor = BLUE; foregroundColor = BLACK; }
};

class SetControl : public Block {
  public:
    int value;
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridWidth+gridInternalWidth; // +gridWidth for 2 columns width
      endY = startY+gridInternalHeight;
    };
    void draw(void) {
      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setTextColor(foregroundColor, backgroundColor);
      myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
      myGLCD.print(String(value), startX+12, startY+11);
    };
};

class PlusButton : public Block {
  public:
    void draw(void) {
      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, startX+62, endY);
      myGLCD.setColor(foregroundColor);
      myGLCD.fillRect(startX+27, startY+15, startX+34, startY+46);
      myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    PlusButton() { backgroundColor = RED; foregroundColor = BLACK; }
};

class Sensors : public Block {
  public:
    int value1;
    int value2;
    MAX6675 Sensor1;
    Sensors(byte pinSensor1DO, byte pinSensor1CS, byte pinSensor1CLK):
      Sensor1(pinSensor1DO, pinSensor1CS, pinSensor1CLK)
    {};

    void highlight(void) { backgroundColor = GREEN; foregroundColor = BLACK; };
    void lowlight(void) { backgroundColor = BLACK; foregroundColor = WHITE; };

    void draw(void) {
      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setTextColor(foregroundColor, backgroundColor);
      myGLCD.setTextSize(SENSOR_TEXT_SIZE);
      myGLCD.print(String(value1), startX+7, startY+7);
      myGLCD.print(String(value2), startX+18+7, startY+35);
      myGLCD.setColor(foregroundColor);
      myGLCD.fillRect(startX+7, startY+35+18 , startX+7+13, startY+35+19);
      myGLCD.fillRect(startX+7, startY+35+7 , startX+7+13, startY+35+8);
      myGLCD.fillRect(startX+7+6, startY+35 , startX+7+7, startY+35+15);
    };
    void update(void) {
      value1 = Sensor1.readCelsius();
    };
};

class Control : public Coordinates {
  public:
    MinusButton minusButton;
    SetControl setControl;
    PlusButton plusButton;
    Block sensors;
    virtual void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = dispX-1 - isOutline;
      endY = startY + gridInternalHeight;
      minusButton.setCoordinates(startX, startY);
      setControl.setCoordinates(startX+gridWidth, startY);
      plusButton.setCoordinates(startX+gridWidth*3, startY);
      sensors.setCoordinates(startX+gridWidth*4, startY);
    };
    virtual void draw(void) {
      minusButton.draw();
      plusButton.draw();
      setControl.draw();
    };
    void decreaseSetControl(void) {
      setControl.value--;
      setControl.draw();
    }
    void increaseSetControl(void) {
      setControl.value++;
      setControl.draw();
    }
} cookTimeControl;

class TempControl : public Control {
  public:
    Sensors sensors;
    TempControl(byte pinSensor1DO, byte pinSensor1CS, byte pinSensor1CLK):
      sensors(pinSensor1DO, pinSensor1CS, pinSensor1CLK)
    {};
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = dispX-1 - isOutline;
      endY = startY + gridInternalHeight;
      minusButton.setCoordinates(startX, startY);
      setControl.setCoordinates(startX+gridWidth, startY);
      plusButton.setCoordinates(startX+gridWidth*3, startY);
      sensors.setCoordinates(startX+gridWidth*4, startY);
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


class Profile : public Block {
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
        { backgroundColor = GREEN; foregroundColor = BLACK; }
      else
        { backgroundColor = BLACK; foregroundColor = WHITE; }

      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, endX, endY);

      myGLCD.setTextColor(foregroundColor, backgroundColor);
      myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
      myGLCD.print(String(id+1), startX+9 , startY+9);
      myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
      myGLCD.print(String(topTemp), startX+38, startY+6);
      myGLCD.print(String(cookTime), startX+38, startY+6+14);
      myGLCD.print(String(bottomTemp), startX+38, startY+6+28);
    };

    void load(void)
    {
      topTempControl.setControl.value = topTemp;
      cookTimeControl.setControl.value = cookTime;
      bottomTempControl.setControl.value = bottomTemp;
      isActive = true;
      draw();
      topTempControl.setControl.draw();
      cookTimeControl.setControl.draw();
      bottomTempControl.setControl.draw();
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
      isActive = true;
      draw();
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


// ###############################################################
// #####################   GLOBAL FUNCTIONS   ####################
// ###############################################################


void loadProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].load();
  activeProfile = id;
}

void saveProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].save();
  activeProfile = id;
}

void calculateProfilesProperties (void)
{
  for (byte i=0; i<profilesSize; i++)
  {
    profiles[i].id = i;
    profiles[i].startX = gridWidth*i + isOutline;
    profiles[i].startY = isOutline;
    profiles[i].endX = profiles[i].startX + gridInternalWidth;
    profiles[i].endY = topTempControl.startY - 2;
  }
}

TSPoint readResistiveTouch(void)
{
  TSPoint tpt = myTouch.getPoint();
  pinMode(YP, OUTPUT);      //restore shared pins
  pinMode(XM, OUTPUT);
  return tpt;
}

TSPoint getTouch(void)
{
  TSPoint tpt;
  byte count = 0;
  #define sampleNum 10
  #define quorum 3
  for (byte i=0; i < sampleNum; i++)
  {
    TSPoint tptmp = readResistiveTouch();
    if (tptmp.z > 50 )
    {
      tpt.x += tptmp.x;
      tpt.y += tptmp.y;
      tpt.z += tptmp.z;
      count++;
    }
  }
  //Serial.print("  count=");Serial.print(count);
  //Serial.print(" z=");Serial.println(tpt.z);
  if (count < quorum) tpt.z=0;
  else { tpt.x /= count; tpt.y /= count; tpt.z /= count; } // get average
  int temp = map(tpt.y, 85, 896, dispX-1, 0);
  tpt.y = map(tpt.x, 907, 130, dispY-1, 0);
  tpt.x = temp;
  return tpt;
}

bool isPressed (TSPoint tpt)
{
  return tpt.z > 0;
}

bool hasTouchStatusChanged () {
  return TouchStatus != lastTouchStatus;
}

void showpoint(TSPoint tpt)
{
  Serial.print("\r\nx="); Serial.print(tpt.x);
  Serial.print(" y="); Serial.print(tpt.y);
  Serial.print(" z="); Serial.print(tpt.z);
}

void drawProfiles(void)
{
  for (byte i=0; i<profilesSize; i++)
  {
    profiles[i].draw();
  }
}

void drawDivisions(void)
{
  if (isOutline) {myGLCD.drawRect(0,0,dispX-1,dispY-1);}
  myGLCD.setColor(WHITE);
  myGLCD.drawLine(gridWidth-1+isOutline , dispY-1 , gridWidth-1+isOutline , 0);
  myGLCD.drawLine(gridWidth*2-1+isOutline , 0 , gridWidth*2-1+isOutline , dispY-gridHeight*3);
  myGLCD.drawLine(gridWidth*3-1+isOutline , dispY-1 , gridWidth*3-1+isOutline , 0);
  myGLCD.drawLine(gridWidth*4-1+isOutline , dispY-1 , gridWidth*4-1+isOutline , 0);
  myGLCD.drawLine(0 , topTempControl.startY-1 , dispX-1 , topTempControl.startY-1);
  myGLCD.drawLine(0 , cookTimeControl.startY-1 , dispX-1 , cookTimeControl.startY-1);
  myGLCD.drawLine(0 , bottomTempControl.startY-1 , dispX-1 , bottomTempControl.startY-1);
}

void draw(void)
{
  drawDivisions();
  drawProfiles();
  topTempControl.draw();
  cookTimeControl.draw();
  bottomTempControl.draw();
}


/*
rant (
  "fucking ugliest code I could come up with, everything was going so fine,
    I was so proud of my code so far, until this shit happened, it wrecked me, I HATE THIS!
  I just couldn't make TouchAction's function to execute it's parent's function, when parents can be
    of differents classes. I wanted to declare TouchAction as an object inside an instance of Profile
    and 3 instances of TempControl and make TouchAction::click() to execute it's parent's click()
    method Profile::click() or TempControl::click(), I just couldn't, maybe something's wrong with my
    undestanding of OOP. I don't know, maybe all i thought I knew about OOP is just worthless.
  Anyway, this is the *best* (¬¬ worst ¬¬) that I could come up with"
);
*/
void findObjectFromCoordAndExecuteAction (TSPoint tpt, byte event)
{
  showpoint(tpt);
  // Profiles
  if (tpt.y > profiles[0].startY+DEAD_ZONE  &&  tpt.y < profiles[0].endY-DEAD_ZONE)
  {
    if (tpt.x > profiles[0].startX+DEAD_ZONE  &&  tpt.x < profiles[0].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      loadProfile(0); break;
        case LONG_CLICK_EVENT : saveProfile(0); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[1].startX+DEAD_ZONE  &&  tpt.x < profiles[1].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      loadProfile(1); break;
        case LONG_CLICK_EVENT : saveProfile(1); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[2].startX+DEAD_ZONE  &&  tpt.x < profiles[2].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      loadProfile(2); break;
        case LONG_CLICK_EVENT : saveProfile(2); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[3].startX+DEAD_ZONE  &&  tpt.x < profiles[3].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      loadProfile(3); break;
        case LONG_CLICK_EVENT : saveProfile(3); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[4].startX+DEAD_ZONE  &&  tpt.x < profiles[4].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      loadProfile(4); break;
        case LONG_CLICK_EVENT : saveProfile(4); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tpt.y > topTempControl.startY+DEAD_ZONE  &&  tpt.y < topTempControl.endY-DEAD_ZONE)
  {
    if (tpt.x > topTempControl.minusButton.startX+DEAD_ZONE  &&  tpt.x < topTempControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topTempControl.decreaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  topTempControl.decreaseSetControl(); break;
      }
    }
    else if (tpt.x > topTempControl.setControl.startX+DEAD_ZONE  &&  tpt.x < topTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > topTempControl.plusButton.startX+DEAD_ZONE  &&  tpt.x < topTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topTempControl.increaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  topTempControl.increaseSetControl(); break;
      }
    }
    else if (tpt.x > topTempControl.sensors.startX+DEAD_ZONE  &&  tpt.x < topTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tpt.y > cookTimeControl.startY+DEAD_ZONE  &&  tpt.y < cookTimeControl.endY-DEAD_ZONE)
  {
    if (tpt.x > cookTimeControl.minusButton.startX+DEAD_ZONE  &&  tpt.x < cookTimeControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      cookTimeControl.decreaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  cookTimeControl.decreaseSetControl(); break;
      }
    }
    else if (tpt.x > cookTimeControl.setControl.startX+DEAD_ZONE  &&  tpt.x < cookTimeControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > cookTimeControl.plusButton.startX+DEAD_ZONE  &&  tpt.x < cookTimeControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      cookTimeControl.increaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  cookTimeControl.increaseSetControl(); break;
      }
    }
    else if (tpt.x > cookTimeControl.sensors.startX+DEAD_ZONE  &&  tpt.x < cookTimeControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tpt.y > bottomTempControl.startY+DEAD_ZONE  &&  tpt.y < bottomTempControl.endY-DEAD_ZONE)
  {
    if (tpt.x > bottomTempControl.minusButton.startX+DEAD_ZONE  &&  tpt.x < bottomTempControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomTempControl.decreaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  bottomTempControl.decreaseSetControl(); break;
      }
    }
    else if (tpt.x > bottomTempControl.setControl.startX+DEAD_ZONE  &&  tpt.x < bottomTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > bottomTempControl.plusButton.startX+DEAD_ZONE  &&  tpt.x < bottomTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomTempControl.increaseSetControl(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  bottomTempControl.increaseSetControl(); break;
      }
    }
    else if (tpt.x > bottomTempControl.sensors.startX+DEAD_ZONE  &&  tpt.x < bottomTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
  }
}


// ###############################################################
// #########################    SETUP    #########################
// ###############################################################


void setup()
{
  Serial.begin(9600);
  Serial.println("SpRvN");

  // Display init
  digitalWrite(A0, HIGH);
  pinMode(A0, OUTPUT);
  myGLCD.InitLCD(TOUCH_ORIENTATION);
  myGLCD.clrScr();
  myGLCD.setFont();

  // Set coordinates
  dispX = myGLCD.getDisplayXSize();
  dispY = myGLCD.getDisplayYSize();
  gridWidth = dispX / 5;
  gridHeight = gridWidth;
  gridInternalWidth = gridWidth - 2;
  gridInternalHeight = gridHeight - 2;
  if ( dispX % 5 >= 1 )  { isOutline = true; } // If there's at least 1 extra pixel, draw outline
  bottomTempControl.setCoordinates( isOutline, ( dispY - 1 ) - isOutline - gridInternalHeight );
  cookTimeControl.setCoordinates( isOutline, bottomTempControl.startY - gridHeight );
  topTempControl.setCoordinates( isOutline, cookTimeControl.startY - gridHeight );
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


// ###############################################################
// ##########################    LOOP    #########################
// ###############################################################


void loop()
{
  topTempControl.sensors.update();  topTempControl.sensors.draw();
  bottomTempControl.sensors.update();  bottomTempControl.sensors.draw();

  //Checking Touch
  tp = getTouch();
  TouchStatus=isPressed(tp);
  lastTimeSinceTouchStarted = timeSinceTouchStarted;
  timeSinceTouchStarted=millis()-timeTouchStarted;
  if (hasTouchStatusChanged())
  {
    if (TouchStatus)
    {
      timeTouchStarted = millis();
    }
    else if (timeSinceTouchStarted > LONG_CLICK_TIME)
    {
      findObjectFromCoordAndExecuteAction(last_tp, LONG_CLICK_EVENT);
    }
    else if (timeSinceTouchStarted > CLICK_TIME)
    {
      findObjectFromCoordAndExecuteAction(last_tp, CLICK_EVENT);
    }
    lastTouchStatus=TouchStatus;
  }
  else
  {
    if (TouchStatus)
    {
      if (lastTimeSinceTouchStarted < LONG_CLICK_TIME && timeSinceTouchStarted > LONG_CLICK_TIME)
        draw();
      if (timeSinceTouchStarted > LONG_HOLD_TIME)
      {
        findObjectFromCoordAndExecuteAction(tp, LONG_HOLD_EVENT);
      }
      else if (timeSinceTouchStarted > HOLD_TIME)
      {
        findObjectFromCoordAndExecuteAction(tp, HOLD_EVENT);
      }
    }
  }
  last_tp = tp;
}