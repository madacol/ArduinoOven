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

// Servos
#define TOP_SERVO_PIN       46
#define CONVEYOR_SERVO_PIN  45
#define BOTTOM_SERVO_PIN    44

// PID
#include <PID_v1.h>
#define PID_KP 2
#define PID_KI 5
#define PID_KD 1
#define TOP_OUTPUT_MIN       70
#define TOP_OUTPUT_MAX       145
#define BOTTOM_OUTPUT_MIN    70
#define BOTTOM_OUTPUT_MAX    145

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

// States
#define CONTROLLING_SETPOINTS   0
#define CONTROLLING_TOP_PID     1
#define CONTROLLING_BOTTOM_PID  2
#define SHOWING_GRAPH           3

#include <Thread.h>
Thread updateSensorsThread = Thread();
Thread drawSensorsThread = Thread();
Thread bottomPIDThread = Thread();
Thread drawGraphPointThread = Thread();

// Colors
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define GREY        0x7BEF
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define PALEGREEN   0x9FD3

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
byte state;
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
    void draw(int number) {
      myGLCD.setColor(backgroundColor);
      myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setTextColor(foregroundColor, backgroundColor);
      myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
      myGLCD.print(String(number), startX+12, startY+11);
    };
    void draw(void) {draw(value);};
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
    double value1;
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
      myGLCD.print(String(int(value1)), startX+7, startY+7);
      myGLCD.print(String(int(value2)), startX+18+7, startY+35);
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
    virtual void draw(int setControlNumber) {
      minusButton.draw();
      setControl.draw(setControlNumber);
      plusButton.draw();
    };
    virtual void draw(void) {draw(setControl.value);};
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
    void draw(int setControlNumber) {
      minusButton.draw();
      setControl.draw(setControlNumber);
      plusButton.draw();
      sensors.draw();
    };
    void draw(void) {draw(setControl.value);};
// TempControl(SensorCLK, SensorCS, SensorDO)
} topTempControl(pinTopTempSensor1CLK, pinTopTempSensor1CS, pinTopTempSensor1DO),
  bottomTempControl(pinBottomTempSensor1CLK, pinBottomTempSensor1CS, pinBottomTempSensor1DO);

class Pid : public PID {
  public:
    int kp, ki, kd;
    double input, output, setpoint;
    Pid(int _kp, int _ki, int _kd, byte P_ON_X=P_ON_E):
      kp(_kp),
      ki(_ki),
      kd(_kd),
      PID(&input, &output, &setpoint, _kp, _ki, _kd, P_ON_X, DIRECT)
    {};

    void updateTuning(void) {SetTunings(kp,ki,kd);};

    void increaseKp (void) {kp++; updateTuning(); topTempControl.setControl.draw(kp);};
    void decreaseKp (void) {kp--; updateTuning(); topTempControl.setControl.draw(kp);};
    void increaseKi (void) {ki++; updateTuning(); cookTimeControl.setControl.draw(ki);};
    void decreaseKi (void) {ki--; updateTuning(); cookTimeControl.setControl.draw(ki);};
    void increaseKd (void) {kd++; updateTuning(); bottomTempControl.setControl.draw(kd);};
    void decreaseKd (void) {kd--; updateTuning(); bottomTempControl.setControl.draw(kd);};

} topPID(PID_KP, PID_KI, PID_KD),
  bottomPID(PID_KP, PID_KI, PID_KD, P_ON_M);

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
  Profile(30,127,30),
  Profile(210,220,230),
  Profile(310,320,330),
  Profile(410,420,430),
  Profile(510,520,530),
};
byte profilesSize = sizeof(profiles) / sizeof(Profile);


// ###############################################################
// #####################   GLOBAL FUNCTIONS   ####################
// ###############################################################


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

void showPIDs(void)
{
  Serial.print("\r\ntopPID Input="); Serial.print(topPID.input);
  Serial.print(" Setpoint="); Serial.print(topPID.setpoint);
  Serial.print(" Output="); Serial.print(topPID.output);
  Serial.print(" kp="); Serial.print(topPID.kp);
  Serial.print(" ki="); Serial.print(topPID.ki);
  Serial.print(" kd="); Serial.print(topPID.kd);
  Serial.print("  |  bottomPID Input="); Serial.print(bottomPID.input);
  Serial.print(" Setpoint="); Serial.print(bottomPID.setpoint);
  Serial.print(" Output="); Serial.print(bottomPID.output);
  Serial.print(" kp="); Serial.print(bottomPID.kp);
  Serial.print(" ki="); Serial.print(bottomPID.ki);
  Serial.print(" kd="); Serial.print(bottomPID.kd);
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
  switch (state) {
    case CONTROLLING_SETPOINTS:
      drawDivisions();
      drawProfiles();
      topTempControl.draw();
      cookTimeControl.draw();
      bottomTempControl.draw();
    break;
    case CONTROLLING_TOP_PID:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(topPID.kp);
      cookTimeControl.draw(topPID.ki);
      bottomTempControl.draw(topPID.kd);
    break;
    case CONTROLLING_BOTTOM_PID:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(bottomPID.kp);
      cookTimeControl.draw(bottomPID.ki);
      bottomTempControl.draw(bottomPID.kd);
    break;
    case SHOWING_GRAPH:
      //clean screen
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(0,0, dispX-1,dispY-1);
    break;
  }
}

void controlSetpoint (void) {
  state = CONTROLLING_SETPOINTS;
  bottomTempControl.sensors.lowlight();
  topTempControl.sensors.lowlight();
  draw();
}
void controlBottomPID (void) {
  state = CONTROLLING_BOTTOM_PID;
  bottomTempControl.sensors.highlight();
  topTempControl.sensors.lowlight();
  draw();
}
void controlTopPID (void) {
  state = CONTROLLING_TOP_PID;
  topTempControl.sensors.highlight();
  bottomTempControl.sensors.lowlight();
  draw();
}
void showGraph (void) {
  state = SHOWING_GRAPH;
  draw();
}

void loadProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].load();
  activeProfile = id;
  controlSetpoint();
}

void saveProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].save();
  activeProfile = id;
  controlSetpoint();
}

void profileClick (byte id=0) {
  if (state == SHOWING_GRAPH) controlSetpoint();  else  loadProfile(id);
}

void profileLongClick (byte id=0) {
  if (state == SHOWING_GRAPH) controlSetpoint();  else  saveProfile(id);
}

void topMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   topTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.decreaseKp(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.decreaseKp(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void centerMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   cookTimeControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.decreaseKi(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.decreaseKi(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void bottomMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   bottomTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.decreaseKd(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.decreaseKd(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void topPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   topTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.increaseKp(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.increaseKp(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void centerPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   cookTimeControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.increaseKi(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.increaseKi(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void bottomPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   bottomTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:     topPID.increaseKd(); break;
    case CONTROLLING_BOTTOM_PID:  bottomPID.increaseKd(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void topSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   controlTopPID(); break;
    case CONTROLLING_TOP_PID:     controlSetpoint(); break;
    case CONTROLLING_BOTTOM_PID:  controlTopPID(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void centerSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   showGraph(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
}
void bottomSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:   controlBottomPID(); break;
    case CONTROLLING_TOP_PID:     controlBottomPID(); break;
    case CONTROLLING_BOTTOM_PID:  controlSetpoint(); break;
    case SHOWING_GRAPH:           controlSetpoint(); break;
  }
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
  // Profiles
  if (tpt.y > profiles[0].startY+DEAD_ZONE  &&  tpt.y < profiles[0].endY-DEAD_ZONE)
  {
    if (tpt.x > profiles[0].startX+DEAD_ZONE  &&  tpt.x < profiles[0].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(0); break;
        case LONG_CLICK_EVENT : profileLongClick(0); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[1].startX+DEAD_ZONE  &&  tpt.x < profiles[1].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(1); break;
        case LONG_CLICK_EVENT : profileLongClick(1); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[2].startX+DEAD_ZONE  &&  tpt.x < profiles[2].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(2); break;
        case LONG_CLICK_EVENT : profileLongClick(2); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[3].startX+DEAD_ZONE  &&  tpt.x < profiles[3].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(3); break;
        case LONG_CLICK_EVENT : profileLongClick(3); break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tpt.x > profiles[4].startX+DEAD_ZONE  &&  tpt.x < profiles[4].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(4); break;
        case LONG_CLICK_EVENT : profileLongClick(4); break;
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
        case CLICK_EVENT :      topMinusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  topMinusButtonClick(); break;
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
        case CLICK_EVENT :      topPlusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  topPlusButtonClick(); break;
      }
    }
    else if (tpt.x > topTempControl.sensors.startX+DEAD_ZONE  &&  tpt.x < topTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topSensorClick(); break;
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
        case CLICK_EVENT :      centerMinusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  centerMinusButtonClick(); break;
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
        case CLICK_EVENT :      centerPlusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  centerPlusButtonClick(); break;
      }
    }
    else if (tpt.x > cookTimeControl.sensors.startX+DEAD_ZONE  &&  tpt.x < cookTimeControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      centerSensorClick(); break;
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
        case CLICK_EVENT :      bottomMinusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  bottomMinusButtonClick(); break;
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
        case CLICK_EVENT :      bottomPlusButtonClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  bottomPlusButtonClick(); break;
      }
    }
    else if (tpt.x > bottomTempControl.sensors.startX+DEAD_ZONE  &&  tpt.x < bottomTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomSensorClick(); break;
        case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  break;
      }
    }
  }
}

void updateSensors (void)
{
  topTempControl.sensors.update();
  bottomTempControl.sensors.update();
}

void drawSensors (void)
{
  topTempControl.sensors.draw();
  bottomTempControl.sensors.draw();
}

void computeTopPID (void)
{
  if (!isnan(topTempControl.sensors.value1))  topPID.input = topTempControl.sensors.value1;
  topPID.setpoint = topTempControl.setControl.value;
  topPID.Compute();
  /*
  if (topPID.output > TOP_SERVO_MAX) topValve.write(TOP_SERVO_MAX);
  else if (topPID.output < TOP_SERVO_MIN) topValve.write(TOP_SERVO_MIN);
  else topValve.write(topPID.output);
  delay(50);
  topValve.detach();
  */
  analogWrite(TOP_SERVO_PIN, topPID.output);
}
void computeBottomPID (void)
{
  if (!isnan(bottomTempControl.sensors.value1))  bottomPID.input = bottomTempControl.sensors.value1;
  bottomPID.setpoint = bottomTempControl.setControl.value;
  bottomPID.Compute();
  //bottomValve.attach(BOTTOM_SERVO_PIN);
  /*
  if (bottomPID.output > BOTTOM_SERVO_MAX) bottomPID.output=BOTTOM_SERVO_MAX;
  else if (bottomPID.output < BOTTOM_SERVO_MIN) bottomPID.output=BOTTOM_SERVO_MIN;
  bottomValve.write(bottomPID.output);
  */
  analogWrite(BOTTOM_SERVO_PIN, bottomPID.output);

  //delay(50);
  //bottomValve.detach();
}

void drawGraphPoint()
{
  static int column;
  // scale so the graph shows 0-400 from bottom to top
  int topInput =        map(topPID.input,       0, 400, dispY-1, 0);
  int topSetpoint =     map(topPID.setpoint,    0, 400, dispY-1, 0);
  int topOutput =       map(topPID.output,      0, 400, dispY-1, 0);
  int bottomInput =     map(bottomPID.input,    0, 400, dispY-1, 0);
  int bottomSetpoint =  map(bottomPID.setpoint, 0, 400, dispY-1, 0);
  int bottomOutput =    map(bottomPID.output,   0, 400, dispY-1, 0);

  // clean column by drawing a BLACK line from top to bottom
  myGLCD.setColor(BLACK);       myGLCD.drawLine(column,0,column,dispY-1);

  // draw points on the graph
  myGLCD.setColor(PALEGREEN);   myGLCD.drawPixel(column, bottomSetpoint);
  myGLCD.setColor(GREEN);       myGLCD.drawPixel(column-1, topSetpoint);
  myGLCD.setColor(CYAN);        myGLCD.drawPixel(column, bottomOutput);
  myGLCD.setColor(BLUE);        myGLCD.drawPixel(column-1, topOutput);
  myGLCD.setColor(MAGENTA);     myGLCD.drawPixel(column, bottomInput);
  myGLCD.setColor(RED);         myGLCD.drawPixel(column-1, topInput);

  column++;
  if (column >= 320) column = 0;
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

  loadProfile(0);
  delay(500);   // wait for MAX chip to stabilize

  // Threads
  updateSensorsThread.onRun(updateSensors);
  updateSensorsThread.setInterval(1000);
  drawSensorsThread.onRun(drawSensors);
  drawSensorsThread.setInterval(3000);
  drawGraphPointThread.onRun(drawGraphPoint);
  drawGraphPointThread.setInterval(200);

  // PIDs
  topPID.input = topTempControl.sensors.value1;
  topPID.setpoint = topTempControl.setControl.value;
  topPID.SetMode(AUTOMATIC);
  topPID.SetSampleTime(1000);
  topPID.SetOutputLimits(TOP_OUTPUT_MIN, TOP_OUTPUT_MAX);
  bottomPID.input = bottomTempControl.sensors.value1;
  bottomPID.setpoint = bottomTempControl.setControl.value;
  bottomPID.SetMode(AUTOMATIC);
  bottomPID.SetSampleTime(1000);
  bottomPID.SetOutputLimits(BOTTOM_OUTPUT_MIN, BOTTOM_OUTPUT_MAX);

  pinMode(CONVEYOR_SERVO_PIN, OUTPUT);
  pinMode(BOTTOM_SERVO_PIN, OUTPUT);
}


// ###############################################################
// ##########################    LOOP    #########################
// ###############################################################


void loop()
{
  // Threads
  if (updateSensorsThread.shouldRun())  {updateSensorsThread.run();  showPIDs();}
  if (drawSensorsThread.shouldRun() && state !=SHOWING_GRAPH)   drawSensorsThread.run();
  if (drawGraphPointThread.shouldRun() && state==SHOWING_GRAPH) drawGraphPointThread.run();

  if (analogRead(CONVEYOR_SERVO_PIN) != cookTimeControl.setControl.value)
    analogWrite(CONVEYOR_SERVO_PIN, cookTimeControl.setControl.value);
  computeTopPID();
  computeBottomPID();

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