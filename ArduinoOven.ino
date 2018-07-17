// Display
  #define  ORIENTATION  1
  #include <Adafruit_GFX.h>
  #include <UTFTGLUE.h>            // Modified file MCUFRIEND_kbv.h: Enabled #define SUPPORT_8347D
  UTFTGLUE myGLCD(0x9341, A2, A1, A3, A4, A0);
  extern uint8_t SmallFont[]; // Declare which fonts we will be using
  // Colors - RGB565 color picker -> https://ee-programming-notepad.blogspot.com.co/2016/10/16-bit-color-generator-picker.html
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

// Thermocouples
  #include <max6675.h>
  #include <SPI.h>
  #define PIN_CS_TOP_TEMP_SENSOR_1     23
  #define PIN_CS_TOP_TEMP_SENSOR_2     25
  #define PIN_CS_BOTTOM_TEMP_SENSOR_1  27
  #define PIN_CS_BOTTOM_TEMP_SENSOR_2  29

// Servos
  #include <Servo.h>  // Modified file ServoTimers.h: disabled timer5 to use analogWrite() on pins 44,45,46. Now timer1 is used.
  Servo topServo;
  Servo bottomServo;
  // pins
    #define TOP_SERVO_PIN               45
    #define BOTTOM_SERVO_PIN            46

// Motor Speed Control with L298N
  #define CONVEYOR_L298N_PWM          44 // ENB
  #define CONVEYOR_L298N_DIR1_PIN     42 // IN3
  #define CONVEYOR_L298N_DIR2_PIN     40 // IN4

// Encoder
  #define ENCODER_PIN             21
  #define STEPS_PER_REVOLUTION    200

// Oven Specific Parameters
  #define STEPS_TO_CROSS_OVEN     STEPS_PER_REVOLUTION * (7+1/3)

// PID
  #include <PID_v1.h>
  // Top
    #define TOP_PID_KP          10
    #define TOP_PID_KI          1
    #define TOP_PID_KD          5
  // Conveyor
    #define CONVEYOR_PID_KP     3
    #define CONVEYOR_PID_KI     0.2
    #define CONVEYOR_PID_KD     0
  // Bottom
    #define BOTTOM_PID_KP       10
    #define BOTTOM_PID_KI       1
    #define BOTTOM_PID_KD       5
  // min/max PWM width in uS
    #define TOP_PID_MIN_WIDTH       780
    #define TOP_PID_MAX_WIDTH       1200
    #define BOTTOM_PID_MIN_WIDTH    750
    #define BOTTOM_PID_MAX_WIDTH    1500
    #define CONVEYOR_PID_MIN_WIDTH  0
    #define CONVEYOR_PID_MAX_WIDTH  255

// Graphs
  #define TOP_TEMP_MIN_RANGE      200
  #define TOP_TEMP_MAX_RANGE      400
  #define BOTTOM_TEMP_MIN_RANGE   200
  #define BOTTOM_TEMP_MAX_RANGE   400
  #define CONVEYOR_MIN_RANGE      -50
  #define CONVEYOR_MAX_RANGE      50

// TouchScreen
  #include <TouchScreen.h>
  #define YP A2   //A3 for ILI9320
  #define YM 6    //9
  #define XM A1
  #define XP 7    //8
  TouchScreen myTouch(XP, YP, XM, YM, 300);
  #define NUM_OF_SAMPLES 10
  #define QUORUM 3
  #define DEAD_ZONE 2 // outest pixels of blocks where Touch-Events are disabled
  // Touch events
    #define CLICK_EVENT       0
    #define LONG_CLICK_EVENT  1+CLICK_EVENT
    #define HOLD_EVENT        1+LONG_CLICK_EVENT
    #define LONG_HOLD_EVENT   1+HOLD_EVENT
    // Time
      #define CLICK_TIME      30              // minimum ms since touch started to trigger CLICK_EVENT
      #define HOLD_TIME       70              // minimum ms since touch started to trigger HOLD_EVENT
      #define LONG_CLICK_TIME 600             // minimum ms since touch started to trigger LONG_CLICK_EVENT
      #define LONG_HOLD_TIME  LONG_CLICK_TIME // minimum ms since touch started to trigger LONG_HOLD_EVENT

// States
  #define CONTROLLING_SETPOINTS                       0
  #define CONTROLLING_TOP_PID                         1+CONTROLLING_SETPOINTS
  #define CONTROLLING_BOTTOM_PID                      1+CONTROLLING_TOP_PID
  #define CONTROLLING_CONVEYOR_PID                    1+CONTROLLING_BOTTOM_PID
  #define SHOWING_GRAPH                               1+CONTROLLING_CONVEYOR_PID
  #define CONTROLLING_TOP_OUTPUT_LIMITS               1+SHOWING_GRAPH
  #define CONTROLLING_CONVEYOR_OUTPUT_LIMITS          1+CONTROLLING_TOP_OUTPUT_LIMITS
  #define CONTROLLING_BOTTOM_OUTPUT_LIMITS            1+CONTROLLING_CONVEYOR_OUTPUT_LIMITS

// Threads
  #include <Thread.h>
  Thread updateSensorsThread    = Thread();
  Thread drawSensorsThread      = Thread();
  Thread topPIDThread           = Thread();
  Thread bottomPIDThread        = Thread();
  Thread conveyorPIDThread      = Thread();
  Thread drawGraphPointThread   = Thread();
  #define UPDATE_SENSORS_INTERVAL      1000
  #define TOP_PID_INTERVAL             1000
  #define BOTTOM_PID_INTERVAL          1000
  #define CONVEYOR_PID_INTERVAL        1000
  #define DRAW_SENSORS_INTERVAL        1000
  #define DRAW_GRAPH_POINT_INTERVAL    300

// Sensors Draw turn
  #define TOP_TURN        0
  #define CONVEYOR_TURN   1+TOP_TURN
  #define BOTTOM_TURN     1+CONVEYOR_TURN

// EEPROM
  #include <EEPROM.h>
  struct PidEEPROM { int kp;  int ki;  int kd; int minOutput;  int startOutput;  int maxOutput; };
  struct ServoEEPROM { int minWidth;  int maxWidth; };
  struct ProfileEEPROM { int topTemp;  int cookTime;  int bottomTemp; };
  // Addresses
    // PIDs
      #define TOP_PID_ADDRESS         0
      #define CONVEYOR_PID_ADDRESS    sizeof(PidEEPROM)+TOP_PID_ADDRESS
      #define BOTTOM_PID_ADDRESS      sizeof(PidEEPROM)+CONVEYOR_PID_ADDRESS
    // Profiles
      #define PROFILE_ADDRESS         sizeof(PidEEPROM)+BOTTOM_PID_ADDRESS

// Relays
    #define SPARK_PIN               35
    #define VALVE_PIN               37
    #define SPARK_IGNITION_TIME     3000 // ms
    //#define VALVE_TIME              200

// ###############################################################
// #####################   GLOBAL VARIABLES   ####################
// ###############################################################


// Display
  int dispX, dispY;
  byte gridWidth, gridHeight, gridInternalWidth, gridInternalHeight;
  bool isOutline = false;

// Touch
  TSPoint avgTouchPoint, last_avgTouchPoint;
  //, touchPoint, touchPointsArr[NUM_OF_SAMPLES];
  // byte touchPointsArrIndex;
  bool TouchStatus;       // the current value read from isPressed()
  bool lastTouchStatus = false;
  long timeTouchStarted, timeSinceTouchStarted, lastTimeSinceTouchStarted;

// Encoder
  long  last_encoderLastStepTime;
  volatile long encoderStepsCounter, encoderStepTime; // volatile to use in interruptions

// Graph
  double *inputGraph, *setpointGraph, *outputGraph;
  int minInputSetpointGraph, maxInputSetpointGraph, minOutputGraph, maxOutputGraph;

// Misc
  byte activeProfile;
  byte state;


// ###############################################################
// ################ GLOBAL FUNCTIONS FOR CLASSES #################
// ###############################################################

String stringifyDouble (double number) {
  char buffer[6];
  if (number == int(number))              return String(int(number));
  else if (number > -1 and number < 10)  return dtostrf(number,4,2, buffer);
  else                                    return dtostrf(number,1,1, buffer);

}

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
    int highlightbackgroundColor = GREEN;
    int highlightforegroundColor = BLACK;
    int lowlightbackgroundColor = BLACK;
    int lowlightforegroundColor = WHITE;
    int old_backgroundColor;

    void highlight(void) { backgroundColor = highlightbackgroundColor; foregroundColor = highlightforegroundColor; };
    void lowlight(void) { backgroundColor = lowlightbackgroundColor; foregroundColor = lowlightforegroundColor; };
    void drawBackground (void) {
      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
    }
    void drawBackgroundIfHasChanged (void) {
      if (old_backgroundColor != backgroundColor) {
        drawBackground();
        old_backgroundColor = backgroundColor;
      }
    }
};

class MinusButton : public Block {
  public:
    void draw(void) {
      drawBackground();
      myGLCD.setColor(foregroundColor);
        myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    MinusButton() {
      backgroundColor = BLUE;
      foregroundColor = BLACK;
      lowlightbackgroundColor = BLUE;
      lowlightforegroundColor = BLACK;
    }
};

class SetControl : public Block {
  public:
    double value;
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridWidth+gridInternalWidth; // +gridWidth for 2 columns width
      endY = startY+gridInternalHeight;
    };
    void draw(double number) {
      drawBackground();
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        String number_str = stringifyDouble(number);
        if (number_str.length() <= 3) {
          myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
          myGLCD.print(number_str, startX+12, startY+11);
        }
        else {
          myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE-1);
          myGLCD.print(number_str, startX+6, startY+14);
        }
    };
    void draw(void) {draw(value);};
};

class PlusButton : public Block {
  public:
    void draw(void) {
      drawBackground();
      myGLCD.setColor(foregroundColor);
        myGLCD.fillRect(startX+27, startY+15, startX+34, startY+46);
        myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    PlusButton() {
      backgroundColor = RED;
      foregroundColor = BLACK;
      lowlightbackgroundColor = RED;
      lowlightforegroundColor = BLACK;
    }
};

class Sensors : public Block {
  public:
    double value1;
    double value2;
    MAX6675 Sensor1;
    MAX6675 Sensor2;
    Sensors(byte pinSensor1CS,byte pinSensor2CS):
      Sensor1(pinSensor1CS),
      Sensor2(pinSensor2CS)
    {};

    void draw(void) {
      drawBackgroundIfHasChanged();
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        myGLCD.setTextSize(SENSOR_TEXT_SIZE);
          myGLCD.print(String(int(value1)), startX+7, startY+7);
          myGLCD.print(String(int(value2)), startX+7, startY+35);

      /* // Draw symbol: ±
      myGLCD.setColor(foregroundColor);
        myGLCD.fillRect(startX+7, startY+35+18 , startX+7+13, startY+35+19);
        myGLCD.fillRect(startX+7, startY+35+7 , startX+7+13, startY+35+8);
        myGLCD.fillRect(startX+7+6, startY+35 , startX+7+7, startY+35+15);
      */
    };
    void update(void) {
      value1 = Sensor1.readCelsius();
      value2 = Sensor2.readCelsius();
    };
};

class Encoder : public Block {
  public:
    double value;

    void draw(void) {
      drawBackground();
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        String value_str = stringifyDouble(value);
        if (value_str.length() <= 3) {
          myGLCD.setTextSize(SENSOR_TEXT_SIZE);
          myGLCD.print(value_str, startX+7, startY+20);
        }
        else {
          myGLCD.setTextSize(SENSOR_TEXT_SIZE-1);
          myGLCD.print(value_str, startX+3, startY+23);
        }
    };
};

class Control : public Coordinates {
  public:
    MinusButton minusButton;
    SetControl setControl;
    PlusButton plusButton;
    Encoder sensors;
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
    virtual void draw(double setControlNumber) {
      minusButton.draw();
      setControl.draw(setControlNumber);
      plusButton.draw();
      sensors.draw();
    };
    virtual void draw(void) {draw(setControl.value);};
    virtual void decreaseSetControl(void) {
      setControl.value-=0.1;
      setControl.draw();
    }
    virtual void increaseSetControl(void) {
      setControl.value+=0.1;
      setControl.draw();
    }
} conveyorControl;

class TempControl : public Control {
  public:
    SetControl setControl;
    Sensors sensors;
    TempControl(byte pinSensor1CS,byte pinSensor2CS):
      sensors(pinSensor1CS, pinSensor2CS)
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
    void draw(double setControlNumber) {
      minusButton.draw();
      setControl.draw(setControlNumber);
      plusButton.draw();
      sensors.draw();
    };
    void draw(void) {draw(setControl.value);};
    void decreaseSetControl(void) {
      setControl.value--;
      setControl.draw();
    }
    void increaseSetControl(void) {
      setControl.value++;
      setControl.draw();
    }
// TempControl(SensorCS1, SensorCS2)
} topTempControl(PIN_CS_TOP_TEMP_SENSOR_1, PIN_CS_TOP_TEMP_SENSOR_2),
  bottomTempControl(PIN_CS_BOTTOM_TEMP_SENSOR_1, PIN_CS_BOTTOM_TEMP_SENSOR_2);

class Pid : public PID {
  public:
    double kp, ki, kd;
    double input, output, setpoint;
    int minOutput, startOutput, maxOutput;
    byte EEPROMaddress, outputLimitsEEPROMaddress;
    Pid(double _kp, double _ki, double _kd, byte pOn, byte DIR, byte address):
      kp(_kp),
      ki(_ki),
      kd(_kd),
      EEPROMaddress(address),
      PID(&input, &output, &setpoint, _kp, _ki, _kd, pOn, DIR)
    {};

    void updateTuning(void) {SetTunings(kp,ki,kd);};

    void updateOutputLimits(void) {SetOutputLimits(minOutput, maxOutput);};

    void increaseKp (void) {kp++;     updateTuning(); topTempControl.setControl.draw(kp);};
    void decreaseKp (void) {kp--;     updateTuning(); topTempControl.setControl.draw(kp);};
    void increaseKi (void) {ki+=0.01;  updateTuning(); conveyorControl.setControl.draw(ki);};
    void decreaseKi (void) {ki-=0.01;  updateTuning(); conveyorControl.setControl.draw(ki);};
    void increaseKd (void) {kd++;     updateTuning(); bottomTempControl.setControl.draw(kd);};
    void decreaseKd (void) {kd--;     updateTuning(); bottomTempControl.setControl.draw(kd);};

    void increaseMinOutput    (void) {minOutput+=10;    updateOutputLimits(); topTempControl.setControl.draw(minOutput);};
    void decreaseMinOutput    (void) {minOutput-=10;    updateOutputLimits(); topTempControl.setControl.draw(minOutput);};
    void increaseStartOutput  (void) {startOutput+=10;  updateOutputLimits(); conveyorControl.setControl.draw(startOutput);};
    void decreaseStartOutput  (void) {startOutput-=10;  updateOutputLimits(); conveyorControl.setControl.draw(startOutput);};
    void increaseMaxOutput    (void) {maxOutput+=10;    updateOutputLimits(); bottomTempControl.setControl.draw(maxOutput);};
    void decreaseMaxOutput    (void) {maxOutput-=10;    updateOutputLimits(); bottomTempControl.setControl.draw(maxOutput);};

    void saveParameters (void) {
      PidEEPROM pid;
      pid.kp = kp;
      pid.ki = ki*100;
      pid.kd = kd;
      pid.minOutput   = minOutput;
      pid.startOutput = startOutput;
      pid.maxOutput   = maxOutput;
      EEPROM.put(EEPROMaddress, pid);
    }
    void loadParameters (void) {
      PidEEPROM pid;
      EEPROM.get(EEPROMaddress, pid);
      if (pid.kp > 0 and pid.kp < 1000) kp = pid.kp;
      if (pid.ki > 0 and pid.ki < 1000) ki = pid.ki/100.0;
      if (pid.kd > 0 and pid.kd < 1000) kd = pid.kd;
      if (pid.minOutput != -1)    minOutput   = pid.minOutput;
      if (pid.startOutput != -1)  startOutput = pid.startOutput;
      if (pid.maxOutput != -1)    maxOutput   = pid.maxOutput;
      updateTuning(); updateOutputLimits();
    }

} topPID     (TOP_PID_KP     , TOP_PID_KI     , TOP_PID_KD     , P_ON_E, REVERSE, TOP_PID_ADDRESS),
  bottomPID  (BOTTOM_PID_KP  , BOTTOM_PID_KI  , BOTTOM_PID_KD  , P_ON_E, REVERSE, BOTTOM_PID_ADDRESS),
  conveyorPID(CONVEYOR_PID_KP, CONVEYOR_PID_KI, CONVEYOR_PID_KD, P_ON_E, DIRECT , CONVEYOR_PID_ADDRESS);

class Profile : public Block {
  public:
    int bottomTemp;
    int topTemp;
    double cookTime;
    bool isActive = false;
    byte id;

    Profile(int _topTemp, int _conveyorCookTime, int _bottomTemp):
      topTemp(_topTemp),
      cookTime(_conveyorCookTime),
      bottomTemp(_bottomTemp)
    {};

    void draw (void)
    {
      if (isActive) highlight();
      else lowlight();
      drawBackground();
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
          myGLCD.print(String(id+1), startX+9 , startY+9);
        myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
          myGLCD.print(String(topTemp), startX+38, startY+6);
          myGLCD.print(stringifyDouble(cookTime), startX+38, startY+6+14);
          myGLCD.print(String(bottomTemp), startX+38, startY+6+28);
    };

    void load(void)
    {
      topTempControl.setControl.value = topTemp;
      conveyorControl.setControl.value = cookTime;
      bottomTempControl.setControl.value = bottomTemp;
      isActive = true;
      draw();
      topTempControl.setControl.draw();
      conveyorControl.setControl.draw();
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
      cookTime = conveyorControl.setControl.value;
      bottomTemp = bottomTempControl.setControl.value;
      isActive = true;
      saveToEEPROM();
      draw();
    };

    void saveToEEPROM(void)
    {
      ProfileEEPROM profile;
      profile.topTemp = topTemp;
      profile.cookTime = int( cookTime * 10 );
      profile.bottomTemp = bottomTemp;
      byte address = PROFILE_ADDRESS + sizeof(ProfileEEPROM) * id;
      EEPROM.put(address, profile);
      isActive = true;
    };

} profiles[] = {
  // Profile(topTemp, cookTime, bottomTemp)
  Profile(0,100,0),
  Profile(340,-150,200),
  Profile(340,150,200),
  Profile(5,5,5),
  Profile(-5,-5,-5),
};
byte profilesSize = sizeof(profiles) / sizeof(Profile);

/*
class Profiles : public Block {
  public:
  Profile profile1;
  Profile profile2;
  Profile profile3;
  Profile profile4;
  Profile profile5;

  Profiles(Profile _profile1, Profile _profile2, Profile _profile3, Profile _profile4, Profile _profile5):
    profile1(_profile1),
    profile2(_profile2),
    profile3(_profile3),
    profile4(_profile4),
    profile5(_profile5)
  {};

  void load(byte id)
  {
    profiles[activeProfile].unload();
    profiles[id].load();
    activeProfile = id;
  }

} profiles2(
  // Profile(topTemp, cookTime, bottomTemp)
  Profile(110,120,130),
  Profile(210,220,230),
  Profile(310,320,330),
  Profile(410,420,430),
  Profile(510,520,530)
);
*/


// ###############################################################
// #####################   GLOBAL FUNCTIONS   ####################
// ###############################################################


void calculateProfilesProperties (void)
{
  for (byte i=0; i<profilesSize; i++)
  {
    ProfileEEPROM profile;
    byte address = PROFILE_ADDRESS + sizeof(ProfileEEPROM) * i;
    EEPROM.get(address, profile);
    if (profile.topTemp > -0 and profile.topTemp < 1000)          profiles[i].topTemp = profile.topTemp;
    if (profile.cookTime > -1000 and profile.cookTime < 1000)        profiles[i].cookTime = profile.cookTime / 10.0;
    if (profile.bottomTemp > -0 and profile.bottomTemp < 1000)    profiles[i].bottomTemp = profile.bottomTemp;
    profiles[i].id = i;
    profiles[i].startX = gridWidth*i + isOutline;
    profiles[i].startY = isOutline;
    profiles[i].endX = profiles[i].startX + gridInternalWidth;
    profiles[i].endY = topTempControl.startY - 2;
  }
}

TSPoint readResistiveTouch(void)
{
  TSPoint tp = myTouch.getPoint();
  pinMode(YP, OUTPUT);      //restore shared pins
  pinMode(XM, OUTPUT);
  return tp;
}

TSPoint getAvgTouchPoint(void)
{
  TSPoint tp;
  byte count = 0;
  for (byte i=0; i < NUM_OF_SAMPLES; i++)
  {
    TSPoint tptmp = readResistiveTouch();
    if (tptmp.z > 50 )
    {
      tp.x += tptmp.x;
      tp.y += tptmp.y;
      tp.z += tptmp.z;
      count++;
    }
  }
  if (count < QUORUM) tp.z=0;
  else { tp.x /= count; tp.y /= count; tp.z /= count; } // get average
  int temp = map(tp.y, 120, 870, 0, dispX-1);
  tp.y = map(tp.x, 907, 130, 0, dispY-1);
  tp.x = temp;
  return tp;
}

bool isPressed (TSPoint tp)
{
  return tp.z > 0;
}

bool hasTouchStatusChanged () {
  return TouchStatus != lastTouchStatus;
}
/*
void showPIDs(Pid pid)
{
  Serial.print("\r\nInput="); Serial.print(pid.input);
  Serial.print(" Setpoint="); Serial.print(pid.setpoint);
  Serial.print(" Output="); Serial.print(pid.output);
  Serial.print(" kp="); Serial.print(pid.GetKp());
  Serial.print(" ki="); Serial.print(pid.GetKi());
  Serial.print(" kd="); Serial.print(pid.GetKd());
  Serial.println();
}
*/

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
    myGLCD.drawLine(0 , conveyorControl.startY-1 , dispX-1 , conveyorControl.startY-1);
    myGLCD.drawLine(0 , bottomTempControl.startY-1 , dispX-1 , bottomTempControl.startY-1);
}

void drawEverything(void)
{
  switch (state) {
    case CONTROLLING_SETPOINTS:
      drawDivisions();
      drawProfiles();
      topTempControl.draw();
      conveyorControl.draw();
      bottomTempControl.draw();
    break;
    case CONTROLLING_TOP_PID:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(topPID.kp);
      conveyorControl.draw(topPID.ki);
      bottomTempControl.draw(topPID.kd);
    break;
    case CONTROLLING_BOTTOM_PID:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(bottomPID.kp);
      conveyorControl.draw(bottomPID.ki);
      bottomTempControl.draw(bottomPID.kd);
    break;
    case CONTROLLING_CONVEYOR_PID:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(conveyorPID.kp);
      conveyorControl.draw(conveyorPID.ki);
      bottomTempControl.draw(conveyorPID.kd);
    break;
    case SHOWING_GRAPH:
      //clean screen
      myGLCD.setColor(BLACK);      myGLCD.fillRect(0,0, dispX-1,dispY-1);
    break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(topPID.minOutput);
      conveyorControl.draw(topPID.startOutput);
      bottomTempControl.draw(topPID.maxOutput);
    break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(bottomPID.minOutput);
      conveyorControl.draw(bottomPID.startOutput);
      bottomTempControl.draw(bottomPID.maxOutput);
    break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:
      drawDivisions();
      drawProfiles();
      topTempControl.draw(conveyorPID.minOutput);
      conveyorControl.draw(conveyorPID.startOutput);
      bottomTempControl.draw(conveyorPID.maxOutput);
    break;
  }
}

void controlSetpoints (void) {
  state = CONTROLLING_SETPOINTS;
  bottomTempControl.setControl.lowlight();
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.sensors.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  drawEverything();
}
void controlBottomPID (void) {
  state = CONTROLLING_BOTTOM_PID;
  bottomTempControl.sensors.highlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  drawEverything();
}
void controlTopPID (void) {
  state = CONTROLLING_TOP_PID;
  topTempControl.sensors.highlight();
  bottomTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  drawEverything();
}
void controlConveyorPID (void) {
  state = CONTROLLING_CONVEYOR_PID;
  conveyorControl.sensors.highlight();
  topTempControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void showTopPIDGraph (void) {
  inputGraph      = &topPID.input;
  setpointGraph   = &topPID.setpoint;
  outputGraph     = &topPID.output;
  minInputSetpointGraph = TOP_TEMP_MIN_RANGE;
  maxInputSetpointGraph = TOP_TEMP_MAX_RANGE;
  minOutputGraph   = topPID.minOutput;
  maxOutputGraph   = topPID.maxOutput;
  state           = SHOWING_GRAPH;
  drawEverything();
}
void showBottomPIDGraph (void) {
  inputGraph      = &bottomPID.input;
  setpointGraph   = &bottomPID.setpoint;
  outputGraph     = &bottomPID.output;
  minInputSetpointGraph = BOTTOM_TEMP_MIN_RANGE;
  maxInputSetpointGraph = BOTTOM_TEMP_MAX_RANGE;
  minOutputGraph   = bottomPID.minOutput;
  maxOutputGraph   = bottomPID.maxOutput;
  state           = SHOWING_GRAPH;
  drawEverything();
}
void showConveyorPIDGraph (void) {
  inputGraph      = &conveyorPID.input;
  setpointGraph   = &conveyorPID.setpoint;
  outputGraph     = &conveyorPID.output;
  minInputSetpointGraph = CONVEYOR_MIN_RANGE;
  maxInputSetpointGraph = CONVEYOR_MAX_RANGE;
  minOutputGraph   = conveyorPID.minOutput;
  maxOutputGraph   = conveyorPID.maxOutput;
  state           = SHOWING_GRAPH;
  drawEverything();
}
void controlTopOutputLimits (void) {
  state = CONTROLLING_TOP_OUTPUT_LIMITS;
  topTempControl.setControl.highlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  drawEverything();
}
void controlConveyorOutputLimits (void) {
  state = CONTROLLING_CONVEYOR_OUTPUT_LIMITS;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.highlight();
  bottomTempControl.setControl.lowlight();
  drawEverything();
}
void controlBottomOutputLimits (void) {
  state = CONTROLLING_BOTTOM_OUTPUT_LIMITS;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.highlight();
  drawEverything();
}

void loadProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].load();
  activeProfile = id;
  if (state != CONTROLLING_SETPOINTS) controlSetpoints();
}

void saveProfile(byte id)
{
  profiles[activeProfile].unload();
  profiles[id].save();
  activeProfile = id;
  if (state != CONTROLLING_SETPOINTS) controlSetpoints();
}

void profileClick (byte id=0) {
  if (state == SHOWING_GRAPH) controlSetpoints();  else  loadProfile(id);
}

void profileLongClick (byte id=0) {
  if (state == SHOWING_GRAPH) controlSetpoints();  else  saveProfile(id);
}

void topMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               topTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKp(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKp(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKp(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseMinOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseMinOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseMinOutput(); break;
  }
}
void topSetcontrolClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               showTopPIDGraph(); break;
    case CONTROLLING_TOP_PID:                 showTopPIDGraph(); break;
    case CONTROLLING_CONVEYOR_PID:            showTopPIDGraph(); break;
    case CONTROLLING_BOTTOM_PID:              showTopPIDGraph(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlSetpoints(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  showTopPIDGraph(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    showTopPIDGraph(); break;
  }
}
void topSetcontrolLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlTopOutputLimits(); break;
    case CONTROLLING_TOP_PID:                 controlTopOutputLimits(); break;
    case CONTROLLING_CONVEYOR_PID:            controlTopOutputLimits(); break;
    case CONTROLLING_BOTTOM_PID:              controlTopOutputLimits(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.saveParameters(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlTopOutputLimits(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlTopOutputLimits(); break;
  }
}
void topPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               topTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKp(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKp(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKp(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseMinOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseMinOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseMinOutput(); break;
  }
}
void topSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlTopPID(); break;
    case CONTROLLING_TOP_PID:                 controlSetpoints(); break;
    case CONTROLLING_CONVEYOR_PID:            controlTopPID(); break;
    case CONTROLLING_BOTTOM_PID:              controlTopPID(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlTopPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlTopPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlTopPID(); break;
  }
}
void topSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlTopPID(); break;
    case CONTROLLING_TOP_PID:                 topPID.saveParameters(); break;
    case CONTROLLING_CONVEYOR_PID:            controlTopPID(); break;
    case CONTROLLING_BOTTOM_PID:              controlTopPID(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlTopPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlTopPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlTopPID(); break;
  }
}
void conveyorMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               conveyorControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKi(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKi(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKi(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseStartOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseStartOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseStartOutput(); break;
  }
}
void conveyorSetcontrolClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               showConveyorPIDGraph(); break;
    case CONTROLLING_TOP_PID:                 showConveyorPIDGraph(); break;
    case CONTROLLING_CONVEYOR_PID:            showConveyorPIDGraph(); break;
    case CONTROLLING_BOTTOM_PID:              showConveyorPIDGraph(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       showConveyorPIDGraph(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlSetpoints(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    showConveyorPIDGraph(); break;
  }
}
void conveyorSetcontrolLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlConveyorOutputLimits(); break;
    case CONTROLLING_TOP_PID:                 controlConveyorOutputLimits(); break;
    case CONTROLLING_CONVEYOR_PID:            controlConveyorOutputLimits(); break;
    case CONTROLLING_BOTTOM_PID:              controlConveyorOutputLimits(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlConveyorOutputLimits(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.saveParameters(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlConveyorOutputLimits(); break;
  }
}
void conveyorPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               conveyorControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKi(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKi(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKi(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseStartOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseStartOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseStartOutput(); break;
  }
}
void conveyorSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlConveyorPID(); break;
    case CONTROLLING_TOP_PID:                 controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_PID:            controlSetpoints(); break;
    case CONTROLLING_BOTTOM_PID:              controlConveyorPID(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlConveyorPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlConveyorPID(); break;
  }
}
void conveyorSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlConveyorPID(); break;
    case CONTROLLING_TOP_PID:                 controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.saveParameters(); break;
    case CONTROLLING_BOTTOM_PID:              controlConveyorPID(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlConveyorPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlConveyorPID(); break;
  }
}
void bottomMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               bottomTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKd(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKd(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKd(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseMaxOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseMaxOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseMaxOutput(); break;
  }
}
void bottomSetcontrolClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               showBottomPIDGraph(); break;
    case CONTROLLING_TOP_PID:                 showBottomPIDGraph(); break;
    case CONTROLLING_CONVEYOR_PID:            showBottomPIDGraph(); break;
    case CONTROLLING_BOTTOM_PID:              showBottomPIDGraph(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       showBottomPIDGraph(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  showBottomPIDGraph(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlSetpoints(); break;
  }
}
void bottomSetcontrolLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlBottomOutputLimits(); break;
    case CONTROLLING_TOP_PID:                 controlBottomOutputLimits(); break;
    case CONTROLLING_CONVEYOR_PID:            controlBottomOutputLimits(); break;
    case CONTROLLING_BOTTOM_PID:              controlBottomOutputLimits(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlBottomOutputLimits(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlBottomOutputLimits(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.saveParameters(); break;
  }
}
void bottomPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               bottomTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKd(); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKd(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKd(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseMaxOutput(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseMaxOutput(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseMaxOutput(); break;
  }
}
void bottomSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlBottomPID(); break;
    case CONTROLLING_TOP_PID:                 controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_PID:            controlBottomPID(); break;
    case CONTROLLING_BOTTOM_PID:              controlSetpoints(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlBottomPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlBottomPID(); break;
  }
}
void bottomSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               controlBottomPID(); break;
    case CONTROLLING_TOP_PID:                 controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_PID:            controlBottomPID(); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.saveParameters(); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  controlBottomPID(); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    controlBottomPID(); break;
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
void findObjectFromCoordAndExecuteAction (TSPoint tp, byte event)
{
  // Profiles
  if (tp.y > profiles[0].startY+DEAD_ZONE  &&  tp.y < profiles[0].endY-DEAD_ZONE)
  {
    if (tp.x > profiles[0].startX+DEAD_ZONE  &&  tp.x < profiles[0].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(0); break;
        case LONG_CLICK_EVENT : profileLongClick(0); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > profiles[1].startX+DEAD_ZONE  &&  tp.x < profiles[1].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(1); break;
        case LONG_CLICK_EVENT : profileLongClick(1); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > profiles[2].startX+DEAD_ZONE  &&  tp.x < profiles[2].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(2); break;
        case LONG_CLICK_EVENT : profileLongClick(2); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > profiles[3].startX+DEAD_ZONE  &&  tp.x < profiles[3].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(3); break;
        case LONG_CLICK_EVENT : profileLongClick(3); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > profiles[4].startX+DEAD_ZONE  &&  tp.x < profiles[4].endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      profileClick(4); break;
        case LONG_CLICK_EVENT : profileLongClick(4); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tp.y > topTempControl.startY+DEAD_ZONE  &&  tp.y < topTempControl.endY-DEAD_ZONE)
  {
    if (tp.x > topTempControl.minusButton.startX+DEAD_ZONE  &&  tp.x < topTempControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topMinusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       topMinusButtonClick(); break;
        case LONG_HOLD_EVENT :  topMinusButtonClick(); break;
      }
    }
    else if (tp.x > topTempControl.setControl.startX+DEAD_ZONE  &&  tp.x < topTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :        topSetcontrolClick(); break;
        case LONG_CLICK_EVENT :   topSetcontrolLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > topTempControl.plusButton.startX+DEAD_ZONE  &&  tp.x < topTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       topPlusButtonClick(); break;
        case LONG_HOLD_EVENT :  topPlusButtonClick(); break;
      }
    }
    else if (tp.x > topTempControl.sensors.startX+DEAD_ZONE  &&  tp.x < topTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topSensorClick(); break;
        case LONG_CLICK_EVENT : topSensorLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tp.y > conveyorControl.startY+DEAD_ZONE  &&  tp.y < conveyorControl.endY-DEAD_ZONE)
  {
    if (tp.x > conveyorControl.minusButton.startX+DEAD_ZONE  &&  tp.x < conveyorControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      conveyorMinusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       conveyorMinusButtonClick(); break;
        case LONG_HOLD_EVENT :  conveyorMinusButtonClick(); break;
      }
    }
    else if (tp.x > conveyorControl.setControl.startX+DEAD_ZONE  &&  tp.x < conveyorControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :        conveyorSetcontrolClick(); break;
        case LONG_CLICK_EVENT :   conveyorSetcontrolLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > conveyorControl.plusButton.startX+DEAD_ZONE  &&  tp.x < conveyorControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      conveyorPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       conveyorPlusButtonClick(); break;
        case LONG_HOLD_EVENT :  conveyorPlusButtonClick(); break;
      }
    }
    else if (tp.x > conveyorControl.sensors.startX+DEAD_ZONE  &&  tp.x < conveyorControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      conveyorSensorClick(); break;
        case LONG_CLICK_EVENT : conveyorSensorLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
  }
  else if (tp.y > bottomTempControl.startY+DEAD_ZONE  &&  tp.y < bottomTempControl.endY-DEAD_ZONE)
  {
    if (tp.x > bottomTempControl.minusButton.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.minusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomMinusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       bottomMinusButtonClick(); break;
        case LONG_HOLD_EVENT :  bottomMinusButtonClick(); break;
      }
    }
    else if (tp.x > bottomTempControl.setControl.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :        bottomSetcontrolClick(); break;
        case LONG_CLICK_EVENT :   bottomSetcontrolLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > bottomTempControl.plusButton.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       bottomPlusButtonClick(); break;
        case LONG_HOLD_EVENT :  bottomPlusButtonClick(); break;
      }
    }
    else if (tp.x > bottomTempControl.sensors.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomSensorClick(); break;
        case LONG_CLICK_EVENT : bottomSensorLongClick(); break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
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
  static byte turn;
  switch (turn) {
    case TOP_TURN:         topTempControl.sensors.draw(); break;
    case CONVEYOR_TURN:    conveyorControl.sensors.draw(); break;
    case BOTTOM_TURN:      bottomTempControl.sensors.draw(); break;
  }
  if (turn == BOTTOM_TURN)  turn = TOP_TURN;
  else                      turn++;
}

void computeTopPID (void)
{
  byte validReads=0; // If it's not set to 0, then "if (validReads > 0)" will always be True
  double valueSum;
  if (!isnan(topTempControl.sensors.value1))
    { valueSum += topTempControl.sensors.value1; validReads++; }
  if (!isnan(topTempControl.sensors.value2))
    { valueSum += topTempControl.sensors.value2; validReads++; }
  if (validReads > 0)   topPID.input = valueSum / validReads;
  else topTempControl.sensors.backgroundColor = MAGENTA;
  // else  To-do: Alert Sensor Error

  topPID.setpoint = topTempControl.setControl.value;
  topPID.Compute();
  topServo.writeMicroseconds(topPID.output);
}
void computeBottomPID (void)
{
  byte validReads=0; // If it's not set to 0, then "if (validReads > 0)" will always be True
  double valueSum;
  if (!isnan(bottomTempControl.sensors.value1))
    { valueSum += bottomTempControl.sensors.value1; validReads++; }
  if (!isnan(bottomTempControl.sensors.value2))
    { valueSum += bottomTempControl.sensors.value2; validReads++; }
  if (validReads > 0)   bottomPID.input = valueSum / validReads;
  else bottomTempControl.sensors.backgroundColor = MAGENTA;
  // else  To-do: Alert Sensor Error

  bottomPID.setpoint = bottomTempControl.setControl.value;
  bottomPID.Compute();
  bottomServo.writeMicroseconds(bottomPID.output);
}
void computeConveyorPID (void)
{
  noInterrupts();
    long encoderLastStepTime = encoderStepTime;
    long encoderSteps_counted = encoderStepsCounter;
    encoderStepsCounter=0;
  interrupts();
  static int oldSetcontrolValue;
  if (conveyorControl.setControl.value != oldSetcontrolValue)
  {
    if (conveyorControl.setControl.value < 0)
      digitalWrite(CONVEYOR_L298N_DIR1_PIN, HIGH), digitalWrite(CONVEYOR_L298N_DIR2_PIN, LOW);
    else
      digitalWrite(CONVEYOR_L298N_DIR1_PIN, LOW), digitalWrite(CONVEYOR_L298N_DIR2_PIN, HIGH);

    oldSetcontrolValue = conveyorControl.setControl.value;
  }
  if (encoderSteps_counted == 0)  encoderLastStepTime = millis(); // If no encoder steps
  long encoderStepsCounter_duration = encoderLastStepTime - last_encoderLastStepTime;
  double msToCrossOven_goal = abs(conveyorControl.setControl.value) * 60000; // convert from minutes-to-cross-oven to miliseconds-to-cross-oven
  double stepsPerMs_goal = STEPS_TO_CROSS_OVEN / msToCrossOven_goal;
  double encoderSteps_counted_goal = stepsPerMs_goal * encoderStepsCounter_duration;
  /*
  Serial.print(" | encoderLastStepTime = "); Serial.print(encoderLastStepTime);
  Serial.print(" | last_encoderLastStepTime = "); Serial.print(last_encoderLastStepTime);
  Serial.print(" | conveyorControl.setControl.value = "); Serial.print(conveyorControl.setControl.value);
  Serial.print(" | stepsPerMs_goal = "); Serial.print(stepsPerMs_goal);
  Serial.print(" | encoderSteps_counted_goal = "); Serial.print(encoderSteps_counted_goal);
  Serial.println();
  */
  conveyorPID.input += encoderSteps_counted - encoderSteps_counted_goal;
  conveyorPID.setpoint = 0;
  conveyorPID.Compute();
  analogWrite(CONVEYOR_L298N_PWM, conveyorPID.output);
  last_encoderLastStepTime = encoderLastStepTime;
  if (encoderSteps_counted == 0) conveyorControl.sensors.value = 9999;
  else conveyorControl.sensors.value = STEPS_TO_CROSS_OVEN / 60000.0 / encoderSteps_counted * encoderStepsCounter_duration;
}

void drawGraphPoint()
{
  static int column;
  // scale so the graph shows 0-400 from bottom to top
  int input    = map(*inputGraph,    minInputSetpointGraph, maxInputSetpointGraph, dispY-1, 0);
  int setpoint = map(*setpointGraph, minInputSetpointGraph, maxInputSetpointGraph, dispY-1, 0);
  int output   = map(*outputGraph,   minOutputGraph,        maxOutputGraph,        dispY-1, 0);

  // clean column by drawing a BLACK line from top to bottom
  myGLCD.setColor(BLACK);       myGLCD.drawLine(column,0,column,dispY-1);

  // draw points on the graph
  myGLCD.setColor(GREEN);       myGLCD.drawPixel(column, setpoint);
  myGLCD.setColor(BLUE);        myGLCD.drawPixel(column, output);
  myGLCD.setColor(RED);         myGLCD.drawPixel(column, input);

  if (column < 320) column++;
  else              column = 0;
}

void increaseEncoderCounter (void)
{
  if (millis()-encoderStepTime > 5)   // filter rebounds
  {
    encoderStepsCounter++;
    encoderStepTime = millis();
  }
}


// ###############################################################
// #########################    SETUP    #########################
// ###############################################################


void setup()
{
  SPI.begin();
  Serial.begin(9600);
  Serial.println("ArduinoOven");

  // Display init
    digitalWrite(A0, HIGH);
    pinMode(A0, OUTPUT);
    myGLCD.InitLCD(ORIENTATION);
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
    conveyorControl.setCoordinates( isOutline, bottomTempControl.startY - gridHeight );
    topTempControl.setCoordinates( isOutline, conveyorControl.startY - gridHeight );
    calculateProfilesProperties();

  controlSetpoints();
  loadProfile(0);

  // Threads
    updateSensorsThread.onRun(updateSensors);
    updateSensorsThread.setInterval(UPDATE_SENSORS_INTERVAL);
    topPIDThread.onRun(computeTopPID);
    topPIDThread.setInterval(TOP_PID_INTERVAL);
    bottomPIDThread.onRun(computeBottomPID);
    bottomPIDThread.setInterval(BOTTOM_PID_INTERVAL);
    conveyorPIDThread.onRun(computeConveyorPID);
    conveyorPIDThread.setInterval(CONVEYOR_PID_INTERVAL);
    drawSensorsThread.onRun(drawSensors);
    drawSensorsThread.setInterval(DRAW_SENSORS_INTERVAL);
    drawGraphPointThread.onRun(drawGraphPoint);
    drawGraphPointThread.setInterval(DRAW_GRAPH_POINT_INTERVAL);

  // PIDs
    topPID.loadParameters();
    topPID.SetSampleTime(TOP_PID_INTERVAL);
    topPID.SetMode(AUTOMATIC);
    bottomPID.loadParameters();
    bottomPID.SetSampleTime(BOTTOM_PID_INTERVAL);
    bottomPID.SetMode(AUTOMATIC);
    conveyorPID.loadParameters();
    conveyorPID.SetSampleTime(CONVEYOR_PID_INTERVAL);
    conveyorPID.SetMode(AUTOMATIC);

  // Servos
    topServo.attach(TOP_SERVO_PIN);         topServo.writeMicroseconds(topPID.startOutput);
    bottomServo.attach(BOTTOM_SERVO_PIN);   bottomServo.writeMicroseconds(bottomPID.startOutput);
    pinMode(CONVEYOR_L298N_PWM      , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR1_PIN , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR2_PIN , OUTPUT);

  // Relays
    pinMode(VALVE_PIN, OUTPUT);
    digitalWrite(VALVE_PIN, LOW);
    pinMode(SPARK_PIN, OUTPUT);
    digitalWrite(SPARK_PIN, LOW);
    delay(SPARK_IGNITION_TIME);     // Turn on Sparks for SPARK_IGNITION_TIME miliseconds
    digitalWrite(SPARK_PIN, HIGH);

  // Encoder
    pinMode(ENCODER_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), increaseEncoderCounter, CHANGE);
    encoderStepTime=millis();
}


// ###############################################################
// ##########################    LOOP    #########################
// ###############################################################


void loop()
{
  // Threads
  if (updateSensorsThread.shouldRun())      updateSensorsThread.run();
  if (topPIDThread.shouldRun())             topPIDThread.run();
  if (bottomPIDThread.shouldRun())          bottomPIDThread.run();
  if (conveyorPIDThread.shouldRun())        conveyorPIDThread.run();
  if (state == SHOWING_GRAPH) {
    if (drawGraphPointThread.shouldRun())   drawGraphPointThread.run();
  } else {
    if (drawSensorsThread.shouldRun())      drawSensorsThread.run();
  }

  // Checking Touch
  avgTouchPoint = getAvgTouchPoint();
  TouchStatus=isPressed(avgTouchPoint);
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
      findObjectFromCoordAndExecuteAction(last_avgTouchPoint, LONG_CLICK_EVENT);
    }
    else if (timeSinceTouchStarted > CLICK_TIME)
    {
      findObjectFromCoordAndExecuteAction(last_avgTouchPoint, CLICK_EVENT);
    }
    lastTouchStatus=TouchStatus;
  }
  else
  {
    if (TouchStatus)
    {
      if (lastTimeSinceTouchStarted < LONG_CLICK_TIME && timeSinceTouchStarted > LONG_CLICK_TIME)
        drawProfiles();
      if (timeSinceTouchStarted > LONG_HOLD_TIME)
      {
        findObjectFromCoordAndExecuteAction(avgTouchPoint, LONG_HOLD_EVENT);
      }
      else if (timeSinceTouchStarted > HOLD_TIME)
      {
        findObjectFromCoordAndExecuteAction(avgTouchPoint, HOLD_EVENT);
      }
    }
  }
  last_avgTouchPoint = avgTouchPoint;
}