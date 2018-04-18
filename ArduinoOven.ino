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
    #define TOP_SERVO_PIN               44
    #define BOTTOM_SERVO_PIN            46
    #define CONVEYOR_L298N_PWM          45    // ENB
    #define CONVEYOR_L298N_DIR1_PIN     41    // IN3
    #define CONVEYOR_L298N_DIR2_PIN     43    // IN4

// Encoder
  #define ENCODER_PIN             21
  #define STEPS_PER_REVOLUTION    200

// PID
  #include <PID_v1.h>
  #define PID_KP 10
  #define PID_KI 0.5
  #define PID_KD 5
  // min/max PWM width in uS
    #define TOP_PID_MIN_WIDTH       780
    #define TOP_PID_MAX_WIDTH       1200
    #define BOTTOM_PID_MIN_WIDTH    750
    #define BOTTOM_PID_MAX_WIDTH    1500
    #define CONVEYOR_PID_MIN_WIDTH  0
    #define CONVEYOR_PID_MAX_WIDTH  255

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
    #define LONG_CLICK_EVENT  1
    #define HOLD_EVENT        2
    #define LONG_HOLD_EVENT   3
    // Time
      #define CLICK_TIME      30              // minimum ms since touch started to trigger CLICK_EVENT
      #define HOLD_TIME       CLICK_TIME      // minimum ms since touch started to trigger HOLD_EVENT
      #define LONG_CLICK_TIME 600             // minimum ms since touch started to trigger LONG_CLICK_EVENT
      #define LONG_HOLD_TIME  LONG_CLICK_TIME // minimum ms since touch started to trigger LONG_HOLD_EVENT

// States
  #define CONTROLLING_SETPOINTS     0
  #define CONTROLLING_TOP_PID       1+CONTROLLING_SETPOINTS
  #define CONTROLLING_BOTTOM_PID    1+CONTROLLING_TOP_PID
  #define CONTROLLING_CONVEYOR_PID  1+CONTROLLING_BOTTOM_PID
  #define SHOWING_GRAPH             1+CONTROLLING_CONVEYOR_PID

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
  #define DRAW_SENSORS_INTERVAL        3000
  #define DRAW_GRAPH_POINT_INTERVAL    1000
// EEPROM
  #include <EEPROM.h>
  struct PidEEPROM { byte kp;  byte ki;  byte kd; };
  struct ServoEEPROM { int minWidth;  int maxWidth; };
  struct ProfileEEPROM { int topTemp;  int conveyorRPH;  int bottomTemp; };
  // Addresses
    // PIDs
      #define TOP_PID_ADDRESS         0
      #define CONVEYOR_PID_ADDRESS    TOP_PID_ADDRESS         +sizeof(PidEEPROM)
      #define BOTTOM_PID_ADDRESS      CONVEYOR_PID_ADDRESS    +sizeof(PidEEPROM)
    // Servos
      #define TOP_SERVO_ADDRESS       BOTTOM_PID_ADDRESS      +sizeof(PidEEPROM)
      #define CONVEYOR_SERVO_ADDRESS  TOP_SERVO_ADDRESS       +sizeof(ServoEEPROM)
      #define BOTTOM_SERVO_ADDRESS    CONVEYOR_SERVO_ADDRESS  +sizeof(ServoEEPROM)
    // Profiles
      #define PROFILE_ADDRESS         BOTTOM_SERVO_ADDRESS    +sizeof(ServoEEPROM)

// ###############################################################
// #####################   GLOBAL VARIABLES   ####################
// ###############################################################


// Display
  int dispX, dispY;
  byte gridWidth, gridHeight, gridInternalWidth, gridInternalHeight;
  bool isOutline = false;

// Touch
  TSPoint avgTouchPoint, last_avgTouchPoint;
  bool TouchStatus;       // the current value read from isPressed()
  bool lastTouchStatus = false;
  long timeTouchStarted, timeSinceTouchStarted, lastTimeSinceTouchStarted;

// Encoder
  long  last_encoderLastStepTime;
  volatile long encoderStepsCounter, encoderStepTime; // volatile to use in interruptions

// Misc
  byte activeProfile;
  byte state;

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

    void highlight(void) { backgroundColor = highlightbackgroundColor; foregroundColor = highlightforegroundColor; };
    void lowlight(void) { backgroundColor = lowlightbackgroundColor; foregroundColor = lowlightforegroundColor; };
};

class MinusButton : public Block {
  public:
    void draw(void) {
      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
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
    int value;
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridWidth+gridInternalWidth; // +gridWidth for 2 columns width
      endY = startY+gridInternalHeight;
    };
    void draw(double number) {
      String number_str;
      char buffer[6];
      if (number == int(number))  number_str=String(int(number));
      else                        number_str=dtostrf(number,1,1, buffer);
      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        myGLCD.setTextSize(SET_CONTROL_TEXT_SIZE);
          myGLCD.print(number_str, startX+12, startY+11);
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
      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
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
    int value;

    void draw(void) {
      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
      myGLCD.setTextColor(foregroundColor, backgroundColor);
        myGLCD.setTextSize(SENSOR_TEXT_SIZE);
          myGLCD.print(String(value), startX+7, startY+20);
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
    void decreaseSetControl(void) {
      setControl.value--;
      setControl.draw();
    }
    void increaseSetControl(void) {
      setControl.value++;
      setControl.draw();
    }
} conveyorControl;

class TempControl : public Control {
  public:
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
// TempControl(SensorCS)
} topTempControl(PIN_CS_TOP_TEMP_SENSOR_1, PIN_CS_TOP_TEMP_SENSOR_2),
  bottomTempControl(PIN_CS_BOTTOM_TEMP_SENSOR_1, PIN_CS_BOTTOM_TEMP_SENSOR_2);

class Pid : public PID {
  public:
    double kp, ki, kd;
    double input, output, setpoint;
    byte EEPROMaddress;
    Pid(double _kp, double _ki, double _kd, byte pOn, byte DIR, byte address):
      kp(_kp),
      ki(_ki),
      kd(_kd),
      EEPROMaddress(address),
      PID(&input, &output, &setpoint, _kp, _ki, _kd, pOn, DIR)
    {};

    void updateTuning(void) {SetTunings(kp,ki,kd);};

    void increaseKp (void) {kp++;     updateTuning(); topTempControl.setControl.draw(kp);};
    void decreaseKp (void) {kp--;     updateTuning(); topTempControl.setControl.draw(kp);};
    void increaseKi (void) {ki+=0.1;  updateTuning(); conveyorControl.setControl.draw(ki);};
    void decreaseKi (void) {ki-=0.1;  updateTuning(); conveyorControl.setControl.draw(ki);};
    void increaseKd (void) {kd++;     updateTuning(); bottomTempControl.setControl.draw(kd);};
    void decreaseKd (void) {kd--;     updateTuning(); bottomTempControl.setControl.draw(kd);};

    void saveTuningParameters (void) {
      PidEEPROM pid;
      pid.kp = kp;
      pid.ki = ki*10;
      pid.kd = kd;
      EEPROM.put(EEPROMaddress, pid);
    }
    void loadTuningParameters (void) {
      PidEEPROM pid;
      EEPROM.get(EEPROMaddress, pid);
      kp = pid.kp;
      ki = pid.ki/10.0;
      kd = pid.kd;
      updateTuning();
    }

} topPID     (TOP_PID_KP     , TOP_PID_KI     , TOP_PID_KD     , P_ON_E, REVERSE, TOP_PID_ADDRESS),
  bottomPID  (BOTTOM_PID_KP  , BOTTOM_PID_KI  , BOTTOM_PID_KD  , P_ON_E, DIRECT , BOTTOM_PID_ADDRESS),
  conveyorPID(CONVEYOR_PID_KP, CONVEYOR_PID_KI, CONVEYOR_PID_KD, P_ON_E, DIRECT , CONVEYOR_PID_ADDRESS);

class Profile : public Block {
  public:
    int bottomTemp;
    int topTemp;
    int conveyorRPH;
    bool isActive = false;
    byte id;

    Profile(int _topTemp, int _conveyorRPH, int _bottomTemp):
      topTemp(_topTemp),
      conveyorRPH(_conveyorRPH),
      bottomTemp(_bottomTemp)
    {};

    void draw (void)
    {
      if (isActive) highlight();
      else lowlight();

      myGLCD.setColor(backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);

      myGLCD.setTextColor(foregroundColor, backgroundColor);
        myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
          myGLCD.print(String(id+1), startX+9 , startY+9);
        myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
          myGLCD.print(String(topTemp), startX+38, startY+6);
          myGLCD.print(String(conveyorRPH), startX+38, startY+6+14);
          myGLCD.print(String(bottomTemp), startX+38, startY+6+28);
    };

    void load(void)
    {
      topTempControl.setControl.value = topTemp;
      conveyorControl.setControl.value = conveyorRPH;
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
      conveyorRPH = conveyorControl.setControl.value;
      bottomTemp = bottomTempControl.setControl.value;
      isActive = true;
      saveToEEPROM();
      draw();
    };

    void saveToEEPROM(void)
    {
      ProfileEEPROM profile;
      profile.topTemp = topTemp;
      profile.conveyorRPH = conveyorRPH;
      profile.bottomTemp = bottomTemp;
      byte address = PROFILE_ADDRESS + sizeof(ProfileEEPROM) * id;
      EEPROM.put(address, profile);
      isActive = true;
      draw();
    };

} profiles[] = {
  // Profile(topTemp, conveyorRPH, bottomTemp)
  Profile(0,0,0),
  Profile(210,100,230),
  Profile(340,180,200),
  Profile(410,100,430),
  Profile(510,100,530),
};
byte profilesSize = sizeof(profiles) / sizeof(Profile);


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
    profiles[i].topTemp = profile.topTemp;
    profiles[i].conveyorRPH = profile.conveyorRPH;
    profiles[i].bottomTemp = profile.bottomTemp;
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
  int temp = map(tp.y, 90, 855, 0, dispX-1);
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
    myGLCD.drawLine(0 , conveyorControl.startY-1 , dispX-1 , conveyorControl.startY-1);
    myGLCD.drawLine(0 , bottomTempControl.startY-1 , dispX-1 , bottomTempControl.startY-1);
}

void draw(void)
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
  }
}

void controlSetpoint (void) {
  state = CONTROLLING_SETPOINTS;
  bottomTempControl.sensors.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  draw();
}
void controlBottomPID (void) {
  state = CONTROLLING_BOTTOM_PID;
  bottomTempControl.sensors.highlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  draw();
}
void controlTopPID (void) {
  state = CONTROLLING_TOP_PID;
  topTempControl.sensors.highlight();
  bottomTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  draw();
}
void controlConveyorPID (void) {
  state = CONTROLLING_CONVEYOR_PID;
  conveyorControl.sensors.highlight();
  topTempControl.sensors.lowlight();
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
  if (state != CONTROLLING_SETPOINTS) controlSetpoint();
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
    case CONTROLLING_SETPOINTS:     topTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.decreaseKp(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.decreaseKp(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.decreaseKp(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void centerMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     conveyorControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.decreaseKi(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.decreaseKi(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.decreaseKi(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void bottomMinusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     bottomTempControl.decreaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.decreaseKd(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.decreaseKd(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.decreaseKd(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void topPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     topTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.increaseKp(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.increaseKp(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.increaseKp(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void centerPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     conveyorControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.increaseKi(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.increaseKi(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.increaseKi(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void bottomPlusButtonClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     bottomTempControl.increaseSetControl(); break;
    case CONTROLLING_TOP_PID:       topPID.increaseKd(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.increaseKd(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.increaseKd(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void topSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlTopPID(); break;
    case CONTROLLING_TOP_PID:       controlSetpoint(); break;
    case CONTROLLING_CONVEYOR_PID:  controlTopPID(); break;
    case CONTROLLING_BOTTOM_PID:    controlTopPID(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void centerSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlConveyorPID(); break;
    case CONTROLLING_TOP_PID:       controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_PID:  controlSetpoint(); break;
    case CONTROLLING_BOTTOM_PID:    controlConveyorPID(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void centerSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlConveyorPID(); break;
    case CONTROLLING_TOP_PID:       controlConveyorPID(); break;
    case CONTROLLING_CONVEYOR_PID:  conveyorPID.saveTuningParameters(); break;
    case CONTROLLING_BOTTOM_PID:    controlConveyorPID(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void bottomSensorClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlBottomPID(); break;
    case CONTROLLING_TOP_PID:       controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_PID:  controlBottomPID(); break;
    case CONTROLLING_BOTTOM_PID:    controlSetpoint(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void bottomSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlBottomPID(); break;
    case CONTROLLING_TOP_PID:       controlBottomPID(); break;
    case CONTROLLING_CONVEYOR_PID:  controlBottomPID(); break;
    case CONTROLLING_BOTTOM_PID:    bottomPID.saveTuningParameters(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
  }
}
void topSensorLongClick (void) {
  switch (state) {
    case CONTROLLING_SETPOINTS:     controlTopPID(); break;
    case CONTROLLING_TOP_PID:       topPID.saveTuningParameters(); break;
    case CONTROLLING_CONVEYOR_PID:  controlTopPID(); break;
    case CONTROLLING_BOTTOM_PID:    controlTopPID(); break;
    case SHOWING_GRAPH:             controlSetpoint(); break;
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
        //case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  topMinusButtonClick(); break;
      }
    }
    else if (tp.x > topTempControl.setControl.startX+DEAD_ZONE  &&  tp.x < topTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        //case CLICK_EVENT :      break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > topTempControl.plusButton.startX+DEAD_ZONE  &&  tp.x < topTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      topPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
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
        case CLICK_EVENT :      centerMinusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  centerMinusButtonClick(); break;
      }
    }
    else if (tp.x > conveyorControl.setControl.startX+DEAD_ZONE  &&  tp.x < conveyorControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        //case CLICK_EVENT :      break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > conveyorControl.plusButton.startX+DEAD_ZONE  &&  tp.x < conveyorControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      centerPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  centerPlusButtonClick(); break;
      }
    }
    else if (tp.x > conveyorControl.sensors.startX+DEAD_ZONE  &&  tp.x < conveyorControl.sensors.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      centerSensorClick(); break;
        case LONG_CLICK_EVENT : centerSensorLongClick(); break;
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
        //case HOLD_EVENT :       break;
        case LONG_HOLD_EVENT :  bottomMinusButtonClick(); break;
      }
    }
    else if (tp.x > bottomTempControl.setControl.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.setControl.endX-DEAD_ZONE)
    {
      switch (event) {
        //case CLICK_EVENT :      break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
        //case LONG_HOLD_EVENT :  break;
      }
    }
    else if (tp.x > bottomTempControl.plusButton.startX+DEAD_ZONE  &&  tp.x < bottomTempControl.plusButton.endX-DEAD_ZONE)
    {
      switch (event) {
        case CLICK_EVENT :      bottomPlusButtonClick(); break;
        //case LONG_CLICK_EVENT : break;
        //case HOLD_EVENT :       break;
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
  topTempControl.sensors.draw();
  conveyorControl.sensors.draw();
  bottomTempControl.sensors.draw();
}

void computeTopPID (void)
{
  if (!isnan(topTempControl.sensors.value1))  topPID.input = topTempControl.sensors.value1;
  // else  To-do: Alert Sensor Error
  else topTempControl.sensors.backgroundColor = MAGENTA;
  topPID.setpoint = topTempControl.setControl.value;
  topPID.Compute();
  topServo.writeMicroseconds(topPID.output);
}
void computeBottomPID (void)
{
  if (!isnan(bottomTempControl.sensors.value1))  bottomPID.input = bottomTempControl.sensors.value1;
  // else  To-do: Alert Sensor Error
  else topTempControl.sensors.backgroundColor = MAGENTA;
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
  double stepsPerMs = conveyorControl.setControl.value * STEPS_PER_REVOLUTION / 3600000.0; // RPH to Steps per milisecond
  double encoderSteps_goal = stepsPerMs * encoderStepsCounter_duration;
  conveyorPID.input += encoderSteps_counted;
  conveyorPID.setpoint += encoderSteps_goal;
  conveyorPID.Compute();
  analogWrite(CONVEYOR_L298N_PWM, conveyorPID.output);
  last_encoderLastStepTime = encoderLastStepTime;
  conveyorControl.sensors.value = encoderSteps_counted * 3600000.0 / STEPS_PER_REVOLUTION / encoderStepsCounter_duration;
}

void drawGraphPoint()
{
  static int column;
  // scale so the graph shows 0-400 from bottom to top
  int topInput =        map(topPID.input,       0, 400, (dispY-1)/2, 0);
  int topSetpoint =     map(topPID.setpoint,    0, 400, (dispY-1)/2, 0);
  int topOutput =       map(topPID.output,      TOP_PID_MIN_WIDTH, TOP_PID_MAX_WIDTH, (dispY-1)/2, 0);
  int bottomInput =     map(bottomPID.input,    0, 400, dispY-1, (dispY-1)/2);
  int bottomSetpoint =  map(bottomPID.setpoint, 0, 400, dispY-1, (dispY-1)/2);
  int bottomOutput =    map(bottomPID.output,   BOTTOM_PID_MIN_WIDTH, BOTTOM_PID_MAX_WIDTH, dispY-1, (dispY-1)/2);

  // clean column by drawing a BLACK line from top to bottom
  myGLCD.setColor(BLACK);       myGLCD.drawLine(column,0,column,dispY-1);

  // draw points on the graph
  myGLCD.setColor(PALEGREEN);   myGLCD.drawPixel(column, bottomSetpoint);
  myGLCD.setColor(CYAN);        myGLCD.drawPixel(column, bottomOutput);
  myGLCD.setColor(MAGENTA);     myGLCD.drawPixel(column, bottomInput);
  myGLCD.setColor(GREEN);       myGLCD.drawPixel(column-1, topSetpoint);
  myGLCD.setColor(BLUE);        myGLCD.drawPixel(column-1, topOutput);
  myGLCD.setColor(RED);         myGLCD.drawPixel(column-1, topInput);

  column++;
  if (column >= 320) column = 0;
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

  controlSetpoint();
  loadProfile(0);
  delay(500);   // wait for MAX chip to stabilize

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
    topPID.loadTuningParameters();
    topPID.input = topTempControl.sensors.value1;
    topPID.setpoint = topTempControl.setControl.value;
    topPID.SetSampleTime(TOP_PID_INTERVAL);
    topPID.SetOutputLimits(TOP_PID_MIN_WIDTH, TOP_PID_MAX_WIDTH);
    topPID.SetMode(AUTOMATIC);
    bottomPID.loadTuningParameters();
    bottomPID.input = bottomTempControl.sensors.value1;
    bottomPID.setpoint = bottomTempControl.setControl.value;
    bottomPID.SetSampleTime(BOTTOM_PID_INTERVAL);
    bottomPID.SetOutputLimits(BOTTOM_PID_MIN_WIDTH, BOTTOM_PID_MAX_WIDTH);
    bottomPID.SetMode(AUTOMATIC);
    conveyorPID.loadTuningParameters();
    conveyorPID.SetSampleTime(CONVEYOR_PID_INTERVAL);
    conveyorPID.SetOutputLimits(CONVEYOR_PID_MIN_WIDTH, CONVEYOR_PID_MAX_WIDTH);
    conveyorPID.SetMode(AUTOMATIC);

  // Servos
    topServo.attach(TOP_SERVO_PIN);
    bottomServo.attach(BOTTOM_SERVO_PIN);
    pinMode(CONVEYOR_L298N_PWM      , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR1_PIN , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR2_PIN , OUTPUT);

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
        draw();
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