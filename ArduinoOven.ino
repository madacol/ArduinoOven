// Serial Options
//#define DEBUG
//#define DEBUG_JSON
  #define DEBUG_PID_JSON
#if defined DEBUG_JSON
  #define DEBUG_CONVEYOR_PID_JSON
  #define DEBUG_SENSOR_TEMP_JSON
#endif
#if defined DEBUG
  #define DEBUG_TOUCHSCREEN
  #define DEBUG_SENSOR_SHOW_ERROR
  #define DEBUG_PID_GRAPH
#endif

  //#define SERIAL_COMMANDER

// Display
  #define  DISPLAY_WIDTH    320
  #define  DISPLAY_HEIGHT   240
  #define  ORIENTATION  1
  #include <Adafruit_GFX.h>
  #include <UTFTGLUE.h>            // Modified file MCUFRIEND_kbv.cpp: Enabled #define SUPPORT_8347D
  UTFTGLUE myGLCD(0x9341, A2, A1, A3, A4, A0);
  extern uint8_t SmallFont[]; // Declare which fonts we will be using
  // Colors - RGB565 color picker -> https://ee-programming-notepad.blogspot.com.co/2016/10/16-bit-color-generator-picker.html
    #define YELLOW      0xFFE0
    #define WHITE       0xFFFF
    #define BLACK       0x0000
    #define BLUE        0x001F
    #define RED         0xF800
    #define GREEN       0x07E0
    #define GRAY        0x7BEF
    #define CYAN        0x07FF
    #define MAGENTA     0xF81F
    #define PALEGREEN   0x9FD3
  // Text sizes
    #define SET_CONTROL_TEXT_SIZE   6
    #define PROFILE_ID_TEXT_SIZE    4
    #define PROFILE_PARAM_TEXT_SIZE 1
    #define SENSOR_TEXT_SIZE        3

// Thermocouples
  #include <max6675.h>      // https://github.com/SirUli/MAX6675
  #include <SPI.h>
  #define PIN_CS_TOP_TEMP_SENSOR_1     23
  #define PIN_CS_TOP_TEMP_SENSOR_2     25
  #define PIN_CS_BOTTOM_TEMP_SENSOR_1  27
  #define PIN_CS_BOTTOM_TEMP_SENSOR_2  29
  #define NUM_OF_MEASUREMENTS_TO_READ  10
  #define MAX_TEMP_SENSOR_ERROR        30

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
  #include <Encoder.h>
  #define ENCODER_PIN_1               20
  #define ENCODER_PIN_2               21
  Encoder myEncoder(ENCODER_PIN_1, ENCODER_PIN_2);
  #define ENCODER_REBOUND_MS          20

// Arduino and Oven Specific Parameters
  #define STEPS_PER_GEAR_REVOLUTION             400.0
  #define GEAR_REVOLUTIONS_TO_CROSS_OVEN        7.333333333
  #define CONVEYOR_MAX_STEPS_PER_MS             STEPS_PER_GEAR_REVOLUTION / 8000.0 // 0.05
  #define STEPS_TO_CROSS_OVEN                   STEPS_PER_GEAR_REVOLUTION * GEAR_REVOLUTIONS_TO_CROSS_OVEN
  #define ARDUINO_TIME_CORRECTION               1.111
  #define STEPS_TO_CROSS_OVEN__TIME_CORRECTED   STEPS_TO_CROSS_OVEN * ARDUINO_TIME_CORRECTION

// PID
  #include <PID_v1.h>
  #define PID_MANUAL_THRESHOLD  50
  // Top
    #define TOP_PID_KP          5
    #define TOP_PID_KI          0.002
    #define TOP_PID_KD          1
  // Conveyor
    #define CONVEYOR_PID_KP     3
    #define CONVEYOR_PID_KI     0.2
    #define CONVEYOR_PID_KD     0
  // Bottom
    #define BOTTOM_PID_KP       5
    #define BOTTOM_PID_KI       0.001
    #define BOTTOM_PID_KD       1
  // min/max PWM width in uS
    #define TOP_PID_MIN_WIDTH       780
    #define TOP_PID_MAX_WIDTH       1200
    #define BOTTOM_PID_MIN_WIDTH    750
    #define BOTTOM_PID_MAX_WIDTH    1500
    #define CONVEYOR_PID_MIN_WIDTH  0
    #define CONVEYOR_PID_MAX_WIDTH  255

// Graphs
  #define TOP_TEMP_GRAPH_RANGE      100
  #define BOTTOM_TEMP_GRAPH_RANGE   100
  #define CONVEYOR_GRAPH_RANGE      100
  #define GRAPH_GRID_HEIGHT         DISPLAY_HEIGHT / 10

// TouchScreen
  #include <TouchScreen.h>      // Adafruit_TouchScreen
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

// States       # Should've used enum
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
  #define DRAW_GRAPH_POINT_INTERVAL    1000

// Sensors Draw
  #define ERROR_DURATION_MS     10000
  //Turns
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
  long  last_computeConveyorTime;

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
  char buffer[10];
  if (number == int(number))              return String(int(number));
  else if (number > 0) {
    if      (number < 0.1)                return String(dtostrf(number,5,4, buffer)).substring(1);
    else if (number < 1)                  return String(dtostrf(number,4,3, buffer)).substring(1);
    else                                  return dtostrf(number,1,1, buffer);
  }
  else if (number > -1 and number < 10)   return dtostrf(number,4,2, buffer);
  else                                    return dtostrf(number,1,1, buffer);
}

byte getFontSize(String number_str, byte normalFontSize) {
  byte number_length = number_str.length();
  return (number_length <= 3) ? normalFontSize : ( normalFontSize - (number_length-3) );
}

//DEBUG
void debug(String str)  { Serial.println(str); }

void serialStartJsonObject(String name)             { Serial.print("{\"name\":\""+name+"\",\"data\":{"); }
void serialEndJsonObject()                          { Serial.println("}},"); }
void serialAddJsonObject(String key, double value)  { Serial.print("\""+key+"\":"+String(value)+","); }
void serialAddJsonObject(String key, String value)  { Serial.print("\""+key+"\":\""+value+"\","); }

void serialStartJsonArray()             { Serial.print("["); }
void serialEndJsonArray()               { Serial.print("],"); }
void serialAddJsonArray(double value)   { Serial.print(String(value)+","); }
void serialAddJsonArray(String value)   { Serial.print("\""+value+"\","); }


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
    uint16_t errorbackgroundColor = MAGENTA;
    uint16_t errorforegroundColor = WHITE;
    uint16_t highlightbackgroundColor = GREEN;
    uint16_t highlightforegroundColor = BLACK;
    uint16_t lowlightbackgroundColor = BLACK;
    uint16_t lowlightforegroundColor = WHITE;
    uint16_t* backgroundColor = & lowlightbackgroundColor;
    uint16_t* foregroundColor = & lowlightforegroundColor;
    uint16_t old_backgroundColor;
    long lastTimeSinceError;
    bool isErrorActive = false;

    void highlight(void) { backgroundColor = & highlightbackgroundColor; foregroundColor = & highlightforegroundColor; };
    void lowlight(void)  { backgroundColor = & lowlightbackgroundColor;  foregroundColor = & lowlightforegroundColor; };
    void drawBackground (void) {
      myGLCD.setColor(*backgroundColor);
        myGLCD.fillRect(startX, startY, endX, endY);
    }
    void drawBackgroundIfHasChanged (void) {
      if (old_backgroundColor != *backgroundColor) {
        drawBackground();
        old_backgroundColor = *backgroundColor;
      }
    }
    void showError (void) {
      lastTimeSinceError = millis();
      backgroundColor = & errorbackgroundColor;
      foregroundColor = & errorforegroundColor;
      isErrorActive = true;
    };
    void showError (String str) {
      showError();
              #if defined DEBUG_SENSOR_SHOW_ERROR
                Serial.print("ERROR: ");Serial.print(str);Serial.println();
              #endif
    }
    void removeError (void) {
      isErrorActive = false;
      lowlight();
    };
};

class MinusButton : public Block {
  public:
    void draw(void) {
      drawBackground();
      myGLCD.setColor(*foregroundColor);
        myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    MinusButton() {
      lowlightbackgroundColor = BLUE;
      lowlightforegroundColor = BLACK;
    }
};

class SetControl : public Block {

    uint16_t getRGB565(byte red, byte green, byte blue)  { return ((red&0b11111000) << 8) | ((green&0b11111100) << 3) | (blue>>3);}

    uint16_t map_7Bits_to_RGB565FromBlueToRed(uint8_t num7Bits) {
    /*
      maps 0-128 to blue-red (passing through green)
      There are 4 cases to map
        - 0-31:   blue=maximum    green=increasing  red=0
        - 32-63:  blue=decreasing green=maximum     red=0
        - 64-95:  blue=0          green=maximum     red=increasing
        - 96-127: blue=0          green=decreasing  red=maximum
    */
      byte red, green, blue;
      if      (num7Bits < 32 ) {  blue = 255;               green = (num7Bits<<3);     red = 0;             }
      else if (num7Bits < 64 ) {  blue = 255-(num7Bits<<3); green = 255;               red = 0;             }
      else if (num7Bits < 96 ) {  blue = 0;                 green = 255;               red = (num7Bits<<3); }
      else if (num7Bits < 128) {  blue = 0;                 green = 255-(num7Bits<<3); red = 255;           }

      uint16_t color = getRGB565(red, green, blue);
      return color;
    }

  public:
    double* value;
    uint8_t servo_status;
    void setCoordinates(int x, int y) {
      startX = x;
      startY = y;
      endX = startX+gridInternalWidth+gridWidth; // +gridWidth for 2 columns width
      endY = startY+gridInternalHeight;
    };
    void draw(double number) {
      drawBackground();
      lowlightforegroundColor = map_7Bits_to_RGB565FromBlueToRed(servo_status);
      myGLCD.setTextColor(*foregroundColor, *backgroundColor);
        String number_str = stringifyDouble(number);
        myGLCD.setTextSize( getFontSize(number_str, SET_CONTROL_TEXT_SIZE) );
        myGLCD.print(number_str, startX+6, startY+11);
    };
    void draw(void) {draw(*value);};
};

class PlusButton : public Block {
  public:
    void draw(void) {
      drawBackground();
      myGLCD.setColor(*foregroundColor);
        myGLCD.fillRect(startX+27, startY+15, startX+34, startY+46);
        myGLCD.fillRect(startX+15, startY+27, startX+46, startY+34);
    };
    PlusButton() {
      lowlightbackgroundColor = RED;
      lowlightforegroundColor = BLACK;
    }
};

class Sensors : public Block {
  public:
  virtual void draw(void){};
  virtual void update(void){};
  virtual double read(void){};
};

class TempSensors : public Sensors {
  public:
    double value1Avg;
    double value2Avg;
    double values1[NUM_OF_MEASUREMENTS_TO_READ];
    double values2[NUM_OF_MEASUREMENTS_TO_READ];
    byte counter;
    MAX6675 Sensor1;
    MAX6675 Sensor2;
    TempSensors(byte pinSensor1CS,byte pinSensor2CS):
      Sensor1(pinSensor1CS),
      Sensor2(pinSensor2CS)
    {};

    void draw(void) {
      if ( isErrorActive and ERROR_DURATION_MS < millis() - lastTimeSinceError )    removeError();
      drawBackgroundIfHasChanged();
      myGLCD.setTextColor(*foregroundColor, *backgroundColor);
        myGLCD.setTextSize(SENSOR_TEXT_SIZE);
          myGLCD.print(String(int(value1Avg)), startX+7, startY+7);
          myGLCD.print(String(int(value2Avg)), startX+7, startY+35);

      /* // Draw symbol: ±
      myGLCD.setColor(foregroundColor);
        myGLCD.fillRect(startX+7, startY+35+18 , startX+7+13, startY+35+19);
        myGLCD.fillRect(startX+7, startY+35+7 , startX+7+13, startY+35+8);
        myGLCD.fillRect(startX+7+6, startY+35 , startX+7+7, startY+35+15);
      */
    };

    double getAvgTemp (double value[NUM_OF_MEASUREMENTS_TO_READ]) {
      double valueSum=0;
      byte validReads=0;
      for (byte i=0; i<NUM_OF_MEASUREMENTS_TO_READ; i++) {
        if (!isnan(value[i])) {
          valueSum += value[i];
          validReads++;
        };
      };
      if (validReads > 0) return valueSum / validReads;   else return NAN;
    };

    double updateSensor(double measurements[NUM_OF_MEASUREMENTS_TO_READ], double* avg_measurements, double new_measurement) {

      if (!isnan(new_measurement)) {
        double valueDiff = new_measurement - *avg_measurements;
        if      ( valueDiff >  MAX_TEMP_SENSOR_ERROR )   { measurements[counter] = *avg_measurements + MAX_TEMP_SENSOR_ERROR;    showError(String(new_measurement)); }
        else if ( valueDiff < -MAX_TEMP_SENSOR_ERROR )   { measurements[counter] = *avg_measurements - MAX_TEMP_SENSOR_ERROR;    showError(String(new_measurement)); }
        else measurements[counter] = new_measurement;
      }
      else measurements[counter] = *avg_measurements;
      *avg_measurements = getAvgTemp(measurements);
    };

    void update(void) {
      double new_measurement1 = Sensor1.readCelsius();
      double new_measurement2 = Sensor2.readCelsius();
              #if defined DEBUG_SENSOR_TEMP_JSON
                serialStartJsonObject("temperatures");
                  serialStartJsonArray();
                    serialAddJsonArray(new_measurement1);
                    serialAddJsonArray(new_measurement2);
                  serialEndJsonArray();
                serialEndJsonObject();
              #endif
      updateSensor(values1, &value1Avg, new_measurement1);
      updateSensor(values2, &value2Avg, new_measurement2);

      if (counter == NUM_OF_MEASUREMENTS_TO_READ-1) counter=0;    else counter++;
    };

    double read(void) {
      byte validReads=0; // If it's not set to 0, then "if (validReads > 0)" will always be True
      double valueSum;
      double reading=0;
      if (value1Avg > 0)    { valueSum += value1Avg; validReads++; }
      if (value2Avg > 0)    { valueSum += value2Avg; validReads++; }
      if (validReads > 0)   reading = valueSum / validReads;
      else showError("No valid reads top sensors");
      return reading;
    };
};

class EncoderSensor : public Sensors {
  public:
    double value;

    void draw(void) {
      drawBackground();
      myGLCD.setTextColor(*foregroundColor, *backgroundColor);
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

    double setpoint;

    // Amount to increase/decrease setcontrol
    double short_event_amount = 0.1;
    double long_event_amount = 1;

    MinusButton minusButton;
    SetControl setControl;
    PlusButton plusButton;
    EncoderSensor sensors;
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
    virtual void draw(void) {draw(*setControl.value);};
    virtual double amountToChange(byte event){ return (event==LONG_HOLD_EVENT) ? long_event_amount : short_event_amount; }
    virtual void decreaseSetControl(byte event) {setpoint-=amountToChange(event);  setControl.draw();}
    virtual void increaseSetControl(byte event) {setpoint+=amountToChange(event);  setControl.draw();}
} conveyorControl;

class TempControl : public Control {
  public:
    TempSensors sensors;
    TempControl(byte pinSensor1CS,byte pinSensor2CS):
      sensors(pinSensor1CS, pinSensor2CS)
    {
      // Amount to increase/decrease setcontrol
      short_event_amount = 1;
      long_event_amount = 10;
    };
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
    void draw(void) {draw(*setControl.value);};
// TempControl(SensorCS1, SensorCS2)
} topTempControl(PIN_CS_TOP_TEMP_SENSOR_1,
                 PIN_CS_TOP_TEMP_SENSOR_2
                 ),
  bottomTempControl(PIN_CS_BOTTOM_TEMP_SENSOR_1,
                    PIN_CS_BOTTOM_TEMP_SENSOR_2
                    );

class Pid : public PID {
  public:
    const String name;
    double kp, ki, kd;
    double input, output, setpoint;
    double minOutput, startOutput, maxOutput;
    byte EEPROMaddress, outputLimitsEEPROMaddress;
    Pid(String _name, double _kp, double _ki, double _kd, byte pOn, byte DIR, byte address):
      name(_name),
      kp(_kp),
      ki(_ki),
      kd(_kd),
      EEPROMaddress(address),
      PID(&input, &output, &setpoint, _kp, _ki, _kd, pOn, DIR)
    {};

    int minOutputGraph(void) { return (GetDirection() == DIRECT) ? minOutput : maxOutput; };
    int maxOutputGraph(void) { return (GetDirection() == DIRECT) ? maxOutput : minOutput; };

    void updateTuning(void) {SetTunings(kp,ki,kd);};

    void updateOutputLimits(void) {SetOutputLimits(minOutput, maxOutput);};

    void increaseKp (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   kp+=scale*0.1;    updateTuning(); topTempControl.setControl.draw(kp);};
    void decreaseKp (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   kp-=scale*0.1;    updateTuning(); topTempControl.setControl.draw(kp);};
    void increaseKi (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   ki+=scale*0.0001; updateTuning(); conveyorControl.setControl.draw(ki);};
    void decreaseKi (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   ki-=scale*0.0001; updateTuning(); conveyorControl.setControl.draw(ki);};
    void increaseKd (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   kd+=scale*0.1;    updateTuning(); bottomTempControl.setControl.draw(kd);};
    void decreaseKd (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   kd-=scale*0.1;    updateTuning(); bottomTempControl.setControl.draw(kd);};

    void increaseMinOutput   (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   minOutput+=  scale*1;   updateOutputLimits();   topTempControl.setControl.draw    (minOutput);};
    void decreaseMinOutput   (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   minOutput-=  scale*1;   updateOutputLimits();   topTempControl.setControl.draw    (minOutput);};
    void increaseStartOutput (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   startOutput+=scale*1;   updateOutputLimits();   conveyorControl.setControl.draw   (startOutput);};
    void decreaseStartOutput (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   startOutput-=scale*1;   updateOutputLimits();   conveyorControl.setControl.draw   (startOutput);};
    void increaseMaxOutput   (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   maxOutput+=  scale*1;   updateOutputLimits();   bottomTempControl.setControl.draw (maxOutput);};
    void decreaseMaxOutput   (byte event) {byte scale=(event==LONG_HOLD_EVENT)?10:1;   maxOutput-=  scale*1;   updateOutputLimits();   bottomTempControl.setControl.draw (maxOutput);};

    void saveParameters (void) {
      PidEEPROM pid;
      pid.kp = kp*10;
      pid.ki = ki*10000;
      pid.kd = kd*10;
      pid.minOutput   = (int)minOutput;
      pid.startOutput = (int)startOutput;
      pid.maxOutput   = (int)maxOutput;
      EEPROM.put(EEPROMaddress, pid);
    }
    void loadParameters (void) {
      PidEEPROM pid;
      EEPROM.get(EEPROMaddress, pid);
      if (pid.kp          >= 0 and pid.kp          < 1000)   kp          = pid.kp/10.0;
      if (pid.ki          >= 0 and pid.ki          < 1000)   ki          = pid.ki/10000.0;
      if (pid.kd          >= 0 and pid.kd          < 1000)   kd          = pid.kd/10.0;
      if (pid.minOutput   >= 0 and pid.minOutput   < 2000)   minOutput   = pid.minOutput;
      if (pid.startOutput >= 0 and pid.startOutput < 2000)   startOutput = pid.startOutput;
      if (pid.maxOutput   >= 0 and pid.maxOutput   < 2000)   maxOutput   = pid.maxOutput;
      updateTuning(); updateOutputLimits();
    }

};
Pid topPID     ("top", TOP_PID_KP     , TOP_PID_KI     , TOP_PID_KD     , P_ON_E, REVERSE, TOP_PID_ADDRESS);
Pid bottomPID  ("bottom", BOTTOM_PID_KP  , BOTTOM_PID_KI  , BOTTOM_PID_KD  , P_ON_E, REVERSE, BOTTOM_PID_ADDRESS);
Pid conveyorPID("conveyor", CONVEYOR_PID_KP, CONVEYOR_PID_KI, CONVEYOR_PID_KD, P_ON_E, DIRECT , CONVEYOR_PID_ADDRESS);

class Profile : public Block {
  public:
    int bottomTemp;
    int topTemp;
    double cookTime;
    bool isActive = false;
    byte id;

    Profile(int _topTemp, int _cookTime, int _bottomTemp):
      topTemp(_topTemp),
      cookTime(_cookTime),
      bottomTemp(_bottomTemp)
    {};

    void draw (void)
    {
      if (isActive) highlight();
      else lowlight();
      drawBackground();
      myGLCD.setTextColor(*foregroundColor, *backgroundColor);
        myGLCD.setTextSize(PROFILE_ID_TEXT_SIZE);
          myGLCD.print(String(id+1), startX+9 , startY+9);
        myGLCD.setTextSize(PROFILE_PARAM_TEXT_SIZE);
          myGLCD.print(String(topTemp), startX+38, startY+6);
          myGLCD.print(stringifyDouble(cookTime), startX+38, startY+6+14);
          myGLCD.print(String(bottomTemp), startX+38, startY+6+28);
    };

    void load(void)
    {
      topTempControl.setpoint = topTemp;
      conveyorControl.setpoint = cookTime;
      bottomTempControl.setpoint = bottomTemp;
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
      topTemp = topTempControl.setpoint;
      cookTime = conveyorControl.setpoint;
      bottomTemp = bottomTempControl.setpoint;
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
  Profile(0,10,0),
  Profile(320,-10,270),
  Profile(320,10,270),
  Profile(5,5,5),
  Profile(-5,-5,-5),
};
byte const profilesSize = sizeof(profiles) / sizeof(Profile);

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
    if (profile.topTemp     >= 0      and profile.topTemp     < 1000)   profiles[i].topTemp = profile.topTemp;
    if (profile.cookTime    >= -500   and profile.cookTime    < 500 )   profiles[i].cookTime = profile.cookTime / 10.0;
    if (profile.bottomTemp  >= 0      and profile.bottomTemp  < 1000)   profiles[i].bottomTemp = profile.bottomTemp;
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
  tp.x=0;
  tp.y=0;
  tp.z=0;
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
  int temp = map(tp.y, 930, 140, 0, dispX-1);
  tp.y = map(tp.x, 150, 900, 0, dispY-1);
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

void serialGraphPIDs(Pid pid); // Compiler complains otherwise ¬¬. Apparently one cannot use functions with parameters that are instances of classes declared in the same file --_(¬.¬)_--
void serialGraphPIDs(Pid pid)
{
  int input    = pid.input;
  int setpoint = pid.setpoint;
  int output   = map(pid.output,  pid.minOutputGraph(),  pid.maxOutputGraph(),  101,  399);

  Serial.print(output);     Serial.print(" ");
  Serial.print(input);      Serial.print(" ");
  Serial.print(setpoint);   Serial.print(" ");
  Serial.print(100);        Serial.print(" ");
  Serial.print(400);        Serial.print(" ");
  Serial.println();
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
  myGLCD.setColor(WHITE);
  if (isOutline) {myGLCD.drawRect(0,0,dispX-1,dispY-1);}
  myGLCD.drawLine(gridWidth-1+isOutline  , dispY-1                   , gridWidth-1+isOutline  , 0);
  myGLCD.drawLine(gridWidth*2-1+isOutline, 0                         , gridWidth*2-1+isOutline, dispY-gridHeight*3);
  myGLCD.drawLine(gridWidth*3-1+isOutline, dispY-1                   , gridWidth*3-1+isOutline, 0);
  myGLCD.drawLine(gridWidth*4-1+isOutline, dispY-1                   , gridWidth*4-1+isOutline, 0);
  myGLCD.drawLine(0                      , topTempControl.startY-1   , dispX-1                , topTempControl.startY-1);
  myGLCD.drawLine(0                      , conveyorControl.startY-1  , dispX-1                , conveyorControl.startY-1);
  myGLCD.drawLine(0                      , bottomTempControl.startY-1, dispX-1                , bottomTempControl.startY-1);
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
      // Clean screen
        myGLCD.setColor(BLACK);      myGLCD.fillRect(0,0, dispX-1,dispY-1);
      // Draw Grid
        myGLCD.setColor(GRAY);
        for (int row=0; row < dispY; row+=GRAPH_GRID_HEIGHT)      myGLCD.drawLine(0, row, dispX-1, row);
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
  topTempControl.setControl.value = & topTempControl.setpoint;
  conveyorControl.setControl.value = & conveyorControl.setpoint;
  bottomTempControl.setControl.value = & bottomTempControl.setpoint;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void controlBottomPID (void) {
  state = CONTROLLING_BOTTOM_PID;
  topTempControl.setControl.value = & bottomPID.kp;
  conveyorControl.setControl.value = & bottomPID.ki;
  bottomTempControl.setControl.value = & bottomPID.kd;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.highlight();
  drawEverything();
}
void controlTopPID (void) {
  state = CONTROLLING_TOP_PID;
  topTempControl.setControl.value = & topPID.kp;
  conveyorControl.setControl.value = & topPID.ki;
  bottomTempControl.setControl.value = & topPID.kd;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.highlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void controlConveyorPID (void) {
  state = CONTROLLING_CONVEYOR_PID;
  topTempControl.setControl.value = & conveyorPID.kp;
  conveyorControl.setControl.value = & conveyorPID.ki;
  bottomTempControl.setControl.value = & conveyorPID.kd;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.highlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void showTopPIDGraph (void) {
  inputGraph                = &topPID.input;
  setpointGraph             = &topPID.setpoint;
  outputGraph               = &topPID.output;
  minInputSetpointGraph     = topPID.setpoint - TOP_TEMP_GRAPH_RANGE;
  maxInputSetpointGraph     = topPID.setpoint + TOP_TEMP_GRAPH_RANGE;
  minOutputGraph            = topPID.minOutputGraph();
  maxOutputGraph            = topPID.maxOutputGraph();
  state                     = SHOWING_GRAPH;
  drawEverything();
}
void showBottomPIDGraph (void) {
  inputGraph                = &bottomPID.input;
  setpointGraph             = &bottomPID.setpoint;
  outputGraph               = &bottomPID.output;
  minInputSetpointGraph     = bottomPID.setpoint - BOTTOM_TEMP_GRAPH_RANGE;
  maxInputSetpointGraph     = bottomPID.setpoint + BOTTOM_TEMP_GRAPH_RANGE;
  minOutputGraph            = bottomPID.minOutputGraph();
  maxOutputGraph            = bottomPID.maxOutputGraph();
  state                     = SHOWING_GRAPH;
  drawEverything();
}
void showConveyorPIDGraph (void) {
  inputGraph                = &conveyorPID.input;
  setpointGraph             = &conveyorPID.setpoint;
  outputGraph               = &conveyorPID.output;
  minInputSetpointGraph     = conveyorPID.setpoint - CONVEYOR_GRAPH_RANGE;
  maxInputSetpointGraph     = conveyorPID.setpoint + CONVEYOR_GRAPH_RANGE;
  minOutputGraph            = conveyorPID.minOutputGraph();
  maxOutputGraph            = conveyorPID.maxOutputGraph();
  state                     = SHOWING_GRAPH;
  drawEverything();
}
void controlTopOutputLimits (void) {
  state = CONTROLLING_TOP_OUTPUT_LIMITS;
  topTempControl.setControl.value = & topPID.minOutput;
  conveyorControl.setControl.value = & topPID.startOutput;
  bottomTempControl.setControl.value = & topPID.maxOutput;
  topTempControl.setControl.highlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void controlConveyorOutputLimits (void) {
  state = CONTROLLING_CONVEYOR_OUTPUT_LIMITS;
  topTempControl.setControl.value = & conveyorPID.minOutput;
  conveyorControl.setControl.value = & conveyorPID.startOutput;
  bottomTempControl.setControl.value = & conveyorPID.maxOutput;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.highlight();
  bottomTempControl.setControl.lowlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
  drawEverything();
}
void controlBottomOutputLimits (void) {
  state = CONTROLLING_BOTTOM_OUTPUT_LIMITS;
  topTempControl.setControl.value = & bottomPID.minOutput;
  conveyorControl.setControl.value = & bottomPID.startOutput;
  bottomTempControl.setControl.value = & bottomPID.maxOutput;
  topTempControl.setControl.lowlight();
  conveyorControl.setControl.lowlight();
  bottomTempControl.setControl.highlight();
  topTempControl.sensors.lowlight();
  conveyorControl.sensors.lowlight();
  bottomTempControl.sensors.lowlight();
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

void topMinusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               topTempControl.decreaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKp(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKp(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKp(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseMinOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseMinOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseMinOutput(event); break;
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
void topPlusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               topTempControl.increaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKp(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKp(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKp(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseMinOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseMinOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseMinOutput(event); break;
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
void conveyorMinusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               conveyorControl.decreaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKi(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKi(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKi(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseStartOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseStartOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseStartOutput(event); break;
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
void conveyorPlusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               conveyorControl.increaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKi(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKi(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKi(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseStartOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseStartOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseStartOutput(event); break;
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
void bottomMinusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               bottomTempControl.decreaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.decreaseKd(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.decreaseKd(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.decreaseKd(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.decreaseMaxOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.decreaseMaxOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.decreaseMaxOutput(event); break;
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
void bottomPlusButtonClick (byte event) {
  switch (state) {
    case CONTROLLING_SETPOINTS:               bottomTempControl.increaseSetControl(event); break;
    case CONTROLLING_TOP_PID:                 topPID.increaseKd(event); break;
    case CONTROLLING_CONVEYOR_PID:            conveyorPID.increaseKd(event); break;
    case CONTROLLING_BOTTOM_PID:              bottomPID.increaseKd(event); break;
    case SHOWING_GRAPH:                       controlSetpoints(); break;
    case CONTROLLING_TOP_OUTPUT_LIMITS:       topPID.increaseMaxOutput(event); break;
    case CONTROLLING_CONVEYOR_OUTPUT_LIMITS:  conveyorPID.increaseMaxOutput(event); break;
    case CONTROLLING_BOTTOM_OUTPUT_LIMITS:    bottomPID.increaseMaxOutput(event); break;
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
        case CLICK_EVENT :      topMinusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       topMinusButtonClick(event); break;
        case LONG_HOLD_EVENT :  topMinusButtonClick(event); break;
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
        case CLICK_EVENT :      topPlusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       topPlusButtonClick(event); break;
        case LONG_HOLD_EVENT :  topPlusButtonClick(event); break;
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
        case CLICK_EVENT :      conveyorMinusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       conveyorMinusButtonClick(event); break;
        case LONG_HOLD_EVENT :  conveyorMinusButtonClick(event); break;
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
        case CLICK_EVENT :      conveyorPlusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       conveyorPlusButtonClick(event); break;
        case LONG_HOLD_EVENT :  conveyorPlusButtonClick(event); break;
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
        case CLICK_EVENT :      bottomMinusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       bottomMinusButtonClick(event); break;
        case LONG_HOLD_EVENT :  bottomMinusButtonClick(event); break;
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
        case CLICK_EVENT :      bottomPlusButtonClick(event); break;
        //case LONG_CLICK_EVENT : break;
        case HOLD_EVENT :       bottomPlusButtonClick(event); break;
        case LONG_HOLD_EVENT :  bottomPlusButtonClick(event); break;
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
    case TOP_TURN:        topTempControl.sensors.draw();    topTempControl.setControl.draw(); break;
    case CONVEYOR_TURN:   conveyorControl.sensors.draw();   conveyorControl.setControl.draw(); break;
    case BOTTOM_TURN:     bottomTempControl.sensors.draw(); bottomTempControl.setControl.draw(); break;
  }
  if (turn == BOTTOM_TURN)  turn = TOP_TURN;
  else                      turn++;
}

void computeTempPID (Pid* pid, TempControl* tempControl, Servo* servo);
void computeTempPID (Pid* pid, TempControl* tempControl, Servo* servo)
{
  double temperature = tempControl->sensors.read();
  if (temperature > 0)   pid->input = temperature;
  else tempControl->sensors.showError("No valid reads in temperature");

  pid->setpoint = tempControl->setpoint;
  double gap = pid->setpoint - pid->input;
  if ( gap > PID_MANUAL_THRESHOLD ) {
    pid->SetMode(MANUAL);
    pid->output = pid->GetDirection() == DIRECT ? pid->maxOutput : pid->minOutput;
  } else {
    if (pid->GetMode() == MANUAL) {
      pid->output = pid->GetDirection() == DIRECT ? pid->minOutput : pid->maxOutput;
      pid->SetMode(AUTOMATIC);
    }
    pid->Compute();
  }
          #if defined DEBUG_PID_JSON
            serialStartJsonObject("PID_"+pid->name);
              serialAddJsonObject("Input"     , pid->input);
              serialAddJsonObject("Setpoint"  , pid->setpoint);
              serialAddJsonObject("Output"    , pid->output);
              serialAddJsonObject("kp"        , pid->GetKp());
              serialAddJsonObject("ki"        , pid->GetKi());
              serialAddJsonObject("kd"        , pid->GetKd());
            serialEndJsonObject();
          #endif
          #if defined DEBUG_PID_GRAPH
            serialGraphPIDs(pid);
          #endif
  servo->writeMicroseconds(pid->output);
  uint8_t servo_status = map(pid->output, pid->minOutputGraph(), pid->maxOutputGraph(), 0, 127);
  tempControl->setControl.servo_status = servo_status;
}

void computeTopPID (void)     {computeTempPID(&topPID,    &topTempControl,    &topServo);}
void computeBottomPID (void)  {computeTempPID(&bottomPID, &bottomTempControl, &bottomServo);}

void computeConveyorPID (void)
{
  noInterrupts();
    long encoderSteps_counted = myEncoder.read();
    myEncoder.write(0);
  interrupts();
  long computeConveyorTime = millis();
  static int oldSetcontrolValue;
  static bool isReverse;
  if (conveyorControl.setpoint != oldSetcontrolValue)
  {
    isReverse = ( conveyorControl.setpoint < 0 );
    digitalWrite(CONVEYOR_L298N_DIR1_PIN, (isReverse?HIGH:LOW) );
    digitalWrite(CONVEYOR_L298N_DIR2_PIN, (isReverse?LOW:HIGH) );
    oldSetcontrolValue = conveyorControl.setpoint;
  }
  if (isReverse)  encoderSteps_counted = -encoderSteps_counted;
  long encoderStepsCounter_duration = computeConveyorTime - last_computeConveyorTime;
  double stepsPerMs_real = (double)encoderSteps_counted / encoderStepsCounter_duration;
  double msToCrossOven_goal = abs(conveyorControl.setpoint) * 60000; // convert from minutes-to-cross-oven to miliseconds-to-cross-oven
  double stepsPerMs_goal = STEPS_TO_CROSS_OVEN__TIME_CORRECTED / msToCrossOven_goal;
  double encoderSteps_counted_goal = stepsPerMs_goal * encoderStepsCounter_duration;
  conveyorPID.input += encoderSteps_counted - encoderSteps_counted_goal;

        #if defined DEBUG_CONVEYOR_PID_JSON
          serialStartJsonObject("PID_conveyor");
            serialAddJsonObject("stepsPerS_real"               , stepsPerMs_real*1000);
            serialAddJsonObject("stepsPerS_goal"               , stepsPerMs_goal*1000);
            serialAddJsonObject("conveyorPID.input"            , conveyorPID.input);
            serialAddJsonObject("encoderStepsCounter_duration" , encoderStepsCounter_duration);
            serialAddJsonObject("encoderSteps_counted"         , encoderSteps_counted);
          serialEndJsonObject();
        #endif

  conveyorPID.setpoint = 0;
  conveyorPID.Compute();
  analogWrite(CONVEYOR_L298N_PWM, conveyorPID.output);
  last_computeConveyorTime = computeConveyorTime;
  conveyorControl.sensors.value = int(conveyorPID.input);
  if (stepsPerMs_real > CONVEYOR_MAX_STEPS_PER_MS)  conveyorControl.sensors.showError("Too fast, Impossible");

  uint8_t servo_status = map(conveyorPID.output, conveyorPID.minOutputGraph(), conveyorPID.maxOutputGraph(), 0, 127);
  conveyorControl.setControl.servo_status = servo_status;
}

void drawGraphPoint()
{
  static int column;
  // scale values to show in graph
  int input    = map(*inputGraph,    minInputSetpointGraph, maxInputSetpointGraph, dispY-1, 0);
  int setpoint = map(*setpointGraph, minInputSetpointGraph, maxInputSetpointGraph, dispY-1, 0);
  int output   = map(*outputGraph,   minOutputGraph,        maxOutputGraph,        dispY-1, 0);

  // draw points on the graph
  myGLCD.setColor(GREEN);       myGLCD.drawPixel(column, setpoint);   myGLCD.drawPixel(column, 1+setpoint);
  myGLCD.setColor(BLUE);        myGLCD.drawPixel(column, output);     myGLCD.drawPixel(column, 1+output);
  myGLCD.setColor(RED);         myGLCD.drawPixel(column, input);      myGLCD.drawPixel(column, 1+input);

  if (column == dispX-1)  column=0;
  else                    column++;
}


// ###############################################################
// #########################    SETUP    #########################
// ###############################################################


void setup()
{
  // SPI is used to communicate with the thermocouples' drivers
  SPI.begin();

  // Serial init
  Serial.begin(38400);
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

  // Load EEPROM
    calculateProfilesProperties();    // Also set coordinates
    topPID.loadParameters();
    bottomPID.loadParameters();
    conveyorPID.loadParameters();
 
  // Servos Init
    topServo.attach(TOP_SERVO_PIN);         topServo.writeMicroseconds(topPID.startOutput);
    bottomServo.attach(BOTTOM_SERVO_PIN);   bottomServo.writeMicroseconds(bottomPID.startOutput);
    pinMode(CONVEYOR_L298N_PWM      , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR1_PIN , OUTPUT);
    pinMode(CONVEYOR_L298N_DIR2_PIN , OUTPUT);

  // Relays
    // Turn ON ElectroValve
      pinMode(VALVE_PIN, OUTPUT);   digitalWrite(VALVE_PIN, LOW);
    // Turn ON Sparks for SPARK_IGNITION_TIME miliseconds
      pinMode(SPARK_PIN, OUTPUT);
      digitalWrite(SPARK_PIN, LOW);
      delay(SPARK_IGNITION_TIME);
      digitalWrite(SPARK_PIN, HIGH);

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
    topPID.SetSampleTime(TOP_PID_INTERVAL);
    topPID.output = topPID.GetDirection() == DIRECT ? topPID.minOutput : topPID.maxOutput;
    topPID.SetMode(AUTOMATIC);
    bottomPID.SetSampleTime(BOTTOM_PID_INTERVAL);
    bottomPID.output = bottomPID.GetDirection() == DIRECT ? bottomPID.minOutput : bottomPID.maxOutput;
    bottomPID.SetMode(AUTOMATIC);
    conveyorPID.SetSampleTime(CONVEYOR_PID_INTERVAL);
    conveyorPID.SetMode(AUTOMATIC);

  // Thermocouples
    // Populate temperatures arrays with current readings
      for (byte i=0; i<NUM_OF_MEASUREMENTS_TO_READ; i++)    topTempControl.sensors.values1[i]    = topTempControl.sensors.Sensor1.readCelsius();   
      for (byte i=0; i<NUM_OF_MEASUREMENTS_TO_READ; i++)    topTempControl.sensors.values2[i]    = topTempControl.sensors.Sensor2.readCelsius();   
      for (byte i=0; i<NUM_OF_MEASUREMENTS_TO_READ; i++)    bottomTempControl.sensors.values1[i] = bottomTempControl.sensors.Sensor1.readCelsius();
      for (byte i=0; i<NUM_OF_MEASUREMENTS_TO_READ; i++)    bottomTempControl.sensors.values2[i] = bottomTempControl.sensors.Sensor2.readCelsius();
    topTempControl.sensors.value1Avg    = topTempControl.sensors.getAvgTemp   ( topTempControl.sensors.values1);
    topTempControl.sensors.value2Avg    = topTempControl.sensors.getAvgTemp   ( topTempControl.sensors.values2);
    bottomTempControl.sensors.value1Avg = bottomTempControl.sensors.getAvgTemp( bottomTempControl.sensors.values1);
    bottomTempControl.sensors.value2Avg = bottomTempControl.sensors.getAvgTemp( bottomTempControl.sensors.values2);

    // Encoder
      myEncoder.write(0);
      last_computeConveyorTime = millis();
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