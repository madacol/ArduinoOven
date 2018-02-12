#define  TOUCH_ORIENTATION  LANDSCAPE
#include <Adafruit_GFX.h>
#include <TouchScreen.h>
#include <UTFTGLUE.h>            //we are using UTFT display methods
UTFTGLUE myGLCD(0x9341, A2, A1, A3, A4, A0);

// MCUFRIEND UNO shield shares pins with the TFT.   Due does NOT work
#define YP A1   //A3 for ILI9320
#define YM 7    //9
#define XM A2
#define XP 6    //8  

TouchScreen myTouch(XP, YP, XM, YM, 300);
TSPoint tp;                      //Touchscreen_due branch uses Point

//Custom defines
#define GREEN 0, 255, 0
#define WHITE 255, 255, 255
#define BLACK 0, 0, 0
#define RED 255, 0, 0
#define BLUE 0, 0, 255
#define YELLOW 255, 255, 0

#define XSCREEN 320
#define YSCREEN 240

#define CONTROLBUTTONWIDTH 63
#define CONTROLBUTTONHEIGHT 63
#define COLUMNWIDTH 64

class TempControl {

  public:
    int sensor1;
    int sensor2;
    int sensor3;
    int x;
    int y;
    int controlTemp;

    TempControl(int _x, int _y)
    {
      x = _x;
      y = _y;
    };

    void drawMinus(void)
    {
      myGLCD.setColor(BLUE);
      myGLCD.fillRect(x, y, x+62, y+62);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(x+15, y+27, x+46, y+34);
    };
    
    void drawControlTemp(void)
    {
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(6);
      myGLCD.print(String(controlTemp), x+76, y+10);

    };

    void drawPlus(void)
    {
      myGLCD.setColor(RED);
      myGLCD.fillRect(x+192, y, x+192+62, y+62);
      myGLCD.setColor(BLACK);
      myGLCD.fillRect(x+192+27, y+15, x+192+34, y+46);
      myGLCD.fillRect(x+192+15, y+27, x+192+46, y+34);
    };

    void drawSensors(void)
    {
      myGLCD.setColor(WHITE);
      myGLCD.setTextSize(3);
      myGLCD.print(String(sensor1), x+263, y+7);
      myGLCD.print(String(sensor2), x+282, y+35);
      myGLCD.fillRect(x+263, y+35+18 , x+263+13, y+35+19);
      myGLCD.fillRect(x+263, y+35+7 , x+263+13, y+35+8);
      myGLCD.fillRect(x+263+6, y+35 , x+263+7, y+35+15);
    };

    void draw(void)
    {
      drawMinus();
      drawPlus();
      drawControlTemp();
      drawSensors();
    };
// TempControl(x, y)
} topTempControl(0,48),
  cookTimeControl(0,112),
  bottomTempControl(0,176);

class Profile {

  public:
    int bottomTemp;
    int topTemp;
    int cookTime;

    Profile(int _topTemp, int _cookTime, int _bottomTemp)
    {
      topTemp = _topTemp;
      cookTime = _cookTime;
      bottomTemp = _bottomTemp;
    };
    void load(void)
    {
      topTempControl.controlTemp = topTemp;
      cookTimeControl.controlTemp = cookTime;
      bottomTempControl.controlTemp = bottomTemp;
    };
     void save(void)
    {
      topTemp = topTempControl.controlTemp;
      cookTime = cookTimeControl.controlTemp;
      bottomTemp = bottomTempControl.controlTemp;
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

class Button {
    const byte pin;
    int state;
    unsigned long buttonDownMs;

  public:
    Button(byte attachTo) :
      pin(attachTo)
    {
    }
};

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

extern uint8_t SmallFont[];

int dispX, dispY;
byte activeProfile;

void loadProfile(byte i)
{
  i--;
  profiles[i].load();
  activeProfile = i;
}

void drawDivisions(void)
{
  myGLCD.setColor(WHITE);
  myGLCD.drawLine(63, 239, 63, 0);
  myGLCD.drawLine(127, 0, 127, 47);
  myGLCD.drawLine(191, 239, 191, 0);
  myGLCD.drawLine(255, 239, 255, 0);
  myGLCD.drawLine(0, 47, 319, 47);
  myGLCD.drawLine(0, 111, 319, 111);
  myGLCD.drawLine(0, 175, 319, 175);
}

void drawProfiles(void)
{
  int x = 64;
  for (int i=0; i<profilesSize; i++)
  { 
    if (i == activeProfile)
    {
      myGLCD.setColor(GREEN);
      myGLCD.fillRect(x*i, 0, x*i+62, 46); 
    }
    
    myGLCD.setColor(WHITE);
    myGLCD.setTextSize(4);
    myGLCD.print(String(i+1),10+x*i , 8);
    myGLCD.setTextSize(1);
    myGLCD.print(String(profiles[i].topTemp), 40+x*i, 6);
    myGLCD.print(String(profiles[i].cookTime), 40+x*i, 20);
    myGLCD.print(String(profiles[i].bottomTemp), 40+x*i, 34);
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
  digitalWrite(A0, HIGH);
  pinMode(A0, OUTPUT);
  myGLCD.InitLCD(TOUCH_ORIENTATION);
  myGLCD.clrScr();
  myGLCD.setFont(SmallFont);
  dispX= myGLCD.getDisplayXSize();
  dispY = myGLCD.getDisplayYSize();
  
  topTempControl.sensor1 = 456;
  topTempControl.sensor2 = 56;
  
  loadProfile(5);

  draw();
}


void loop()
{  
}


