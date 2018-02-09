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

class PresetParameters {

  public:
    int topTemp;
    int bottomTemp;
    int cookTime;
    PresetParameters(int topTempAttachTo, int bottomTempAttachTo, int cookTimeAttachTo) :
      topTemp(topTempAttachTo), 
      bottomTemp(bottomTempAttachTo), 
      cookTime(cookTimeAttachTo)
    {
    }
};

class Settings {

  public:
    byte active;
    int topTemp;
    int bottomTemp;
    int cookTime;
    PresetParameters option1(int x, int y, int z);
    
} settings;

class asd {
    const byte pin;
    int state;
    unsigned long buttonDownMs;

  public:
    asd(byte attachTo) :
      pin(attachTo)
    {
    }
};

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



void drawDivisions(void)
{
    myGLCD.setColor(WHITE);
    //myGLCD.drawRect(0, 0, 319, 239);
    myGLCD.drawLine(63, 239, 63, 0);
    myGLCD.drawLine(127, 0, 127, 47);
    myGLCD.drawLine(191, 239, 191, 0);
    myGLCD.drawLine(255, 239, 255, 0);
    myGLCD.drawLine(0, 47, 319, 47);
    myGLCD.drawLine(0, 111, 319, 111);
    myGLCD.drawLine(0, 175, 319, 175);
}

void drawPlusButtom(int x,int y)
{
  myGLCD.setColor(RED);
  myGLCD.fillRect(x, y, x+62, y+62);
  myGLCD.setColor(BLACK);
  myGLCD.fillRect(x+27, y+15, x+34, y+46);
  myGLCD.fillRect(x+15, y+27, x+46, y+34);
}

void drawMinusButtom(int x,int y)
{
  myGLCD.setColor(BLUE);
  myGLCD.fillRect(x, y, x+62, y+62);
  myGLCD.setColor(BLACK);
  myGLCD.fillRect(x+15, y+27, x+46, y+34);
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
    //myGLCD.setTextSize(1.5);
    drawDivisions();
    drawPlusButtom(192, 48);
    drawPlusButtom(192, 112);
    drawPlusButtom(192, 176);
    
    drawMinusButtom(0, 48);
    drawMinusButtom(0, 112);
    drawMinusButtom(0, 176);

    settings.topTemp=320;
    settings.cookTime=650;
    settings.bottomTemp=230;
}


void loop()
{
    
  
    //myGLCD.drawRect(100, 100, 100, 100);
    //myGLCD.drawLine(150, 60, 100, 5);
    //myGLCD.drawLine(319, 239, 318, 0);
    myGLCD.setColor(WHITE);
    myGLCD.setTextSize(6);
    myGLCD.print(String(settings.topTemp), 76, 122-64);
    myGLCD.print(String(settings.cookTime), 76, 122);
    myGLCD.print(String(settings.bottomTemp), 76, 122+64);
    //myGLCD.fillRect(10, 10, 10, 10);
}
