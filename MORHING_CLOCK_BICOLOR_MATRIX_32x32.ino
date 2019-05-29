//************************************************************************************************************//
  
//********************* THE 32x32 RED GREEN LED MATRIX USING BIT ANGLE MODULATION METHOD ********************//
/*
MORPHING DIGITAL CLOCK IS REFERED FROM HARIFUN: https://github.com/hwiguna/HariFun_166_Morphing_Clock/tree/master/Latest/MorphingClock

*/

//************************************************************************************************************//
#include <SPI.h>
#include "Wire.h"

// REAL PIN ON ARDUINO

#define blank_pin       3   // BLANK PIN - 74HC595
#define latch_pin       2   // LATCH PIN - 74HC595
#define clock_pin       52  // CLOCK PIN - 74HC595
#define data_pin        51  // DATA PIN - 74HC595

#define RowA_Pin        22  // A PIN - 74HC238
#define RowB_Pin        24  // B PIN - 74HC238
#define RowC_Pin        26  // C PIN - 74HC238
#define RowD_Pin        28  // D PIN - 74HC238
//#define OE_Pin          30  // ENABLE OUTPUT PIN - 74HC238

//MAPPING TO PORT

#define Blank_Pin_Bit   5 // PORTE bit 5 - PE5
#define Latch_Pin_Bit   4 // PORTE bit 4 - PE4

#define RowA_Pin_Bit    0 // PORTA bit 0 - PA0
#define RowB_Pin_Bit    2 // PORTA bit 2 - PA2
#define RowC_Pin_Bit    4 // PORTA bit 4 - PA4
#define RowD_Pin_Bit    6 // PORTA bit 6 - PA6
#define OE_Pin_Bit      7 // PORTC bit 7 - PC7

/**   AN RED GREEN COLOR TEMPLATE */
struct Color
{
  unsigned char red, green;

  Color(int r, int g) : red(r), green(g) {}
  Color() : red(0), green(0) {}
};

const Color redcolor        = Color(0x0F,0x00);
const Color orangecolor     = Color(0x0F,0x0F);
const Color yellowcolor     = Color(0x0F,0x09);
const Color greencolor      = Color(0x00,0x0F);
const Color clearcolor      = Color(0x00,0x00);


// *************************************************For RTC DS3231******************************************//
#define DS3231_I2C_ADDRESS 0x68
unsigned long samplingtime = 0;
byte ssecond, sminute, shour, sdayOfWeek, sdayOfMonth, smonth, syear;

//*************************************************MORPHING******************************************//
const byte sA = 0;
const byte sB = 1;
const byte sC = 2;
const byte sD = 3;
const byte sE = 4;
const byte sF = 5;
const byte sG = 6;
const int segHeight = 5;
const int segWidth = segHeight;
const uint16_t height = 31;
const uint16_t width = 31;
int animSpeed = 30;
Color bgcolor;

byte digitBits[] = {
  B11111100, // 0 ABCDEF--
  B01100000, // 1 -BC-----
  B11011010, // 2 AB-DE-G-
  B11110010, // 3 ABCD--G-
  B01100110, // 4 -BC--FG-
  B10110110, // 5 A-CD-FG-
  B10111110, // 6 A-CDEFG-
  B11100000, // 7 ABC-----
  B11111110, // 8 ABCDEFG-
  B11110110, // 9 ABCD_FG-
};


typedef struct
{
  byte _value;
  uint16_t xOffset;
  uint16_t yOffset;
  Color _color;
  Color _bgcolor;
} Digit;


byte prevhour;
byte prevminute;
byte prevsecond;
Color updatecolor;

//

Digit digit0{0, 17, 10, redcolor, bgcolor};
Digit digit1{0, 25, 10, redcolor, bgcolor};
Digit digit2{0, 0, 1, yellowcolor, bgcolor};
Digit digit3{0, 8, 1, yellowcolor, bgcolor};
Digit digit4{0, 0, 18, orangecolor, bgcolor};
Digit digit5{0, 8, 18, orangecolor, bgcolor};


//************************************************************************************************************//

#ifndef _swap_int16_t
#define _swap_int16_t(a, b)   { int16_t t = a; a = b; b = t; }
#endif


//****************************************************BAM**************************************************//

#define BAM_RESOLUTION 4    // EG 4 bit colour = 15 variation of R, G (256 colours)

const  byte Size_Y = 32;    //Number of LEDS in Y axis (Top to Bottom)
const  byte Size_X = 32;    //Number of LEDs in X axis (Left to Right)

byte red[4][128];
byte green[4][128];

int level=0;                //keeps track of which level we are shifting data to
int row=0;
int BAM_Bit, BAM_Counter=0; // Bit Angle Modulation variables to keep track of things

void setup()
{
SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2);

noInterrupts();

TCCR1A = B00000000;
TCCR1B = B00001011;
TIMSK1 = B00000010;
OCR1A = 15;

pinMode(latch_pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
pinMode(data_pin, OUTPUT);
pinMode(clock_pin, OUTPUT);

pinMode(RowA_Pin, OUTPUT);
pinMode(RowB_Pin, OUTPUT);
pinMode(RowC_Pin, OUTPUT);
pinMode(RowD_Pin, OUTPUT);
//pinMode(OE_Pin, OUTPUT);

//digitalWrite(OE_Pin, HIGH);
SPI.begin();
interrupts();
Wire.begin();


setBGcolor(greencolor);               // Set clock's background color
fillTable(bgcolor.red,bgcolor.green);
ReadDS3231Time(&ssecond, &sminute, &shour, &sdayOfWeek, &sdayOfMonth, &smonth, &syear); // Read RTC and print TIME when start-up

Digit_Draw(digit0, (ssecond >> 4 & 0x0f));
Digit_Draw(digit1, (ssecond & 0x0f));
Digit_Draw(digit2, (sminute >> 4 & 0x0f));
Digit_Draw(digit3, (sminute & 0x0f));
Digit_Draw(digit4, (shour >> 4 & 0x0f));
Digit_Draw(digit5, (shour & 0x0f));

}

void loop()
{
  Display_HMS();
}

void LED(int X, int Y, int R, int G)
{
  X = constrain(X, 0, Size_X - 1); 
  Y = constrain(Y, 0, Size_Y - 1);
  
  R = constrain(R, 0, (1 << BAM_RESOLUTION) - 1);
  G = constrain(G, 0, (1 << BAM_RESOLUTION) - 1); 

  int WhichByte = int(Y*4+ X/8);
  int WhichBit = 7-(X%8);

  for (byte BAM = 0; BAM < BAM_RESOLUTION; BAM++) 
  {
    bitWrite(red[BAM][WhichByte], WhichBit, bitRead(R, BAM));

    bitWrite(green[BAM][WhichByte], WhichBit, bitRead(G, BAM));
  }

}

void rowScan(byte row)
{  
  if (row & 0x01) PORTA |= _BV(RowA_Pin_Bit);   //PORTA |= _BV(0)
    else PORTA &= ~_BV(RowA_Pin_Bit);           //PORTA &= ~_BV(0)
  
  if (row & 0x02) PORTA |= _BV(RowB_Pin_Bit);   //PORTA |= _BV(2)
    else PORTA &= ~_BV(RowB_Pin_Bit);           //PORTA &= ~_BV(2)

  if (row & 0x04) PORTA |= _BV(RowC_Pin_Bit);   //PORTA |= _BV(4)
    else PORTA &= ~_BV(RowC_Pin_Bit);           //PORTA &= ~_BV(4)

  if (row & 0x08) PORTA |= _BV(RowD_Pin_Bit);   //PORTA |= _BV(6)
    else PORTA &= ~_BV(RowD_Pin_Bit);           //PORTA &= ~_BV(6)
}

ISR(TIMER1_COMPA_vect){
  
PORTE |= ((1<<Blank_Pin_Bit));                  // Set BLANK PIN high - 74HC595
//PORTC |= _BV(OE_Pin_Bit);                       // Set OUTPUT ENABLE high - 74HC238

if(BAM_Counter==4)
BAM_Bit++;
else
if(BAM_Counter==12)
BAM_Bit++;
else
if(BAM_Counter==28)
BAM_Bit++;

BAM_Counter++;

switch (BAM_Bit)
{
    case 0:

      //Red
      
        myTransfer(red[0][level + 0]);
        myTransfer(red[0][level + 1]);
        myTransfer(red[0][level + 2]);
        myTransfer(red[0][level + 3]);
        myTransfer(red[0][level + 64]);
        myTransfer(red[0][level + 65]);
        myTransfer(red[0][level + 66]);
        myTransfer(red[0][level + 67]);

      //Green
        
        myTransfer(green[0][level + 0]);
        myTransfer(green[0][level + 1]);
        myTransfer(green[0][level + 2]);
        myTransfer(green[0][level + 3]);
        myTransfer(green[0][level + 64]);
        myTransfer(green[0][level + 65]);
        myTransfer(green[0][level + 66]);
        myTransfer(green[0][level + 67]);

      break;
    case 1:
       
      //Red

        myTransfer(red[1][level + 0]);
        myTransfer(red[1][level + 1]);
        myTransfer(red[1][level + 2]);
        myTransfer(red[1][level + 3]);  

        myTransfer(red[1][level + 64]);
        myTransfer(red[1][level + 65]);
        myTransfer(red[1][level + 66]);
        myTransfer(red[1][level + 67]);             
      //Green

        myTransfer(green[1][level + 0]);
        myTransfer(green[1][level + 1]);
        myTransfer(green[1][level + 2]);
        myTransfer(green[1][level + 3]);
        
        myTransfer(green[1][level + 64]);
        myTransfer(green[1][level + 65]);
        myTransfer(green[1][level + 66]);
        myTransfer(green[1][level + 67]);
        
      break;
    case 2:
      
      //Red

        myTransfer(red[2][level + 0]);
        myTransfer(red[2][level + 1]);
        myTransfer(red[2][level + 2]);
        myTransfer(red[2][level + 3]);
        
        myTransfer(red[2][level + 64]);
        myTransfer(red[2][level + 65]);
        myTransfer(red[2][level + 66]);
        myTransfer(red[2][level + 67]);
                      
       //Green

        myTransfer(green[2][level + 0]);
        myTransfer(green[2][level + 1]);
        myTransfer(green[2][level + 2]);
        myTransfer(green[2][level + 3]);
        
        myTransfer(green[2][level + 64]);
        myTransfer(green[2][level + 65]);
        myTransfer(green[2][level + 66]);
        myTransfer(green[2][level + 67]);

      break;
    case 3:
      //Red

        myTransfer(red[3][level + 0]);
        myTransfer(red[3][level + 1]);
        myTransfer(red[3][level + 2]);
        myTransfer(red[3][level + 3]); 
               
        myTransfer(red[3][level + 64]);
        myTransfer(red[3][level + 65]);
        myTransfer(red[3][level + 66]);
        myTransfer(red[3][level + 67]);    

      //Green

        myTransfer(green[3][level + 0]);
        myTransfer(green[3][level + 1]);
        myTransfer(green[3][level + 2]);
        myTransfer(green[3][level + 3]);
              
        myTransfer(green[3][level + 64]);
        myTransfer(green[3][level + 65]);
        myTransfer(green[3][level + 66]);
        myTransfer(green[3][level + 67]);
        
  if(BAM_Counter==60){
  BAM_Counter=0;
  BAM_Bit=0;
  }
  break;
}

rowScan(row);

PORTE |= 1<<Latch_Pin_Bit;

PORTE &= ~(1<<Latch_Pin_Bit);

PORTE &= ~(1<<Blank_Pin_Bit);       // Set BLANK PIN low - 74HC595
//PORTC &= ~_BV(OE_Pin_Bit);          // Set OUTPUT ENABLE low - 74HC238

row++;
level = row<<2;
if(row==16)
row=0;
if(level==64)
level=0;

DDRE |= _BV (Blank_Pin_Bit);    // pinMode (blank_pin, OUTPUT);
//DDRC |= _BV (OE_Pin_Bit);       // pinMode (OE_Pin, OUTPUT);
//pinMode(blank_pin, OUTPUT);
//pinMode(OE_Pin, OUTPUT);

}

inline static uint8_t myTransfer(uint8_t C_data){
  SPDR = C_data;
  asm volatile("nop"); 
  asm volatile("nop");
  
}

void clearfast ()
{
    memset(red, 0, sizeof(red[0][0]) * 4 * 128);
    memset(green, 0, sizeof(green[0][0]) * 4 * 128);
}

void fillTable(byte R, byte G)
{
    for (byte x=0; x<32; x++)
    {
      for (byte y=0; y<32; y++)
      {
        LED(x, y, R, G);
      }
    }
}

/*--------------------------------------------------------------------------------------
 Draw or clear a line (horizontal and vertical line) from x1, y1 to x2, y2
--------------------------------------------------------------------------------------*/
// Bresenham's algorithm
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, Color color) 
{
  int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) 
      {
        _swap_int16_t(x0, y0);
        _swap_int16_t(x1, y1);
      }
  
    if (x0 > x1) 
      {
        _swap_int16_t(x0, x1);
        _swap_int16_t(y0, y1);
      }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) 
    {
      ystep = 1;
    } else 
    {
      ystep = -1;
    }

    for (; x0<=x1; x0++) 
    {
      if (steep) 
      {
      LED(y0, x0, color.red, color.green);
      } 
      else 
      {
      LED(x0, y0, color.red, color.green);
      }
      err -= dy;
      if (err < 0) 
      {
        y0 += ystep;
        err += dx;
      }
    }
}

void fillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, Color color)
{
    for (uint16_t x = x1; x <= x2; x++) {
        for (uint16_t y = y1; y <= y2; y++) {
            LED(x,y, color.red, color.green);      
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Color setBGcolor(Color backgroundcolor)
{
  bgcolor = backgroundcolor;
  return bgcolor;
}

byte Value(Digit digit) {
  return digit._value;
}


void drawPixel(Digit digit, uint16_t x, uint16_t y, Color setcolor)
{
  LED(digit.xOffset + x, height - (y + digit.yOffset), setcolor.red, setcolor.green);
}

void Digit_drawLine(Digit digit, uint16_t x, uint16_t y, uint16_t x2, uint16_t y2, Color setcolor)
{
  drawLine(digit.xOffset + x, height - (y + digit.yOffset), digit.xOffset + x2, height - (y2 + digit.yOffset), setcolor);
}

void drawFillRect(Digit digit, uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
  fillRectangle(digit.xOffset + x, height - (y + digit.yOffset), digit.xOffset + x + w, height - (y + digit.yOffset) + h, digit._color); 
}

void DrawColon(Digit digit)
{
  // Colon is drawn to the left of this digit
  drawFillRect(digit, -3, segHeight - 1, 2, 2);
  drawFillRect(digit, -3, segHeight + 1 + 3, 2, 2);
}

void drawSeg(Digit digit, byte seg)
{
  switch (seg) {
    case sA: Digit_drawLine(digit, 1, segHeight * 2 + 2, segWidth, segHeight * 2 + 2, digit._color); break;
    case sB: Digit_drawLine(digit, segWidth + 1, segHeight * 2 + 1, segWidth + 1, segHeight + 2, digit._color); break;
    case sC: Digit_drawLine(digit, segWidth + 1, 1, segWidth + 1, segHeight, digit._color); break;
    case sD: Digit_drawLine(digit, 1, 0, segWidth, 0, digit._color); break;
    case sE: Digit_drawLine(digit, 0, 1, 0, segHeight, digit._color); break;
    case sF: Digit_drawLine(digit, 0, segHeight * 2 + 1, 0, segHeight + 2, digit._color); break;
    case sG: Digit_drawLine(digit, 1, segHeight + 1, segWidth, segHeight + 1, digit._color); break;
  }
}

void Digit_Draw(Digit digit, byte value) {
  byte pattern = digitBits[value];
  if (bitRead(pattern, 7)) drawSeg(digit, sA);
  if (bitRead(pattern, 6)) drawSeg(digit, sB);
  if (bitRead(pattern, 5)) drawSeg(digit, sC);
  if (bitRead(pattern, 4)) drawSeg(digit, sD);
  if (bitRead(pattern, 3)) drawSeg(digit, sE);
  if (bitRead(pattern, 2)) drawSeg(digit, sF);
  if (bitRead(pattern, 1)) drawSeg(digit, sG);
  digit._value = value;
}

void Morph2(Digit digit) {
  // TWO
  for (int i = 0; i <= segWidth; i++)
  {
    if (i < segWidth) {
      drawPixel(digit, segWidth - i, segHeight * 2 + 2, digit._color);
      drawPixel(digit, segWidth - i, segHeight + 1, digit._color);
      drawPixel(digit, segWidth - i, 0, digit._color);
    }

    Digit_drawLine(digit, segWidth + 1 - i, 1, segWidth + 1 - i, segHeight, bgcolor);
    Digit_drawLine(digit, segWidth - i, 1, segWidth - i, segHeight, digit._color);
    delay(animSpeed);
  }
}

void Morph3(Digit digit) {
  // THREE
  for (int i = 0; i <= segWidth; i++)
  {
    Digit_drawLine(digit, 0 + i, 1, 0 + i, segHeight, bgcolor);
    Digit_drawLine(digit, 1 + i, 1, 1 + i, segHeight, digit._color);
    delay(animSpeed);
  }
}

void Morph4(Digit digit) {
  // FOUR
  for (int i = 0; i < segWidth; i++)
  {
    drawPixel(digit, segWidth - i, segHeight * 2 + 2, bgcolor); // Erase A
    drawPixel(digit, 0, segHeight * 2 + 1 - i, digit._color); // Draw as F
    drawPixel(digit, 1 + i, 0, bgcolor); // Erase D
    delay(animSpeed);
  }
}

void Morph5(Digit digit) {
  // FIVE
  for (int i = 0; i < segWidth; i++)
  {
    drawPixel(digit, segWidth + 1, segHeight + 2 + i, bgcolor); // Erase B
    drawPixel(digit, segWidth - i, segHeight * 2 + 2, digit._color); // Draw as A
    drawPixel(digit, segWidth - i, 0, digit._color); // Draw D
    delay(animSpeed);
  }
}

void Morph6(Digit digit) {
  // SIX
  for (int i = 0; i <= segWidth; i++)
  {
    // Move C right to left
    Digit_drawLine(digit, segWidth - i, 1, segWidth - i, segHeight, digit._color);
    if (i > 0) Digit_drawLine(digit, segWidth - i + 1, 1, segWidth - i + 1, segHeight, bgcolor);
    delay(animSpeed);
  }
}

void Morph7(Digit digit) {
  // SEVEN
  for (int i = 0; i <= (segWidth + 1); i++)
  {
    // Move E left to right
    Digit_drawLine(digit, 0 + i - 1, 1, 0 + i - 1, segHeight, bgcolor);
    Digit_drawLine(digit, 0 + i, 1, 0 + i, segHeight, digit._color);

    // Move F left to right
    Digit_drawLine(digit, 0 + i - 1, segHeight * 2 + 1, 0 + i - 1, segHeight + 2, bgcolor);
    Digit_drawLine(digit, 0 + i, segHeight * 2 + 1, 0 + i, segHeight + 2, digit._color);

    // Erase D and G gradually
    drawPixel(digit, 1 + i, 0, bgcolor); // D
    drawPixel(digit, 1 + i, segHeight + 1, bgcolor); // G
    delay(animSpeed);
  }
}

void Morph8(Digit digit) {
  // EIGHT
  for (int i = 0; i <= segWidth; i++)
  {
    // Move B right to left
    Digit_drawLine(digit, segWidth - i, segHeight * 2 + 1, segWidth - i, segHeight + 2, digit._color);
    if (i > 0) Digit_drawLine(digit, segWidth - i + 1, segHeight * 2 + 1, segWidth - i + 1, segHeight + 2, bgcolor);

    // Move C right to left
    Digit_drawLine(digit, segWidth - i, 1, segWidth - i, segHeight, digit._color);
    if (i > 0) Digit_drawLine(digit, segWidth - i + 1, 1, segWidth - i + 1, segHeight, bgcolor);

    // Gradually draw D and G
    if (i < segWidth) {
      drawPixel(digit, segWidth - i, 0, digit._color); // D
      drawPixel(digit, segWidth - i, segHeight + 1, digit._color); // G
    }
    delay(animSpeed);
  }
}

void Morph9(Digit digit) {
  // NINE
  for (int i = 0; i <= (segWidth + 1); i++)
  {
    // Move E left to right
    Digit_drawLine(digit, 0 + i - 1, 1, 0 + i - 1, segHeight, bgcolor);
    Digit_drawLine(digit, 0 + i, 1, 0 + i, segHeight, digit._color);
    delay(animSpeed);
  }
}

void Morph0(Digit digit) {
  // ZERO
  for (int i = 0; i <= segWidth; i++)
  {
    if (digit._value==1) { // If 1 to 0, slide B to F and E to C  
      // slide B to F 
      Digit_drawLine(digit, segWidth - i, segHeight * 2+1 , segWidth - i, segHeight + 2, digit._color);
      if (i > 0) Digit_drawLine(digit, segWidth - i + 1, segHeight * 2+1, segWidth - i + 1, segHeight + 2, bgcolor);

      // slide E to C
      Digit_drawLine(digit, segWidth - i, 1, segWidth - i, segHeight, digit._color);
      if (i > 0) Digit_drawLine(digit, segWidth - i + 1, 1, segWidth - i + 1, segHeight, bgcolor);

      if (i<segWidth) drawPixel(digit, segWidth - i, segHeight * 2 + 2 , digit._color); // Draw A
      if (i<segWidth) drawPixel(digit, segWidth - i, 0, digit._color); // Draw D
    }
    
    if (digit._value==2) { // If 2 to 0, slide B to F and Flow G to C
      // slide B to F 
      Digit_drawLine(digit, segWidth - i, segHeight * 2+1 , segWidth - i, segHeight + 2, digit._color);
      if (i > 0) Digit_drawLine(digit, segWidth - i + 1, segHeight * 2+1, segWidth - i + 1, segHeight + 2, bgcolor);
    
      drawPixel(digit, 1+i, segHeight + 1, bgcolor); // Erase G left to right
      if (i<segWidth) drawPixel(digit, segWidth + 1, segHeight + 1- i, digit._color);// Draw C
    }

    if (digit._value==3) { // B to F, C to E
      // slide B to F 
      Digit_drawLine(digit, segWidth - i, segHeight * 2+1 , segWidth - i, segHeight + 2, digit._color);
      if (i > 0) Digit_drawLine(digit, segWidth - i + 1, segHeight * 2+1, segWidth - i + 1, segHeight + 2, bgcolor);
      
      // Move C to E
      Digit_drawLine(digit, segWidth - i, 1, segWidth - i, segHeight, digit._color);
      if (i > 0) Digit_drawLine(digit, segWidth - i + 1, 1, segWidth - i + 1, segHeight, bgcolor);

      // Erase G from right to left
      drawPixel(digit, segWidth - i, segHeight + 1, bgcolor); // G
    }
    
    if (digit._value==5) { // If 5 to 0, we also need to slide F to B
      if (i<segWidth) {
        if (i>0) Digit_drawLine(digit, 1 + i, segHeight * 2 + 1, 1 + i, segHeight + 2, bgcolor);
        Digit_drawLine(digit, 2 + i, segHeight * 2 + 1, 2 + i, segHeight + 2, digit._color);
      }
    }
    
    if (digit._value==5 || digit._value==9) { // If 9 or 5 to 0, Flow G into E
      if (i<segWidth) drawPixel(digit, segWidth - i, segHeight + 1, bgcolor);
      if (i<segWidth) drawPixel(digit, 0, segHeight - i, digit._color);
    }
    delay(animSpeed);
  }
}

void Morph1(Digit digit) {
  // Zero to One
  for (int i = 0; i <= (segWidth + 1); i++)
  {
    // Move E left to right
    Digit_drawLine(digit, 0 + i - 1, 1, 0 + i - 1, segHeight, bgcolor);
    Digit_drawLine(digit, 0 + i, 1, 0 + i, segHeight, digit._color);

    // Move F left to right
    Digit_drawLine(digit, 0 + i - 1, segHeight * 2 + 1, 0 + i - 1, segHeight + 2, bgcolor);
    Digit_drawLine(digit, 0 + i, segHeight * 2 + 1, 0 + i, segHeight + 2, digit._color);

    // Gradually Erase A, G, D
    drawPixel(digit, 1 + i, segHeight * 2 + 2, bgcolor); // A
    drawPixel(digit, 1 + i, 0, bgcolor); // D
    drawPixel(digit, 1 + i, segHeight + 1, bgcolor); // G

    delay(animSpeed);
  }
}

void Morph(Digit digit, byte newValue) {
  switch (newValue) {
    case 2: Morph2(digit); break;
    case 3: Morph3(digit); break;
    case 4: Morph4(digit); break;
    case 5: Morph5(digit); break;
    case 6: Morph6(digit); break;
    case 7: Morph7(digit); break;
    case 8: Morph8(digit); break;
    case 9: Morph9(digit); break;
    case 0: Morph0(digit); break;
    case 1: Morph1(digit); break;
  }
  digit._value = newValue;
}

// Convert decimal numbers to binary coded decimal
byte decToBcd(byte val)
{
  return ( (val / 10 * 16) + (val % 10) );
}

// Convert binary coded decimal to decimal numbers
byte bcdToDec(byte val)
{
  return ( (val / 16 * 10) + (val % 16) );
}

void ReadDS3231Time(byte *second, byte *minute, byte *hour, byte *dayOfWeek, byte *dayOfMonth, byte *month, byte *year)
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);  // Set DS3231 register pointer to 00h
  Wire.endTransmission();

  // Request seven bytes of data from DS3231 starting from register 00h
  Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
  *second = (Wire.read() & 0x7f);
  *minute = (Wire.read());
  *hour = (Wire.read() & 0x3f); 
}

void Display_HMS()
{
  if ( (unsigned long) (millis() - samplingtime) > 50  )
  {
    byte csecond, cminute, chour, cdayOfWeek, cdayOfMonth, cmonth, cyear;
    ReadDS3231Time(&csecond, &cminute, &chour, &cdayOfWeek, &cdayOfMonth, &cmonth, &cyear); 
      if (csecond!=prevsecond) 
      { 
        byte s0 = ((csecond >> 4) & 0x0f);
        byte s1 = (csecond & 0x0f);
        if (s0!=Value(digit0)) Morph(digit0, s0);
        if (s1!=Value(digit1)) Morph(digit1, s1);
        prevsecond = csecond;
        digit0._value=s0;
        digit1._value=s1;
      }

      if (cminute!=prevminute) 
      {
        byte m0 = ((cminute >> 4) & 0x0f);
        byte m1 = (cminute & 0x0f);
        if (m0!=Value(digit2)) Morph(digit2, m0);
        if (m1!=Value(digit3)) Morph(digit3, m1);
        prevminute = cminute;
        digit2._value=m0;
        digit3._value=m1;
      }
      
      if (chour!=prevhour) 
      {
        byte h0 = ((chour >> 4) & 0x0f);
        byte h1 = (chour & 0x0f);
        if (h0!=Value(digit4)) Morph(digit4, h0);
        if (h1!=Value(digit5)) Morph(digit5, h1);
        prevhour = chour;
        digit4._value=h0;
        digit5._value=h1;
        
      }
      
      samplingtime = millis();
  }
}
