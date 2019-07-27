/*  draw text's APP
 drawChar(INT8U ascii, poX,  poY, size,  fgcolor);
 drawString(char *string, poX,  poY, size, fgcolor);
 */
/*  draw number's APP
 drawNumber( long_num, poX,  poY, size, fgcolor);
 drawFloat(float floatNumber, decimal, poX,  poY, size, fgcolor);
 drawFloat(float floatNumber, poX,  poY, size, fgcolor);
 */
/*  Draw Boxes - Demonstrate drawRectangle and fillRectangle
 fillScreen( XL, XR, YU, YD, color);
 fillRectangle( poX,  poY,  length,  width,  color);
 drawRectangle( poX,  poY,  length, width, color);
 */


// Connect pin #1 of boty expander to Analog 5 (i2c clock)
// Connect pin #2 of both expanders to Analog 4 (i2c data)
// Connect pin #6 and 18 of the expander to 5V (power and reset disable)
// Connect pin #9 of the expander to ground (common ground)

/*
;Connect the following pins from MCP23008 to LCD
 ;
 ;P0 - D4
 ;P1 - D5
 ;P2 - D6
 ;P3 - D7
 ;P4 - RS
 ;P5 - RW (not used, set to 0 to ground for write)
 ;P6 - Bl (backlight switch)
 ;P7 - E
 */
#include <stdint.h>
#include <TFTv2.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_MCP23008.h"
#include <DDS.h>
#include <LiquidCrystal.h>
#include "rotary.h"
#include <U8x8lib.h>
//#include <U8g2lib.h>



//OLED shit oh yeahhhhh

// 8x8 display code does stuff like that
//#define U8LOG_WIDTH 16
//#define U8LOG_HEIGHT 8
//uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
//U8X8LOG u8x8log;
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8( A4 , A5 , U8X8_PIN_NONE);   // OLEDs without Reset of the Display


//
U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, A4, A5, U8X8_PIN_NONE);
//8x8 display
//U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8( SCL, SDA, U8X8_PIN_NONE);   // OLEDs without Reset of the Display
//page buffer
//U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display
//full buffer
//U8G2_SSD1306_128X64_ALT0_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);   // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 13, /* data=*/ 11, /* reset=*/ 8);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display


//mcp23008 pin assignments for ythe FIRST ONE

Adafruit_MCP23008 mcp;
#define expander_tuning_button  A0       // analog input 0
#define butt_matrix             A1    // analog input 1
#define mix_out                 18           // GPIO 0 
#define W_CLK_PIN               17                  // GPIO 1 word load clock pin
#define FQ_UD_PIN               16                   // GPIO 2 freq update pin
#define DATA_PIN                15                    // GPIO 3 serial data load pin
#define RESET_PIN               14                   // GPIO 4 reset pin

//mcp23008 pin assignments for ythe SECOND ONE

//#define A0      
//#define A1      ID ODNT FUCKING KNOW     


// define states for pin assignments on the multiplexer using an individual function for 
//each so's as i can get me some IF ELSE action going

#define mux_1a_high() {mcp.digitalWrite(18, HIGH);}  // MUX 1 A 
#define mux_1b_high() {mcp.digitalWrite(17, HIGH);}
#define mux_1c_high() {mcp.digitalWrite(16, HIGH);}
#define mux_2a_high() {mcp.digitalWrite(15, HIGH);}
#define mux_2b_high() {mcp.digitalWrite(14, HIGH);}
#define mux_2c_high() {mcp.digitalWrite(13, HIGH);}

#define mux_1a_low() {mcp.digitalWrite(18, LOW);}  // MUX 1 A 
#define mux_1b_low() {mcp.digitalWrite(17, LOW);}//wurd
#define mux_1c_low() {mcp.digitalWrite(16, LOW);}
#define mux_2a_low() {mcp.digitalWrite(15, LOW);}
#define mux_2b_low() {mcp.digitalWrite(14, LOW);}
#define mux_2c_low() {mcp.digitalWrite(13, LOW);}




// 4051 1:8 MUX/DEMUX input pins on the Arduino!
#define mux_1   4
#define mux_2   5


//for DDS output to MCP:
/*
init (bool = FALSE) 
 pulsehigh(bool = FLASE)
 setFrequency(int FREQ, bool FALSE)
 
 */
//AD9850 module and mixer Mux/Demux pin assignments, taking optputs from the signal generator and routing them around the radio
// the multiplexer is used only as output from the module, all input is cumming from pin headers soldered into the module connecting 
// to the mcp23008's or directly to an arduino.
DDS dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);

#define ard_2_mux  5 // pin from arduino to catch the multiplexer output on the COM line
#define sin_1      13  // y0 used as input
#define sin_2      14 // y1 used as input
#define sq_1       15  // y2 used as input
#define sq_2       12  // y3 used as input
#define mix_a      16//mixer output a and b to pin 1,4 of mux
#define mix_b      17 // look up
//pinMode(ard_2_mux, INPUT); //COM line to mux
//



//#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

int val =  123;
int x =    0 ;
int y =    0 ;
int w =    0 ;
int l =    0 ;


//setings for the encoder and frequency display
#define rot_pin1    A1
#define rot_pin2    A0


int_fast32_t rx           = 0000000; // Base (starting) frequency of VFO.  This only loads once.  To force load again see ForceFreq variable below.
int_fast32_t rx2          = 1;// variable to hold the updated frequency
int_fast32_t increment    = 10; // starting VFO update increment in HZ.
int_fast32_t iffreq       = 0000000; // Intermedite Frequency - Amount to subtract (-) from base frequency. ********************************************
int GoIF                  = 1;
Rotary r                  = Rotary(rot_pin1, rot_pin2); // sets the pins the rotary encoder uses.  Must be interrupt pins.
int buttonstate           = 0;
int buttonstate2          = 0;
int  hertzPosition        = 5;
String hertz              = "10 hz";
int out                   = 0 ;
byte                         ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
int_fast32_t timepassed   = millis() ;// int to hold the arduino miilis since startup
int memstatus             = 1  ;// value to notify if memory is current or old. 0=old, 1=current.
int ForceFreq             = 1; // Change this to 0 after you upload and run a working sketch to activate the EEPROM memory.  YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!
int phase                 = 180;
int disp                  = 1   // which display mode to put it on, have to code an auto select based on which lid is used 

// Interrupt routine to catch the rotary encoder
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result) {    
    if (result == DIR_CW){
      rx=rx+increment;
    }
    else {
      rx=rx-increment;
    };       
    if (rx >=30000000){
      rx=rx2;
    }; // UPPER VFO LIMIT
    if (rx <=0000000){
      rx=rx2;
    }; // LOWER VFO LIMIT
  } 
}



void displayValue_TFT(int valu, int x_, int y_, int w_, int l_, char col) {
  // make rectangle clear the space for yhe chars plus padding of  1pixel on both sides
  Tft.fillRectangle(0,15,150,30,BLACK);
  delay(100);
  Tft.drawNumber(valu,0,25,2,WHITE);  
}

//this oarticular bit o codey boi is fo' muxie boi numBAH'2... she's on the ads9850 mixer board!
void ads9850_o_sel(int out){

  if (out = 1){
    // square wave 1 
    mux_chan_sel(sq_1, 2);
  }
  else if (out = 2){
    // square wave 2 
    mux_chan_sel(sq_2, 2);
  }

  else if (out = 3){
    // sine wave 1 
    mux_chan_sel(sin_1, 2);
  }
  else if (out = 4){
    // sine wave 2 
    mux_chan_sel(sin_2, 2);
  }
  else if (out = 5){
    // mixer output wave 1 
    mux_chan_sel(mix_a, 2);
  }
  else if (out = 6){
    // mixer output wave 2
    mux_chan_sel(mix_b, 2);
  }
}

void mux_chan_sel(int ch, int mux_num) {

  if (mux_num = 1) {

    if (ch = 1) {
      mux_1a_low()
        mux_1a_low()
          mux_1a_low()
          }
    else if (ch = 2) {
      mux_1a_low()
        mux_1a_low()
          mux_1a_high()
          }
    else if (ch = 3) {
      mux_1a_low()
        mux_1a_high()
          mux_1a_low()
          }
    else if (ch = 4) {
      mux_1a_low()
        mux_1a_high()
          mux_1a_high()
          }
    else if (ch = 5) {
      mux_1a_high()
        mux_1a_low()
          mux_1a_low()
          }
    else if (ch = 6) {
      mux_1a_high()
        mux_1a_low()
          mux_1a_high()
          }
    else if (ch = 7) {
      mux_1a_high()
        mux_1a_high()
          mux_1a_low()
          }
    else if (ch = 8) {
      mux_1a_high()
        mux_1a_high()
          mux_1a_high()
          }
    else if (mux_num = 2) {
      if (ch = 1) {
        mux_1a_low()
          mux_1a_low()
            mux_1a_low()
            }
      else if (ch = 2) {
        mux_1a_low()
          mux_1a_low()
            mux_1a_high()
            }
      else if (ch = 3) {
        mux_1a_low()
          mux_1a_high()
            mux_1a_low()
            }
      else if (ch = 4) {
        mux_1a_low()
          mux_1a_high()
            mux_1a_high()
            }
      else if (ch = 5) {
        mux_1a_high()
          mux_1a_low()
            mux_1a_low()
            }
      else if (ch = 6) {
        mux_1a_high()
          mux_1a_low()
            mux_1a_high()
            }
      else if (ch = 7) {
        mux_1a_high()
          mux_1a_high()
            mux_1a_low()
            }   
      else if (ch = 8) {
        mux_1a_high()
          mux_1a_high()
            mux_1a_high()
            }   

          }
        }
      }
}



//STARTUP FREQUENCY I PUT THIS HERE BECAUSE REASON I DONT UNDERSTAND... I MEAN YOU
//REASONS YOU WOULDNT UNDERSTAND... I THINK
double freq = 10000000;
double trimFreq = 124999500;

void setup() {

  //u8x8log.begin(u8x8, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  //  u8x8log.setRedrawMode(1);		// 0: Update screen with newline, 1: Update screen for every char  

    //establish pins used for communication with other boards
  //Tft.TFTinit();                                  // 
  Serial.begin(9600);
 // TFT_BL_ON;      // turn on the background light
  Wire.begin(); // join i2c bus (address optional for master)

  mcp.begin();      // use default address 0
  mcp.pinMode(0, OUTPUT);

  //digitalWrite(A0,HIGH);
  //digitalWrite(A5,HIGH);
  mcp.pinMode(mix_out, INPUT);
  mcp.pinMode(W_CLK_PIN, OUTPUT);
  mcp.pinMode(FQ_UD_PIN, OUTPUT); 
  mcp.pinMode(DATA_PIN, OUTPUT);
  mcp.pinMode(RESET_PIN, OUTPUT);
  //mcp.pinMode(

  //  pinMode(FQ_UD_PIN, OUTPUT);
  //  pinMode(W_CLK_PIN, OUTPUT);
  //  pinMode(DATA_PIN, OUTPUT);
  //6  pinMode(RESET_PIN, OUTPUT); 
  dds.init(); 
  dds.trim(trimFreq);

}

void loop() {
  // makes it so everytime you press the tuning button, the tuning increment will change on the encoder knob.
  buttonstate = digitalRead(mcp.digitalRead(expander_tuning_button));

}







