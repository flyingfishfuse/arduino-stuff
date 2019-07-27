
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <U8x8lib.h>
#include "bitlash.h"
#include "Adafruit_MCP23008.h"


U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, A5, A4, U8X8_PIN_NONE);
Adafruit_MCP23008 mcp;

//constants stay the same, DEFINE makes a constant because it DEFINES A CONCEPT

//#define INT0 A4                          //setting pin 2 to variable
//#define INT1 A5                          //setting pin 3 to var
const int buttPin              =  A1;     // the number of the pushbutton pin

// variables will change:
int BUTTPRESS                  =  0;         // variable for reading the pushbutton status
int hertz_int                  =  0;
int  hertzPosition             =  5;
String hertz                   =  "10 hz";
int out                        =  0 ;
byte                              ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
int increment                  =  10;
int fr                         =  0;     // FREQUENCY OF THE DDS, INITIALIZE THIS VARIABLE WITH AN ACCEPTABLRE VALUE
int event                      =  analogRead(buttPin);   


void setincrement(){

  if(increment == 10){
    increment        =  50; 
    hertz            =  "50 Hz";
    hertz_int        =  10;
    hertzPosition    =  5;
  }
  else if (increment == 50){
    increment        =  100;   
    hertz            =  "100 Hz";
    hertz_int        =  50;
    hertzPosition    =  4;
  }
  else if (increment == 100){
    increment        =  500; 
    hertz            =  "500 Hz";
    hertz_int        =  100; 
    hertzPosition    =  4;
  }
  else if (increment == 500){
    increment        =  1000; 
    hertz            = "1 Khz";
    hertz_int        =  500; 
    hertzPosition    =  6;
  }
  else if (increment == 1000){
    increment        =  2500; 
    hertz            =  "2.5 Khz";
    hertz_int        =  1000; 
    hertzPosition    = 4;
  }
  else if (increment == 2500){
    increment        =  5000; 
    hertz            = "5 Khz";
    hertz_int        =  2500;
    hertzPosition    = 6;
  }
  else if (increment == 5000){
    increment        =  10000; 
    hertz            =  "10 Khz";
    hertz_int        =  5000;
    hertzPosition    =  5;
  }
  else if (increment == 10000){
    increment        =  100000; 
    hertz            =  "100 Khz";
    hertz_int        =  10000;
    hertzPosition    =  4;
  }
  else if (increment == 100000){
    increment        =  1000000; 
    hertz            =  "1 Mhz";
    hertz_int        =  1000000; 
    hertzPosition    =  6;
  }    
  else{
    increment        =  10; 
    hertz            =  "10 Hz"; 
    hertz_int        =  10;
    hertzPosition    =  5;
  };  
};

int showFreq(){
  millions           =  int(fr/1000000);
  hundredthousands   =  ((fr/100000)%10);
  tenthousands       =  ((fr/10000)%10);
  thousands          =  ((fr/1000)%10);
  hundreds           =  ((fr/100)%10);
  tens               =  ((fr/10)%10);
  ones               =  ((fr/1)%10);

  return (millions + hundredthousands + tenthousands  + thousands  + hundreds  + tens  + ones);
};


void setup() {
  initBitlash(57600);		// must be first to initialize serial port
  mcp.begin();      // use default address 0

  Serial.begin(9600);
  pinMode(1, INPUT_PULLUP);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);

  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  u8g2.begin(/*Select=*/ 3, /*Right/Next=*/ 1, /*Left/Prev=*/ 2, /*Up=*/10 , /*Down=*/ 11, /*Home/Cancel=*/ 4 );
u8g2.clearBuffer();
  //buttPressed() ,  buttPressed(), buttPressed(), buttPressed(), buttPressed(), buttPressed())
  u8g2.setFont(u8g2_font_6x12_tr);

}



const char *string_list = 
"Set Frequency\n"
"Read Frequency\n"
"Batt. Stats\n"
"Arduino Stats\n"
"Serial Monitor\n"
"Terminal\n"
"FM Radio\n"
"Light\n"
"Hardware Settings";

uint8_t item =  1;
//int sel = 0;

void loop(void) {



  item   =  u8g2.userInterfaceSelectionList("Firefly", item, string_list);
  //setDefaultForegroundColor()
  if ( item == 0 ) {
    u8g2.userInterfaceMessage(
    "WAT.", 
    "",
    "",
    " ok ");
  } 

  else if ( item == 1 ) {
    u8g2.userInterfaceMessage(
    "Set Frequency.", 
    "",
    "",
    " ok ");
  } 
  else if( item == 2 ) {
    u8g2.userInterfaceMessage(
    "Read Frequency.", 
    "",
    "",
    " ok ");
  } 
  else if( item == 3 ) {
    u8g2.userInterfaceMessage(
    "Batt. Stats.", 
    "",
    "",
    " ok ");
  } 
  else if( item == 4 ) {
    u8g2.userInterfaceMessage(
    "Arduino Stats.", 
    "",
    "",
    " ok ");
  } 

  else if( item == 5 ) {
     if (u8g2.userInterfaceMessage("Terminal", "", "", " ok /n Cancel ") == 1){ ;
      runBitlash();
    }
  } 

  else {
    u8g2.userInterfaceMessage(
    "Selection:", 
    u8x8_GetStringLineStart(item-1, string_list ),
    "",
    " ok \n cancel ");
  };
};



