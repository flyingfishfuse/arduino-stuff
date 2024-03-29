#include <Wire.h>
#include "Adafruit_MCP23008.h"
#include <AD9850.h>
#include <LiquidCrystal.h>
#include <rotary.h>

// Connect pin #1 of the expander to Analog 5 (i2c clock)
// Connect pin #2 of the expander to Analog 4 (i2c data)
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

//Setup some items

Adafruit_MCP23008 mcp;

//mcp23008 pin assignments

#define expander_tuning_button 5       // analog input 0
#define front_panel_button_matrix 4    // analog input 1
#define out_from_mixer 18              // GPIO 0 
#define W_CLK_PIN 17                   // GPIO 1 word load clock pin
#define FQ_UD_PIN 16                   // GPIO 2 freq update pin
#define DATA_PIN 15                    // GPIO 3 serial data load pin
#define RESET_PIN 14                   // GPIO 4 reset pin

//AD9850 module and mixer Mux/Demux pin assignments, taking optputs from the signal generator and routing them around the radio
// the multiplexer is used only as output from the module, all input is comming from pin headers soldered into the module connecting 
// to the mcp23008's or directly to an arduino.

#define ard_2_mux 5 // pin from arduino to catch the multiplexer output on the COM line

#define muxPin(addr) {digitalWrite(addr)
#define sin_1 13  // y0 used as input
#define sin_2 14  // y1 used as input
#define sq_1 15   // y2 used as input
#define sq_2 12   // y3 used as input
#define mix_o_a   //mixer output a and b to pin 1,4 of mux
#define mix_o_b   // look up


#define pulseHigh(pin) {digitalWrite(pin, HIGH); digitalWrite(pin, LOW); }

Rotary r = Rotary(2,3); // sets the pins the rotary encoder uses.  Must be interrupt pins.
LiquidCrystal lcd(12, 13, 7, 6, 5, 4); // I used an odd pin combination because I need pin 2 and 3 for the interrupts.
int_fast32_t rx=0000000; // Base (starting) frequency of VFO.  This only loads once.  To force load again see ForceFreq variable below.
int_fast32_t rx2=1; // variable to hold the updated frequency
int_fast32_t increment = 10; // starting VFO update increment in HZ.
int_fast32_t iffreq = 0000000; // Intermedite Frequency - Amount to subtract (-) from base frequency. ********************************************
int buttonstate = 0;
int buttonstate2 = 0;
int GoIF = 1;
String hertz = "10 Hz";
int  hertzPosition = 5;
byte ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders
String freq; // string to hold the frequency
int_fast32_t timepassed = millis(); // int to hold the arduino miilis since startup
int memstatus = 1;  // value to notify if memory is current or old. 0=old, 1=current.
int ForceFreq = 1;  // Change this to 0 after you upload and run a working sketch to activate the EEPROM memory.  YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!



void setup() {
  
  Serial.begin(9600);
  Wire.begin(); // join i2c bus (address optional for master)
  lcd.init();
  lcd.printIn("test");
  WriteLCD(ADDR,'a');
  lcd.clear();
  WriteLCD(ADDR,'c');
  a = analogRead(5);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("  analogRead() ");
  lcd.setCursor(0,1);
  lcd.print("  value is :");
  lcd.print(a);
  mcp.begin();      // use default address 0
  mcp.pinMode(0, OUTPUT);
  pinMode(A0,INPUT); // Connect to a button that goes to GND on push
  pinMode(A5,INPUT); // IF sense **********************************************
  digitalWrite(A0,HIGH);
  digitalWrite(A5,HIGH);
  lcd.begin(16, 2);
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
  pinMode(FQ_UD, OUTPUT);
  pinMode(W_CLK, OUTPUT);
  pinMode(DATA, OUTPUT);
  pinMode(RESET, OUTPUT); 
  pulseHigh(RESET);
  pulseHigh(W_CLK);
  pulseHigh(FQ_UD);  // this pulse enables serial mode on the AD9850 - Datasheet page 12.
  lcd.setCursor(hertzPosition,1);    
  lcd.print(hertz);  
}

void loop() {    

  // makes it so everytime you press the tuning button, the tuning increment will change on the encoder knob.
  buttonstate = digitalRead(mcp.digitalRead(expander_tuning_button);
  if(buttonstate == LOW) {
        setincrement();        
    };
    
  // Check for PIN low to drive IF offset Freq
  buttonstate = digitalRead(A5);  
    if (buttonstate != buttonstate2){
        if(buttonstate == LOW) {       
              lcd.setCursor(15,1);
              lcd.print("."); 
              GoIF = 0; 
              buttonstate2 = buttonstate; 
              sendFrequency(rx);         
          }
        else{
            lcd.setCursor(15,1);
            lcd.print(" ");
            GoIF = 1;
            buttonstate2 = buttonstate;
            sendFrequency(rx);       
          };
    };
    
    // Write the frequency to memory if not stored and 2 seconds have passed since the last frequency change.
    if(memstatus == 0){   
      if(timepassed+2000 < millis()){
        storeMEM();
        }
      }  
    
 }
 
 
// Interrupt routine to catch the rotary encoder
ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result) {    
    if (result == DIR_CW){rx=rx+increment;}
    else {rx=rx-increment;};       
      if (rx >=30000000){rx=rx2;}; // UPPER VFO LIMIT
      if (rx <=0000000){rx=rx2;}; // LOWER VFO LIMIT
  } 
}

// frequency calc from datasheet page 8 = <sys clock> * <frequency tuning word>/2^32
void sendFrequency(double frequency) {  
  if (GoIF == 1){frequency=frequency-iffreq;}; //If pin = low, subtract the IF frequency.
  int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850.  You can make 'slight' tuning variations here by adjusting the clock frequency.
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD);  // Done!  Should see output
}
// transfers a byte, a bit at a time, LSB first to the 9850 via serial DATA line
void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA, data & 0x01);
    pulseHigh(W_CLK);   //after each bit sent, CLK is pulsed high
  }
};

void setincrement(){
  if(increment == 10){increment = 50; hertz = "50 Hz"; hertzPosition=5;}
  else if (increment == 50){increment = 100;  hertz = "100 Hz"; hertzPosition=4;}
  else if (increment == 100){increment = 500; hertz="500 Hz"; hertzPosition=4;}
  else if (increment == 500){increment = 1000; hertz="1 Khz"; hertzPosition=6;}
  else if (increment == 1000){increment = 2500; hertz="2.5 Khz"; hertzPosition=4;}
  else if (increment == 2500){increment = 5000; hertz="5 Khz"; hertzPosition=6;}
  else if (increment == 5000){increment = 10000; hertz="10 Khz"; hertzPosition=5;}
  else if (increment == 10000){increment = 100000; hertz="100 Khz"; hertzPosition=4;}
  else if (increment == 100000){increment = 1000000; hertz="1 Mhz"; hertzPosition=6;}  
  else{increment = 10; hertz = "10 Hz"; hertzPosition=5;};  
   lcd.setCursor(0,1);
   lcd.print("                ");
   lcd.setCursor(hertzPosition,1); 
   lcd.print(hertz); 
   delay(250); // Adjust this delay to speed up/slow down the button menu scroll speed.
};

void showFreq(){
    millions = int(rx/1000000);
    hundredthousands = ((rx/100000)%10);
    tenthousands = ((rx/10000)%10);
    thousands = ((rx/1000)%10);
    hundreds = ((rx/100)%10);
    tens = ((rx/10)%10);
    ones = ((rx/1)%10);
    lcd.setCursor(0,0);
    lcd.print("                ");
   if (millions > 9){lcd.setCursor(1,0);}
   else{lcd.setCursor(2,0);}
    lcd.print(millions);
    lcd.print(".");
    lcd.print(hundredthousands);
    lcd.print(tenthousands);
    lcd.print(thousands);
    lcd.print(".");
    lcd.print(hundreds);
    lcd.print(tens);
    lcd.print(ones);
    lcd.print(" Mhz  ");
    timepassed = millis();
    memstatus = 0; // Trigger memory write
};



