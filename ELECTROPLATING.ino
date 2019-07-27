/*========================================================

ELECTROPLATING CODE


 Micah's Electroplator MARK IV, 

 Created by Micah Casteel
 02JUN2011
 
 Modified 
 03NOV2011
 
 
 FURTHER MODIFIED by moop the person typing.
 
 
==========================================================*/



#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <SPI.h>
#include <LiquidCrystal.h>
#include <TimerOne.h>
#include "Adafruit_MCP23008.h"
#include <rotary.h>
#include <EEPROM.h>


Adafruit_MCP23008 mcp;

//Buck converter with power mosfet for final stage
//digital pins
#define   MOSFET_PIN_1    10 //forward
#define   MOSFET_PIN_2    11 //reverse

#define   LCD_RS          12  //wired in dont change!!
#define   LCD_EN          8    //!!!!!!!!!!!!
#define   LCD_D4          2  //  ANALOG PIN USED AS DIGITAL!!!!
#define   LCD_D5          4  //!!!!!!!!!!!!
#define   LCD_D6          7  //!!!!!!!!!
#define   LCD_D7          8  // !!!!!!!!!!!!!!!!
#define   KillSwitch      1
#define   Activate        6
#define   _UNDEFINED_     3
#define   _UNDEFINED_     5

//analog pins
#define   BUCK_PIN_1      A0
#define   MUX_DEMUX       A1
#define   FEEDBACK        A2
#define   pot_trim        A3
#define   _UNDEFINED_     A4
#define   button_panel    A5

/*========================================================================================
SIMULATED MCP23008 pins be careful of the numbering! these pins are on a seperate chip!!

* Setings for the encoder and frequency display
* Settings for the AD9850 signal generator module
* Settings for the SA612 Mixer

=========================================================================================*/


#define W_CLK_PIN          15       // GPIO 1 word load clock pin
#define FQ_UD_PIN          14       // GPIO 2 freq update pin
#define DATA_PIN           13       // GPIO 3 serial data load pin
#define RESET_PIN          12       // GPIO 4 reset pin
#define rot_pin1           10
#define rot_pin2           11

//Constant Functions
#define pulseHigh(pin) {mcp.digitalWrite(pin, HIGH); mcp.digitalWrite(pin, LOW); }


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Rotary  r  =  Rotary(mcp.digitalRead(rot_pin1), mcp.digitalRead(rot_pin2)); // sets the pins the rotary encoder uses.  Must be interrupt pins.
//DDS dds(W_CLK_PIN, FQ_UD_PIN, DATA_PIN, RESET_PIN);

U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, A5, A4, U8X8_PIN_NONE);
//U8GLIB_SSD1306_64X48 u8g(U8G_I2C_OPT_NONE);	// I2C / TWI 

//miscellaneous flags, variables, etc...
boolean   Reset                 = false;         // Reset Function
boolean   Run                   = false;         // Running Bool
boolean   Kill                  = false;         // Kill Switch bool 
long      PulseOn               = 0;              // Simple pulse on time
long      PulseOff              = 0;             // Simple pulse off time
int       b                     =  0;
byte      address               =  0x00;
int       CS                    =  10;
int       analogInput           =  0;
float     vout                  =  0.0;
float     vin                   =  0.0;
float     R1                    =  100000.0; // resistance of R1 (100K)
float     R2                    =  10000.0; // resistance of R2 (10K) 
int       value                 =  0;
int       pwm                   =  0;
float     V_Out                 =  0.0;
float     I_Out                 =  0.0;
float     V_Bat                 =  0.0;
float     V_Bst                 =  0.0;
float     V_Chg                 =  0.0;
int       pot_trim_pos          =  0;
int       buttonstate           =  0;
int       buttonstate2          =  0;
String    hertz                 =  "10 Hz";
int       hertzPosition         =  5;
String    freq_string;          // string to hold the frequency
int       GoIF                  =  1;
int       memstatus             =  1;  // value to notify if memory is current or old. 0=old, 1=current.

/*=============================================
Frequency source :

  1 = ADS9850 signal generator module
  2 = Software Algorithm

=============================================*/
int       frequency_source      =  1; 


double    freq                  =  10000000;
double    trimFreq              =  124999500;


const char         *mode        = "simple"; //or "directional"
uint32_t           time         =  0;
volatile uint8_t   Switch1      =  1;
volatile uint8_t   Switch2      =  1;
static const int   dataPin      =  3;
static const int   clockPin     =  4 ;
int_fast32_t       rx           =  0000000;  // Base (starting) frequency of VFO.  This only loads once.  To force load again see ForceFreq variable below.
int_fast32_t       rx2          =  1;        // variable to hold the updated frequency
int_fast32_t       increment    =  10;       // starting VFO update increment in HZ.
int_fast32_t       iffreq       =  0000000;  // Intermedite Frequency - Amount to subtract (-) from base frequency. ********************************************
int_fast32_t       timepassed   =  millis(); // int to hold the arduino miilis since startup
byte               ones,tens,hundreds,thousands,tenthousands,hundredthousands,millions ;  //Placeholders

//This section sets up the waveform parameters, these will
//be changed to the desired run parameters
long               PlateDurationMil   =  7.5;              // Plating time duration (in minutes)
long               PlateDuration      =  PlateDurationMil; //Variable Plate Duration
long               Period             =  1000;             // Waveform period (ms)
float              DutyCycle          =  0.5;              // Percentage "on time" or "forward time"
long               RunTime            =  0;                // Run time variable

// Change this to 0 after you upload and run a working sketch to activate the EEPROM memory. 
// YOU MUST PUT THIS BACK TO 0 AND UPLOAD THE SKETCH AGAIN AFTER STARTING FREQUENCY IS SET!
int ForceFreq = 1;


//////////////////////////////////////////
// BUTTON PANEL
// this is the multiple button sensing device panel
// its got multiple buttons....
/////////////////////////////////////////

void readButtons(int pin) {
  // returns the button number pressed, or zero for none pressed
  // int pin is the analog pin number to read
  int b,c = 0;
  c=analogRead(pin); // get the analog value  

  if (c>1000) {

    b=0; // buttons have not been pressed

  } else if (c>440 && c<470) {

    b=1; // button 1 pressed

  }  else if (c<400 && c>370) {   // switches between simple and pulsed

    b=2;

  } else if (c>280 && c<310) {

    b=3; // button 3 pressed

  } else if (c>150 && c<180) {

    b=4; // button 4 pressed

  } else if (c<20) {

    b=5; // button 5 pressed

  }
  //return b;
}

/*=============================================
ENCODER CONTROL FUNCTIONS

  * set frequency
  * set PWM
  
=============================================*/

void read_encoder_set_freq() {
  
  unsigned char result = r.process();
  if (result) {    
    if (result == DIR_CW){rx=rx+increment;}
    else {rx=rx-increment;};       
      if (rx >=30000000){rx=rx2;}; // UPPER VFO LIMIT
      if (rx <=1000000){rx=rx2;}; // LOWER VFO LIMIT
      showFreq();
 
  
  }
}

//read the tuning knob for the voltage syetting
// MUST be a regular potentiometer, the encoder knob is a different device!
void digipot_func(boolean inc_dec, int amount) {
  
  pot_trim_pos = analogRead(pot_trim);
  int x= analogRead(pot_trim) ;
  int w= map(x,0,1023,0,255) ;
  analogWrite(6,w); // write mapped value on pin 6
  Serial.print("w    "); //print mapped value on screen
 
 }

void voltmeter(){
  // read the value at analog input
   value = analogRead(FEEDBACK);
   vout = (value * 5.0) / 1024.0; // see text
   vin = vout / (R2/(R1+R2)); 

        /*
  if (vin<0.09) {
         vin=0.0;//statement to quash undesired reading !
         }
         //u8g.firstPage();  
         do 
         {
         //draw_volt();      
         }
         while( );//u8g.nextPage() );
         delay(500);
         */
      }


void OutputSet(boolean For,boolean Rev){
  digitalWrite(MOSFET_PIN_1, For);
  digitalWrite(MOSFET_PIN_2, Rev);
}

void SimplePulseMain(long StartTime, long PlateDuration){
      while (millis() - StartTime < PlateDuration ){
        if (digitalRead(KillSwitch)==HIGH){
          Kill = true;
          PlateDuration = 0;
        }
        else{
          SimplePulseFunction(StartTime); 
         //Serial.println(RunTime);     debug help
         //Serial.println(x);           debug help 
        }
      }
      RunComplete();
}

void ConstantMain(long StartTime, long PlateDuration){    
      while (millis() - StartTime  < PlateDuration){
        if (digitalRead(KillSwitch)==HIGH){
          Kill = true;
          PlateDuration = 0;
        }
        else {
          OutputSet(1,0); 
        }
      }
      RunComplete();
}

void DirectionalMain(long StartTime, long PlateDuration){                              
      while (millis() - StartTime  < PlateDuration ){
      if (digitalRead(KillSwitch)==HIGH){
          Kill = true;
          PlateDuration = 0;
        }
        else{
         ModeDirectionalFunction(StartTime);
        }
      } 
      RunComplete();                                                                     
}    
  
void SimplePulseFunction(long StartTime){
  long Time = millis();
  long x = (Time-StartTime) % Period;  //establish period location from start time
  if (x <  PulseOn){ //pulse On
    OutputSet(1,0);
    }
  else{  //pulse Off
    OutputSet(0,0); 
    }
}

void ModeDirectionalFunction(long StartTime){
  long Time = millis();
  long x = (Time-StartTime) % Period; 
  if   (x <  PulseOn){           //Pulse Forward
    OutputSet(1,0);
    }
  else {
    OutputSet(1,1);          //pulse backwards
    }  
}

void RunComplete(){
  OutputSet(0,0);  
  Run = false; //Sets run flag
  if (Kill == true){
  }
  
  else{
   
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Run");
    lcd.setCursor(0,1);
    lcd.print("Complete");   //Initialize indicators
  }       
}

void KillLoop(){ 
  OutputSet(0,0);
  Reset = false;        //ensure reset is initialized
  //watch for reset  
  
  while (!Reset) {    //check for reset
   
    if (digitalRead(Activate)==HIGH && digitalRead(KillSwitch)==HIGH){   //double press and hold for reset
      Reset = true;
      Kill = false;
    }
    
    else{
      
      delay(3000);   //pause before checking reset again
    } 
  }
  
  delay (5000);    // pause to ensure hands are off the button



}                      // Reset complete 


/*=========================================================

ADS9850 module code

===========================================================*/
void set_freq () {
  
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print("Set Frequency");
       lcd.setCursor(0,1);
       lcd.print("");
       read_encoder_set_freq();
       sendFrequency(rx);
  
  
}

void sendFrequency(double frequency) {  
  if (GoIF == 1){frequency=frequency-iffreq;}; //If pin = low, subtract the IF frequency.
  int32_t freq = frequency * 4294967295/125000000;  // note 125 MHz clock on 9850.  You can make 'slight' tuning variations here by adjusting the clock frequency.
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x000);   // Final control byte, all 0 for 9850 chip
  pulseHigh(FQ_UD_PIN);  // Done!  Should see output
}

void tfr_byte(byte data)
{
  for (int i=0; i<8; i++, data>>=1) {
    digitalWrite(DATA_PIN, data & 0x01);
    pulseHigh(W_CLK_PIN);   //after each bit sent, CLK is pulsed high
  }
}

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

void storeMEM(){
  //Write each frequency section to a EPROM slot.  Yes, it's cheating but it works!
   EEPROM.write(0,millions);
   EEPROM.write(1,hundredthousands);
   EEPROM.write(2,tenthousands);
   EEPROM.write(3,thousands);
   EEPROM.write(4,hundreds);       
   EEPROM.write(5,tens);
   EEPROM.write(6,ones);   
   memstatus = 1;  // Let program know memory has been written
};


void setup() { 

  lcd.begin(16, 2);

  SPI.begin();

  analogReference(INTERNAL);


// frequency from ADS9850 signal generator
  if (frequency_source == 1 ) {
  
    mcp.pinMode(W_CLK_PIN, OUTPUT);
    mcp.pinMode(FQ_UD_PIN, OUTPUT); 
    mcp.pinMode(DATA_PIN, OUTPUT);
    mcp.pinMode(RESET_PIN, OUTPUT);
    
    // Update the display and frequency if the new Freq NEQ the old Freq  
    if (rx != rx2){   
        showFreq();
        sendFrequency(rx);
        rx2 = rx;        
      } 
      
   if (ForceFreq == 0) {
    freq_string = String(EEPROM.read(0))+String(EEPROM.read(1))+String(EEPROM.read(2))+String(EEPROM.read(3))+String(EEPROM.read(4))+String(EEPROM.read(5))+String(EEPROM.read(6));
    rx = freq_string.toInt();  
   } 

// frequency from ADS9850 signal generator
  } else if (frequency_source == 2 ) {
     
     TCCR2B = TCCR2B & B11111000 | B00000001;    // pin 3 and 11 PWM frequency of 31372.55 Hz
     //Perform initialization calcs 
     // Period = 10 * (Period / 10); //  trim period to 10 millisecond intervals
     PlateDurationMil = Period*((PlateDuration * 60 * 1000)/Period); //conversion to milliseconds and chops off scrap
     PulseOn = Period * DutyCycle; // set on time variable
     PulseOff = Period - PulseOn; // set off time variable   

}

//---------------------------------------------------

 
  pinMode(pot_trim, INPUT);
  pinMode (CS, OUTPUT);
  pinMode(FEEDBACK, INPUT);
  pinMode(BUCK_PIN_1, OUTPUT);  
  pinMode(MOSFET_PIN_1, OUTPUT) ;
  pinMode(MOSFET_PIN_2, OUTPUT) ;
  
//---------------------------------------------------
  
  lcd.print("BuZZb0X V1\n");   
}



void loop() {
  
  // read the button panel every looop for instruction.
  // activate the LCD display.
  readButtons(button_panel);
  lcd.display();
  
 /*===================
  Button one selects the frequency increment
 ====================*/
 if (b = 0) {
  
  setincrement();         
  
/*===================
  Button two SETS the frequency!
====================*/   
 } else if (b = 1) {  
    if(memstatus == 0){   
    if(timepassed+2000 < millis()){
    storeMEM();
        }
      }   
      
 /*===================
  Button THREE selects the mode of operation
  for the electroplating software algorithm
 ====================*/   
} else if (b = 2) {

    if (mode == "simple") {
      mode  = "directional";
       lcd.clear();
       lcd.setCursor(0,0);
       lcd.print("Type:");
       lcd.setCursor(0,1);
       lcd.print("Directional");
    } else if (mode == "directional") {
        mode = "simple";
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Type:");
        lcd.setCursor(0,1);
        lcd.print("Simple Pulse");
    } 
          
 /*===================
  Button FOUR starts the plating process
 ====================*/          
} else if (b = 4) {
    lcd.print("Electroplating:");
    
    /*====================================
    
    signal from ADS9850
    
    ====================================*/
    if (frequency_source = 1) {
     
      // send signal through mosfyet, use square wave output
      // for electroplating, sinewave for mixer.
     
      
    /*====================================
    
    Signal from software
    
    ====================================*/      
    } else if (frequency_source = 2) {
      
      Run = true; //Sets run flag
      long StartTime = millis();
      PlateDuration = PlateDurationMil;
     
      if (mode == "simple" ) { //simple program
          SimplePulseMain(StartTime, PlateDuration);
        } else if(mode == "directional") {    //directional program
          DirectionalMain(StartTime, PlateDuration); 
        }  else {      //only on program  
          ConstantMain(StartTime, PlateDuration);
        }
        float voltage = analogRead(pot_trim);
        float output  = analogRead(FEEDBACK);
        //update_display();

      if (voltage > output) {
        pwm = pwm-1;
        pwm = constrain(pwm, 1, 254);
      } if (voltage < output) {
        pwm = pwm+1;
        pwm = constrain(pwm, 1, 254);
      }

    analogWrite(MOSFET_PIN_1,pwm);

     
                   
/*===================
  Button FIVE sets the voltage
====================*/         
} else if (b = 5) {     
     
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Set Voltage");
    lcd.setCursor(0,1);
    float voltage_set = analogRead(pot_trim);
    lcd.print(voltage_set);

        
 /*===================
  Button NOTHING does NOTHING!!
 ====================*/          
} else {

    //do nothing
 }


}
}
// Read feedback 




