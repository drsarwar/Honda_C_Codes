#include <Wire.h>


//AD7746 definitions
#define I2C_ADDRESS  0x48 //0x90 shift one to the rigth

#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_VT_DATA 0x04
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_VT_SETUP 0x08
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A
#define REGISTER_CAP_DAC_A 0x0B
#define REGISTER_CAP_DAC_B 0x0B
#define REGISTER_CAP_OFFSET 0x0D
#define REGISTER_CAP_GAIN 0x0F
#define REGISTER_VOLTAGE_GAIN 0x11

#define RESET_ADDRESS 0xBF

#define VALUE_UPPER_BOUND 16000000L
#define VALUE_LOWER_BOUND 0xFL
#define MAX_OUT_OF_RANGE_COUNT 3
#define CALIBRATION_INCREASE 1



# define touch_thresh 9      //Set the threshold for touch detection here. Capacitance must be > touch_thresh * baseline value to trigger a touch
# define hover_thresh 0.95     // //Set the threshold for proximty detection here. Capacitance must be < hover_thresh * baseline value to trigger a touch


boolean touch_flag=0;         //flag set if a touch OR proximity is detected



boolean calibration_run=true;    //Ensures that an initial set of baseline values are sent once at the beginning
long value=0;           //Stores unconverted value from CDC
long average1[16];      //Used to store data for averaging for baseline level
long average2[16];
long average3[16];





float converted = 0;             //Stores converted pF value
int state_mux1 = 0;              // 2 bit state of mux1   
int state_mux2 = 0;              // 2 bit state of mux2   
 
int mux1_A0;                     //1 bit value to send to mux control pin
int mux1_A1;
int mux2_A0;
int mux2_A1;

int elapsed;
int start_time;

unsigned char address_bit0=0;    //Similar to mux1_A0 except for sending baseline addresses
unsigned char address_bit1=0;
unsigned char address_bit2=0;
unsigned char address_bit3=0;

boolean average_flag=true;       //If true an averaging calculation is done
unsigned char average_counter=0; //A counter to set which array will store 1 set of the 4 sets of data to be averaged. E.g. average_counter==0 means data is stored in average1[]



long BASELINE [16];    //Baseline capacitance of each position stored here, initially a set of the first values read at reset, then becomes a running average of 4 sets of data


unsigned char LED_POSITION = 0; //Stores led position. Also used as a counter to cycle through the various data arrays. 

unsigned char address[16]={B1111,B0000,B0001,B0010,B0011,B0100,B0101,B0110,B0111,B1000,B1001,B1010,B1011,B1100,B1101,B1110};     //Makes it easier to print baseline addresses





void setup()
{
  //Set Mux control pins to output mode
  pinMode(52, OUTPUT);
  pinMode(53, OUTPUT);
  pinMode(50, OUTPUT);
  pinMode(51, OUTPUT);





  Wire.begin(); // sets up i2c for operation
  Serial.begin(9600); // set up baud rate for serial communication to com port

  //Serial.println("Initializing");

  Wire.beginTransmission(I2C_ADDRESS); // start i2c cycle
  Wire.write(RESET_ADDRESS); // reset the device
  Wire.endTransmission(); // ends i2c cycle

  //wait a tad for reboot
  delay(1);

  writeRegister(REGISTER_EXC_SETUP, _BV(3) | _BV(1) | _BV(0)); // EXC source a register in data sheet, use this to set bits for changing excitation modes

 // writeRegister(REGISTER_CAP_DAC_A, 0x00);

  writeRegister(REGISTER_CAP_SETUP, _BV(7)); // cap setup reg - cap enabled

 ;

 // writeRegister(0x0A, _BV(7) | _BV(6) | _BV(5) | _BV(4) | _BV(3) | _BV(2) | _BV(0));  // set configuration to calib. mode, slow sample

  //wait for calibration
  delay(10);

  displayStatus();
 // Serial.print("Calibrated offset: ");

  //Serial.println(offset);

  writeRegister(REGISTER_CAP_SETUP, _BV(7)); // cap setup reg - cap enabled

  //writeRegister(REGISTER_EXC_SETUP, _BV(3)); // EXC source A



  //////////////////////////////////////////Final setup of CDC operation modes, see pg18 of datasheet///////////////////////////////////////////////////////////////
  writeRegister(REGISTER_CONFIGURATION, B00000001); // continuous mode       Changed to quickest speed



  displayStatus();
  //calibrate();

 // Serial.println("done\n\n");
 
}


void loop() // main program begins
{
   
 


      //Read in initial baseline levels (no averaging done for first set of baseline values)
   
      if( calibration_run==true)               
      {
        while(LED_POSITION<=15)           
        {
            mux();
            delay(10);
          BASELINE[LED_POSITION]=readValue();
          LED_POSITION++;

    
        }
                //Printing obtained baseline data to com port for the GUI
                for(int i=0; i<16;i++)
               {
                 start_time=millis();
                 
                 Serial.print("BASELINE ");
                 Serial.print("(");
                 
                 address_bit0=bitRead(address[i],0);
                 address_bit1=bitRead(address[i],1);
                 address_bit2=bitRead(address[i],2);
                 address_bit3=bitRead(address[i],3);
              
                 
                 Serial.print(address_bit3,BIN);
                 Serial.print(address_bit2,BIN);
                 Serial.print(address_bit1,BIN);
                 Serial.print(address_bit0,BIN);
               
                 Serial.print(")");
                 converted=BASELINE[i];
                 converted=  ((converted-8388608)/8388607)*4.096;         // 0pF is at 8388608   // -4.096 is 0  //4.096 is 16777215 (0xFFFFFF)
                 Serial.println(converted,4);

                 elapsed=-(start_time-millis());
                 Serial.println(elapsed);
               }
               
        LED_POSITION=0;
        calibration_run=false;        //Initial calibration over, use averaging data from now on
      
      }

      //Once all 16 positions read 

      if (LED_POSITION > 15)
      {     
             
          average_counter++;
             // average_counter>=4 ensures that 4 sets of data have been taken and averaged together
             if(average_counter>=4  && touch_flag==false)
             {
               average_counter=0;
               
               for(int i=0; i<16;i++)
               {

                 //Printing averaged baseline data
                 Serial.print("BASELINE ");
                 Serial.print("(");
                 address_bit0=bitRead(address[i],0);
                 address_bit1=bitRead(address[i],1);
                 address_bit2=bitRead(address[i],2);
                 address_bit3=bitRead(address[i],3);
              
                 
                 Serial.print(address_bit3,BIN);
                 Serial.print(address_bit2,BIN);
                 Serial.print(address_bit1,BIN);
                 Serial.print(address_bit0,BIN);
               
                 Serial.print(")");
                 converted=BASELINE[i];
                 converted=  ((converted-8388608)/8388607)*4.096;         // 0pF is at 8388608   // -4.096 is 0  //4.096 is 16777215 (0xFFFFFF)
                 Serial.println(converted,4);
               }
               
             
             }

          
              LED_POSITION = 0;
           touch_flag=false;
           
        
      }



  mux();  //cycle muxs to next input

  
  value = readValue();   //Read capacitance.



    //These if statements ensure that averages are not taken during a touch or hover.  hover_thresh and touch_thresh constants must be tuned to the specific sensor
    //First set hover_thresh so that in the serial monitor baseline values are constantly printed.  Then increase hover_thresh until a hover stops the printing of baselines
    if (value<BASELINE[LED_POSITION]*hover_thresh)
    {
      touch_flag=true;
    }
   
    else if (value>BASELINE[LED_POSITION]*touch_thresh)
    {
      touch_flag=true;
    }

   

  
  //Store averaging data in 3 arrays
  if(average_counter==0 && touch_flag==false)
  {
    average1[LED_POSITION] =value;
  }

  if(average_counter==1 && touch_flag==false)
  {
    average2[LED_POSITION] =value;
  }

  if(average_counter==2 && touch_flag==false)
  {
    average3[LED_POSITION] =value;
  }

  //Average current value and 3 previous values together for a single cell
  if(average_counter==3 && touch_flag==false)
  {
    BASELINE[LED_POSITION]=value/4 +average1[LED_POSITION]/4 +average2[LED_POSITION]/4 +average3[LED_POSITION]/4;
    
  }




 
  LED_POSITION++;



  //Convert data into a pF value
  
  converted=  value;
  converted=  ((converted/16777215)*8.192)-4.096;         // 0pF is at 8388608   // -4.096 is 0  //4.096 is 16777215 (0xFFFFFF)
  
  Serial.println(converted,4);
 

 
  
  
  delay(15);    //need a delay here or data will be transmitted out of order (or not at all)





}







long readValue() {
  long ret = 0;
  uint8_t data[3];

  char status = 0;
  //wait until a conversion is done
  while (!(status & (_BV(0) | _BV(2)))) {
    //wait for the next conversion
    status = readRegister(REGISTER_STATUS);
  }

  unsigned long value =  readLong(REGISTER_CAP_DATA);

  value >>= 8;
  //we have read one byte to much, now we have to get rid of it
  ret =  value;

  return ret;
}







/* Change the state of the multiplexers.
 * Cycles automatically
 * 
 * 
 * 
*/

void mux() {



  if (state_mux2 > B11)    
  {
    state_mux2 = 0;
    state_mux1+=B1;
  }


  if (state_mux1 > B11)
  {
    state_mux1 = 0;
  }



 if( calibration_run==false)         //only print non calibration data
 {

  Serial.print("(");
  Serial.print(mux1_A1, BIN);
  Serial.print(mux1_A0, BIN);
  //Serial.println("mux2");
  Serial.print(mux2_A1, BIN);
  Serial.print(mux2_A0, BIN);
  Serial.print(")");
  
  
 }
 
  
  mux1_A0 = bitRead(state_mux1, 0);
  mux1_A1 = bitRead(state_mux1, 1);
  mux2_A0 = bitRead(state_mux2, 0);
  mux2_A1 = bitRead(state_mux2, 1);


  
  digitalWrite(52, mux1_A0);
  digitalWrite(53, mux1_A1);
  digitalWrite(50, mux2_A0);
  digitalWrite(51, mux2_A1);


  state_mux2+=B1;


}




