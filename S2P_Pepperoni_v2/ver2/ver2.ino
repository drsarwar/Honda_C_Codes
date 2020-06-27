#include <Wire.h>

#define I2C_ADDRESS  0x48 //0x90 shift one to the right
#define REGISTER_STATUS 0x00
#define REGISTER_CAP_DATA 0x01
#define REGISTER_CAP_SETUP 0x07
#define REGISTER_EXC_SETUP 0x09
#define REGISTER_CONFIGURATION 0x0A
#define RESET_ADDRESS 0xBF

//FDC constant
#define FDC_I2C_ADDRESS 0x50
#define FDC_RESET_ADDRESS 0x80

#define REG_MEAS1_MSB 0x00
#define REG_MEAS1_LSB 0x01
#define REG_CONF_MEAS1 0x08
#define REG_FDC_CONF 0x0C
#define REG_OFFSET1 0x0D

#define VALUE_UPPER_BOUND 16000000L
#define VALUE_LOWER_BOUND 0xFL
#define MAX_OUT_OF_RANGE_COUNT 3
#define CALIBRATION_INCREASE 1

#define touch_thresh 2.5 //Set the threshold for touch detection here. Capacitance must be > touch_thresh * baseline value to trigger a touch
#define hover_thresh 0.65 //Set the threshold for proximty detection here. Capacitance must be < hover_thresh * baseline value to trigger a touch

#define mux_1addr_3 12
#define mux_1addr_2 11
#define mux_1addr_1 10
#define mux_1addr_0 9
#define mux_2addr_0 8
#define PIN_COVER_MUX 7

long value = 0; //Unconverted value from CDC

float converted = 0; //Converted pF value

int mux1_A0; //1 bit value to send to mux control pin
int mux1_A1;
int mux1_A2;
int mux1_A3;
int mux2_A0;

int state_mux1 = 1; // 2 bit state of mux1

//Baseline address bits
unsigned char address_bit0 = 0;
unsigned char address_bit1 = 0;
unsigned char address_bit2 = 0;
unsigned char address_bit3 = 0;

long BASELINE [4]; //Baseline capacitance of each position stored here, initially a set of the first values read at reset, 
                   //then becomes a running average of 4 sets of data
                   
unsigned char current_sensor = 0; //Used as a counter to cycle through the 12 sensors.
//unsigned char address[4] = {B0000, B0001, B0010, B0011}; //Makes it easier to print baseline addresses TODO: what is this?

unsigned char SPDTAddressRegA[16] = {B00000000, B00000000, B00000010, B00000000, \
									 B00000000, B00000000, B00000000, B00000000, \
									 B00000000, B00000000, B00000000, B00000001, \
									 B00000100, B00001000, B00000000, B00000000}; //SPDT select mapping
unsigned char SPDTAddressRegC[16] = {B00001000, B01000000, B00000000, B00000001, \
									 B00000000, B00000000, B00000000, B00000000, \
									 B00010000, B00100000, B10000000, B00000000, \
									 B00000000, B00000000, B00000010, B00000100}; //SPDT select mapping RegC is from 37 (low) to 30 (high)

unsigned char state_mux1Mapping[16] = {12, 9, 1, 15, 6, 6, 6, 6, 11, 10, 8, 0, 2, 3, 14, 13};
//unsigned char current_sensorMapping[12] = {13, 10, 2, 16, 12, 11, 9, 1, 3, 4, 15, 14};

//Rolling Average Values
int window_size = 10;
float sum = 0.0;
float avg = 0.0;
float last_n_vals [4] [10];
int avg_index = 0;

void setup()
{
	pinMode(mux_1addr_0, OUTPUT);
	pinMode(mux_1addr_1, OUTPUT);
	pinMode(mux_1addr_2, OUTPUT);
	pinMode(mux_1addr_3, OUTPUT);
	pinMode(mux_2addr_0, OUTPUT);
	pinMode(PIN_COVER_MUX, OUTPUT);
	DDRA = B11111111;  // sets Arduino pins 22 to 29 as outputs
	DDRC = B11111111;  // sets Arduino pins 30 to 37 as outputs

	//Set up i2c for operation  
	Wire.begin();

	//Set up baud rate for serial communication to com port
	Serial.begin(9600);

	delay(1);
	//config_fdc
	config_fdc();

	//config_cdc:
	//Start i2c cycle
	Wire.beginTransmission(I2C_ADDRESS);
	//Reset the device
	Wire.write(RESET_ADDRESS);
	//End i2c cycle
	Wire.endTransmission();
	//Wait a tad for reboot
	delay(1);

	writeRegister(I2C_ADDRESS, REGISTER_EXC_SETUP, _BV(3) | _BV(1) | _BV(0));
	writeRegister(I2C_ADDRESS, REGISTER_CAP_SETUP, _BV(7)); //Cap setup reg - cap enabled
	delay(10);
	writeRegister(I2C_ADDRESS, REGISTER_CONFIGURATION, _BV(0)); //Continuous mode - changed to quickest speed


	//Prevent overshoot of the first BASELINE value
    delay(200);
 
}

//Main Program
void loop()
{
	current_sensor = state_mux1-1;
	if(current_sensor > 7)
		current_sensor = current_sensor-4;


	//Cycle mux to next input
	mux();


	//Read capacitance.
	value = readValue();

	//Convert data into a pF value
	//0pF is at 8388608
	//-4.096 is 0
	//4.096 is 16777215 (0xFFFFFF)
	converted = value;
	converted = ((converted / 16777215) * 8.192) - 4.096;

	   
	   	Serial.print("(");
	    if (current_sensor<1)
	      Serial.print("000");
	    else if (current_sensor<2)
	      Serial.print("000");
	    else if (current_sensor<4)
	      Serial.print("00");
	    else if (current_sensor<8)
	    Serial.print("0");
	    Serial.print(current_sensor,BIN);
	    Serial.print(")");
		
	  	Serial.print(converted, 4);
		//  	Serial.print(current_sensor+1);
		//  	Serial.print("  ");
		//  	Serial.print(state_mux1-1);
	  	Serial.print("\n");

	  	if(current_sensor == 11) {
	  		writeRegister(I2C_ADDRESS, REGISTER_CAP_SETUP, 0); //this is where the CDC is turned off
	  		digitalWrite(PIN_COVER_MUX, LOW);
	  		delay(10);

	  		Serial.print("(1100)");
			Serial.println(FDC_readValue(), 4);
			
			digitalWrite(PIN_COVER_MUX, HIGH);
			writeRegister(I2C_ADDRESS, REGISTER_CAP_SETUP, _BV(7));
		}

	delay(15);  //Need a delay here or data will be transmitted out of order (or not at all)
	//current_sensor++;   //Increment to the next sensor
}

//This function checks the sensor's status reg until the data is ready and then reads it in
long readValue()
{
	long ret = 0;
	uint8_t data[3];
	char status = 0;
  
	//Wait until a conversion is done
	while (!(status & (_BV(0) | _BV(2))))
	{
	//Wait for the next conversion
	status = readRegister(I2C_ADDRESS,REGISTER_STATUS);
	}

	//Size of unsigned long : 4 bytes (32bits)
	unsigned long value =  readLong(I2C_ADDRESS,REGISTER_CAP_DATA);

	//We have read one byte too many - now we have to get rid of it - sensor is 24-bit
	value >>= 8;
	ret = value;
	return ret;
}

//Nima's MUX
void mux()
{
  		//state_mux1=16;
	mux1_A0 = bitRead(state_mux1Mapping[state_mux1-1], 0);
	mux1_A1 = bitRead(state_mux1Mapping[state_mux1-1], 1);
	mux1_A2 = bitRead(state_mux1Mapping[state_mux1-1], 2);
	mux1_A3 = bitRead(state_mux1Mapping[state_mux1-1], 3);

	/*********
	 * Channels S5-S8 are not used in the *
	 * MUX Hence, they will be skipped    *        *
	*******/

	if (state_mux1Mapping[state_mux1-1] < 4 || state_mux1Mapping[state_mux1-1] > 7 ){
  
	    digitalWrite(mux_1addr_0, mux1_A0);
	    digitalWrite(mux_1addr_1, mux1_A1);
	    digitalWrite(mux_1addr_2, mux1_A2);
	    digitalWrite(mux_1addr_3, mux1_A3);

 
    /***************************
    * Shear Pads are connected to S2,S10,S13 and S16 in the MUX.    *
    * Based on the datasheet (ADG1606) the controlling inputs for   * 
    * these channels are checked here. If the channel in the MUX is *
    * for shear the SPDT switches to the channel 0. Otherwise it is *
    * connected to channel 1.
    *****************************/
    
		if(state_mux1 <= 4){
		  	mux2_A0 = 1;
		}
		else {
		  	mux2_A0 = 0;
		}
        //digitalWrite(mux_2addr_0, 1);
  		digitalWrite(mux_2addr_0, mux2_A0);
 	}

    /***************************
    * select specific SPDT
    ****************************/

  	PORTA = SPDTAddressRegA[state_mux1-1];
   	PORTC = SPDTAddressRegC[state_mux1-1];

    //PORTA = B11111111;
    //PORTC = B11111111;

	if (state_mux1 < 16)
    {
      	state_mux1++;
    }
    else{
      	state_mux1 = 1;
    }
    
    if(state_mux1 == 5){
    	state_mux1 = 9; 
    }

    //state_mux1=1;
}

void config_fdc()
{
	Wire.begin();																// Set up I2C for operation
	Serial.begin(9600);															// Set up baud rate for serial communication to com port
	writeRegister_Word(FDC_I2C_ADDRESS, REG_FDC_CONF, FDC_RESET_ADDRESS, 0x00); // Reset device
	delay(100);																	// Wait a tad for reboot

	/* Measurement Configuration */
	// +C input: CIN1 -- enable CAPDAC --  0pF offset
	// if offset = 6, c = 0;  if offset = 10, c approx. = -16
	/// Offset range: 0 <= offset < 32
	uint8_t offset_c = 6; 
	// Measurement configuration registers: REG_CONF_MEAS1 = 0x1000
	// [15:13] = 0b000
	//		select positive input as channel 1
	// [12:10] = 0b100
	//		select negative input as CAPDAC
	// [9:5]   = 0b00000
	// 		reserve for CAPDAC setting; 1|0 = 1; 0|0 = 0;
	// [4:0]   = 0b00000
	// 		reserved, always 0 and read only
	uint16_t conf = 0x1000;
	// "(offset_c & 0x1F)" ensures it has only lower 5 bits, other bits are all 0
	// " << 5" moves the offset_c value to its right place
	// " |= " merges the offset value to the configuration
	conf |= (offset_c & 0x1F) << 5;
	writeRegister_Word(FDC_I2C_ADDRESS, REG_CONF_MEAS1, (conf >> 8) & 0xFF, conf & 0xFF);
	delay(20);

	/* Trigger Configuration */
	// Normal operation -- msr rate: 400 S/s -- repeat enabled -- msr 1 enabled
	writeRegister_Word(FDC_I2C_ADDRESS, REG_FDC_CONF, 0x0D, 0x80);
	delay(20);

	/* Offset Configuration */
	// Integer part is from bit 11 to 15
	// 0x20 means offset +4pF
	// 0xF7 means offset -2pF
	// 0xB8 means offset -9pF
	// 0x98 means offset -13pF
	writeRegister_Word(FDC_I2C_ADDRESS, REG_OFFSET1, 0x98, 0x00);
	delay(20);
}

float FDC_readValue()
{
	digitalWrite(PIN_COVER_MUX, LOW);
	delay(15);

	unsigned int status = 0;
	long raw_data;

	/* Wait for Measurement Completion */
	// Data ready when bit[3] of REG_FDC_CONF = 1
	while (!(0x8 == (status & 0x8)))
	{
		status = readRegister(FDC_I2C_ADDRESS, REG_FDC_CONF); // Wait for the next conversion
		delay(10);
	}

	raw_data = readLong(FDC_I2C_ADDRESS, REG_MEAS1_MSB); // Size of unsigned long: 4 bytes (32 bits)
	raw_data >>= 8;										 // Have 1 byte too many - have to get rid of it - data is 24 bit

	digitalWrite(PIN_COVER_MUX, HIGH); //set the mux open via channel 4
	delay(15);

	return (float)raw_data / 524288;
}
