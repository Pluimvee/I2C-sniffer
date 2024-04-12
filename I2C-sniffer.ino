/**
 * @AUTHOR Erik Veer
 * Continue the work of Whitehawk Tailer https://github.com/WhitehawkTailor/I2C-sniffer
 * Some cleaning up from global to local variables
 * resolved the false starts issue
 * Stabilized detection of START/STOP
 *
 * @AUTHOR Ákos Szabó (Whitehawk Tailor) - aaszabo@gmail.com
 * This is an I2C sniffer that logs traffic on I2C BUS.
 * 
 * It is not part of the I2C BUS. It is neither a Master, nor a Slave and puts no data to the lines.
 * It just listens and logs the communication.
 * 
 * Two pins as imput are attached to SDC and SDA lines.
 * Since the I2C communications runs on 400kHz so,
 * the tool that runs this program should be fast.
 * This was tested on an ESP32 bord Heltec WiFi Lora32 v2
 * ESP32 core runs on 240MHz.
 * It means there are 600 ESP32 cycles during one I2C clock tick.
 *
 * The program uses interrupts to detect
 * the raise edge of the SCL - bit transfer 
 * the falling edge of SDA if SCL is HIGH- START
 * the raise edge of SDA if SCL is HIGH - STOP 
 * 
 * In the interrupt routines there is just a few line of code
 * that mainly sets the status and stores the incoming bits.
 * Otherwise the program gets timeout panic in interrupt handler and
 * restart the CPU.
 * 
 */
#include <Arduino.h>

#define PIN_SDA D2 // GPIO4
#define PIN_SCL D1 // GPIO5

static volatile byte dataBuffer[9600];//Array for storing data of the I2C communication
static volatile uint16_t bufferPoiW=0;//points to the first empty position in the dataBufer to write
static uint16_t bufferPoiR=0;//points to the position where to start read from
static volatile uint16_t byteCount =0;//counter of bytes were writen in one communication.

//static volatile byte i2c_reg[256];
static volatile bool i2cIdle = true; // Status of the I2C BUS is idle
static volatile byte bitCount = 0;//counter of bit appeared on the BUS

////////////////////////////
//// Interrupt handlers
/////////////////////////////
/**
 * Rising SCL makes reading the SDA
 */
void IRAM_ATTR i2cTriggerOnRaisingSCL() 
{
  if (i2cIdle)
    return;

	//get the value from SDA
	int sda = digitalRead(PIN_SDA); // SDA is stable on rising SCL

	//decide wherewe are and what to do with incoming data
	int i2cCase = 0;    // data bit

	if (bitCount == 8)  //we are already at 8 bits, so this is the (N)ACK bit
		i2cCase = 1;

	if (bitCount == 7 && byteCount == 0 ) // first byte is 7bits address, 8th bit is R/W
		i2cCase = 2;

	bitCount++;

	switch (i2cCase)
	{
		case 0: // data bit
		  dataBuffer[bufferPoiW++] = '0' + sda;
	  	break;//end of case 0 general

		case 1: //(N)ACK
			if (sda > 0) // SDA HIGH -> NACK
        dataBuffer[bufferPoiW++] = '-';
      else        // SDA LOW ->  ACK
        dataBuffer[bufferPoiW++] = '+';
			byteCount++;
			bitCount=0;
  		break;//end of case 1 ACK

		case 2:
			if (sda > 0)
				dataBuffer[bufferPoiW++] = 'R';
			else
				dataBuffer[bufferPoiW++] = 'W';
  		break;
	}//end of switch

}

/*
 * This is for recognizing I2C START and STOP
 * This is called when the SDA line is changing
 * It is decided inside the function wheather it is a rising or falling change.
 * If SCL is on High then the falling change is a START and the rising is a STOP.
 * If SCL is LOW, then this is the action to set a data bit, so nothing to do.
 */
void IRAM_ATTR i2cTriggerOnChangeSDA()
{
  // SCL is table on rising SDA -> its NOT on Falling SDA !!!
	if (digitalRead(PIN_SDA) > 0) //  if SDA is HIGH (1) -> RISING -> STOP 
	{
    if (digitalRead(PIN_SCL) == 0)  // if SCL is low we are still communicating
      return;

    i2cIdle = true;
    bufferPoiW--;   // remove the last bit which we stored while running, as it is a pre-stop
    dataBuffer[bufferPoiW++] = 's';  // stop
    dataBuffer[bufferPoiW++] = '\n'; // new line
	}
	else // if SDA is LOW -> FALLING -> START?
	{
		if (i2cIdle) //If we are idle than this is a START
		{
			i2cIdle = false;
			bitCount = 0;
			byteCount =0;
			dataBuffer[bufferPoiW++] = 'S';// START
		}
	}
}

////////////////////////////////
//// Functions
////////////////////////////////
/**
 * Reset all important variable
 */
void resetI2cVariable()
{
	i2cIdle = true;
	bufferPoiW=0;
	bufferPoiR=0;
}

/**
* @DESC Write out the buffer to the serial console
*
*/
void processDataBuffer()
{
	if (bufferPoiW == bufferPoiR)//There is nothing to say
		return;

	uint16_t pw = bufferPoiW;
	for(int i=bufferPoiR; i< pw; i++)
	{
		Serial.write(dataBuffer[i]);
		bufferPoiR++;		
	}
	//if there is no I2C action in progress and there wasn't during the Serial.print then buffer was printed out completly and can be reset.
	if (i2cIdle && pw==bufferPoiW)
	{
		bufferPoiW =0;
		bufferPoiR =0;
	}	
}

/////////////////////////////////
////  MAIN entry point of the program
/////////////////////////////////
void setup() 
{
	//Define pins for SCL, SDA
  pinMode(PIN_SCL, INPUT_PULLUP);   
  pinMode(PIN_SDA, INPUT_PULLUP);

  //reset variables
  resetI2cVariable();

  //Atach interrupt handlers to the interrupts on GPIOs
  attachInterrupt(PIN_SCL, i2cTriggerOnRaisingSCL, RISING); //trigger for reading data from SDA
  attachInterrupt(PIN_SDA, i2cTriggerOnChangeSDA,  CHANGE); //for I2C START and STOP

	Serial.begin(115200);
}//END of setup

/**
 * LOOP
 */
void loop() 
{
  //if it is in IDLE, then write out the databuffer to the serial consol
  if (i2cIdle)
	{
		processDataBuffer();
    Serial.print("Start buffering.....");		
    delay(5000);
    Serial.print("....dump\n");
		delay(500);
	}
}//END of loop
