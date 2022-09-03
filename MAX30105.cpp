/***************************************************
  This is a library written for the Maxim MAX30105 Optical Smoke Detector
  It should also work with the MAX30102. However, the MAX30102 does not have a Green LED.

  These sensors use I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  Written by Peter Jansen and Nathan Seidle (SparkFun)
  BSD license, all text above must be included in any redistribution.
 *****************************************************/

#include "MAX30105.h"

#ifndef __MICROBIT_H_
#define __MICROBIT_H_
#include "MicroBit.h"
#endif

//static MicroBitI2C i2c(I2C_SDA0, I2C_SCL0);

int16_t placeholder;

static const char MAX30105_ADDRESS = 0xAE;


// Status Registers
static const char MAX30105_INTSTAT1 =		0x00;
static const char MAX30105_INTSTAT2 =		0x01;
static const char MAX30105_INTENABLE1 =		0x02;
static const char MAX30105_INTENABLE2 =		0x03;

// FIFO Registers
static const char MAX30105_FIFOWRITEPTR = 	0x04;
static const char MAX30105_FIFOOVERFLOW = 	0x05;
static const char MAX30105_FIFOREADPTR = 	0x06;
static const char MAX30105_FIFODATA =		0x07;

// Configuration Registers
static const char MAX30105_FIFOCONFIG = 		0x08;
static const char MAX30105_MODECONFIG = 		0x09;
static const char MAX30105_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const char MAX30105_LED1_PULSEAMP = 	0x0C;
static const char MAX30105_LED2_PULSEAMP = 	0x0D;
static const char MAX30105_MULTILEDCONFIG1 = 0x11;
static const char MAX30105_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const char MAX30105_DIETEMPINT = 		0x1F;
static const char MAX30105_DIETEMPFRAC = 	0x20;
static const char MAX30105_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const char MAX30105_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const char MAX30105_REVISIONID = 		0xFE;
static const char MAX30105_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30105 Commands
// Interrupt configuration (pg 13, 14)
static const char MAX30105_INT_A_FULL_MASK =		(char)~0b10000000;
static const char MAX30105_INT_A_FULL_ENABLE = 	0x80;
static const char MAX30105_INT_A_FULL_DISABLE = 	0x00;

static const char MAX30105_INT_DATA_RDY_MASK = (char)~0b01000000;
static const char MAX30105_INT_DATA_RDY_ENABLE =	0x40;
static const char MAX30105_INT_DATA_RDY_DISABLE = 0x00;

static const char MAX30105_INT_ALC_OVF_MASK = (char)~0b00100000;
static const char MAX30105_INT_ALC_OVF_ENABLE = 	0x20;
static const char MAX30105_INT_ALC_OVF_DISABLE = 0x00;

static const char MAX30105_INT_PROX_INT_MASK = (char)~0b00010000;
static const char MAX30105_INT_PROX_INT_ENABLE = 0x10;
static const char MAX30105_INT_PROX_INT_DISABLE = 0x00;

static const char MAX30105_INT_DIE_TEMP_RDY_MASK = (char)~0b00000010;
static const char MAX30105_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const char MAX30105_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const char MAX30105_SAMPLEAVG_MASK =	(char)~0b11100000;
static const char MAX30105_SAMPLEAVG_1 = 	0x00;
static const char MAX30105_SAMPLEAVG_2 = 	0x20;
static const char MAX30105_SAMPLEAVG_4 = 	0x40;
static const char MAX30105_SAMPLEAVG_8 = 	0x60;
static const char MAX30105_SAMPLEAVG_16 = 	0x80;
static const char MAX30105_SAMPLEAVG_32 = 	0xA0;

static const char MAX30105_ROLLOVER_MASK = 	0xEF;
static const char MAX30105_ROLLOVER_ENABLE = 0x10;
static const char MAX30105_ROLLOVER_DISABLE = 0x00;

static const char MAX30105_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const char MAX30105_SHUTDOWN_MASK = 	0x7F;
static const char MAX30105_SHUTDOWN = 		0x80;
static const char MAX30105_WAKEUP = 			0x00;

static const char MAX30105_RESET_MASK = 		0xBF;
static const char MAX30105_RESET = 			0x40;

static const char MAX30105_MODE_MASK = 		0xF8;
static const char MAX30105_MODE_REDONLY = 	0x02;
static const char MAX30105_MODE_REDIRONLY = 	0x03;

// Particle sensing configuration commands (pgs 19-20)
static const char MAX30105_ADCRANGE_MASK = 	0x9F;
static const char MAX30105_ADCRANGE_2048 = 	0x00;
static const char MAX30105_ADCRANGE_4096 = 	0x20;
static const char MAX30105_ADCRANGE_8192 = 	0x40;
static const char MAX30105_ADCRANGE_16384 = 	0x60;

static const char MAX30105_SAMPLERATE_MASK = 0xE3;
static const char MAX30105_SAMPLERATE_50 = 	0x00;
static const char MAX30105_SAMPLERATE_100 = 	0x04;
static const char MAX30105_SAMPLERATE_200 = 	0x08;
static const char MAX30105_SAMPLERATE_400 = 	0x0C;
static const char MAX30105_SAMPLERATE_800 = 	0x10;
static const char MAX30105_SAMPLERATE_1000 = 0x14;
static const char MAX30105_SAMPLERATE_1600 = 0x18;
static const char MAX30105_SAMPLERATE_3200 = 0x1C;

static const char MAX30105_PULSEWIDTH_MASK = 0xFC;
static const char MAX30105_PULSEWIDTH_69 = 	0x00;
static const char MAX30105_PULSEWIDTH_118 = 	0x01;
static const char MAX30105_PULSEWIDTH_215 = 	0x02;
static const char MAX30105_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const char MAX30105_SLOT1_MASK = 		0xF8;
static const char MAX30105_SLOT2_MASK = 		0x8F;
static const char MAX30105_SLOT3_MASK = 		0xF8;
static const char MAX30105_SLOT4_MASK = 		0x8F;

static const char SLOT_NONE = 				0x00;
static const char SLOT_RED_LED = 			0x01;
static const char SLOT_IR_LED = 				0x02;
static const char SLOT_GREEN_LED = 			0x03;
static const char SLOT_NONE_PILOT = 			0x04;
static const char SLOT_RED_PILOT =			0x05;
static const char SLOT_IR_PILOT = 			0x06;
static const char SLOT_GREEN_PILOT = 		0x07;

static const char MAX_30105_EXPECTEDPARTID = 0x15;

uint8_t activeDiodes = 3; //Gets set during setup. Allows check() to calculate how many bytes to read from FIFO

#define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro

int16_t IR_AC_Max = 20;
int16_t IR_AC_Min = -20;

int16_t IR_AC_Signal_Current = 0;
int16_t IR_AC_Signal_Previous;
int16_t IR_AC_Signal_min = 0;
int16_t IR_AC_Signal_max = 0;
int16_t IR_Average_Estimated;

int16_t positiveEdge = 0;
int16_t negativeEdge = 0;
int32_t ir_avg_reg = 0;

int16_t cbuf[32];
uint8_t offset = 0;

static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

typedef struct Record
{
  uint32_t red[STORAGE_SIZE];
  uint32_t IR[STORAGE_SIZE];
  uint32_t green[STORAGE_SIZE];
  uint8_t head;
  uint8_t tail;
} sense_struct; //This is our circular buffer of readings from the sensor

sense_struct sense;

MAX30105::MAX30105() {
  // Constructor
}

void MAX30105::beginParticle() {

  // Step 1: Initial Communication and Verification
  // Check that a MAX30105 is connected
  if (readPartID() != MAX_30105_EXPECTEDPARTID) {
    // Error -- Part ID read from MAX30105 does not match expected part ID.
    // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
  }

  // Populate revision ID
  readRevisionID();
  
}

//
// Configuration
//

//Begin Interrupt configuration
uint8_t MAX30105::getINT1(void) {
  return (readRegister8(MAX30105_ADDRESS, MAX30105_INTSTAT1));
}
uint8_t MAX30105::getINT2(void) {
  return (readRegister8(MAX30105_ADDRESS, MAX30105_INTSTAT2));
}

void MAX30105::enableAFULL(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_ENABLE);
}
void MAX30105::disableAFULL(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_A_FULL_MASK, MAX30105_INT_A_FULL_DISABLE);
}

void MAX30105::enableDATARDY(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_ENABLE);
}
void MAX30105::disableDATARDY(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_DATA_RDY_MASK, MAX30105_INT_DATA_RDY_DISABLE);
}

void MAX30105::enableALCOVF(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_ENABLE);
}
void MAX30105::disableALCOVF(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_ALC_OVF_MASK, MAX30105_INT_ALC_OVF_DISABLE);
}

void MAX30105::enablePROXINT(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_ENABLE);
}
void MAX30105::disablePROXINT(void) {
  bitMask(MAX30105_INTENABLE1, MAX30105_INT_PROX_INT_MASK, MAX30105_INT_PROX_INT_DISABLE);
}

void MAX30105::enableDIETEMPRDY(void) {
  bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30105::disableDIETEMPRDY(void) {
  bitMask(MAX30105_INTENABLE2, MAX30105_INT_DIE_TEMP_RDY_MASK, MAX30105_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30105::softReset(void) {
  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  unsigned long startTime = system_timer_current_time();
  while (system_timer_current_time() - startTime < 100)
  {
    uint8_t response = readRegister8(MAX30105_ADDRESS, MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0) break; //We're done!
    fiber_sleep(1); //Let's not over burden the I2C bus
  }
}

void MAX30105::shutDown(void) {
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void MAX30105::wakeUp(void) {
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void MAX30105::setLEDMode(uint8_t mode) {
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  activeDiodes = mode - 1;
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void MAX30105::setADCRange(uint8_t adcRange) {
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void MAX30105::setSampleRate(uint8_t sampleRate) {
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void MAX30105::setPulseWidth(uint8_t pulseWidth) {
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30105::setPulseAmplitudeRed(uint8_t amplitude) {
  writeRegister8(MAX30105_ADDRESS, MAX30105_LED1_PULSEAMP, amplitude);
}

void MAX30105::setPulseAmplitudeIR(uint8_t amplitude) {
  writeRegister8(MAX30105_ADDRESS, MAX30105_LED2_PULSEAMP, amplitude);
}

void MAX30105::setProximityThreshold(uint8_t threshMSB) {
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister8(MAX30105_ADDRESS, MAX30105_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30105::enableSlot(uint8_t slotNumber, uint8_t device) {

  switch (slotNumber) {
    case (1):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
      break;
    case (2):
      bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
      break;
    case (3):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
      break;
    case (4):
      bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
      break;
    default:
      //Shouldn't be here!
      break;
  }
}

//Clears all slot assignments
void MAX30105::disableSlots(void) {
  writeRegister8(MAX30105_ADDRESS, MAX30105_MULTILEDCONFIG1, 0);
  writeRegister8(MAX30105_ADDRESS, MAX30105_MULTILEDCONFIG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30105::setFIFOAverage(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30105::clearFIFO(void) {
  writeRegister8(MAX30105_ADDRESS, MAX30105_FIFOWRITEPTR, 0);
  writeRegister8(MAX30105_ADDRESS, MAX30105_FIFOOVERFLOW, 0);
  writeRegister8(MAX30105_ADDRESS, MAX30105_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX30105::enableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30105::disableFIFORollover(void) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30105::setFIFOAlmostFull(uint8_t numberOfSamples) {
  bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX30105::getWritePointer(void) {
  return (readRegister8(MAX30105_ADDRESS, MAX30105_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX30105::getReadPointer(void) {
  return (readRegister8(MAX30105_ADDRESS, MAX30105_FIFOREADPTR));
}


// Die Temperature
// Returns temp in C
float MAX30105::readTemperature() {
	
  //DIE_TEMP_RDY interrupt must be enabled
  //See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19
  
  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8(MAX30105_ADDRESS, MAX30105_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  unsigned long startTime = system_timer_current_time();
  while (system_timer_current_time() - startTime < 100)
  {
    //uint8_t response = readRegister8(MAX30105_ADDRESS, MAX30105_DIETEMPCONFIG); //Original way
    //if ((response & 0x01) == 0) break; //We're done!
    
	//Check to see if DIE_TEMP_RDY interrupt is set
	uint8_t response = readRegister8(MAX30105_ADDRESS, MAX30105_INTSTAT2);
    if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0) break; //We're done!
    fiber_sleep(1); //Let's not over burden the I2C bus
  }
  //TODO How do we want to fail? With what type of error?
  //? if(uBit.systemTime() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(MAX30105_ADDRESS, MAX30105_DIETEMPINT);
  uint8_t tempFrac = readRegister8(MAX30105_ADDRESS, MAX30105_DIETEMPFRAC); //Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30105::readTemperatureF() {
  float temp = readTemperature();

  if (temp != -999.0) temp = temp * 1.8 + 32.0;

  return (temp);
}

// Set the PROX_INT_THRESHold
void MAX30105::setPROXINTTHRESH(uint8_t val) {
  writeRegister8(MAX30105_ADDRESS, MAX30105_PROXINTTHRESH, val);
}


//
// Device ID and Revision
//
uint8_t MAX30105::readPartID() {
  return readRegister8(MAX30105_ADDRESS, MAX30105_PARTID);
}

void MAX30105::readRevisionID() {
  revisionID = readRegister8(MAX30105_ADDRESS, MAX30105_REVISIONID);
}

uint8_t MAX30105::getRevisionID() {
  return revisionID;
}


//Setup the sensor
//The MAX30105 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30105 sensor
void MAX30105::setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange) {
  softReset(); //Reset all configuration, threshold, and data registers to POR values
  //FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1) setFIFOAverage(MAX30105_SAMPLEAVG_1); //No averaging per FIFO record
  else if (sampleAverage == 2) setFIFOAverage(MAX30105_SAMPLEAVG_2);
  else if (sampleAverage == 4) setFIFOAverage(MAX30105_SAMPLEAVG_4);
  else if (sampleAverage == 8) setFIFOAverage(MAX30105_SAMPLEAVG_8);
  else if (sampleAverage == 16) setFIFOAverage(MAX30105_SAMPLEAVG_16);
  else if (sampleAverage == 32) setFIFOAverage(MAX30105_SAMPLEAVG_32);
  else setFIFOAverage(MAX30105_SAMPLEAVG_4);

  //setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); //Allow FIFO to wrap/roll over
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 2) setLEDMode(MAX30105_MODE_REDIRONLY); //Red and IR
  else setLEDMode(MAX30105_MODE_REDONLY); //Red only
  activeDiodes = ledMode; //Used to control how many uint8_ts to read from FIFO buffer
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if(adcRange < 4096) setADCRange(MAX30105_ADCRANGE_2048); //7.81pA per LSB
  else if(adcRange < 8192) setADCRange(MAX30105_ADCRANGE_4096); //15.63pA per LSB
  else if(adcRange < 16384) setADCRange(MAX30105_ADCRANGE_8192); //31.25pA per LSB
  else if(adcRange == 16384) setADCRange(MAX30105_ADCRANGE_16384); //62.5pA per LSB
  else setADCRange(MAX30105_ADCRANGE_2048);

  if (sampleRate < 100) setSampleRate(MAX30105_SAMPLERATE_50); //Take 50 samples per second
  else if (sampleRate < 200) setSampleRate(MAX30105_SAMPLERATE_100);
  else if (sampleRate < 400) setSampleRate(MAX30105_SAMPLERATE_200);
  else if (sampleRate < 800) setSampleRate(MAX30105_SAMPLERATE_400);
  else if (sampleRate < 1000) setSampleRate(MAX30105_SAMPLERATE_800);
  else if (sampleRate < 1600) setSampleRate(MAX30105_SAMPLERATE_1000);
  else if (sampleRate < 3200) setSampleRate(MAX30105_SAMPLERATE_1600);
  else if (sampleRate == 3200) setSampleRate(MAX30105_SAMPLERATE_3200);
  else setSampleRate(MAX30105_SAMPLERATE_50);

  //The longer the pulse width the longer range of detection you'll have
  //At 69us and 0.4mA it's about 2 inches
  //At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118) setPulseWidth(MAX30105_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215) setPulseWidth(MAX30105_PULSEWIDTH_118); //16 bit resolution
  else if (pulseWidth < 411) setPulseWidth(MAX30105_PULSEWIDTH_215); //17 bit resolution
  else if (pulseWidth == 411) setPulseWidth(MAX30105_PULSEWIDTH_411); //18 bit resolution
  else setPulseWidth(MAX30105_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  //Default is 0x1F which gets us 6.4mA
  //powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  //powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  //powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  //powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  //Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
  //enableSlot(1, SLOT_RED_PILOT);
  //enableSlot(2, SLOT_IR_PILOT);
  //enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); //Reset the FIFO before we begin checking the sensor
}
	
//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX30105::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}

//Report the most recent red value
uint32_t MAX30105::getRed(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250)){
    return sense.red[sense.head];
  }
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t MAX30105::getIR(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX30105::getGreen(void)
{
  //Check the sensor for new data for 250ms
  if(safeCheck(250))
    return (sense.green[sense.head]);
  else
    return(0); //Sensor failed to find new data
}

//Report the next Red value in the FIFO
uint32_t MAX30105::getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO
uint32_t MAX30105::getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX30105::getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

//Advance the tail
bool MAX30105::nextSample(void)
{
  if(available()) //Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; //Wrap condition
	return true;
  }
  else
  {
	return false;
  }
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
uint16_t MAX30105::check(void)
{
  //Read register FIDO_DATA in (3-uint8_t * number of active LED) chunks
  //Until FIFO_RD_PTR = FIFO_WR_PTR

  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();
  int numberOfSamples = 0;

  //Do we have new data?
  if (readPointer != writePointer)
  {
    //Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0) 
		numberOfSamples += I2C_BUFFER_LENGTH; //Wrap condition

    //We now have the number of readings, now calc uint8_ts to read
    //For this example we are just doing Red and IR (3 uint8_ts each)
    uint8_t bytesLeftToRead = numberOfSamples * activeDiodes * 3;

    //Get ready to read a burst of data from the FIFO register

    //We may need to read as many as 288 uint8_ts so we read in blocks no larger than I2C_BUFFER_LENGTH
    //I2C_BUFFER_LENGTH changes based on the platform. 64 uint8_ts for SAMD21, 32 uint8_ts for Uno.
    while (bytesLeftToRead > 0)
    {
		
      uint8_t toGet = activeDiodes * 3;

      //Request toGet number of uint8_ts from sensor
      //i2c.requestFrom(MAX30105_ADDRESS, toGet);
      while(toGet > 0)
	  {
		uint8_t temp[9]; //Array of 9 uint8_ts that we will convert into longs
		uint8_t temp2[4];
        uint32_t tempLong;
	
		uBit.i2c.readRegister(MAX30105_ADDRESS, (uint8_t)MAX30105_FIFODATA, temp, toGet);

        sense.head++; //Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; //Wrap condition
		for (int led = 0; led < activeDiodes; led++)
		{
			uint8_t checkOffset = led * 3;
			temp2[3] = 0;
			temp2[0] = temp[2 + checkOffset];
			temp2[1] = temp[1 + checkOffset];
			temp2[2] = temp[checkOffset];
			memcpy(&tempLong, temp2, sizeof(tempLong)); //tempLong is 4 bytes, we only need 3
			tempLong &= 0x3FFFF;
			switch (led)
			{
				case 0:
					sense.red[sense.head] = tempLong;//Long;//Store this reading into the sense array
					break;
				case 1:
					sense.IR[sense.head] = tempLong;
					break;
				case 2:
					sense.green[sense.head] = tempLong;
					break;
				default:
					break;
			}
		}
		bytesLeftToRead -= toGet;
		toGet -= activeDiodes * 3;
	  }
    } //End while (bytesLeftToRead > 0)
  } //End readPtr != writePtr
  return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool MAX30105::safeCheck(uint8_t maxTimeToCheck)
{
  uint64_t markTime = system_timer_current_time();
  
  while(1)
  {
	if(system_timer_current_time() - markTime > maxTimeToCheck){
		
		return(false);
	}

	if(check() == true)
	{ //We found new data!
	  return(true);
	}
	fiber_sleep(1);
  }
}

//Given a register, read it, mask it, and then set the thing
void MAX30105::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  // Grab current register context
  uint8_t originalContents = readRegister8(MAX30105_ADDRESS, reg);

  // Zero-out the portions of the register we're interested in
  originalContents = originalContents & mask;

  // Change contents
  writeRegister8(MAX30105_ADDRESS, reg, originalContents | thing);
}

//  Heart Rate Monitor functions takes a sample value and the sample number
//  Returns true if a beat is detected
//  A running average of four samples is recommended for display on the screen.
bool MAX30105::checkForBeat(uint32_t sample)
{
  bool beatDetected = false;
  
  //  Save current state
  IR_AC_Signal_Previous = IR_AC_Signal_Current;
  
  //This is good to view for debugging
  //uBit.serial.send("Signal_Current: ");
  //uBit.serial.send(IR_AC_Signal_Current);

  //  Process next data sample
  IR_Average_Estimated = averageDCEstimator(&ir_avg_reg, sample);
  IR_AC_Signal_Current = lowPassFIRFilter(sample - IR_Average_Estimated);

  //  Detect positive zero crossing (rising edge)
  if ((IR_AC_Signal_Previous < 0) && (IR_AC_Signal_Current >= 0))
  {
  
    IR_AC_Max = IR_AC_Signal_max; //Adjust our AC max and min
    IR_AC_Min = IR_AC_Signal_min;

    positiveEdge = 1;
    negativeEdge = 0;
    IR_AC_Signal_max = 0;

    //if ((IR_AC_Max - IR_AC_Min) > 100 & (IR_AC_Max - IR_AC_Min) < 1000)
    if (((IR_AC_Max - IR_AC_Min) > 20) && ((IR_AC_Max - IR_AC_Min) < 1000))
    {
	  placeholder++;
      //Heart beat!!!
      beatDetected = true;
    }
  }

  //  Detect negative zero crossing (falling edge)
  if ((IR_AC_Signal_Previous > 0) && (IR_AC_Signal_Current <= 0))
  {
    positiveEdge = 0;
    negativeEdge = 1;
    IR_AC_Signal_min = 0;
  }

  //  Find Maximum value in positive cycle
  if (positiveEdge && (IR_AC_Signal_Current > IR_AC_Signal_Previous))
  {
    IR_AC_Signal_max = IR_AC_Signal_Current;
  }

  //  Find Minimum value in negative cycle
  if (negativeEdge && (IR_AC_Signal_Current < IR_AC_Signal_Previous))
  {
    IR_AC_Signal_min = IR_AC_Signal_Current;
  }
  
  return(beatDetected);
}

int16_t MAX30105::getPlaceholder()
{
	return placeholder;
}

//  Average DC Estimator
int16_t MAX30105::averageDCEstimator(int32_t *p, uint16_t x)
{
  *p += ((((long) x << 15) - *p) >> 4);
  return (*p >> 15);
}

//  Low Pass FIR Filter
int16_t MAX30105::lowPassFIRFilter(int16_t din)
{  
  cbuf[offset] = din;

  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);
  
  for (uint8_t i = 0 ; i < 11 ; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  offset++;
  offset %= 32; //Wrap condition

  return(z >> 15);
}

//  Integer multiplier
int32_t MAX30105::mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}

//
// Low-level I2C Communication
//
uint8_t MAX30105::readRegister8(uint8_t address, uint8_t reg) {
	uint8_t data;
	uBit.i2c.readRegister(address, reg, &data, 1);
	return data;
}

void MAX30105::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {

#if MICROBIT_CODAL
	uint8_t temp[2];
#else 
	char temp[2];
#endif
	temp[0] = reg;
	memcpy(&temp[1], &value, 2); 

	uBit.i2c.write(address, temp, 2);

}
