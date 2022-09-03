/**
* Andy England @ SparkFun Electronics
* June 4, 2019
* https://github.com/sparkfun/pxt-gator-particle
*
* Development environment specifics:
* Written in Microsoft PXT
* Tested with a SparkFun Gator:Particle Sensor
*
* This code is released under the [MIT License](http://opensource.org/licenses/MIT).
* Please review the LICENSE.md file included with this example. If you have any questions
* or concerns with licensing, please contact techsupport@sparkfun.com.
* Distributed as-is; no warranty is given.
*/


#include "pxt.h"
#include <cstdint>
#include <math.h>
#include "MAX30105.h"

enum LEDToRead{
	//% block="Red" enumval=1
	Red=1,	
	//% block="Infrared" enumval=2
	Infrared=2,
};
 
enum HeartbeatType{
	//% block="BPM" enumval=0
	BPM=0,
	//% block="AVG" enumval=1
	AVG=1,
};

enum LEDMode{
	//% block="Red" enumval=2
	RedLED=2,
	//% block="Red & Infrared" enumval=3
	RedAndIR=3,
};

using namespace pxt;

namespace Microbit {
	MAX30105 *particleSensor;
	const uint8_t RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
	uint8_t rates[RATE_SIZE]; //Array of heart rates
	uint8_t rateSpot = 0;
	unsigned long lastBeat = 0; //Time at which the last beat occurred
	
	float beatsPerMinute;
	int beatAvg;

	//%
	void begin()
	{
		particleSensor->beginParticle();
		particleSensor->setup(0x1F, 4, 2, 400, 411, 4096);
	}
	
	//%
	uint32_t color(uint8_t type)
	{
		uint32_t colorValue;
		switch(type)
		{
			case 1:
				colorValue = particleSensor->getRed();
				break;
			case 2:
				colorValue = particleSensor->getIR();
				break;
		}
		return colorValue;
	}

	//%
	void setReadMode(uint8_t mode)
	{
		particleSensor->setLEDMode(mode);
	}

	//%
	void setAmplitude(uint8_t led, uint8_t myBrightness)
	{
		switch(led)
		{
			case 1:
				particleSensor->setPulseAmplitudeRed(myBrightness);
				break;
			case 2:
				particleSensor->setPulseAmplitudeIR(myBrightness);
				break;
		}
	}

	//%
	int16_t heartbeat(uint8_t type)
	{
		uint8_t myBeat;
		particleSensor->safeCheck(100);
		do
		{
			uint32_t irValue = particleSensor->getFIFOIR();
			if (particleSensor->checkForBeat(irValue) == true)
			{
				//We sensed a beat!
				unsigned long delta = uBit.systemTime() - lastBeat;
				lastBeat = uBit.systemTime();

				beatsPerMinute = 60 / (delta / 1000.0);

				if (beatsPerMinute < 255 && beatsPerMinute > 20)
				{
					rates[rateSpot++] = (uint8_t)beatsPerMinute; //Store this reading in the array
					rateSpot %= RATE_SIZE; //Wrap variable

					//Take average of readings
					beatAvg = 0;
					for (uint8_t x = 0 ; x < RATE_SIZE ; x++){
						beatAvg += rates[x];
					}
					beatAvg /= RATE_SIZE;
				}
			}
		} while(particleSensor->nextSample());
		switch(type)
		{
			case 0:
				myBeat = (uint8_t)beatsPerMinute;
				break;
				
			case 1:
				myBeat = (uint8_t)beatAvg;
				break;
				
		}
		return myBeat;
	}
}
