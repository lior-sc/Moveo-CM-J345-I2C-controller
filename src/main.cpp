#include <Arduino.h>
#include <Wire.h>
#include "TrapezStepper.h"

#define DEBUG			      	false

// Pin definitions
#define J3_EN_PIN		    	2
#define J3_DIR_PIN	    		3
#define J3_STEPPIN		    	4

#define J4_EN_PIN		    	5
#define J4_DIR_PIN        		6
#define J4_STEPPIN        		7

#define J5_EN_PIN		    	8
#define J5_DIR_PIN        		9
#define J5_STEPPIN        		10


// motor parameters
#define J3_StepPerRev      		400
#define J3_ReductionRatio		26.25

#define J4_StepPerRev      		1600
#define J4_ReductionRatio		1

#define J5_StepPerRev      		400
#define J5_ReductionRatio		4.4

// motion parameters
#define J3_MAX_VEL				6.28	/// [rad/s]
#define J3ACC					2.25		/// [rad^2/s]

#define J4_MAX_VEL				6.28	/// [rad/s]
#define J4ACC					2.25		/// [rad^2/s]

#define J5_MAX_VEL			    2.25		/// [rad/s]
#define J5ACC				    3.14		/// [rad^2/s]

// Constants
#define REGULAR_DIR				1
#define INVERTED_DIR			0



byte inposState = false;

Stepper J3, J4, J5;

// Misc functions
void emptySerialBuffer(int iterations)
{
	for (int i = 0; i < iterations; i++)
	{
		Serial.read();
	}
}
double getSerialfloat(String str)
{
	emptySerialBuffer(100);
	Serial.println(str);
	while (!Serial.available());
	return (double) Serial.parseFloat();
}
void blink_led(int times, unsigned long int dt)
{
  for(int i=0;i<times;i++)
  {
    digitalWrite(13,HIGH);
    delay(dt);
    digitalWrite(13,LOW);
    delay(dt);
  }
}

// Stepper movement functions
void moveSystem()
{
	while (J3.INPOS == false || J4.INPOS == false || J5.INPOS == false)
	{
		if (inposState == true)
		{
			// set inpos flag to be false
			inposState = false;
		}

		// move motors to position
		J3.PTP_update();
		J4.PTP_update();
		J5.PTP_update();

		// this loop will run until motors are in position
	}

	if (inposState == false)
	{
		inposState = true;
	}
}
void moveSystem(double t3, double t4, double t5)
{
	J3.inputSetpointRad(t3);
	J4.inputSetpointRad(t4);
	J5.inputSetpointRad(t5);
	while (J3.INPOS == false || J4.INPOS == false || J5.INPOS == false)
	{
		//unsigned long t1 = micros();
		J3.PTP_update();
		J4.PTP_update();
		J5.PTP_update();
		//unsigned long t2 = micros();
		//Serial.println(t2 - t1);
	}
}
void StepperSetup()
{


	J3.init(J3_STEPPIN,J3_DIR_PIN,J3_EN_PIN,false, J3_StepPerRev,J3_ReductionRatio);
	J3.setVEL(J3_MAX_VEL);
	J3.setACC(J3ACC);
	J3.attach();

	J4.init(J4_STEPPIN, J4_DIR_PIN, J4_EN_PIN, false, J4_StepPerRev,J4_ReductionRatio);
	J4.setVEL(J4_MAX_VEL);
	J4.setACC(J4ACC);
	J4.attach();

	J5.init(J5_STEPPIN, J5_DIR_PIN, J5_EN_PIN, false, J5_StepPerRev, J5_ReductionRatio);
	J5.setVEL(J5_MAX_VEL);
	J5.setACC(J5ACC);
	J5.attach();

}
void moveSystemJoints(double t3, double t4, double t5)
{
	J3.inputSetpointRad(t3);
	J4.inputSetpointRad(t4);
	J5.inputSetpointRad(t5);
}

// I2C functions
void process_cmd(int size)
{
	// The buffers in the message received are arranged in Big Endian configuration (MSB first)
	uint8_t buff[7];
	uint8_t _cmd_type;
	double _cmd_value[3];

	Wire.readBytes(buff, 7);

	_cmd_type = buff[0];

	for(int i=0; i<3; i++)
	{
		int16_t tmp = buff[2*i + 1] << 8 | buff[2*i + 2];
		_cmd_value[i] = (double)tmp / 1000;
	}
	switch (_cmd_type)
	{
	default:
		if(DEBUG)
		{
			Serial.println("no such option. enter 1 in cmd_type");
		}
		break;
	case 1:
		for(int i=0; i<3; i++)
		{
			if(abs(_cmd_value[i])>(2*PI))
			{
				Serial.println("Abs value of joints is too high (>2*PI)");
				return;
			}
			else;
		}
		J3.inputSetpointRad(_cmd_value[0]);
		J4.inputSetpointRad(_cmd_value[1]);
		J5.inputSetpointRad(_cmd_value[2]);
		break;
	}
}
void SendFlag()
{
	uint8_t package = 5;
	Wire.write(package);
}
void WireSetup()
{
	Wire.begin(3);
	Wire.onReceive(process_cmd);
	Wire.onRequest(SendFlag);
}

// Main loop functions
void SlaveLoop()
{
	moveSystem();
}
void TestLoop()
{
	double J3RequiredPosition = getSerialfloat("input J3 setpoint in rad");
	double J4RequiredPosition = getSerialfloat("input J4 setpoint in rad");
	double J5RequiredPosition = getSerialfloat("input J5 setpoint in rad");
	Serial.println("Motion staring in 1 sec.");
	delay(1000);
	moveSystem(J3RequiredPosition, J4RequiredPosition, J5RequiredPosition);

}
void demoLoop()
{
	Serial.println("hello");
	double dist = PI/4;
	J3.inputSetpointRad(dist);
	J4.inputSetpointRad(dist);
	J5.inputSetpointRad(dist);
	moveSystem();
	J3.inputSetpointRad(-dist);
	J4.inputSetpointRad(-dist);
	J5.inputSetpointRad(-dist);
	moveSystem();
	Serial.println("demo loop");
	delay(1000);
}

// Define some steppers and the pins the will use
void setup(){
	if (DEBUG)
		Serial.begin(115200);

	StepperSetup();
	WireSetup();
}

// Add the main program code into the continuous loop() function
void loop(){
	if (DEBUG == true)
	{
		//demoLoop();
		TestLoop();
	}
	else
	{
		SlaveLoop();
	}
}