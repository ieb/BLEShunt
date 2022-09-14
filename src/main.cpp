
#include <Arduino.h>
#include <Wire.h>

#include <avr/sleep.h>
#include "commandline.h"
#include "ina219.h"
#include "ntcsensor.h"
#include "jdy25m.h"


// pins
#define WAKEUP_PIN    PIN_PA6  // hardware pin 4
#define RESET_PIN     PIN_PA7  // hardware pin 5
#define TEMP_NTC1_PIN PIN_PA4  // hardware pin 2
#define TEMP_NTC2_PIN PIN_PA5  // hardware pin 3
#define LED_PIN       PIN_PA3  // hardware pin 13

// JDY25M is on RX0/TX0 Serial
// Debug header is on Serial1

#define jdy25mUart Serial
#define debug Serial1

Ina219 ina219(&debug);
CommandLine commandLine(&debug);
NtcSensor ntcSensor(&debug);
Jdy25m jdy25m(RESET_PIN, &jdy25mUart, &debug);


unsigned long send = millis();
// must be volatile so the compiler doesnt optimnise
volatile bool shouldWakeUp=false;
bool ina219Ok = true;
bool jdy25mOk = true;

#define OUTPUT_BUFFER_LEN 12
uint8_t outputBuffer[OUTPUT_BUFFER_LEN];





// see https://github.com/SpenceKonde/megaTinyCore/blob/e59fc3046dd69f951497506e8f441a6818c28be5/megaavr/extras/Ref_PinInterrupts.md
// Used to detect wake up from sleep.
ISR(PORTA_PORT_vect) {
	byte flags = VPORTA.INTFLAGS;
	// wake up pin is pin PA 6
	if (flags & (1 << 6) ) {
		shouldWakeUp=true;
		VPORTA.INTFLAGS |= (1 << 6); // clear the flag, so it doesnt fire continuously.
	}
}




// in role 0 these are hard coded into the JDY-25M device
// and cant be changed
// They appear as services and Gatt characteristics with write+notify
// but this is only for reference. the AT+MESH command decides where the 
// data is going based on the CMD.
// These are here for reference only.
#define JDY25_UUID 0xFFE0  
#define JDY25_TRANSPARENT_CHARACTERISTIC_CH0 0xFFE1
#define JDY25_TRANSPARENT_CHARACTERISTIC_CH1 0xFFE2
#define JDY25_MESH_DATA_CHARACTERISTIC 0xFFE3


void flashLed(int n) {
	for ( int i = 0; i < n; i++) {
		digitalWrite(LED_PIN, HIGH);
		delay(250);
		commandLine.checkCommand();
		digitalWrite(LED_PIN, LOW);
		delay(250);
		commandLine.checkCommand();
	}
	for ( int i = 0; i < 8; i++) {
		delay(250);
		commandLine.checkCommand();
	}

}

void dumpBuffer() {
	for (int i = 0; i < OUTPUT_BUFFER_LEN; i++) {
		if ( outputBuffer[i] < 0x10 ) {
			debug.print('0');
		}
		debug.print(outputBuffer[i],HEX);
	}
	debug.println("");
}



void setup() {

  pinMode(WAKEUP_PIN, INPUT);
  pinMode(RESET_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  // Open serial communications and wait for port to open:
  debug.begin(115200);

  // try 115200, if that fails drop back to 9600 and set.


  if ( ina219.begin() != I2C_OK) {
  	ina219Ok = false;
	  debug.print("Failed to find INA219 chip, error:");
	  debug.println(ina219.getError());
  }
  commandLine.begin();

  ntcSensor.addSensor(TEMP_NTC1_PIN, 0);
  ntcSensor.addSensor(TEMP_NTC2_PIN, 1);
  ntcSensor.begin();

  commandLine.diagnosticsEnabled =true;

  // These only work when the device is powered on or the RST 11 on device) pin is pulled low
  // reset the device.
  if ( !jdy25m.begin((const char *)&commandLine.deviceName[0]) ) {
	  debug.print("Failed to find JDY25M board");
	  jdy25mOk = false;
  }

	 if (ina219Ok && jdy25mOk ) {
	 	 flashLed(2);
	 } 
	 if (!ina219Ok) {
	 	 debug.println("INA219 not responding");
	 }
	 if (!jdy25mOk) {
	 	 debug.println("JDY25M not responding");

	 }

	 set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	 sleep_enable();

}


void loop() {
	static unsigned long clKeepAlive = 60000;
	static int state = 0;
	unsigned long now = millis();
	if ( !(ina219Ok && jdy25mOk )) {
	 	if ( !ina219Ok ) {
	 		flashLed(3);
	 	} 
	 	if ( !jdy25mOk) {
	 		flashLed(5);
	 	}
	} else if ( now < clKeepAlive || digitalRead(WAKEUP_PIN) == HIGH ) {
	  if ( commandLine.checkCommand() ) {
	  	clKeepAlive = millis() + 60000;
	  }


	  jdy25m.echo();


	  unsigned long now = millis();
	  if ( now > send ) {
			ina219.enableLogging(commandLine.diagnosticsEnabled);
			ntcSensor.enableLogging(commandLine.diagnosticsEnabled);
			jdy25m.enableLogging(commandLine.diagnosticsEnabled);
	  	uint16_t error = 0x0000;
	  	// read the ina219 and notify BLE.
	  	send = now+1000;
	  	digitalWrite(LED_PIN, CHANGE);
	  	// 1mV per bit, +-32V max
  		int16_t voltage = (int16_t)(1000.0*ina219.getVoltage());
  		if ( ina219.getError() != I2C_OK) {
  			error |= 0x0001;
  		}
  		// 10mA per bit 320A  0.01*32000
  	  int16_t current = (int16_t)(100*ina219.getCurrent());
  		if ( ina219.getError() != I2C_OK) {
  			error |= 0x0002;
  		}
  		// temperature 0.01C per bit, 320C max
  		ntcSensor.refresh();
  		int16_t temp1 = ntcSensor.getTemperature(0);
  		int16_t temp2 = ntcSensor.getTemperature(1);
  		uint16_t tons = millis()/1000;
  		outputBuffer[0] = voltage>>8;
  		outputBuffer[1] = voltage&0xff;
  		outputBuffer[2] = current>>8;
  		outputBuffer[3] = current&0xff;
  		outputBuffer[4] = temp1>>8;
  		outputBuffer[5] = temp1&0xff;
  		outputBuffer[6] = temp2>>8;
  		outputBuffer[7] = temp2&0xff;
  		outputBuffer[8] = tons>>8;
  		outputBuffer[9] = tons&0xff;
  		outputBuffer[10] = error>>8;
  		outputBuffer[11] = error&0xff;
  		jdy25m.notify(outputBuffer,OUTPUT_BUFFER_LEN);
  		if ( commandLine.diagnosticsEnabled ) {
  			debug.print(" v=");debug.print(voltage);
  			debug.print(" c=");debug.print(current);
  			debug.print(" t1=");debug.print(temp1);
  			debug.print(" t2=");debug.print(temp2);
  			debug.print(" tons=");debug.print(tons);
  			debug.print(" error=");debug.print(error);
  			debug.print(" gatt=");
	  		dumpBuffer();
  		}
	  }

	} else {
		delay(100); // to avoid bouncing
		// clear the read buffer.
		// The JDY25 will go into sleep mode when the last connection is removed
		// It will also take the WAKE PIN to low.
		// turn off the ina219
		ina219.powerDown(true);
		jdy25m.powerDown(true);
		// turn off anything else that can be.
	  digitalWrite(LED_PIN, LOW);
		debug.println(F("Sleeping "));
		debug.flush();
		delay(100); // to avoid bouncing
		// sleep

		shouldWakeUp = false;
		USART0.CTRLB != 1<<4; // set the SFDEN bit, start of frame detection.
		// clear the flag on pin6
		VPORTA.INTFLAGS |= (1 << 6);
		// enable the ISR on pin6 rising, with pullup.
		// 0xx010 == rising.
		// 0x1xxx == input pullup
		PORTA.PIN6CTRL  = 0b00001010;
		do {
			sleep_cpu();
		} while (!shouldWakeUp);
		// diable the ISR
		// 0xx000 == disabled
		// 0x1xxx == input pullup
		PORTA.PIN6CTRL = 0b00001000;


		debug.println(F("Woke up "));
		debug.flush();
		// clear the read buffer.
		jdy25m.powerDown(false);
	  // restart ina219
		ina219.powerDown(false);
		delay(10); // to avoid bouncing

	}
}

