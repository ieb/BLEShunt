
#include <Arduino.h>
#include <Wire.h>

#include <avr/sleep.h>
#include "commandline.h"
#include "ina219.h"


// pins
#define WAKEUP_PIN    PIN_PA6  // hardware pin 4
#define RESET_PIN     PIN_PA7  // hardware pin 5
#define TEMP_NTC1_PIN PIN_PA4  // hardware pin 2
#define TEMP_NTC2_PIN PIN_PA5  // hardware pin 3
#define LED_PIN       PIN_PA3  // hardware pin 13

// JDY25M is on RX0/TX0 Serial
// Debug header is on Serial1

#define jdy25m Serial
#define debug Serial1

#define ATTINY3224_BOARD 1




Ina219 ina219(&debug);
CommandLine commandLine(&debug);

unsigned long send = millis();
// must be volatile so the compiler doesnt optimnise
volatile bool shouldWakeUp=false;
bool ina219Ok = true;
bool jdy25mOk = true;

#define OUTPUT_BUFFER_LEN 12
uint8_t outputBuffer[OUTPUT_BUFFER_LEN];



#ifdef PROMINI_BOARD
void wakeUp() {
   // Just a handler for the pin interrupt.
}
#endif


#ifdef ATTINY3224_BOARD

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
#endif

/**
 *  Waits for the expected sequence of serial characters.
 */ 
bool expect(const __FlashStringHelper * send, const char * expect, int timeout=1000) {
	if ( send != NULL) {
		if ( commandLine.diagnosticsEnabled ) {
			debug.print(F(">"));
			debug.println(send);
			debug.print(F("<"));			
		}
		jdy25m.print(send);
		jdy25m.print(F("\r\n"));
	}
	int i = 0;
	int l = strlen(expect);
	unsigned long end = millis()+timeout;
	while(true) {
		while (jdy25m.available()) {
			char c = jdy25m.read();
			if ( commandLine.diagnosticsEnabled ) {
				debug.write(c);
			}
			if ( i < l ) {
				if ( c == expect[i] ) {
					i++;
					continue;
				}
			} else if ( i == l ) {
				if ( c == '\r') {
					i++;
					continue;
				}
			} else if ( i == l+1 ) {
				if ( c == '\n' ) {
					return true;
				}
			} 
			i=0;
			if ( c == expect[i]) {
				i++;
			}
		}
		if ( millis() > end ) {
			debug.print(F("Failed to get :"));
			debug.println(expect);
			return false;
		}
		delay(100);
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

void resetJdy25m() {
   digitalWrite(RESET_PIN,LOW);
   delay(100);
   digitalWrite(RESET_PIN,HIGH);
   // It takes about 1000ms for the JDY25M to startup and get to a sleep state.
   // measured by experimentation.
   delay(1000); 	
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


  // These only work when the device is powered on or the RST 11 on device) pin is pulled low
  // reset the device.
  resetJdy25m();

  jdy25m.begin(115200);
	debug.println(F("Connecting 115200"));
  if ( !expect(F("AT+BAUD"),"+BAUD=8") ) {
	  jdy25m.begin(9600);
	  debug.println(F("Connecting 9600, and setting to 115200"));
	  expect(F("AT+BAUD8"),"");
	  jdy25m.begin(115200);
	  resetJdy25m();
	  expect(F("AT+BAUD"),"");
	}

	// clear the read buffer.
	while(jdy25m.available())
    	debug.write(jdy25m.read());



	 // configure on first start
    commandLine.diagnosticsEnabled =true;
    jdy25m.print(F("AT+NAME"));
    jdy25m.print(commandLine.deviceName);
    jdy25m.print("\r\n");
    debug.print(F("AT+NAME"));
    debug.print(commandLine.deviceName);
    debug.print("\n");
	 jdy25mOk = expect(NULL,"OK");
	 jdy25mOk = expect(F("AT+NAME"),commandLine.deviceName) && jdy25mOk;
	 jdy25mOk = expect(NULL,"OK");	 
	 jdy25mOk = expect(F("AT+VERSION"),"") && jdy25mOk;
	 jdy25mOk = expect(F("AT+LADDR"),"") && jdy25mOk;
	 jdy25mOk = expect(F("AT+ROLE"),"") && jdy25mOk;
	 jdy25mOk = expect(F("AT+ROLE0"),"OK") && jdy25mOk; // transparent mode, which will notify 
	 jdy25mOk = expect(F("AT+TYPE0"),"OK") && jdy25mOk; // nopassword connection mode
	 jdy25mOk = expect(F("AT+STARTEN0"),"OK") && jdy25mOk; // enter light sleep. 
	 jdy25mOk = expect(F("AT+ADVIN8"),"OK") && jdy25mOk; // 3s Broadcast, probably about 20uA
	 jdy25mOk = expect(F("AT+RESET"),"+SLEEP", 3000) && jdy25mOk; // reset to enter sleep
 // this will cause the device to go to sleep until there is a connection
 // seen
 // OK
 // +JDY-25M-START
 // +SLEEP
 //expect("AT+SLEEP","");
 // should now be sleeping and advertising.
	 if (ina219Ok && jdy25mOk ) {
	 	 flashLed(2);
	 } 
	 if (!ina219Ok) {
	 	 debug.println("INA219 not responding");
	 }
	 if (!jdy25mOk) {
	 	 debug.println("JDY25M not responding");

	 }

#ifdef ATTINY3224_BOARD
	 set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	 sleep_enable();
#endif

}

int16_t readTemperature(int ch) {
	return 204+ch*10; 
}

void loop() // run over and over
{
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


	  if (jdy25m.available())
	    debug.write(jdy25m.read());


	  unsigned long now = millis();
	  if ( now > send ) {
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
  		int16_t temp1 = readTemperature(0);
  		int16_t temp2 = readTemperature(1);
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
  		jdy25m.write(outputBuffer,OUTPUT_BUFFER_LEN);
  		jdy25m.write("\r\n");
  		jdy25m.flush();
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
		while(jdy25m.available())
	    	debug.write(jdy25m.read());
		// The JDY25 will go into sleep mode when the last connection is removed
		// It will also take the WAKE PIN to low.
		// turn off the ina219
		ina219.powerDown(true);
		// turn off anything else that can be.
	  digitalWrite(LED_PIN, LOW);
		debug.println(F("Sleeping "));
		debug.flush();
		delay(100); // to avoid bouncing
		// sleep

#ifdef ATTINY3224_BOARD
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

#endif
#ifdef PROMINI_BOARD
		attachInterrupt(digitalPinToInterrupt(WAKEUP_PIN), wakeUp, RISING);
		// Enter power down state with ADC and BOD module disabled.
   		// Wake up when wake up pin is low.
   		LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
   		// Disable external pin interrupt on wake up pin.
		detachInterrupt(digitalPinToInterrupt(WAKEUP_PIN));
#endif

		debug.println(F("Woke up "));
		debug.flush();
		// clear the read buffer.
		while(jdy25m.available())
	    	debug.write(jdy25m.read());


	  // restart ina219
		ina219.powerDown(false);
		delay(10); // to avoid bouncing

	}
}

