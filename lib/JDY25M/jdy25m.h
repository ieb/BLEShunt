#pragma once
#include <Arduino.h>

class Jdy25m {
public:
    Jdy25m(uint8_t resetPin, UartClass *port, UartClass *debug):
            resetPin{resetPin},  
            port{port}, 
            debug{debug} {};

    bool begin(const char * deviceName) {
        port->begin(115200);

        debug->println(F("Connecting 115200"));
        if ( !expect(F("AT+BAUD"),"+BAUD=8", false) ) {
          port->begin(9600);
          debug->println(F("Connecting 9600, and setting to 115200"));
          expect(F("AT+BAUD8"),"", false);
          port->begin(115200);
          reset();
          expect(F("AT+BAUD"),"", false);
        }

        // clear the read buffer.
        while(port->available())
            debug->write(port->read());



         // configure on first start
        port->print(F("AT+NAME"));
        port->print(deviceName);
        port->print("\r\n");
        debug->print(F("AT+NAME"));
        debug->print(deviceName);
        debug->print("\n");
        expect(NULL,"OK");
        debug->println("Starting JDY25m board ");
        bool sucess = expect(F("AT+NAME"),deviceName);
        sucess = expect(NULL,"OK") && sucess;    
        sucess = expect(F("AT+VERSION"),"") && sucess;
        sucess = expect(F("AT+LADDR"),"") && sucess;
        sucess = expect(F("AT+ROLE"),"") && sucess;
        sucess = expect(F("AT+ROLE0"),"OK") && sucess; // transparent mode, which will notify 
        sucess = expect(F("AT+TYPE0"),"OK") && sucess; // nopassword connection mode
        sucess = expect(F("AT+STARTEN0"),"OK") && sucess; // enter light sleep. 
        sucess = expect(F("AT+ADVIN8"),"OK") && sucess; // 3s Broadcast, probably about 20uA
        sucess = expect(F("AT+RESET"),"+SLEEP", 3000) && sucess; // reset to enter sleep
         // this will cause the device to go to sleep until there is a connection
         // seen
         // OK
         // +JDY-25M-START
         // +SLEEP
         //expect("AT+SLEEP","");
         // should now be sleeping and advertising.
        if ( sucess ) {
            debug->println("JDY Start Ok");
        } else {
            debug->println("JDY Start Fail");
        }
        return sucess;
    };

    void enableLogging(bool logEnabled) {
        this->logEnabled = logEnabled;
    };    
    void echo() {
      if (port->available())
        debug->write(port->read());
    };
    void powerDown(bool down) {
        // nothing to do.
        echo();
    };

    void notify(uint8_t *outputBuffer, uint8_t len) {
        port->write(outputBuffer,len);
        port->write("\r\n");
        port->flush();
    };

    void reset() {
       digitalWrite(resetPin,LOW);
       delay(100);
       digitalWrite(resetPin,HIGH);
       // It takes about 1000ms for the port->to startup and get to a sleep state.
       // measured by experimentation.
       delay(1000);     
    };
private:
    uint8_t resetPin;
    UartClass *port;
    UartClass *debug;
    bool logEnabled = false;

    /**
     *  Waits for the expected sequence of serial characters.
    */ 
    bool expect(const __FlashStringHelper * send, 
            const char * expect, 
            int timeout=1000,
            bool logfails = true) {
        if ( send != NULL) {
            if ( logEnabled ) {
                debug->print(F(">"));
                debug->println(send);
                debug->print(F("<"));           
            }
            port->print(send);
            port->print(F("\r\n"));
        }
        int i = 0;
        int l = strlen(expect);
        unsigned long end = millis()+timeout;
        while(true) {
            while (port->available()) {
                char c = port->read();
                if ( logEnabled ) {
                    debug->write(c);
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
                if ( logfails ) {
                    debug->print(F("Failed to get :"));
                    debug->println(expect);                    
                }
                return false;
            }
            delay(100);
        }
    };


};