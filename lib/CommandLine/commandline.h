#pragma once

#include <avr/eeprom.h>


#define HR_DEVICE_SERIAL 0
#define HR_SHUNT_RANGE_CURRENT 2 
#define HR_SHUNT_RANGE_VOLTAGE 4
#define HR_CRC 6
#define HR_SIZE 8




// symbols from main.cpp so we dont have to get complicated.

// stored little endian, since the attiny is little endian.
const uint8_t epromDefaultValues[HR_CRC] PROGMEM = {
  0x04, 0x00,  // serial number
  0x64, 0x00,  // 100A
  0x4B, 0x00   // 75mV
};





class CommandLine {
    public:
        CommandLine(UartClass * io) : io{io} {};
        int16_t serialNumber;
        int16_t shutRangeCurrent = 100;
        int16_t shutRangeVoltage = 75;
        bool diagnosticsEnabled = false;
        char deviceName[10];


        void begin() {
            loadDefaults();
        };

        void showHelp() {
            io->println(F("Pressure Monitor - key presses"));
            io->println(F("  - 'h' or '?' to show this message"));
            io->println(F("  - 's' show status"));
            io->println(F("  - 'd' toggle diagnostics"));
            io->println(F("  - 'r' read sensor"));            
            io->println(F("  - 'S' setup"));
            io->println(F("  - 'R' reset"));
            io->println(F("  - 'F' factory reset"));
        };

        bool checkCommand() {
            if (io->available()) {
                char chr = io->read();
                switch ( chr ) {
                    case 'h': showHelp(); return true;
                    case 's': showStatus(); return true;
                    case 'r': readSensors();  return true;
                    case 'S': doSetup(); return true;
                    case 'R': doReset(); return true;
                    case 'F': loadDefaults(); return true;
                    case 'd': toggleDiagnostics(); return true;
                }
            }
            return diagnosticsEnabled;
        };
    private:
        UartClass * io;


        void doReset() {
            _PROTECTED_WRITE(RSTCTRL.SWRR, 1);
        }

        void toggleDiagnostics() {
            diagnosticsEnabled = !diagnosticsEnabled;
        };

        /**
         * @brief crc for mdbus, polynomial = 0x8005, reverse in, reverse out, init 0xffff;
         * 
         * @param array 
         * @param length 
         * @return uint16_t 
         */
        uint16_t crc16(const uint8_t *array, uint16_t length) {
            uint16_t crc = 0xffff;
            while (length--) {
                if ((length & 0xFF) == 0) yield();  // RTOS
                uint8_t data = *array++;
                data = (((data & 0xAA) >> 1) | ((data & 0x55) << 1));
                data = (((data & 0xCC) >> 2) | ((data & 0x33) << 2));
                data =          ((data >> 4) | (data << 4));
                crc ^= ((uint16_t)data) << 8;
                for (uint8_t i = 8; i; i--) {
                if (crc & (1 << 15)) {
                    crc <<= 1;
                    crc ^= 0x8005;
                } else {
                    crc <<= 1;
                }
                }
            }
            crc = (((crc & 0XAAAA) >> 1) | ((crc & 0X5555) << 1));
            crc = (((crc & 0xCCCC) >> 2) | ((crc & 0X3333) << 2));
            crc = (((crc & 0xF0F0) >> 4) | ((crc & 0X0F0F) << 4));
            //  crc = (( crc >> 8) | (crc << 8));
            //  crc ^= endmask;
            return crc;
        };

        void loadDefaults() {
            uint8_t eepromContents[HR_SIZE];
            eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
            uint16_t crcv =  crc16(&eepromContents[0], HR_SIZE-2);
            uint16_t crcvs = (0xff&eepromContents[HR_CRC]) | (0xff00&(eepromContents[HR_CRC+1]<<8)); // little endian
            if (crcv != crcvs || eepromContents[HR_DEVICE_SERIAL] == 0 ) {
                io->print(F("EPROM not initialised"));
                for(uint8_t i = 0; i < HR_SIZE-2; i++) {
                    eepromContents[i] = pgm_read_byte_near(epromDefaultValues+i);
                }
                crcv =  crc16(&eepromContents[0], HR_SIZE-2);
                eepromContents[HR_CRC] = 0xff&(crcv);
                eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
                eeprom_update_block(&eepromContents[0], (void*)0, HR_SIZE);
            }
            loadSettings(&eepromContents[0]);
            showStatus();

        };
        char toHex(int n) {
            if ( n <= 9) {
                return '0'+n;
            } else {
                return 'a'-10+n;
            }
        }

        void loadSettings(const uint8_t *eepromContents) {            
            serialNumber = asInt16(eepromContents,HR_DEVICE_SERIAL);
            // ShuntXXXX
            deviceName[0] = 'S';
            deviceName[1] = 'h';
            deviceName[2] = 'u';
            deviceName[3] = 'n';
            deviceName[4] = 't';
            deviceName[5] = toHex((serialNumber&0xf000)>>12); 
            deviceName[6] = toHex((serialNumber&0x0f00)>>8); 
            deviceName[7] = toHex((serialNumber&0x00f0)>>4); 
            deviceName[8] = toHex((serialNumber&0x000f)); 
            deviceName[9] = 0; 
            shutRangeCurrent = asInt16(eepromContents,HR_SHUNT_RANGE_CURRENT);
            shutRangeVoltage = asInt16(eepromContents,HR_SHUNT_RANGE_VOLTAGE);
        }

        void doSetup() {
            showStatus();
            io->setTimeout(10000);
            uint8_t eepromContents[HR_SIZE];
            eeprom_read_block((void*)&eepromContents[0], (const void*)0, HR_SIZE);
            io->print(F("Setup - shunt serial number "));io->print(asInt16(eepromContents, HR_DEVICE_SERIAL));io->print(F("A >"));
            bool changed = updateSetting(eepromContents, HR_DEVICE_SERIAL) || changed;
            io->print(F("Setup - shunt range current "));io->print(asInt16(eepromContents, HR_SHUNT_RANGE_CURRENT));io->print(F("A >"));
            changed = updateSetting(eepromContents, HR_SHUNT_RANGE_CURRENT) || changed;
            io->print(F("Setup - shunt range voltage "));io->print(asInt16(eepromContents, HR_SHUNT_RANGE_VOLTAGE));io->print(F("mV >"));
            changed = updateSetting(eepromContents, HR_SHUNT_RANGE_VOLTAGE) || changed;

            if ( changed ) {
                // update CRC and save.
                uint16_t crcv =  crc16(&eepromContents[0], HR_SIZE-2);
                eepromContents[HR_CRC] = 0xff&(crcv);
                eepromContents[HR_CRC+1] = 0xff&(crcv>>8); // little endian
                eeprom_update_block((const void*)&eepromContents[0], (void*)0, HR_SIZE);
                // update device address, which may have changed.
                loadSettings(&eepromContents[0]);
                showStatus();
                io->println(F("Settings changed, device will restart in 5s "));
                delay(5000);
                doReset();
            }
            io->setTimeout(1000);
        };

        void readSensors() {
            io->println("Todo");
        };

        void showStatus() {

            io->println(F("Status"));
            io->print(F("Device Name    : "));
            io->println(deviceName);
            io->print(F("Serial Number  : "));
            io->println(serialNumber);
            io->print(F("Shunt Range Current : "));io->print(shutRangeCurrent);io->println(F(" A"));
            io->print(F("Shunt Range Voltage : "));io->print(shutRangeVoltage);io->println(F(" mV"));
            io->print((F("MCU V=")));
            io->print(readMCUVoltage());
            io->print((F("mV T=")));
            int32_t t = readMCUTemperature();
            t -= 273;
            io->print(t);
            io->println((F("C")));

        };


        bool readInt(int16_t *v) {
            String line = io->readStringUntil('\r');
            line.trim();
            if ( line.length() > 0 ) {
                *v = line.toInt();
                return true;
            }
            return false;
        };
        bool updateSetting(int8_t * registers, uint8_t offset, int16_t minval =  INT16_MIN, int16_t maxval = INT16_MAX) {
            int16_t value;
            if (readInt(&value)) {
                if ( value > minval && value < maxval ) {
                    // little endian
                    registers[offset] = 0xff&value;
                    registers[offset+1] = 0xff&(value>>8);
                    io->println(F(" - changed"));
                    return true;
                } else {
                    io->println(F(" - invalid, no change"));
                }
            } else {
                io->println(F(" - no change"));
            }
            return false;
        };
        int16_t asInt16(const uint8_t * registers, uint8_t offset) {
            // little endian
            return registers[offset] | (0xff00&(registers[offset+1]<<8));
        };

        void dumpRegisters(const uint8_t * b) {
            for(int i = 0; i < HR_SIZE; i+=2 ) {
                io->print(i);
                io->print(':');
                io->println(asInt16(b,i));
            }
        }






        

        uint16_t readMCUVoltage() {
            analogReference(INTERNAL1V024);
            delayMicroseconds(100);
            int32_t vddmeasure = analogReadEnh(ADC_VDDDIV10, 12); // Take it at 12 bits
            vddmeasure *= 10; // since we measured 1/10th VDD
            int16_t returnval = vddmeasure >> 2; // divide by 4 to get into millivolts.
            if (vddmeasure & 0x02) {
                // if last two digits were 0b11 or 0b10 we should round up
                returnval++;
            }
            return returnval;
        }

        uint16_t readMCUTemperature() {
            int8_t sigrowOffset = SIGROW.TEMPSENSE1;
            uint8_t sigrowGain = SIGROW.TEMPSENSE0;
            analogSampleDuration(128); // must be >= 32µs * f_CLK_ADC per datasheet 30.3.3.7
            analogReference(INTERNAL1V024);
            uint32_t reading = analogRead(ADC_TEMPERATURE);
            reading -= sigrowOffset;
            reading *= sigrowGain;
            reading += 0x80; // Add 1/2 to get correct rounding on division below
            reading >>= 8; // Divide result to get Kelvin
            return reading;

        }

};


