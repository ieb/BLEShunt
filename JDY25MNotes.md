# Notes on JDY-25M


## Broadcasting with light sleep

	AT+ROLE0       ; selects Transparent transmission of slave
	AT+STARTEN0    ; light sleep transmission, 500mS gives 30uA
	AT+ADVIN5      ; 700m period  see user manual for LUT.
    AT+SLEEP       ; enter light sleep

## Mesh Addressing 

	0xFFFF broadcast
	0x0002 to 0xFF00 uncast addresses
	0x0001 central machine
	0xFF01 to 0xFFFE system dont use

## Mesh addressing commands

commands are sent as <header 7 byte> + <cmd 1byte> + <Maddr 2 byte> + <data 1-17 bytes> + 0D0A

Header is the AT command, often expressed in hex in the manuals, very confusing.


    0x00 Send data no ACK
    0x01 Send data with ACK
    0x10 Control IO pin on target, no ACK
    0x11 Control IO pin on target, ACK
    0x31 Read target device parameters
    0x41 Configure target device paramters, NETID, MADDR, PIN, KEY, TYEP
    0xA1 Send one-to-one data to all friend nodes.
    0xA2 Send one-to-one data to private friends


## IO Control 

IO control is sent as <header 7 byte> + <cmd 1byte> + <Maddr 2 byte> + <io instruction 3 bytes> + <io number 1 byte> + <io level 1 byte> + 0D0A


    0xAA__E7   set single IO pin   leve is 0 or 1
    0xAB__E7   sets all IO poins   level is bitmap, lower 5 bytes.
    0xA1__E7   flips single IO pin 
    0xA2__E7   flips all IO pins

    0x__B1__   no serial port output
    0x__B2__   serial port output



    0xAAB1E7   Set IO pin no serial output
    0xAAB2E7   Set IO pin with serial output  
    0xABB1E7   Set all IO pins, no serial output
    0xABB2E7   Set all IO pins serial output

See more examples in documentation.



