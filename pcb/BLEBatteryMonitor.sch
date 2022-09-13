EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:BLEBatteryMonitor-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L INA219-module U102
U 1 1 62F8F2F5
P 7150 3550
F 0 "U102" H 7150 3550 60  0000 C CNN
F 1 "INA219-module" H 7150 3550 60  0000 C CNN
F 2 "Divers:INA219Module" H 7150 3550 60  0001 C CNN
F 3 "" H 7150 3550 60  0001 C CNN
	1    7150 3550
	1    0    0    -1  
$EndComp
$Comp
L ATTINY3224-SS IC101
U 1 1 62F8F322
P 7250 2100
F 0 "IC101" H 6500 2850 60  0000 C CNN
F 1 "ATTINY3224-SS" H 7800 1350 60  0000 C CNN
F 2 "Housings_SOIC:SOIC-14_3.9x8.7mm_Pitch1.27mm" H 6550 1350 60  0001 C CNN
F 3 "" H 7250 2100 60  0001 C CNN
	1    7250 2100
	1    0    0    -1  
$EndComp
Text GLabel 8300 2250 2    60   BiDi ~ 0
SCL
Text GLabel 8300 2400 2    60   BiDi ~ 0
SDA
Text GLabel 6200 2400 0    60   Output ~ 0
RS-TX
Text GLabel 6200 2250 0    60   Input ~ 0
RS-RX
Text GLabel 8800 2100 2    60   BiDi ~ 0
UDPI
Text GLabel 8300 1800 2    60   Input ~ 0
RX
Text GLabel 8300 1950 2    60   Output ~ 0
TX
Text GLabel 5450 1500 0    60   BiDi ~ 0
3V
$Comp
L GND #PWR01
U 1 1 62F8F4C4
P 8500 1550
F 0 "#PWR01" H 8500 1550 30  0001 C CNN
F 1 "GND" H 8500 1480 30  0001 C CNN
F 2 "" H 8500 1550 60  0001 C CNN
F 3 "" H 8500 1550 60  0001 C CNN
	1    8500 1550
	1    0    0    -1  
$EndComp
Wire Wire Line
	8300 1500 8500 1500
Wire Wire Line
	8500 1500 8500 1550
Text GLabel 6200 1650 0    60   Input ~ 0
TEMP1
Text GLabel 6200 1800 0    60   Input ~ 0
TEMP2
Text GLabel 7750 3500 2    60   BiDi ~ 0
SDA
Text GLabel 7750 3600 2    60   BiDi ~ 0
SCL
Text GLabel 7750 3800 2    60   BiDi ~ 0
3V
$Comp
L GND #PWR02
U 1 1 62F8F671
P 7750 3700
F 0 "#PWR02" H 7750 3700 30  0001 C CNN
F 1 "GND" H 7750 3630 30  0001 C CNN
F 2 "" H 7750 3700 60  0001 C CNN
F 3 "" H 7750 3700 60  0001 C CNN
	1    7750 3700
	1    0    0    -1  
$EndComp
Text GLabel 9900 2600 0    60   Input ~ 0
TEMP1
$Comp
L R R102
U 1 1 62F8F813
P 10100 2300
F 0 "R102" V 10180 2300 40  0000 C CNN
F 1 "10K" V 10107 2301 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 10030 2300 30  0001 C CNN
F 3 "" H 10100 2300 30  0000 C CNN
	1    10100 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	9900 2600 10100 2600
Wire Wire Line
	10100 2600 10100 2550
Text GLabel 10100 2050 0    60   BiDi ~ 0
3V
Text GLabel 10600 2600 0    60   Input ~ 0
TEMP2
$Comp
L R R103
U 1 1 62F8F89A
P 10800 2250
F 0 "R103" V 10880 2250 40  0000 C CNN
F 1 "10K" V 10807 2251 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 10730 2250 30  0001 C CNN
F 3 "" H 10800 2250 30  0000 C CNN
	1    10800 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	10600 2600 10800 2600
Wire Wire Line
	10800 2600 10800 2500
Text GLabel 10800 2000 0    60   BiDi ~ 0
3V
$Comp
L C C102
U 1 1 62F8F8E4
P 4250 1650
F 0 "C102" H 4250 1750 40  0000 L CNN
F 1 "10uF" H 4256 1565 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 4288 1500 30  0001 C CNN
F 3 "" H 4250 1650 60  0000 C CNN
	1    4250 1650
	-1   0    0    1   
$EndComp
$Comp
L C C103
U 1 1 62F8F962
P 5650 1750
F 0 "C103" H 5650 1850 40  0000 L CNN
F 1 "100nF" H 5656 1665 40  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 5688 1600 30  0001 C CNN
F 3 "" H 5650 1750 60  0000 C CNN
	1    5650 1750
	1    0    0    -1  
$EndComp
Text GLabel 7750 3300 2    60   BiDi ~ 0
IN+
Text GLabel 7750 3400 2    60   BiDi ~ 0
IN-
$Comp
L CONN_5 P104
U 1 1 62F8FA97
P 10900 3400
F 0 "P104" V 10850 3400 50  0000 C CNN
F 1 "DEBUG" V 10950 3400 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x05" H 10900 3400 60  0001 C CNN
F 3 "" H 10900 3400 60  0001 C CNN
	1    10900 3400
	1    0    0    -1  
$EndComp
Text GLabel 10500 3200 0    60   BiDi ~ 0
3V
$Comp
L GND #PWR03
U 1 1 62F8FB41
P 10500 3300
F 0 "#PWR03" H 10500 3300 30  0001 C CNN
F 1 "GND" H 10500 3230 30  0001 C CNN
F 2 "" H 10500 3300 60  0001 C CNN
F 3 "" H 10500 3300 60  0001 C CNN
	1    10500 3300
	1    0    0    -1  
$EndComp
Text GLabel 10500 3600 0    60   BiDi ~ 0
UDPI
Text GLabel 10500 3400 0    60   Output ~ 0
RX
Text GLabel 10500 3500 0    60   Input ~ 0
TX
$Comp
L R R101
U 1 1 62F8FC80
P 8550 2100
F 0 "R101" V 8630 2100 40  0000 C CNN
F 1 "270R" V 8557 2101 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 8480 2100 30  0001 C CNN
F 3 "" H 8550 2100 30  0000 C CNN
	1    8550 2100
	0    1    1    0   
$EndComp
Wire Wire Line
	5450 1500 6200 1500
Wire Wire Line
	5650 1550 5650 1500
Connection ~ 5650 1500
$Comp
L GND #PWR04
U 1 1 62F8FDF9
P 5650 1950
F 0 "#PWR04" H 5650 1950 30  0001 C CNN
F 1 "GND" H 5650 1880 30  0001 C CNN
F 2 "" H 5650 1950 60  0001 C CNN
F 3 "" H 5650 1950 60  0001 C CNN
	1    5650 1950
	1    0    0    -1  
$EndComp
Text GLabel 10450 4750 0    60   Input ~ 0
TEMP1
Text GLabel 10450 5100 0    60   Input ~ 0
TEMP2
Text GLabel 10500 5500 0    60   BiDi ~ 0
IN+
Text GLabel 10500 5600 0    60   BiDi ~ 0
IN-
$Comp
L GND #PWR05
U 1 1 62F90018
P 10500 5700
F 0 "#PWR05" H 10500 5700 30  0001 C CNN
F 1 "GND" H 10500 5630 30  0001 C CNN
F 2 "" H 10500 5700 60  0001 C CNN
F 3 "" H 10500 5700 60  0001 C CNN
	1    10500 5700
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P102
U 1 1 62F9028E
P 10800 4850
F 0 "P102" V 10750 4850 40  0000 C CNN
F 1 "CONN_2" V 10850 4850 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 10800 4850 60  0001 C CNN
F 3 "" H 10800 4850 60  0001 C CNN
	1    10800 4850
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR06
U 1 1 62F902F4
P 10450 4950
F 0 "#PWR06" H 10450 4950 30  0001 C CNN
F 1 "GND" H 10450 4880 30  0001 C CNN
F 2 "" H 10450 4950 60  0001 C CNN
F 3 "" H 10450 4950 60  0001 C CNN
	1    10450 4950
	1    0    0    -1  
$EndComp
$Comp
L CONN_2 P103
U 1 1 62F90323
P 10800 5200
F 0 "P103" V 10750 5200 40  0000 C CNN
F 1 "CONN_2" V 10850 5200 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02" H 10800 5200 60  0001 C CNN
F 3 "" H 10800 5200 60  0001 C CNN
	1    10800 5200
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 62F904A6
P 10450 5300
F 0 "#PWR07" H 10450 5300 30  0001 C CNN
F 1 "GND" H 10450 5230 30  0001 C CNN
F 2 "" H 10450 5300 60  0001 C CNN
F 3 "" H 10450 5300 60  0001 C CNN
	1    10450 5300
	1    0    0    -1  
$EndComp
$Comp
L NCP1117ST50T3G U101
U 1 1 62F906EF
P 3700 1350
F 0 "U101" H 3850 1154 40  0000 C CNN
F 1 "NCP1117ST50T3G" H 3700 1550 40  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-223-3Lead_TabPin2" H 3700 1350 60  0000 C CNN
F 3 "" H 3700 1350 60  0000 C CNN
	1    3700 1350
	1    0    0    -1  
$EndComp
$Comp
L C C101
U 1 1 62F9089E
P 3050 1650
F 0 "C101" H 3050 1750 40  0000 L CNN
F 1 "1uF" H 3056 1565 40  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 3088 1500 30  0001 C CNN
F 3 "" H 3050 1650 60  0000 C CNN
	1    3050 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4250 1850 4250 2000
Wire Wire Line
	4250 2000 3050 2000
Wire Wire Line
	3050 2000 3050 1850
Wire Wire Line
	3700 1600 3700 2100
Connection ~ 3700 2000
Wire Wire Line
	2300 1300 3300 1300
Wire Wire Line
	3050 1300 3050 1450
Wire Wire Line
	4100 1300 4250 1300
Wire Wire Line
	4250 1300 4250 1450
Text GLabel 4250 1300 2    60   BiDi ~ 0
3V
Text GLabel 1700 1300 0    60   BiDi ~ 0
IN+
Connection ~ 3050 1300
$Comp
L GND #PWR08
U 1 1 62F90ACC
P 3700 2100
F 0 "#PWR08" H 3700 2100 30  0001 C CNN
F 1 "GND" H 3700 2030 30  0001 C CNN
F 2 "" H 3700 2100 60  0001 C CNN
F 3 "" H 3700 2100 60  0001 C CNN
	1    3700 2100
	1    0    0    -1  
$EndComp
$Comp
L CONN_3 K101
U 1 1 62F90DF3
P 10850 5600
F 0 "K101" V 10800 5600 50  0000 C CNN
F 1 "CONN_3" V 10900 5600 40  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 10850 5600 60  0001 C CNN
F 3 "" H 10850 5600 60  0001 C CNN
	1    10850 5600
	1    0    0    -1  
$EndComp
$Comp
L JDY25M U104
U 1 1 631A1145
P 6600 4800
F 0 "U104" H 6600 4800 60  0000 C CNN
F 1 "JDY25M" H 6700 4550 60  0000 C CNN
F 2 "Divers:JDY-25M" H 6600 4800 60  0001 C CNN
F 3 "" H 6600 4800 60  0001 C CNN
	1    6600 4800
	1    0    0    -1  
$EndComp
Text GLabel 6000 4600 0    60   BiDi ~ 0
3V
$Comp
L GND #PWR09
U 1 1 631A125E
P 7550 4600
F 0 "#PWR09" H 7550 4600 30  0001 C CNN
F 1 "GND" H 7550 4530 30  0001 C CNN
F 2 "" H 7550 4600 60  0001 C CNN
F 3 "" H 7550 4600 60  0001 C CNN
	1    7550 4600
	1    0    0    -1  
$EndComp
NoConn ~ 7550 4800
NoConn ~ 7550 4900
NoConn ~ 7550 5200
NoConn ~ 7550 5300
NoConn ~ 7100 5600
NoConn ~ 7000 5600
NoConn ~ 6900 5600
NoConn ~ 6800 5600
NoConn ~ 6700 5600
NoConn ~ 6600 5600
NoConn ~ 6500 5600
NoConn ~ 6400 5600
NoConn ~ 6000 5300
NoConn ~ 6000 5200
NoConn ~ 6000 5000
NoConn ~ 6000 4900
NoConn ~ 6000 4800
NoConn ~ 6000 4700
NoConn ~ 10200 1600
NoConn ~ 10350 1800
$Comp
L NCP718 U103
U 1 1 631A1898
P 3550 3050
F 0 "U103" H 3550 3200 60  0000 C CNN
F 1 "NCP718" H 3600 3100 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 3550 3050 60  0001 C CNN
F 3 "" H 3550 3050 60  0001 C CNN
	1    3550 3050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 631A18F5
P 3550 3500
F 0 "#PWR010" H 3550 3500 30  0001 C CNN
F 1 "GND" H 3550 3430 30  0001 C CNN
F 2 "" H 3550 3500 60  0001 C CNN
F 3 "" H 3550 3500 60  0001 C CNN
	1    3550 3500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3050 2900 3000 2900
Wire Wire Line
	3000 2900 3000 2300
Wire Wire Line
	3000 2300 3200 2300
Wire Wire Line
	3200 2300 3200 1300
Connection ~ 3200 1300
Wire Wire Line
	4250 2900 4450 2900
Wire Wire Line
	4450 2900 4450 2500
Wire Wire Line
	4450 2500 4100 2500
Wire Wire Line
	4100 2500 4100 1300
$Comp
L CONN_1 P101
U 1 1 631A162C
P 9650 5450
F 0 "P101" H 9730 5450 40  0000 L CNN
F 1 "CONN_1" H 9650 5505 30  0001 C CNN
F 2 "Divers:M5Bolt" H 9650 5450 60  0001 C CNN
F 3 "" H 9650 5450 60  0001 C CNN
	1    9650 5450
	1    0    0    -1  
$EndComp
$Comp
L CONN_1 P105
U 1 1 631A16B4
P 9650 5600
F 0 "P105" H 9730 5600 40  0000 L CNN
F 1 "CONN_1" H 9650 5655 30  0001 C CNN
F 2 "Divers:M5Bolt" H 9650 5600 60  0001 C CNN
F 3 "" H 9650 5600 60  0001 C CNN
	1    9650 5600
	1    0    0    -1  
$EndComp
Text GLabel 9500 5450 0    60   BiDi ~ 0
IN+
Text GLabel 9500 5600 0    60   BiDi ~ 0
IN-
Text GLabel 7550 5100 2    60   Output ~ 0
RS-RX
Text GLabel 7550 5000 2    60   Input ~ 0
RS-TX
$Comp
L LED D101
U 1 1 631A3157
P 5400 3000
F 0 "D101" H 5400 3100 50  0000 C CNN
F 1 "LED" H 5400 2900 50  0000 C CNN
F 2 "LEDs:LED_0805" H 5400 3000 60  0001 C CNN
F 3 "" H 5400 3000 60  0000 C CNN
	1    5400 3000
	1    0    0    -1  
$EndComp
$Comp
L THERMISTOR TH101
U 1 1 631A31C6
P 2050 1300
F 0 "TH101" V 2150 1350 50  0000 C CNN
F 1 "THERMISTOR" V 1950 1300 50  0000 C CNN
F 2 "Resistors_ThroughHole:Resistor_Horizontal_RM7mm" H 2050 1300 60  0001 C CNN
F 3 "" H 2050 1300 60  0000 C CNN
	1    2050 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	1700 1300 1800 1300
$Comp
L R R104
U 1 1 631A35A4
P 5850 3000
F 0 "R104" V 5930 3000 40  0000 C CNN
F 1 "2.2K" V 5857 3001 40  0000 C CNN
F 2 "Resistors_SMD:R_0805" V 5780 3000 30  0001 C CNN
F 3 "" H 5850 3000 30  0000 C CNN
	1    5850 3000
	0    1    1    0   
$EndComp
$Comp
L GND #PWR011
U 1 1 631A3607
P 6100 3000
F 0 "#PWR011" H 6100 3000 30  0001 C CNN
F 1 "GND" H 6100 2930 30  0001 C CNN
F 2 "" H 6100 3000 60  0001 C CNN
F 3 "" H 6100 3000 60  0001 C CNN
	1    6100 3000
	1    0    0    -1  
$EndComp
Text GLabel 8300 1650 2    60   Output ~ 0
LED
Text GLabel 5200 3000 0    60   Input ~ 0
LED
Text GLabel 7550 4700 2    60   Input ~ 0
RESET
Text GLabel 6000 5100 0    60   Output ~ 0
WAKEUP
Text GLabel 6200 1950 0    60   Input ~ 0
WAKEUP
Text GLabel 6200 2100 0    60   Output ~ 0
RESET
Text Notes 4650 2700 0    60   ~ 0
Wakeup is on an Async interrupt pin\nDo not move.
Text Notes 650  6200 0    60   ~ 0
https://github.com/SpenceKonde/megaTinyCore/blob/e59fc3046dd69f951497506e8f441a6818c28be5/megaavr/extras/PowerSave.md\n\nLDL117 has 250uA Iq and has input of 20V https://www.st.com/resource/en/datasheet/ldl1117.pdf\n
$EndSCHEMATC