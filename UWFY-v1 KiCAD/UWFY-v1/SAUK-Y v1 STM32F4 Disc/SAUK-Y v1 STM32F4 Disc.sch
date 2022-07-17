EESchema Schematic File Version 4
EELAYER 30 0
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
Text Notes 7000 7050 0    50   ~ 0
STM32F4 Discovery Kartı büyüklüğünde PCB parçası bırakılacak.\n6 Adet +5V, GND, PWM olmak üzere üçlü erkek header. Motor Headerı\n2 Adet UART pini + 2 tane kullanıcı UART\n2 Adet I2C pini + 1 tane kullanıcı I2C\nTIM2 ve TIM 3 Motorlar için\n3 tane ADC pin girişi
Text Notes 9350 7050 0    50   ~ 0
TIM2, TIM3, TIM4, TIM5, TIM9, TIM12 → GPTimers\nTIM1, TIM8 → ACTimer
Wire Notes Line
	3770 470  3770 7800
$Comp
L Connector:Conn_01x03_Male R_B1
U 1 1 6234782A
P 4000 750
F 0 "R_B1" H 4000 950 50  0000 C CNN
F 1 "SAG_ARKA" H 4500 550 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 4000 750 50  0001 C CNN
F 3 "~" H 4000 750 50  0001 C CNN
	1    4000 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR01
U 1 1 62349833
P 4400 650
F 0 "#PWR01" H 4400 400 50  0001 C CNN
F 1 "GND" V 4405 522 50  0000 R CNN
F 2 "" H 4400 650 50  0001 C CNN
F 3 "" H 4400 650 50  0001 C CNN
	1    4400 650 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 6234AA59
P 4400 750
F 0 "#PWR02" H 4400 600 50  0001 C CNN
F 1 "+5V" V 4415 878 50  0000 L CNN
F 2 "" H 4400 750 50  0001 C CNN
F 3 "" H 4400 750 50  0001 C CNN
	1    4400 750 
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 650  4400 650 
Wire Wire Line
	4400 750  4200 750 
Wire Wire Line
	4200 850  4400 850 
$Comp
L Connector:Conn_01x03_Male R_V1
U 1 1 6234DEBD
P 4000 1250
F 0 "R_V1" H 4000 1450 50  0000 C CNN
F 1 "SAG_DIKEY" H 4500 1050 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 4000 1250 50  0001 C CNN
F 3 "~" H 4000 1250 50  0001 C CNN
	1    4000 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR03
U 1 1 6234DEC3
P 4400 1150
F 0 "#PWR03" H 4400 900 50  0001 C CNN
F 1 "GND" V 4405 1022 50  0000 R CNN
F 2 "" H 4400 1150 50  0001 C CNN
F 3 "" H 4400 1150 50  0001 C CNN
	1    4400 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR04
U 1 1 6234DEC9
P 4400 1250
F 0 "#PWR04" H 4400 1100 50  0001 C CNN
F 1 "+5V" V 4415 1378 50  0000 L CNN
F 2 "" H 4400 1250 50  0001 C CNN
F 3 "" H 4400 1250 50  0001 C CNN
	1    4400 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 1150 4400 1150
Wire Wire Line
	4400 1250 4200 1250
Wire Wire Line
	4200 1350 4400 1350
$Comp
L Connector:Conn_01x03_Male R_F1
U 1 1 6234EEE6
P 4000 1750
F 0 "R_F1" H 4000 1950 50  0000 C CNN
F 1 "SAG_ON" H 4500 1550 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 4000 1750 50  0001 C CNN
F 3 "~" H 4000 1750 50  0001 C CNN
	1    4000 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR05
U 1 1 6234EEEC
P 4400 1650
F 0 "#PWR05" H 4400 1400 50  0001 C CNN
F 1 "GND" V 4405 1522 50  0000 R CNN
F 2 "" H 4400 1650 50  0001 C CNN
F 3 "" H 4400 1650 50  0001 C CNN
	1    4400 1650
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR06
U 1 1 6234EEF2
P 4400 1750
F 0 "#PWR06" H 4400 1600 50  0001 C CNN
F 1 "+5V" V 4415 1878 50  0000 L CNN
F 2 "" H 4400 1750 50  0001 C CNN
F 3 "" H 4400 1750 50  0001 C CNN
	1    4400 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 1650 4400 1650
Wire Wire Line
	4400 1750 4200 1750
Wire Wire Line
	4200 1850 4400 1850
$Comp
L Connector:Conn_01x03_Male L_V1
U 1 1 6236221B
P 5150 750
F 0 "L_V1" H 5150 950 50  0000 C CNN
F 1 "SOL_DIKEY" H 5650 550 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 5150 750 50  0001 C CNN
F 3 "~" H 5150 750 50  0001 C CNN
	1    5150 750 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR023
U 1 1 62362221
P 5550 650
F 0 "#PWR023" H 5550 400 50  0001 C CNN
F 1 "GND" V 5555 522 50  0000 R CNN
F 2 "" H 5550 650 50  0001 C CNN
F 3 "" H 5550 650 50  0001 C CNN
	1    5550 650 
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR024
U 1 1 62362227
P 5550 750
F 0 "#PWR024" H 5550 600 50  0001 C CNN
F 1 "+5V" V 5565 878 50  0000 L CNN
F 2 "" H 5550 750 50  0001 C CNN
F 3 "" H 5550 750 50  0001 C CNN
	1    5550 750 
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 650  5550 650 
Wire Wire Line
	5550 750  5350 750 
Wire Wire Line
	5350 850  5550 850 
$Comp
L Connector:Conn_01x03_Male L_B1
U 1 1 62362231
P 5150 1250
F 0 "L_B1" H 5150 1450 50  0000 C CNN
F 1 "SOL_ARKA" H 5650 1050 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 5150 1250 50  0001 C CNN
F 3 "~" H 5150 1250 50  0001 C CNN
	1    5150 1250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR025
U 1 1 62362237
P 5550 1150
F 0 "#PWR025" H 5550 900 50  0001 C CNN
F 1 "GND" V 5555 1022 50  0000 R CNN
F 2 "" H 5550 1150 50  0001 C CNN
F 3 "" H 5550 1150 50  0001 C CNN
	1    5550 1150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR026
U 1 1 6236223D
P 5550 1250
F 0 "#PWR026" H 5550 1100 50  0001 C CNN
F 1 "+5V" V 5565 1378 50  0000 L CNN
F 2 "" H 5550 1250 50  0001 C CNN
F 3 "" H 5550 1250 50  0001 C CNN
	1    5550 1250
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 1150 5550 1150
Wire Wire Line
	5550 1250 5350 1250
Wire Wire Line
	5350 1350 5550 1350
$Comp
L Connector:Conn_01x03_Male OP_MOT_1
U 1 1 62362246
P 5150 1750
F 0 "OP_MOT_1" H 5150 1950 50  0000 C CNN
F 1 "OPS_MOT1" H 5650 1550 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 5150 1750 50  0001 C CNN
F 3 "~" H 5150 1750 50  0001 C CNN
	1    5150 1750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 6236224C
P 5550 1650
F 0 "#PWR027" H 5550 1400 50  0001 C CNN
F 1 "GND" V 5555 1522 50  0000 R CNN
F 2 "" H 5550 1650 50  0001 C CNN
F 3 "" H 5550 1650 50  0001 C CNN
	1    5550 1650
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR028
U 1 1 62362252
P 5550 1750
F 0 "#PWR028" H 5550 1600 50  0001 C CNN
F 1 "+5V" V 5565 1878 50  0000 L CNN
F 2 "" H 5550 1750 50  0001 C CNN
F 3 "" H 5550 1750 50  0001 C CNN
	1    5550 1750
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 1650 5550 1650
Wire Wire Line
	5550 1750 5350 1750
Wire Wire Line
	5350 1850 5550 1850
$Comp
L Connector:Conn_01x03_Male L_F1
U 1 1 62364DC4
P 4000 2250
F 0 "L_F1" H 4000 2450 50  0000 C CNN
F 1 "SOL_ON" H 4500 2050 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S3B-EH_1x03_P2.50mm_Horizontal" H 4000 2250 50  0001 C CNN
F 3 "~" H 4000 2250 50  0001 C CNN
	1    4000 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR07
U 1 1 62364DCA
P 4400 2150
F 0 "#PWR07" H 4400 1900 50  0001 C CNN
F 1 "GND" V 4405 2022 50  0000 R CNN
F 2 "" H 4400 2150 50  0001 C CNN
F 3 "" H 4400 2150 50  0001 C CNN
	1    4400 2150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR08
U 1 1 62364DD0
P 4400 2250
F 0 "#PWR08" H 4400 2100 50  0001 C CNN
F 1 "+5V" V 4415 2378 50  0000 L CNN
F 2 "" H 4400 2250 50  0001 C CNN
F 3 "" H 4400 2250 50  0001 C CNN
	1    4400 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 2150 4400 2150
Wire Wire Line
	4400 2250 4200 2250
Wire Wire Line
	4200 2350 4400 2350
$Comp
L Connector:Conn_01x03_Male OP_MOT_2
U 1 1 623678DD
P 5150 2250
F 0 "OP_MOT_2" H 5150 2450 50  0000 C CNN
F 1 "OPS_MOT2" H 5650 2050 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B3B-EH-A_1x03_P2.50mm_Vertical" H 5150 2250 50  0001 C CNN
F 3 "~" H 5150 2250 50  0001 C CNN
	1    5150 2250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR029
U 1 1 623678E3
P 5550 2150
F 0 "#PWR029" H 5550 1900 50  0001 C CNN
F 1 "GND" V 5555 2022 50  0000 R CNN
F 2 "" H 5550 2150 50  0001 C CNN
F 3 "" H 5550 2150 50  0001 C CNN
	1    5550 2150
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR030
U 1 1 623678E9
P 5550 2250
F 0 "#PWR030" H 5550 2100 50  0001 C CNN
F 1 "+5V" V 5565 2378 50  0000 L CNN
F 2 "" H 5550 2250 50  0001 C CNN
F 3 "" H 5550 2250 50  0001 C CNN
	1    5550 2250
	0    1    1    0   
$EndComp
Wire Wire Line
	5350 2150 5550 2150
Wire Wire Line
	5550 2250 5350 2250
Wire Wire Line
	5350 2350 5550 2350
Text Notes 3810 2760 0    39   ~ 0
6+2 Motor Connections\n6 for our design\n2 for optional design
Text Notes 5320 2780 0    39   ~ 0
*TIM3_CH3 & TIM3_CH4 optional 
$Comp
L Connector:Conn_01x04_Male A02YYUW-1
U 1 1 6236BBB0
P 4000 3050
F 0 "A02YYUW-1" H 4000 3250 50  0000 C CNN
F 1 "A02YYUW-1" H 4500 2750 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S4B-EH_1x04_P2.50mm_Horizontal" H 4000 3050 50  0001 C CNN
F 3 "~" H 4000 3050 50  0001 C CNN
	1    4000 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR09
U 1 1 6236C3F2
P 4400 2950
F 0 "#PWR09" H 4400 2800 50  0001 C CNN
F 1 "+5V" V 4415 3078 50  0000 L CNN
F 2 "" H 4400 2950 50  0001 C CNN
F 3 "" H 4400 2950 50  0001 C CNN
	1    4400 2950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR010
U 1 1 6236CA07
P 4400 3050
F 0 "#PWR010" H 4400 2800 50  0001 C CNN
F 1 "GND" V 4405 2922 50  0000 R CNN
F 2 "" H 4400 3050 50  0001 C CNN
F 3 "" H 4400 3050 50  0001 C CNN
	1    4400 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4200 2950 4400 2950
Wire Wire Line
	4200 3050 4400 3050
Wire Wire Line
	4200 3150 4400 3150
Wire Wire Line
	4200 3250 4400 3250
Text Notes 3820 4550 0    39   ~ 0
2+2 USART Connections\nUSART1 → A02YYUW\nUSART2 → USB-TTL\nUSART3 → Optional\nUSART6 → Optional
$Comp
L Connector:Conn_01x04_Male A02YYUW-2
U 1 1 6237F8F1
P 5200 3050
F 0 "A02YYUW-2" H 5200 3250 50  0000 C CNN
F 1 "A02YYUW-2" H 5700 2750 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S4B-EH_1x04_P2.50mm_Horizontal" H 5200 3050 50  0001 C CNN
F 3 "~" H 5200 3050 50  0001 C CNN
	1    5200 3050
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR011
U 1 1 62381A54
P 5600 2950
F 0 "#PWR011" H 5600 2800 50  0001 C CNN
F 1 "+5V" V 5615 3078 50  0000 L CNN
F 2 "" H 5600 2950 50  0001 C CNN
F 3 "" H 5600 2950 50  0001 C CNN
	1    5600 2950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR012
U 1 1 62381E2C
P 5600 3050
F 0 "#PWR012" H 5600 2800 50  0001 C CNN
F 1 "GND" V 5605 2922 50  0000 R CNN
F 2 "" H 5600 3050 50  0001 C CNN
F 3 "" H 5600 3050 50  0001 C CNN
	1    5600 3050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 2950 5600 2950
Wire Wire Line
	5400 3050 5600 3050
Wire Wire Line
	5400 3150 5600 3150
Wire Wire Line
	5400 3250 5600 3250
$Comp
L Connector:Conn_01x04_Male A02YYUW-3
U 1 1 6235600C
P 4000 3600
F 0 "A02YYUW-3" H 4000 3800 50  0000 C CNN
F 1 "A02YYUW-3" H 4500 3300 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S4B-EH_1x04_P2.50mm_Horizontal" H 4000 3600 50  0001 C CNN
F 3 "~" H 4000 3600 50  0001 C CNN
	1    4000 3600
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR031
U 1 1 62356012
P 4400 3500
F 0 "#PWR031" H 4400 3350 50  0001 C CNN
F 1 "+5V" V 4415 3628 50  0000 L CNN
F 2 "" H 4400 3500 50  0001 C CNN
F 3 "" H 4400 3500 50  0001 C CNN
	1    4400 3500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR032
U 1 1 62356018
P 4400 3600
F 0 "#PWR032" H 4400 3350 50  0001 C CNN
F 1 "GND" V 4405 3472 50  0000 R CNN
F 2 "" H 4400 3600 50  0001 C CNN
F 3 "" H 4400 3600 50  0001 C CNN
	1    4400 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4200 3500 4400 3500
Wire Wire Line
	4200 3600 4400 3600
Wire Wire Line
	4200 3700 4400 3700
Wire Wire Line
	4200 3800 4400 3800
Wire Notes Line
	6300 4600 3770 4600
Wire Notes Line
	6300 970  6300 8300
$Comp
L Connector:Conn_01x04_Male M300
U 1 1 6237B15B
P 5200 4850
F 0 "M300" H 5200 5050 50  0000 C CNN
F 1 "M300" H 5700 4550 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S4B-EH_1x04_P2.50mm_Horizontal" H 5200 4850 50  0001 C CNN
F 3 "~" H 5200 4850 50  0001 C CNN
	1    5200 4850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR033
U 1 1 6237B161
P 5600 4750
F 0 "#PWR033" H 5600 4600 50  0001 C CNN
F 1 "+5V" V 5615 4878 50  0000 L CNN
F 2 "" H 5600 4750 50  0001 C CNN
F 3 "" H 5600 4750 50  0001 C CNN
	1    5600 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 4750 5600 4750
Wire Wire Line
	5400 4850 5600 4850
Wire Wire Line
	5400 4950 5600 4950
Wire Wire Line
	5400 5050 5600 5050
Text GLabel 5600 5350 2    50   Input ~ 0
I2C1_SCL
Text GLabel 5600 5450 2    50   Input ~ 0
I2C1_SDA
Text GLabel 4450 5050 2    50   Input ~ 0
I2C2_SDA
Text GLabel 4450 4950 2    50   Input ~ 0
I2C2_SCL
$Comp
L power:GND #PWR034
U 1 1 6237DF97
P 5600 5050
F 0 "#PWR034" H 5600 4800 50  0001 C CNN
F 1 "GND" V 5605 4922 50  0000 R CNN
F 2 "" H 5600 5050 50  0001 C CNN
F 3 "" H 5600 5050 50  0001 C CNN
	1    5600 5050
	0    -1   -1   0   
$EndComp
$Comp
L Connector:Conn_01x04_Male OPS-I2C1
U 1 1 6237E19E
P 5200 5350
F 0 "OPS-I2C1" H 5200 5550 50  0000 C CNN
F 1 "OPS_I2C" H 5700 5050 50  0000 C CNN
F 2 "Connector_JST:JST_EH_B4B-EH-A_1x04_P2.50mm_Vertical" H 5200 5350 50  0001 C CNN
F 3 "~" H 5200 5350 50  0001 C CNN
	1    5200 5350
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR015
U 1 1 623804CA
P 5600 5250
F 0 "#PWR015" H 5600 5100 50  0001 C CNN
F 1 "+5V" V 5615 5378 50  0000 L CNN
F 2 "" H 5600 5250 50  0001 C CNN
F 3 "" H 5600 5250 50  0001 C CNN
	1    5600 5250
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR016
U 1 1 623808C5
P 5600 5550
F 0 "#PWR016" H 5600 5300 50  0001 C CNN
F 1 "GND" V 5605 5422 50  0000 R CNN
F 2 "" H 5600 5550 50  0001 C CNN
F 3 "" H 5600 5550 50  0001 C CNN
	1    5600 5550
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 5250 5600 5250
Wire Wire Line
	5600 5350 5400 5350
Wire Wire Line
	5400 5450 5600 5450
Wire Wire Line
	5600 5550 5400 5550
Text Notes 5200 6000 0    50   ~ 0
2+1 I2C Connections\nI2C1 → MPU6050\nI2C2 → M300\nI2C3 → Optional
Wire Wire Line
	7000 850  7300 850 
Wire Wire Line
	7800 850  8000 850 
NoConn ~ 7300 950 
NoConn ~ 7800 950 
Wire Wire Line
	7000 1050 7300 1050
NoConn ~ 7800 1050
NoConn ~ 7800 1550
NoConn ~ 7800 1750
NoConn ~ 7300 1750
NoConn ~ 7800 1950
NoConn ~ 7300 2050
NoConn ~ 7800 2050
NoConn ~ 7300 2150
NoConn ~ 7800 2150
NoConn ~ 7300 2250
NoConn ~ 7800 2250
NoConn ~ 7300 2350
NoConn ~ 7800 2350
NoConn ~ 7300 2450
NoConn ~ 7800 2550
NoConn ~ 7300 2650
NoConn ~ 7800 2650
NoConn ~ 7300 2750
NoConn ~ 7800 2850
NoConn ~ 7300 2950
NoConn ~ 7800 3150
$Comp
L power:+3V0 #PWR053
U 1 1 624077F0
P 10350 1050
F 0 "#PWR053" H 10350 900 50  0001 C CNN
F 1 "+3V0" V 10365 1178 50  0000 L CNN
F 2 "" H 10350 1050 50  0001 C CNN
F 3 "" H 10350 1050 50  0001 C CNN
	1    10350 1050
	0    1    1    0   
$EndComp
$Comp
L power:+3V0 #PWR049
U 1 1 62407E48
P 9450 1050
F 0 "#PWR049" H 9450 900 50  0001 C CNN
F 1 "+3V0" V 9465 1178 50  0000 L CNN
F 2 "" H 9450 1050 50  0001 C CNN
F 3 "" H 9450 1050 50  0001 C CNN
	1    9450 1050
	0    -1   -1   0   
$EndComp
NoConn ~ 10150 1150
NoConn ~ 9650 1150
NoConn ~ 9650 1250
NoConn ~ 10150 1250
NoConn ~ 10150 1350
NoConn ~ 9650 1450
NoConn ~ 10150 1550
NoConn ~ 9650 1550
NoConn ~ 10150 1650
NoConn ~ 9650 1650
NoConn ~ 10150 1750
NoConn ~ 9650 1750
NoConn ~ 10150 1850
NoConn ~ 9650 1850
NoConn ~ 10150 2050
NoConn ~ 9650 2050
NoConn ~ 10150 2150
NoConn ~ 9650 2150
NoConn ~ 10150 2350
NoConn ~ 9650 2350
NoConn ~ 9650 2450
NoConn ~ 10150 2550
NoConn ~ 9650 2750
NoConn ~ 10150 2750
NoConn ~ 10150 2850
NoConn ~ 9650 3050
Wire Wire Line
	7000 1950 7300 1950
Wire Wire Line
	7000 3250 7300 3250
Wire Wire Line
	7800 3250 8050 3250
Wire Wire Line
	9450 850  9650 850 
Wire Wire Line
	10150 850  10350 850 
Wire Wire Line
	10150 950  10350 950 
Wire Wire Line
	9650 950  9450 950 
Wire Wire Line
	9450 1050 9650 1050
Wire Wire Line
	10150 1050 10350 1050
Wire Wire Line
	9450 3250 9650 3250
Wire Wire Line
	10150 3250 10350 3250
$Comp
L Connector_Generic:Conn_02x25_Odd_Even D-SOL1
U 1 1 6262FCAC
P 7500 2050
F 0 "D-SOL1" H 7550 3467 50  0000 C CNN
F 1 "SOL" H 7550 3376 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x25_P2.54mm_Vertical" H 7500 2050 50  0001 C CNN
F 3 "~" H 7500 2050 50  0001 C CNN
	1    7500 2050
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_02x25_Odd_Even D-SAG1
U 1 1 62633E2E
P 9850 2050
F 0 "D-SAG1" H 9900 3467 50  0000 C CNN
F 1 "SAG" H 9900 3376 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_2x25_P2.54mm_Vertical" H 9850 2050 50  0001 C CNN
F 3 "~" H 9850 2050 50  0001 C CNN
	1    9850 2050
	1    0    0    -1  
$EndComp
NoConn ~ 7300 1150
NoConn ~ 7300 1250
NoConn ~ 7800 1250
NoConn ~ 7800 1150
$Comp
L MCU_ST_STM32F4:STM32F407VGTx U1
U 1 1 6247301D
P 2000 3300
F 0 "U1" H 2000 411 50  0000 C CNN
F 1 "STM32F407VGTx" H 2000 320 50  0000 C CNN
F 2 "Package_QFP:LQFP-100_14x14mm_P0.5mm" H 1300 700 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00037051.pdf" H 2000 3300 50  0001 C CNN
	1    2000 3300
	1    0    0    -1  
$EndComp
NoConn ~ 1700 600 
NoConn ~ 1800 600 
NoConn ~ 1900 600 
NoConn ~ 2000 600 
NoConn ~ 2100 600 
NoConn ~ 2200 600 
NoConn ~ 2300 600 
NoConn ~ 2400 600 
NoConn ~ 1100 900 
NoConn ~ 1100 1100
NoConn ~ 1100 1400
NoConn ~ 1100 1300
NoConn ~ 1100 1500
NoConn ~ 1100 2300
NoConn ~ 1100 2400
NoConn ~ 1800 6100
NoConn ~ 1900 6100
NoConn ~ 2000 6100
NoConn ~ 2100 6100
NoConn ~ 2200 6100
Wire Wire Line
	1100 4800 900  4800
Wire Wire Line
	900  4900 1100 4900
Wire Wire Line
	1100 5100 900  5100
Wire Wire Line
	900  5200 1100 5200
Wire Wire Line
	2900 5200 3100 5200
Wire Wire Line
	3100 3700 2900 3700
Wire Wire Line
	2900 3600 3100 3600
Wire Wire Line
	3100 3300 2900 3300
Wire Wire Line
	2900 3200 3100 3200
Wire Wire Line
	3100 2700 2900 2700
Wire Wire Line
	2900 2600 3100 2600
Wire Wire Line
	3100 1900 2900 1900
Wire Wire Line
	2900 1800 3100 1800
Wire Wire Line
	3100 1700 2900 1700
Wire Wire Line
	2900 1600 3100 1600
Wire Wire Line
	3100 1500 2900 1500
Wire Wire Line
	3100 1000 2900 1000
NoConn ~ 2900 1300
NoConn ~ 2900 2000
NoConn ~ 2900 2100
NoConn ~ 2900 2200
NoConn ~ 2900 2300
NoConn ~ 2900 2400
NoConn ~ 2900 2800
NoConn ~ 2900 2900
NoConn ~ 2900 3000
NoConn ~ 2900 3100
NoConn ~ 2900 3400
NoConn ~ 2900 3500
NoConn ~ 2900 3800
NoConn ~ 2900 3900
NoConn ~ 2900 4000
NoConn ~ 2900 4100
NoConn ~ 2900 4700
NoConn ~ 2900 4800
NoConn ~ 2900 5100
NoConn ~ 2900 5600
NoConn ~ 2900 5700
NoConn ~ 2900 5800
NoConn ~ 1100 5400
NoConn ~ 1100 5300
NoConn ~ 1100 5000
NoConn ~ 1100 4700
NoConn ~ 1100 4600
NoConn ~ 1100 4400
NoConn ~ 1100 4300
NoConn ~ 1100 4100
NoConn ~ 1100 4000
NoConn ~ 1100 3900
NoConn ~ 1100 3800
NoConn ~ 1100 3700
NoConn ~ 1100 3600
NoConn ~ 1100 3500
NoConn ~ 1100 3400
NoConn ~ 1100 3300
NoConn ~ 1100 3000
NoConn ~ 1100 2900
NoConn ~ 1100 2800
NoConn ~ 1100 2700
NoConn ~ 1100 2600
NoConn ~ 2900 4600
NoConn ~ 2900 4500
NoConn ~ 2900 4400
NoConn ~ 2900 4300
Text Notes 6550 3600 0    118  ~ 0
STM32F4 Discovery Kartı 100 pin giriş çıkışı
Wire Notes Line
	6300 3650 11250 3650
$Comp
L Connector:Screw_Terminal_01x02 PWR1
U 1 1 62571F7F
P 4000 6550
F 0 "PWR1" H 4000 6650 50  0000 C CNN
F 1 "GDK_Güc_Girisleri" H 3350 6350 50  0000 C CNN
F 2 "TerminalBlock_MetzConnect:TerminalBlock_MetzConnect_Type011_RT05502HBWC_1x02_P5.00mm_Horizontal" H 4000 6550 50  0001 C CNN
F 3 "~" H 4000 6550 50  0001 C CNN
	1    4000 6550
	-1   0    0    -1  
$EndComp
Text GLabel 4400 6550 2    50   Input ~ 0
+5V_GDK_giris
Text GLabel 4400 6650 2    50   Input ~ 0
GND_GDK_giris
Text GLabel 10350 950  2    50   Input ~ 0
+5V_GDK_giris
Text GLabel 10350 850  2    50   Input ~ 0
GND_GDK_giris
Text GLabel 9450 950  0    50   Input ~ 0
+5V_GDK_giris
Text GLabel 9450 850  0    50   Input ~ 0
GND_GDK_giris
Text GLabel 10350 3250 2    50   Input ~ 0
GND_GDK_giris
Text GLabel 9450 3250 0    50   Input ~ 0
GND_GDK_giris
Text GLabel 8050 3250 2    50   Input ~ 0
GND_GDK_giris
Text GLabel 7000 3250 0    50   Input ~ 0
GND_GDK_giris
Text GLabel 7000 1950 0    50   Input ~ 0
GND_GDK_giris
Text GLabel 7000 1050 0    50   Input ~ 0
GND_GDK_giris
Text GLabel 7000 850  0    50   Input ~ 0
GND_GDK_giris
Text GLabel 8000 850  2    50   Input ~ 0
GND_GDK_giris
Wire Notes Line
	6300 955  6300 470 
Wire Notes Line
	6300 6005 3770 6005
Text Notes 3800 6200 0    79   ~ 0
STM32F4 Discovery Kartı Güç Beslemesi\n
Wire Wire Line
	4400 6550 4200 6550
Wire Wire Line
	4200 6650 4400 6650
Text GLabel 5700 6550 0    50   Input ~ 0
+5V_GDK_giris
Text GLabel 5700 6650 0    50   Input ~ 0
GND_GDK_giris
$Comp
L power:+5V #PWR017
U 1 1 625D8393
P 5900 6550
F 0 "#PWR017" H 5900 6400 50  0001 C CNN
F 1 "+5V" V 5915 6678 50  0000 L CNN
F 2 "" H 5900 6550 50  0001 C CNN
F 3 "" H 5900 6550 50  0001 C CNN
	1    5900 6550
	0    1    1    0   
$EndComp
Wire Wire Line
	5900 6550 5700 6550
$Comp
L power:GND #PWR018
U 1 1 625E0902
P 5900 6650
F 0 "#PWR018" H 5900 6400 50  0001 C CNN
F 1 "GND" H 5905 6477 50  0000 C CNN
F 2 "" H 5900 6650 50  0001 C CNN
F 3 "" H 5900 6650 50  0001 C CNN
	1    5900 6650
	1    0    0    -1  
$EndComp
Wire Wire Line
	5900 6650 5700 6650
$Comp
L Connector:Conn_01x08_Female MPU1
U 1 1 625F2324
P 4000 5050
F 0 "MPU1" H 3950 5450 50  0000 L CNN
F 1 "MPU6050" H 3250 4500 50  0000 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Horizontal" H 4000 5050 50  0001 C CNN
F 3 "~" H 4000 5050 50  0001 C CNN
	1    4000 5050
	-1   0    0    -1  
$EndComp
$Comp
L power:+5V #PWR013
U 1 1 6260C006
P 4450 4750
F 0 "#PWR013" H 4450 4600 50  0001 C CNN
F 1 "+5V" V 4465 4878 50  0000 L CNN
F 2 "" H 4450 4750 50  0001 C CNN
F 3 "" H 4450 4750 50  0001 C CNN
	1    4450 4750
	0    1    1    0   
$EndComp
Wire Wire Line
	4450 4750 4200 4750
$Comp
L power:GND #PWR014
U 1 1 626149C4
P 4450 4850
F 0 "#PWR014" H 4450 4600 50  0001 C CNN
F 1 "GND" V 4455 4722 50  0000 R CNN
F 2 "" H 4450 4850 50  0001 C CNN
F 3 "" H 4450 4850 50  0001 C CNN
	1    4450 4850
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4450 4850 4200 4850
Text GLabel 5600 4850 2    50   Input ~ 0
I2C3_SCL
Wire Wire Line
	4450 4950 4200 4950
Text GLabel 5600 4950 2    50   Input ~ 0
I2C3_SDA
Wire Wire Line
	4450 5050 4200 5050
Text GLabel 3100 900  2    50   Input ~ 0
UART4_TX
Text GLabel 3100 1000 2    50   Input ~ 0
UART4_RX
Wire Wire Line
	2900 900  3100 900 
NoConn ~ 2900 1100
NoConn ~ 2900 1200
NoConn ~ 2900 1400
Text GLabel 3100 1500 2    50   Input ~ 0
TIM3_CH1
Text GLabel 3100 1600 2    50   Input ~ 0
TIM3_CH2
Text GLabel 3100 1700 2    50   Input ~ 0
I2C3_SCL
Text GLabel 3100 1800 2    50   Input ~ 0
USART1_TX
Text GLabel 3100 1900 2    50   Input ~ 0
USART1_RX
Text GLabel 3100 2700 2    50   Input ~ 0
TIM3_CH4
Text GLabel 3100 3200 2    50   Input ~ 0
I2C1_SCL
Text GLabel 3100 3300 2    50   Input ~ 0
I2C1_SDA
Text GLabel 3100 3600 2    50   Input ~ 0
I2C2_SCL
Text GLabel 3100 3700 2    50   Input ~ 0
I2C2_SDA
Text GLabel 3100 5200 2    50   Input ~ 0
I2C3_SDA
Text GLabel 3100 5500 2    50   Input ~ 0
UART5_TX
Wire Wire Line
	3100 5500 2900 5500
NoConn ~ 2900 4900
NoConn ~ 2900 5000
Text GLabel 3100 5300 2    50   Input ~ 0
USART3_TX
Text GLabel 3100 5400 2    50   Input ~ 0
USART3_RX
Text GLabel 900  5500 0    50   Input ~ 0
TIM4_CH1
Wire Wire Line
	900  5500 1100 5500
Text GLabel 900  5600 0    50   Input ~ 0
TIM4_CH2
Text GLabel 900  5700 0    50   Input ~ 0
TIM4_CH3
Text GLabel 900  5800 0    50   Input ~ 0
TIM4_CH4
Wire Wire Line
	900  5800 1100 5800
Wire Wire Line
	1100 5700 900  5700
Wire Wire Line
	900  5600 1100 5600
Text GLabel 900  4500 0    50   Input ~ 0
UART5_RX
Text GLabel 900  4900 0    50   Input ~ 0
USART2_RX
Text GLabel 900  4800 0    50   Input ~ 0
USART2_TX
NoConn ~ 1100 3100
NoConn ~ 1100 3200
Text GLabel 5550 1350 2    50   Input ~ 0
TIM4_CH4
Text GLabel 5550 850  2    50   Input ~ 0
TIM4_CH3
Text GLabel 4400 2350 2    50   Input ~ 0
TIM4_CH2
Text GLabel 4400 1850 2    50   Input ~ 0
TIM4_CH1
Text GLabel 5550 1850 2    50   Input ~ 0
TIM3_CH1
Text GLabel 5550 2350 2    50   Input ~ 0
TIM3_CH2
Text GLabel 4400 850  2    50   Input ~ 0
TIM3_CH3
Text GLabel 4400 1350 2    50   Input ~ 0
TIM3_CH4
$Comp
L Connector:Conn_01x04_Male TTL1
U 1 1 624FFF68
P 5200 4200
F 0 "TTL1" H 5200 4400 50  0000 C CNN
F 1 "USB_TTL" H 5700 3900 50  0000 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x04_P2.54mm_Vertical" H 5200 4200 50  0001 C CNN
F 3 "~" H 5200 4200 50  0001 C CNN
	1    5200 4200
	1    0    0    -1  
$EndComp
Wire Wire Line
	5400 4300 5600 4300
Wire Wire Line
	5400 4200 5600 4200
Wire Notes Line
	6300 2800 3770 2800
$Comp
L Connector:Conn_01x04_Male A02YYUW-4
U 1 1 6298860A
P 5200 3600
F 0 "A02YYUW-4" H 5200 3800 50  0000 C CNN
F 1 "A02YYUW-3" H 5700 3300 50  0000 C CNN
F 2 "Connector_JST:JST_EH_S4B-EH_1x04_P2.50mm_Horizontal" H 5200 3600 50  0001 C CNN
F 3 "~" H 5200 3600 50  0001 C CNN
	1    5200 3600
	1    0    0    -1  
$EndComp
Text GLabel 5600 4200 2    50   Input ~ 0
USART1_TX
Text GLabel 5600 4300 2    50   Input ~ 0
USART1_RX
Text GLabel 4400 3150 2    50   Input ~ 0
USART2_RX
Text GLabel 4400 3250 2    50   Input ~ 0
USART2_TX
Text GLabel 5600 3150 2    50   Input ~ 0
USART3_RX
Text GLabel 5600 3250 2    50   Input ~ 0
USART3_TX
Text GLabel 4400 3700 2    50   Input ~ 0
UART4_RX
Text GLabel 4400 3800 2    50   Input ~ 0
UART4_TX
$Comp
L power:+5V #PWR021
U 1 1 62989EE3
P 5600 3500
F 0 "#PWR021" H 5600 3350 50  0001 C CNN
F 1 "+5V" V 5615 3628 50  0000 L CNN
F 2 "" H 5600 3500 50  0001 C CNN
F 3 "" H 5600 3500 50  0001 C CNN
	1    5600 3500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR022
U 1 1 62989EE9
P 5600 3600
F 0 "#PWR022" H 5600 3350 50  0001 C CNN
F 1 "GND" V 5605 3472 50  0000 R CNN
F 2 "" H 5600 3600 50  0001 C CNN
F 3 "" H 5600 3600 50  0001 C CNN
	1    5600 3600
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 3500 5600 3500
Wire Wire Line
	5400 3600 5600 3600
Text GLabel 5600 3700 2    50   Input ~ 0
UART5_RX
Text GLabel 5600 3800 2    50   Input ~ 0
UART5_TX
Wire Wire Line
	5600 3800 5400 3800
Wire Wire Line
	5400 3700 5600 3700
$Comp
L power:GND #PWR019
U 1 1 629DCC73
P 4450 5350
F 0 "#PWR019" H 4450 5100 50  0001 C CNN
F 1 "GND" V 4455 5222 50  0000 R CNN
F 2 "" H 4450 5350 50  0001 C CNN
F 3 "" H 4450 5350 50  0001 C CNN
	1    4450 5350
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR020
U 1 1 629DCF15
P 4450 5450
F 0 "#PWR020" H 4450 5300 50  0001 C CNN
F 1 "+5V" V 4465 5578 50  0000 L CNN
F 2 "" H 4450 5450 50  0001 C CNN
F 3 "" H 4450 5450 50  0001 C CNN
	1    4450 5450
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 5150 4450 5150
Wire Wire Line
	4450 5250 4200 5250
Wire Wire Line
	4200 5350 4450 5350
Wire Wire Line
	4450 5450 4200 5450
Text GLabel 8000 1350 2    50   Input ~ 0
UART4_TX
Text GLabel 7000 1350 0    50   Input ~ 0
UART4_RX
Wire Wire Line
	7300 1350 7000 1350
Wire Wire Line
	7800 1350 8000 1350
Text GLabel 8000 1650 2    50   Input ~ 0
TIM3_CH1
Text GLabel 7000 1650 0    50   Input ~ 0
TIM3_CH2
Wire Wire Line
	7300 1650 7000 1650
Wire Wire Line
	7800 1650 8000 1650
NoConn ~ 7300 1450
NoConn ~ 7300 1550
NoConn ~ 7800 1450
Text GLabel 3100 2600 2    50   Input ~ 0
TIM3_CH3
Text GLabel 8000 1850 2    50   Input ~ 0
TIM3_CH3
Text GLabel 7000 1850 0    50   Input ~ 0
TIM3_CH4
Wire Wire Line
	8000 1850 7800 1850
Wire Wire Line
	7300 1850 7000 1850
Text GLabel 8000 2450 2    50   Input ~ 0
I2C2_SCL
Wire Wire Line
	8000 2450 7800 2450
Text GLabel 7000 2550 0    50   Input ~ 0
I2C2_SDA
Wire Wire Line
	7000 2550 7300 2550
NoConn ~ 900  5200
NoConn ~ 900  5100
Wire Wire Line
	2900 5300 3100 5300
Wire Wire Line
	3100 5400 2900 5400
NoConn ~ 7300 2850
NoConn ~ 7800 2750
Text GLabel 8000 2950 2    50   Input ~ 0
TIM4_CH1
Text GLabel 7000 3050 0    50   Input ~ 0
TIM4_CH2
Text GLabel 8000 3050 2    50   Input ~ 0
TIM4_CH3
Text GLabel 7000 3150 0    50   Input ~ 0
TIM4_CH4
Wire Wire Line
	7300 3050 7000 3050
Wire Wire Line
	7000 3150 7300 3150
Wire Wire Line
	7800 3050 8000 3050
Wire Wire Line
	7800 2950 8000 2950
Text GLabel 9450 1950 0    50   Input ~ 0
I2C1_SCL
Text GLabel 10350 1950 2    50   Input ~ 0
I2C1_SDA
Wire Wire Line
	9450 1950 9650 1950
Wire Wire Line
	10150 1950 10350 1950
NoConn ~ 10150 1450
NoConn ~ 9650 1350
Text GLabel 10350 2250 2    50   Input ~ 0
USART2_RX
Text GLabel 9450 2250 0    50   Input ~ 0
USART2_TX
Wire Wire Line
	10350 2250 10150 2250
Wire Wire Line
	9450 2250 9650 2250
Wire Wire Line
	1100 4500 900  4500
Text GLabel 10350 2450 2    50   Input ~ 0
UART5_RX
Wire Wire Line
	10350 2450 10150 2450
Text GLabel 9450 2550 0    50   Input ~ 0
UART5_TX
Wire Wire Line
	9650 2550 9450 2550
Text GLabel 9450 2650 0    50   Input ~ 0
USART3_TX
Text GLabel 10350 2650 2    50   Input ~ 0
USART3_RX
Wire Wire Line
	10150 2650 10350 2650
Wire Wire Line
	9650 2650 9450 2650
Text GLabel 9450 2850 0    50   Input ~ 0
USART1_RX
Wire Wire Line
	9450 2850 9650 2850
Text GLabel 9450 2950 0    50   Input ~ 0
I2C3_SCL
Text GLabel 10350 2950 2    50   Input ~ 0
USART1_TX
Wire Wire Line
	10150 2950 10350 2950
Wire Wire Line
	9450 2950 9650 2950
Text GLabel 10350 3050 2    50   Input ~ 0
I2C3_SDA
Wire Wire Line
	10350 3050 10150 3050
NoConn ~ 9650 3150
NoConn ~ 10150 3150
Text GLabel 4450 5150 2    50   Input ~ 0
I2C2_SDA
Text GLabel 4450 5250 2    50   Input ~ 0
I2C2_SCL
$Comp
L power:GND #PWR035
U 1 1 62C85C4B
P 5600 4100
F 0 "#PWR035" H 5600 3850 50  0001 C CNN
F 1 "GND" V 5605 3972 50  0000 R CNN
F 2 "" H 5600 4100 50  0001 C CNN
F 3 "" H 5600 4100 50  0001 C CNN
	1    5600 4100
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR036
U 1 1 62C86312
P 5600 4400
F 0 "#PWR036" H 5600 4150 50  0001 C CNN
F 1 "GND" V 5605 4272 50  0000 R CNN
F 2 "" H 5600 4400 50  0001 C CNN
F 3 "" H 5600 4400 50  0001 C CNN
	1    5600 4400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	5400 4100 5600 4100
Wire Wire Line
	5400 4400 5600 4400
$EndSCHEMATC
