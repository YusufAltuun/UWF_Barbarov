EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 8600 750  0    50   Input ~ 0
VDD_11
Text GLabel 8600 850  0    50   Input ~ 0
VDD_19
Text GLabel 8600 950  0    50   Input ~ 0
VDD_28
Text GLabel 8600 1050 0    50   Input ~ 0
VDD_50
Text GLabel 8600 1150 0    50   Input ~ 0
VDD_75
Text GLabel 8600 1250 0    50   Input ~ 0
VDD_100
Text GLabel 7700 2850 0    50   Input ~ 0
VBAT_6
Text GLabel 8600 2550 0    50   Input ~ 0
VDDA_22
Text GLabel 6550 7050 2    50   Input ~ 0
VSS_10
Text GLabel 6550 7150 2    50   Input ~ 0
VSS_27
Text GLabel 6550 7250 2    50   Input ~ 0
VSS_74
Text GLabel 5600 7700 0    50   Input ~ 0
VSS_99
Text GLabel 6550 6950 2    50   Input ~ 0
VSSA_20
Text GLabel 8600 2250 0    50   Input ~ 0
VREF+_21
Text GLabel 7900 4800 0    50   Input ~ 0
VCAP_1-49
Text GLabel 7900 5450 0    50   Input ~ 0
VCAP_2-73
$Comp
L pspice:INDUCTOR L?
U 1 1 622ECB3F
P 10650 1300
F 0 "L?" H 10400 1400 50  0000 C CNN
F 1 "fcm1608-0603" H 10650 1250 50  0000 C CNN
F 2 "" H 10650 1300 50  0001 C CNN
F 3 "~" H 10650 1300 50  0001 C CNN
	1    10650 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	8600 750  8650 750 
Wire Wire Line
	8600 850  8650 850 
Wire Wire Line
	8650 850  8650 750 
Connection ~ 8650 750 
Wire Wire Line
	8650 750  10900 750 
Wire Wire Line
	8650 850  8650 950 
Wire Wire Line
	8650 950  8600 950 
Connection ~ 8650 850 
Wire Wire Line
	8650 950  8650 1050
Wire Wire Line
	8650 1050 8600 1050
Connection ~ 8650 950 
Wire Wire Line
	8650 1050 8650 1150
Wire Wire Line
	8650 1150 8600 1150
Connection ~ 8650 1050
Wire Wire Line
	8650 1150 8650 1250
Wire Wire Line
	8650 1250 8600 1250
Connection ~ 8650 1150
Wire Wire Line
	10900 750  10900 1300
$Comp
L Device:R R?
U 1 1 622EDD8B
P 10050 1300
F 0 "R?" V 9950 1150 50  0000 C CNN
F 1 "47" V 10150 1300 50  0000 C CNN
F 2 "" V 9980 1300 50  0001 C CNN
F 3 "~" H 10050 1300 50  0001 C CNN
	1    10050 1300
	0    1    1    0   
$EndComp
Wire Wire Line
	10400 1300 10300 1300
Wire Wire Line
	8750 1300 8750 2250
Wire Wire Line
	8750 2250 8600 2250
Wire Wire Line
	8750 1300 9400 1300
$Comp
L pspice:CAP C?
U 1 1 622EEE4B
P 9750 1550
F 0 "C?" H 9600 1650 50  0000 L CNN
F 1 "100uF" H 9800 1400 50  0000 L CNN
F 2 "" H 9750 1550 50  0001 C CNN
F 3 "~" H 9750 1550 50  0001 C CNN
	1    9750 1550
	1    0    0    -1  
$EndComp
Connection ~ 9750 1300
Wire Wire Line
	9750 1300 9900 1300
$Comp
L pspice:CAP C?
U 1 1 622EF635
P 9400 1550
F 0 "C?" H 9250 1650 50  0000 L CNN
F 1 "1uF" H 9450 1400 50  0000 L CNN
F 2 "" H 9400 1550 50  0001 C CNN
F 3 "~" H 9400 1550 50  0001 C CNN
	1    9400 1550
	1    0    0    -1  
$EndComp
Connection ~ 9400 1300
Wire Wire Line
	9400 1300 9750 1300
$Comp
L pspice:CAP C?
U 1 1 622EFE21
P 9750 2250
F 0 "C?" H 9600 2350 50  0000 L CNN
F 1 "CAP" H 9800 2100 50  0000 L CNN
F 2 "" H 9750 2250 50  0001 C CNN
F 3 "~" H 9750 2250 50  0001 C CNN
	1    9750 2250
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F06AE
P 9400 2250
F 0 "C?" H 9250 2350 50  0000 L CNN
F 1 "CAP" H 9450 2100 50  0000 L CNN
F 2 "" H 9400 2250 50  0001 C CNN
F 3 "~" H 9400 2250 50  0001 C CNN
	1    9400 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	9400 1800 9400 1900
Wire Wire Line
	9400 1900 9750 1900
Wire Wire Line
	9750 1900 9750 1800
Wire Wire Line
	9400 2000 9400 1900
Connection ~ 9400 1900
Wire Wire Line
	9750 2000 9750 1900
Connection ~ 9750 1900
$Comp
L power:GND #PWR?
U 1 1 622F1748
P 9950 1900
F 0 "#PWR?" H 9950 1650 50  0001 C CNN
F 1 "GND" V 9955 1772 50  0000 R CNN
F 2 "" H 9950 1900 50  0001 C CNN
F 3 "" H 9950 1900 50  0001 C CNN
	1    9950 1900
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9750 1900 9950 1900
Wire Wire Line
	10300 1300 10300 2550
Wire Wire Line
	10300 2550 9750 2550
Connection ~ 10300 1300
Wire Wire Line
	10300 1300 10200 1300
Wire Wire Line
	9750 2500 9750 2550
Connection ~ 9750 2550
Wire Wire Line
	9750 2550 9400 2550
Wire Wire Line
	9400 2500 9400 2550
Connection ~ 9400 2550
Wire Wire Line
	9400 2550 8600 2550
$Comp
L pspice:CAP C?
U 1 1 622F3318
P 10900 3350
F 0 "C?" H 10750 3450 50  0000 L CNN
F 1 "100uF" H 10950 3200 50  0000 L CNN
F 2 "" H 10900 3350 50  0001 C CNN
F 3 "~" H 10900 3350 50  0001 C CNN
	1    10900 3350
	1    0    0    -1  
$EndComp
Connection ~ 10900 1300
Wire Wire Line
	10900 1300 10900 2850
Wire Wire Line
	10900 2850 10550 2850
Connection ~ 10900 2850
Wire Wire Line
	10900 2850 10900 3100
$Comp
L pspice:CAP C?
U 1 1 622F8EF6
P 10550 3350
F 0 "C?" H 10400 3450 50  0000 L CNN
F 1 "100uF" H 10600 3200 50  0000 L CNN
F 2 "" H 10550 3350 50  0001 C CNN
F 3 "~" H 10550 3350 50  0001 C CNN
	1    10550 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F916E
P 10200 3350
F 0 "C?" H 10050 3450 50  0000 L CNN
F 1 "100uF" H 10250 3200 50  0000 L CNN
F 2 "" H 10200 3350 50  0001 C CNN
F 3 "~" H 10200 3350 50  0001 C CNN
	1    10200 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F954E
P 9850 3350
F 0 "C?" H 9700 3450 50  0000 L CNN
F 1 "100uF" H 9900 3200 50  0000 L CNN
F 2 "" H 9850 3350 50  0001 C CNN
F 3 "~" H 9850 3350 50  0001 C CNN
	1    9850 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F982F
P 9500 3350
F 0 "C?" H 9350 3450 50  0000 L CNN
F 1 "100uF" H 9550 3200 50  0000 L CNN
F 2 "" H 9500 3350 50  0001 C CNN
F 3 "~" H 9500 3350 50  0001 C CNN
	1    9500 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F9B15
P 9150 3350
F 0 "C?" H 9000 3450 50  0000 L CNN
F 1 "100uF" H 9200 3200 50  0000 L CNN
F 2 "" H 9150 3350 50  0001 C CNN
F 3 "~" H 9150 3350 50  0001 C CNN
	1    9150 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F9D15
P 8750 3350
F 0 "C?" H 8600 3450 50  0000 L CNN
F 1 "100uF" H 8800 3200 50  0000 L CNN
F 2 "" H 8750 3350 50  0001 C CNN
F 3 "~" H 8750 3350 50  0001 C CNN
	1    8750 3350
	1    0    0    -1  
$EndComp
$Comp
L pspice:CAP C?
U 1 1 622F9FAB
P 8350 3350
F 0 "C?" H 8200 3450 50  0000 L CNN
F 1 "100uF" H 8400 3200 50  0000 L CNN
F 2 "" H 8350 3350 50  0001 C CNN
F 3 "~" H 8350 3350 50  0001 C CNN
	1    8350 3350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 622FA1B0
P 10900 3850
F 0 "#PWR?" H 10900 3600 50  0001 C CNN
F 1 "GND" H 10905 3677 50  0000 C CNN
F 2 "" H 10900 3850 50  0001 C CNN
F 3 "" H 10900 3850 50  0001 C CNN
	1    10900 3850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 3600 10900 3800
Wire Wire Line
	10550 3600 10550 3800
Wire Wire Line
	10550 3800 10900 3800
Connection ~ 10900 3800
Wire Wire Line
	10900 3800 10900 3850
Wire Wire Line
	10200 3600 10200 3800
Wire Wire Line
	10200 3800 10550 3800
Connection ~ 10550 3800
Wire Wire Line
	9850 3600 9850 3800
Wire Wire Line
	9850 3800 10200 3800
Connection ~ 10200 3800
Wire Wire Line
	9500 3600 9500 3800
Wire Wire Line
	9500 3800 9850 3800
Connection ~ 9850 3800
Wire Wire Line
	9150 3600 9150 3800
Wire Wire Line
	9150 3800 9500 3800
Connection ~ 9500 3800
Wire Wire Line
	8750 3600 8750 3800
Wire Wire Line
	8750 3800 9150 3800
Connection ~ 9150 3800
Wire Wire Line
	8350 3600 8350 3800
Wire Wire Line
	8350 3800 8750 3800
Connection ~ 8750 3800
$Comp
L pspice:CAP C?
U 1 1 622FF09A
P 7950 3350
F 0 "C?" H 7800 3450 50  0000 L CNN
F 1 "1uF" H 8000 3200 50  0000 L CNN
F 2 "" H 7950 3350 50  0001 C CNN
F 3 "~" H 7950 3350 50  0001 C CNN
	1    7950 3350
	1    0    0    -1  
$EndComp
Wire Wire Line
	7950 3600 7950 3800
Wire Wire Line
	7950 3800 8350 3800
Connection ~ 8350 3800
$Comp
L Device:R R?
U 1 1 622FFE48
P 8150 2850
F 0 "R?" V 8050 2700 50  0000 C CNN
F 1 "[N/A]" V 8250 2850 50  0000 C CNN
F 2 "" V 8080 2850 50  0001 C CNN
F 3 "~" H 8150 2850 50  0001 C CNN
	1    8150 2850
	0    1    1    0   
$EndComp
Wire Wire Line
	8000 2850 7950 2850
Wire Wire Line
	7950 3100 7950 2850
Connection ~ 7950 2850
Wire Wire Line
	7950 2850 7700 2850
Wire Wire Line
	8350 3100 8350 2850
Connection ~ 8350 2850
Wire Wire Line
	8350 2850 8300 2850
Wire Wire Line
	8750 2850 8750 3100
Connection ~ 8750 2850
Wire Wire Line
	8750 2850 8350 2850
Wire Wire Line
	9150 2850 9150 3100
Connection ~ 9150 2850
Wire Wire Line
	9150 2850 8750 2850
Wire Wire Line
	9500 2850 9500 3100
Connection ~ 9500 2850
Wire Wire Line
	9500 2850 9150 2850
Wire Wire Line
	9850 2850 9850 3100
Connection ~ 9850 2850
Wire Wire Line
	9850 2850 9500 2850
Wire Wire Line
	10200 2850 10200 3100
Connection ~ 10200 2850
Wire Wire Line
	10200 2850 9850 2850
Wire Wire Line
	10550 2850 10550 3100
Connection ~ 10550 2850
Wire Wire Line
	10550 2850 10200 2850
Wire Notes Line
	11250 4300 6900 4300
Text Notes 6950 4250 0    118  ~ 0
VDD Connections
Wire Notes Line
	6900 450  6900 6550
Text Notes 6950 6500 0    118  ~ 0
Capacitor Connections\n
$Comp
L pspice:CAP C?
U 1 1 62329C68
P 8650 4800
F 0 "C?" V 8965 4800 50  0000 C CNN
F 1 "2.2uF" V 8874 4800 50  0000 C CNN
F 2 "" H 8650 4800 50  0001 C CNN
F 3 "~" H 8650 4800 50  0001 C CNN
	1    8650 4800
	0    -1   -1   0   
$EndComp
$Comp
L pspice:CAP C?
U 1 1 6232DDE4
P 8650 5450
F 0 "C?" V 8965 5450 50  0000 C CNN
F 1 "2.2uF" V 8874 5450 50  0000 C CNN
F 2 "" H 8650 5450 50  0001 C CNN
F 3 "~" H 8650 5450 50  0001 C CNN
	1    8650 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7900 4800 8400 4800
Wire Wire Line
	7900 5450 8400 5450
Wire Wire Line
	8900 4800 9050 4800
Wire Wire Line
	9050 4800 9050 5100
Wire Wire Line
	9050 5450 8900 5450
$Comp
L power:GND #PWR?
U 1 1 62332E45
P 9050 5100
F 0 "#PWR?" H 9050 4850 50  0001 C CNN
F 1 "GND" V 9055 4972 50  0000 R CNN
F 2 "" H 9050 5100 50  0001 C CNN
F 3 "" H 9050 5100 50  0001 C CNN
	1    9050 5100
	0    -1   -1   0   
$EndComp
Connection ~ 9050 5100
Wire Wire Line
	9050 5100 9050 5450
$Comp
L power:GND #PWR?
U 1 1 622FAF2F
P 6400 7550
F 0 "#PWR?" H 6400 7300 50  0001 C CNN
F 1 "GND" H 6405 7377 50  0000 C CNN
F 2 "" H 6400 7550 50  0001 C CNN
F 3 "" H 6400 7550 50  0001 C CNN
	1    6400 7550
	1    0    0    -1  
$EndComp
Wire Wire Line
	6400 7550 6400 7450
Wire Wire Line
	6400 6950 6550 6950
Wire Wire Line
	6400 7050 6550 7050
Connection ~ 6400 7050
Wire Wire Line
	6400 7050 6400 6950
Wire Wire Line
	6400 7150 6550 7150
Connection ~ 6400 7150
Wire Wire Line
	6400 7150 6400 7050
Wire Wire Line
	6400 7250 6550 7250
Connection ~ 6400 7250
Wire Wire Line
	6400 7250 6400 7150
$Comp
L Device:R R?
U 1 1 6230877C
P 5450 7450
F 0 "R?" V 5243 7450 50  0000 C CNN
F 1 "[N/A]" V 5334 7450 50  0000 C CNN
F 2 "" V 5380 7450 50  0001 C CNN
F 3 "~" H 5450 7450 50  0001 C CNN
	1    5450 7450
	0    1    1    0   
$EndComp
$Comp
L Device:R R?
U 1 1 62308C5B
P 6000 7450
F 0 "R?" V 5793 7450 50  0000 C CNN
F 1 "0" V 5884 7450 50  0000 C CNN
F 2 "" V 5930 7450 50  0001 C CNN
F 3 "~" H 6000 7450 50  0001 C CNN
	1    6000 7450
	0    1    1    0   
$EndComp
Wire Wire Line
	5150 7200 5150 7450
Wire Wire Line
	5150 7450 5300 7450
Wire Wire Line
	5600 7450 5750 7450
Wire Wire Line
	6150 7450 6400 7450
Connection ~ 6400 7450
Wire Wire Line
	6400 7450 6400 7250
Wire Wire Line
	5600 7700 5750 7700
Wire Wire Line
	5750 7700 5750 7450
Connection ~ 5750 7450
Wire Wire Line
	5750 7450 5850 7450
$Comp
L power:+3.3V #PWR?
U 1 1 6231B1C3
P 5150 7200
F 0 "#PWR?" H 5150 7050 50  0001 C CNN
F 1 "+3.3V" H 5165 7373 50  0000 C CNN
F 2 "" H 5150 7200 50  0001 C CNN
F 3 "" H 5150 7200 50  0001 C CNN
	1    5150 7200
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 6231C335
P 10900 700
F 0 "#PWR?" H 10900 550 50  0001 C CNN
F 1 "+3.3V" H 10915 873 50  0000 C CNN
F 2 "" H 10900 700 50  0001 C CNN
F 3 "" H 10900 700 50  0001 C CNN
	1    10900 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 700  10900 750 
Connection ~ 10900 750 
Wire Notes Line
	4550 6550 4550 7800
Wire Notes Line
	4550 7800 4600 7800
Wire Notes Line
	4550 6550 11250 6550
Text Notes 4600 6750 0    118  ~ 0
VSS Connections\n
Text Notes 1850 5650 0    197  ~ 0
swd de mxden bak\nsolder bridge yuvarlak olarak
$EndSCHEMATC
