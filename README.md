# SensorModule
Sensor managing microcontroller software

## Introduction
SensorModule is a sensor managing microcontroller. It controls various sensors and sensing capable modules.
(I.e. temperature, humidity, light amount, etc.)
High level management provided by ManagerModule.
SensorModule is connected to ManagerModule through I2C. Together they provide distibuted environment measurement
system.

## Device description
### Hardware
SensorModule implemented on Texas Instruments MSP-EXP430FR5739.
Sensors are attached through channels, connected on GPIO. Also the board has an integrated NTC Thermistor. 

### Software
Board connects to manager through I2C. Manager sends request for measurement and receives measurement result.
- Output: Raw ADC value
- Output: Measurement value
- Output: Measurement unit (Register 2) 'C'elsius, %'R'H

Board is configured through UART console. One sensor / channel
- Channel ID (serial number)
- Channel Sensor type ('T'emperature, 'H'umidity, 'P'ressure, 'L'ight intensity)
- Channel Sensor unit ('C'elsius, 'R'H, 'P'ascal)
- Channel Sensor address (0 = NTC, 0x03-0x77 = I2C address)
- Channel Slave I2C address (toward the master controller)
- Calibration values:
	- Scale
	- Offset

I2C bus registers (Read only):
0 - Raw measurement	(word)
1 - Adjusted measurement value	(word)
2 - Sensor type	(byte)
3 - Sensor unit	(byte)

Required components / information:
Power MM, Clock system, CPUX (Interrupts), Digital I/O, Timer_A, ADC10_B, eUSCI - UART, eUSCI - I2C

Optional components:
FRAM controller, Watchdog Timer

## Further development
Implement interrupt based hardware handling instead of polling
FreeRTOS based, multiple concurrent channel handling
