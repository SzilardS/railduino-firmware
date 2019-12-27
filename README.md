# Railduino v2.1 firmware

New firmware for Railduino v2.1 based on official firmware from Pavel Sedláček

# Compile

## Required libraries

### MgsModbus

- Download the archive from http://myarduinoprojects.com/modbus.html
- Extract the MgsModbus-v0.1.1.zip into libraries folder

### SimpleModbusSlave

- Clone https://github.com/jecrespo/simple-modbus repository
- Copy *simple-modbus/Modbus RTU libraries for Arduino/SimpleModbusSlaveV10* folder into libraries folder

### DS2438

- Clone https://github.com/jbechter/arduino-onewire-DS2438
- Move to the libraries folder named

### Other

You can install the following libraries via Manage Libraries feature of Arduino IDE:

- OneWire

# UDP syntax
## Signals

| Description | Message to server  |
| :--- | :--- |
DS18B20 1wire sensor packet|rail1 1w 2864fc3008082 25.44
DS2438 1wire sensor packet|rail1 1w 2612c3102004f 25.44 1.23 0.12
digital input state|rail1 di1 1
analog input state|rail1 ai1 250

## Commands

| Description | Message from server  |
| :--- | :--- |
relay on command|rail1 do12 on
relay off command|rail1 do5 off
high side switch on command|rail1 ho2 on
high side switch off command|rail1 ho4 off
low side switch on command|rail1 lo1 on
low side switch off command|rail1 lo2 off
analog output command|rail1 ao1 180
status command|rail1 stat10
reset command|rail1 rst

# Default scan cycles

| Cycle | Tim beetween checks
--- | :---:
1wire cycle|30000 ms
analog input cycle|10000 ms
heart beat cycle(only RS485)|60000 ms

# MODBUS

* MODBUS TCP commands: FC1 - FC16
* MODBUS RTU commands: FC3, FC6, FC16

## MODBUS register map (1 register = 2 bytes = 16 bits)
 
register number|description
 :---: | :--- 
0|relay outputs 1-8
1|relay outputs 9-12
2|digital outputs HSS 1-4, LSS 1-4
3|analog output 1
4|analog output 2
5|digital inputs 1-8
6|digital inputs 9-16
7|digital inputs 17-24
8|analog input 1           0-256
9|analog input 2           0-256
10|service - reset (bit 0)
11|1st DS2438 Temp (value multiplied by 100)
12|1st DS2438 Vad (value multiplied by 100)
13|1st DS2438 Vsens (value multiplied by 100)
-| 
39|DS2438 values (up to 10 sensors)
40-50|DS18B20 Temperature (up to 10 sensors) (value multiplied by 100)

# Notes

Combination of 1wire sensors must be up to 10pcs maximum (DS18B20 or DS2438)

Using RS485 the UDP syntax must have \n symbol at the end of the command line