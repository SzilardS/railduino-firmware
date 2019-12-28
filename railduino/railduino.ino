/*
    Copyright (C) 2019  Ing. Pavel Sedlacek
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#define dbg(x) Serial.print(x);
#define dbg(x) ;
//#define dbgln(x) Serial.println(x);
#define dbgln(x) ;

// Define Railduino board version
// RESERVED FOR FUTURE USE: #define BOARD_RAILDUINO_1_3
//#define BOARD_RAILDUINO_2_0
#define BOARD_RAILDUINO_2_1

#include <SimpleModbusSlave.h>
#include <OneWire.h>
#include <DS2438.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MgsModbus.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0};
unsigned int listenPort = 44444;
unsigned int sendPort = 55554;
unsigned int remPort = 55555;
IPAddress listenIpAddress;
IPAddress sendIpAddress(255, 255, 255, 255);

typedef struct rs485_parameter {
  long baudRate;
  int txDelay;
  int timeOut;
  String modeStr;
} rs485_parameter;

rs485_parameter rs485Parameters[] = {
  {115200, 10, 20, "RS485, 115200kbps, RTU OFF"},
  {19200, 30, 500, "RS485, 19200kbps, RTU OFF"},
  {9600, 30, 500, "RS485, 9600kbps, RTU OFF"},
  {115200, 10, 20, "RS485, 115200kbps, RTU ON"},
  {0, 0, 0}
};

#define RS485_MODE_MODBUS_RTU       0
#define RS485_MODE_RAILDUINO_115200 1
#define RS485_MODE_RAILDUINO_19200  2
#define RS485_MODE_RAILDUINO_9600   3
#define RS485_MODE_UNUSED           4

#if defined(BOARD_RAILDUINO_1_3)

# define BOARD_VERSION 1.3
# define RS485_MODE_0 RS485_MODE_RAILDUINO_115200
# define RS485_MODE_1 RS485_MODE_UNUSED

#elif defined(BOARD_RAILDUINO_2_0)

# define BOARD_VERSION 2.0
# define numOfLSSwitches 4
int LSSwitchPins[numOfLSSwitches] = {9, 11, 12, 13};
# define numOfLedPins 2
int ledPins[numOfLedPins] = {18, 17};
# define RS485_MODE_0 RS485_MODE_RAILDUINO_115200
# define RS485_MODE_1 RS485_MODE_RAILDUINO_19200

#elif defined(BOARD_RAILDUINO_2_1)

#define BOARD_VERSION 2.1
# define numOfLSSwitches 4
int LSSwitchPins[numOfLSSwitches] = {9, 11, 12, 18};
# define numOfLedPins 2
int ledPins[numOfLedPins] = {13, 17};
# define RS485_MODE_0 RS485_MODE_RAILDUINO_115200
# define RS485_MODE_1 RS485_MODE_MODBUS_RTU

#else
# error "Boardtype undefined. You must define one of the BOARD_RAILDUINO_"
#endif


#define relOut1Byte 0
#define relOut2Byte 1
#define hssLssByte 2
#define anaOut1Byte 3
#define anaOut2Byte 4
#define digInp1Byte 5
#define digInp2Byte 6
#define digInp3Byte 7
#define anaInp1Byte 8
#define anaInp2Byte 9
#define serviceByte 10
#define oneWireTempByte 11
#define oneWireVadByte 12
#define oneWireVsensByte 13
#define oneWireDS18B20Byte 40

#define inputPacketBufferSize UDP_TX_PACKET_MAX_SIZE
char inputPacketBuffer[UDP_TX_PACKET_MAX_SIZE];
#define outputPacketBufferSize 100
char outputPacketBuffer[outputPacketBufferSize];

EthernetUDP udpRecv;
EthernetUDP udpSend;

#define oneWireCycle 30000
#define oneWireSubCycle 5000
#define anaInputCycle 10000
#define heartBeatCycle 60000
#define statusLedTimeOn 50
#define statusLedTimeOff 990
#define commLedTimeOn 50
#define commLedTimeOff 50
#define debouncingTime 5
#define serial3TxControl 16
#define numOfRelays 12
int relayPins[numOfRelays] = {37, 35, 33, 31, 29, 27, 39, 41, 43, 45, 47, 49};
#define numOfHSSwitches 4
int HSSwitchPins[numOfHSSwitches] = {5, 6, 7, 8};
#define numOfAnaOuts 2
int anaOutPins[numOfAnaOuts] = {3, 2};
#define numOfAnaInputs 2
int analogPins[numOfAnaInputs] = {64, 63};
float analogStatus[numOfAnaInputs];
#define numOfDigInputs 24
int inputPins[numOfDigInputs] = {34, 32, 30, 28, 26, 24, 22, 25, 23, 21, 20, 19, 36, 38, 40, 42, 44, 46, 48, 69, 68, 67, 66, 65};
int inputStatus[numOfDigInputs];
int inputStatusNew[numOfDigInputs];
int inputChangeTimestamp[numOfDigInputs];
#define numOfDipSwitchPins 6
int dipSwitchPins[numOfDipSwitchPins] = {57, 56, 55, 54, 58, 59};
int boardAddress = 0;
int ethOn = 0;
int rs485Mode = -1;
bool ticTac = 0;

String boardAddressStr;
String boardAddressRailStr;
String railStr = "rail";
String digInputStr = "di";
String anaInputStr = "ai";
String relayStr = "ro";
String HSSwitchStr = "ho";
String LSSwitchStr = "lo";
String anaOutStr = "ao";
String digStatStr = "stat";
String rstStr = "rst";
String heartBeatStr = "hb";
String relayOnCommands[numOfRelays];
String relayOffCommands[numOfRelays];
String HSSwitchOnCommands[numOfHSSwitches];
String HSSwitchOffCommands[numOfHSSwitches];
String LSSwitchOnCommands[numOfLSSwitches];
String LSSwitchOffCommands[numOfLSSwitches];

String digStatCommand[numOfDigInputs];
String anaOutCommand[numOfAnaOuts];

class Timer {
    private:
        unsigned long timestampLastHitMs;
        unsigned long sleepTimeMs;
    public:
        boolean isOver();
        void sleep(unsigned long sleepTimeMs);
};

boolean Timer::isOver() {
    if (millis() - timestampLastHitMs < sleepTimeMs) {
        return false;
    }
    timestampLastHitMs = millis();
    return true;
}

void Timer::sleep(unsigned long sleepTimeMs) {
    this->sleepTimeMs = sleepTimeMs;
    timestampLastHitMs = millis();
}

Timer statusLedTimerOn;
Timer statusLedTimerOff;
Timer oneWireTimer;
Timer oneWireSubTimer;
Timer analogTimer;
Timer heartBeatTimer;

MgsModbus Mb;

OneWire ds(62);
byte oneWireData[12];
byte oneWireAddr[8];

#define maxSensors 10
byte readstage = 0, resolution = 11;
byte sensors[maxSensors][8], DS2438count, DS18B20count;
byte sensors2438[maxSensors][8], sensors18B20[maxSensors][8];
DS2438 ds2438(&ds);



void setup() {

    Serial.begin(9600);


    dbg("Railduino firmware version: ");
    dbgln(BOARD_VERSION);

    for (int i = 0; i < numOfDigInputs; i++) {
        pinMode(inputPins[i], INPUT);
        inputStatus[i] = 1;
        inputStatusNew[i] = 0;
        digStatCommand[i] = digStatStr + String(i + 1, DEC);
    }

    for (int i = 0; i < numOfRelays; i++) {
        pinMode(relayPins[i], OUTPUT);
        relayOnCommands[i] = relayStr + String(i + 1, DEC) + " on";
        relayOffCommands[i] = relayStr + String(i + 1, DEC) + " off";
        setRelay(i, 0);
    }

    for (int i = 0; i < numOfHSSwitches; i++) {
        pinMode(HSSwitchPins[i], OUTPUT);
        HSSwitchOnCommands[i] = HSSwitchStr + String(i + 1, DEC) + " on";
        HSSwitchOffCommands[i] = HSSwitchStr + String(i + 1, DEC) + " off";
        setHSSwitch(i, 0);
    }

    for (int i = 0; i < numOfLSSwitches; i++) {
        pinMode(LSSwitchPins[i], OUTPUT);
        LSSwitchOnCommands[i] = LSSwitchStr + String(i + 1, DEC) + " on";
        LSSwitchOffCommands[i] = LSSwitchStr + String(i + 1, DEC) + " off";
        setLSSwitch(i, 0);
    }

    for (int i = 0; i < numOfAnaOuts; i++) {
        pinMode(anaOutPins[i], OUTPUT);
        anaOutCommand[i] = anaOutStr + String(i + 1, DEC);
        setAnaOut(i, 0);
    }

    for (int i = 0; i < numOfAnaInputs; i++) {
        pinMode(analogPins[i], INPUT);
    }

    for (int i = 0; i < numOfLedPins; i++) {
        pinMode(ledPins[i], OUTPUT);
    }

    analogTimer.sleep(anaInputCycle);
    statusLedTimerOn.sleep(statusLedTimeOn);
    statusLedTimerOff.sleep(statusLedTimeOff);
    heartBeatTimer.sleep(heartBeatCycle);

    for (int i = 0; i < 4; i++) {
        pinMode(dipSwitchPins[i], INPUT);
        if (!digitalRead(dipSwitchPins[i])) {
            boardAddress |= (1 << i);
        }
    }

    pinMode(dipSwitchPins[4], INPUT);
    if (!digitalRead(dipSwitchPins[4]))  {
        ethOn = 1;
        dbgln("Ethernet ON");
    } else {
        ethOn = 0;
        dbgln("Ethernet OFF");
    }

    pinMode(dipSwitchPins[5], INPUT);
    if (!digitalRead(dipSwitchPins[5]))  {
        rs485Mode = RS485_MODE_0;
    } else {
        rs485Mode = RS485_MODE_1;
    }
    dbgln(rs485Parameters[rs485Mode].modeStr);

    dbg(rs485Parameters[rs485Mode].baudRate);
    dbg(" Bd, Tx Delay: ");
    dbg(rs485Parameters[rs485Mode].txDelay);
    dbg(" ms, Timeout: ");
    dbg(rs485Parameters[rs485Mode].timeOut);
    dbgln(" ms");

    boardAddressStr = String(boardAddress);
    boardAddressRailStr = railStr + String(boardAddress);

    if (ethOn) {
        mac[5] = (0xED + boardAddress);
        listenIpAddress = IPAddress(192, 168, 150, 150 + boardAddress);
        if (Ethernet.begin(mac) == 0)
        {
            dbgln("Failed to configure Ethernet using DHCP, using Static Mode");
            Ethernet.begin(mac, listenIpAddress);
        }

        udpRecv.begin(listenPort);
        udpSend.begin(sendPort);

        dbg("IP: ");
        printIPAddress();
        dbgln();
    }


    memset(Mb.MbData, 0, sizeof(Mb.MbData));

    if (rs485Mode == RS485_MODE_MODBUS_RTU) {
        modbus_configure(&Serial3, rs485Parameters[rs485Mode].baudRate, SERIAL_8N1, boardAddress, serial3TxControl, sizeof(Mb.MbData), Mb.MbData);
    }

    Serial3.begin(rs485Parameters[rs485Mode].baudRate);
    Serial3.setTimeout(rs485Parameters[rs485Mode].timeOut);
    pinMode(serial3TxControl, OUTPUT);
    digitalWrite(serial3TxControl, 0);

    dbg("Address: ");
    dbgln(boardAddressStr);

    lookUpSensors();

}

void loop() {

    readDigInputs();

    readAnaInputs();

    processCommands();

    processOnewire();

    statusLed();

    if (ethOn) {
        Mb.MbsRun();
    } else {
        heartBeat();
    }

    if (rs485Mode == RS485_MODE_MODBUS_RTU) {
        modbus_update();
    }

}

void (* resetFunc) (void) = 0;


void printIPAddress()
{
    for (byte thisByte = 0; thisByte < 4; thisByte++) {
        dbg(Ethernet.localIP()[thisByte]);
        dbg(".");
    }
}

void heartBeat() {
    if (!heartBeatTimer.isOver()) {
        return;
    }

    heartBeatTimer.sleep(heartBeatCycle);
    if (ticTac) {
        sendMsg(heartBeatStr + " 1");
        ticTac = 0;
    } else {
        sendMsg(heartBeatStr + " 0");
        ticTac = 1;
    }
}

void statusLed() {
    if (statusLedTimerOff.isOver()) {
        statusLedTimerOn.sleep(statusLedTimeOn);
        statusLedTimerOff.sleep(statusLedTimeOff);
        digitalWrite(ledPins[0], HIGH);
    }
    if (statusLedTimerOn.isOver()) {
        digitalWrite(ledPins[0], LOW);
    }
}

String oneWireAddressToString(byte addr[]) {
    String s = "";
    for (int i = 0; i < 8; i++) {
        s += String(addr[i], HEX);
    }
    return s;
}

void lookUpSensors() {
    byte j = 0, k = 0, l = 0, m = 0;
    while ((j <= maxSensors) && (ds.search(sensors[j]))) {
        if (!OneWire::crc8(sensors[j], 7) != sensors[j][7]) {
            if (sensors[j][0] == 38) {
                for (l = 0; l < 8; l++) {
                    sensors2438[k][l] = sensors[j][l];
                }
                k++;
            } else {
                for (l = 0; l < 8; l++) {
                    sensors18B20[m][l] = sensors[j][l];
                }
                m++;
                dssetresolution(ds, sensors[j], resolution);
            }
        }
        j++;
    }
    DS2438count = k;
    DS18B20count = m;
    dbg("1-wire sensors found: ");
    dbgln(k + m);
}

void dssetresolution(OneWire ow, byte addr[8], byte resolution) {
    byte resbyte = 0x1F;
    if (resolution == 12) {
        resbyte = 0x7F;
    }
    else if (resolution == 11) {
        resbyte = 0x5F;
    }
    else if (resolution == 10) {
        resbyte = 0x3F;
    }

    ow.reset();
    ow.select(addr);
    ow.write(0x4E);
    ow.write(0);
    ow.write(0);
    ow.write(resbyte);
    ow.write(0x48);
}

void dsconvertcommand(OneWire ow, byte addr[8]) {
    ow.reset();
    ow.select(addr);
    ow.write(0x44, 1);
}

float dsreadtemp(OneWire ow, byte addr[8]) {
    int i;
    byte data[12];
    float celsius;
    ow.reset();
    ow.select(addr);
    ow.write(0xBE);
    for ( i = 0; i < 9; i++) {
        data[i] = ow.read();
    }

    int16_t TReading = (data[1] << 8) | data[0];
    celsius = 0.0625 * TReading;
    return celsius;
}


void processOnewire() {
    static byte oneWireState = 0;
    static byte oneWireCnt = 0;

    switch (oneWireState)
    {
        case 0:
            if (!oneWireTimer.isOver()) {
                return;
            }
            oneWireTimer.sleep(oneWireCycle);
            oneWireSubTimer.sleep(oneWireSubCycle);
            oneWireCnt = 0;
            oneWireState++;
            break;
        case 1:
            if (!oneWireSubTimer.isOver()) {
                return;
            }
            if ((oneWireCnt < DS2438count)) {
                ds2438.begin();
                ds2438.update(sensors2438[oneWireCnt]);
                if (!ds2438.isError()) {
                    if (rs485Mode == RS485_MODE_MODBUS_RTU) {
                        Mb.MbData[oneWireTempByte + (oneWireCnt * 3)] = ds2438.getTemperature() * 100;
                        Mb.MbData[oneWireVadByte + (oneWireCnt * 3)] = ds2438.getVoltage(DS2438_CHA) * 100;
                        Mb.MbData[oneWireVsensByte + (oneWireCnt * 3)] = ds2438.getVoltage(DS2438_CHB) * 100;
                    } else {
                        sendMsg("1w " + oneWireAddressToString(sensors2438[oneWireCnt]) + " " + String(ds2438.getTemperature(), 2) + " " + String(ds2438.getVoltage(DS2438_CHA), 2) + " " + String(ds2438.getVoltage(DS2438_CHB), 2));
                    }
                }
                oneWireCnt++;
            } else {
                oneWireCnt = 0;
                oneWireState++;
            }
            break;
        case 2:
            if (!oneWireSubTimer.isOver()) {
                return;
            }
            if ((oneWireCnt < DS18B20count)) {
                dsconvertcommand(ds, sensors18B20[oneWireCnt]);
                oneWireCnt++;
            } else {
                oneWireCnt = 0;
                oneWireState++;
            }
            break;
        case 3:
            if (!oneWireSubTimer.isOver()) {
                return;
            }
            if ((oneWireCnt < DS18B20count)) {
                if (rs485Mode == RS485_MODE_MODBUS_RTU) {
                    Mb.MbData[oneWireDS18B20Byte + oneWireCnt] = dsreadtemp(ds, sensors18B20[oneWireCnt]) * 100;
                } else {
                    sendMsg("1w " + oneWireAddressToString(sensors18B20[oneWireCnt]) + " " + String(dsreadtemp(ds, sensors18B20[oneWireCnt]), 2));
                }
                oneWireCnt++;
            } else {
                oneWireState = 0;
            }
            break;
    }
}


void readDigInputs() {

    int timestamp = millis();
    for (int i = 0; i < numOfDigInputs; i++) {
        int oldValue = inputStatus[i];
        int newValue = inputStatusNew[i];
        int curValue = digitalRead(inputPins[i]);
        int byteNo = i / 8;
        int bitPos = i - (byteNo * 8);

        if (oldValue != newValue) {
            if (newValue != curValue) {
                inputStatusNew[i] = curValue;
            } else if (timestamp - inputChangeTimestamp[i] > debouncingTime) {
                inputStatus[i] = newValue;
                if (!newValue) {
                    bitWrite(Mb.MbData[byteNo + 5], bitPos, 1);
                    sendInputOn(i + 1);
                } else {
                    bitWrite(Mb.MbData[byteNo + 5], bitPos, 0);
                    sendInputOff(i + 1);
                }
            }

        } else {
            if (oldValue != curValue) {
                inputStatusNew[i] = curValue;
                inputChangeTimestamp[i] = timestamp;
            }
        }
    }
}

void readAnaInputs() {

    if (!analogTimer.isOver()) {
        return;
    }

    analogTimer.sleep(anaInputCycle);
    for (int i = 0; i < numOfAnaInputs; i++) {
        int pin = analogPins[i];
        float value = analogRead(pin) * (255 / 1023.0);
        float oldValue = analogStatus[i];
        analogStatus[i] = value;
        if (value != oldValue) {
            Mb.MbData[i + 8] = (byte) value;
            sendAnaInput(i + 1, Mb.MbData[i + 8]);
        }
    }
}

void sendInputOn(int input) {
    sendMsg(digInputStr + String(input, DEC) + " 1");
}

void sendInputOff(int input) {
    sendMsg(digInputStr + String(input, DEC) + " 0");
}

void sendAnaInput(int input, float value) {
    sendMsg(anaInputStr + String(input, DEC) + " " + String(value, 2));
}

void sendMsg(String message) {
    message = railStr + boardAddressStr + " " + message;
    message.toCharArray(outputPacketBuffer, outputPacketBufferSize);

    digitalWrite(ledPins[1], HIGH);

    if (ethOn) {
        udpSend.beginPacket(sendIpAddress, remPort);
        udpSend.write(outputPacketBuffer, message.length());
        udpSend.endPacket();
    }

    if (rs485Mode != RS485_MODE_MODBUS_RTU) {
        digitalWrite(serial3TxControl, HIGH);
        Serial3.print(message + "\n");
        delay(rs485Parameters[rs485Mode].txDelay);
        digitalWrite(serial3TxControl, LOW);
    }

    digitalWrite(ledPins[1], LOW);

    dbg("Sending packet: ");
    dbgln(message);
}

void setRelay(int relay, int value) {
    if (relay >= numOfRelays) {
        return;
    }
    dbgln("Writing to relay " + String(relay + 1) + " value " + String(value));
    digitalWrite(relayPins[relay], value);
}

void setHSSwitch(int hsswitch, int value) {
    if (hsswitch >= numOfHSSwitches) {
        return;
    }
    dbgln("Writing to high side switch" + String(hsswitch + 1) + " value " + String(value));
    digitalWrite(HSSwitchPins[hsswitch], value);
}

void setLSSwitch(int lsswitch, int value) {
    if (lsswitch >= numOfLSSwitches) {
        return;
    }
    dbgln("Writing to low side switch" + String(lsswitch + 1) + " value " + String(value));
    digitalWrite(LSSwitchPins[lsswitch], value);
}

void setAnaOut(int pwm, int value) {
    if (pwm >= numOfAnaOuts) {
        return;
    }
    dbgln("Writing to analog output " + String(pwm + 1) + " value " + String(value));
    analogWrite(anaOutPins[pwm], value);
}

boolean receivePacket(String *cmd) {

    if (rs485Mode != RS485_MODE_MODBUS_RTU) {
        while (Serial3.available() > 0) {
            *cmd = Serial3.readStringUntil('\n');
            if (cmd->startsWith(boardAddressRailStr)) {
                cmd->replace(boardAddressRailStr, "");
                cmd->trim();
                return true;
            }
        }
    }

    if (ethOn) {
        int packetSize = udpRecv.parsePacket();
        if (packetSize) {
            memset(inputPacketBuffer, 0, sizeof(inputPacketBuffer));
            udpRecv.read(inputPacketBuffer, inputPacketBufferSize);
            *cmd = String(inputPacketBuffer);
            if (cmd->startsWith(boardAddressRailStr)) {
                cmd->replace(boardAddressRailStr, "");
                cmd->trim();
                return true;
            }
        }
    }

    return false;
}


void processCommands() {
    String cmd;
    byte byteNo, curBitValue, bitPos;
    int curAnaVal;

    for (int i = 0; i < numOfRelays; i++) {
        if (i < 8) {
            setRelay(i, bitRead(Mb.MbData[relOut1Byte], i));
        } else {
            setRelay(i, bitRead(Mb.MbData[relOut2Byte], i - 8));
        }

    }

    for (int i = 0; i < numOfHSSwitches; i++) {
        setHSSwitch(i, bitRead(Mb.MbData[hssLssByte], i));
    }

    for (int i = 0; i < numOfLSSwitches; i++) {
        setLSSwitch(i, bitRead(Mb.MbData[hssLssByte], i + numOfHSSwitches));
    }

    for (int i = 0; i < numOfAnaOuts; i++) {
        setAnaOut(i, Mb.MbData[anaOut1Byte + i]);
    }

    if (bitRead(Mb.MbData[serviceByte], 0)) {
        resetFunc();
    }

    if (receivePacket(&cmd)) {
        dbg("Received packet: ")
        dbgln(cmd);
        digitalWrite(ledPins[1], HIGH);
        if (cmd.startsWith(relayStr)) {
            for (int i = 0; i < numOfRelays; i++) {
                if (i < 8) {
                    byteNo = relOut1Byte;
                    bitPos = i;
                } else {
                    byteNo = relOut2Byte;
                    bitPos = i - 8;
                }
                if (cmd == relayOnCommands[i]) {
                    bitWrite(Mb.MbData[byteNo], bitPos, 1);
                } else if (cmd == relayOffCommands[i]) {
                    bitWrite(Mb.MbData[byteNo], bitPos, 0);
                }
            }
        } else if (cmd.startsWith(HSSwitchStr)) {
            for (int i = 0; i < numOfHSSwitches; i++) {
                if (cmd == HSSwitchOnCommands[i]) {
                    bitWrite(Mb.MbData[hssLssByte], i, 1);
                } else if (cmd == HSSwitchOffCommands[i]) {
                    bitWrite(Mb.MbData[hssLssByte], i, 0);
                }
            }
        } else if (cmd.startsWith(LSSwitchStr)) {
            for (int i = 0; i < numOfLSSwitches; i++) {
                if (cmd == LSSwitchOnCommands[i]) {
                    bitWrite(Mb.MbData[hssLssByte], i + numOfHSSwitches, 1);
                } else if (cmd == LSSwitchOffCommands[i]) {
                    bitWrite(Mb.MbData[hssLssByte], i + numOfHSSwitches, 0);
                }
            }
        } else if (cmd.startsWith(anaOutStr)) {
            String anaOutValue = cmd.substring(anaOutStr.length() + 2);
            for (int i = 0; i < numOfAnaOuts; i++) {
                if (cmd.substring(0, anaOutStr.length() + 1) == anaOutCommand[i]) {
                    Mb.MbData[anaOut1Byte + i] = anaOutValue.toInt();
                }
            }
        } else if (cmd.startsWith(rstStr)) {
            bitWrite(Mb.MbData[serviceByte], 0, 1);
        }
        digitalWrite(ledPins[1], LOW);
    }
}
