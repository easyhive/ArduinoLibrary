// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "EasyHive.h"
#include <Arduino.h>
#include "OneWire.h"
#include "DallasTemperature.h"
#include "ADS1231.h"
#include "FlashStorage.h"
#include "Sodaq_nbIOT.h"
#include "Sodaq_wdt.h"

// Server variables
// define Server variables
const char* apn = "iot.1nce.net";
const char* cdp = "";
unsigned char cid = 1;
const char* forceOperator = "26201"; // optional - depends on SIM / network
const char band = 8;

// Sodaq NBIOT Class init
Sodaq_nbIOT nbiot;

unsigned long temp_value;
unsigned long weight_value;

//float weight_calib = 8388607.0;
//float weight_offset = 4.2;

float weight_calib = 1.0;
float weight_offset = 1.0;

//calibration weights in gram
const unsigned long calibration_weight = 1000;
const unsigned long tare_weight = 0;

String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
//Weight ADS
ADS1231 adc(CLK_PIN,DATA_PIN);

// Reserve a portion of flash memory to store an "float" variable
// and call it "flash_c".
FlashStorage(flash_c, float);

// Reserve a portion of flash memory to store an "float" variable
// and call it "flash_o".
FlashStorage(flash_o, float);

// Reserve a portion of flash memory to store an "bool" variable
// and call it "calibrated".
FlashStorage(flash_calibrated, bool);

struct EEPROM_field {
  float value;
  char name[10];
};


void enable_USB_in(void){
    // initialize serial:
    SerialUSB.begin(9600);
    // reserve 200 bytes for the inputString:
    inputString.reserve(200);
    return; 
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

void serialEvent(void) {
    while (SerialUSB.available()) {
        // get the new byte:
        char inChar = (char)SerialUSB.read();
        // add it to the inputString:
        inputString += inChar;
        // if the incoming character is a newline, set a flag so the main loop can
        // do something about it:
        if (inChar == '\n') {
            stringComplete = true;
        }
    }
    return;
}

void check_USB_in(void){
    if (stringComplete) {
        SerialUSB.println(inputString);
        // clear the string:
        inputString = "";
        stringComplete = false;
    }
    return;   
}

void init_LEDs(void){
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(BUZZER, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);  

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return; 
}

void LEDs_off(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_green(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_red(void){
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);
    return;
}

void LEDs_blue(void){
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, LOW);
    return;
}

void init_Temp(void){
    // start serial port
    SerialUSB.begin(9600);
    SerialUSB.println("Dallas Temperature IC Control Library Demo");
    // Start up the library
    sensors.begin();
    return; 
}

void init_Weight(void){
    SerialUSB.begin(9600);
    SerialUSB.println("Init ADS1232");
    pinMode(PDWN, OUTPUT);
    digitalWrite(PDWN, HIGH);

    pinMode(TEMP, OUTPUT);
    digitalWrite(TEMP, LOW);

    if (!adc.begin()) {
        Serial.println("Invalid pins bro!");
    }    
    return; 
}

void set_Weight_calib(float val_c, float val_o){

    // Save into "flash_c"
    flash_c.write(val_c);
    // Save into "flash_o"
    flash_o.write(val_o);

    //needs to be calibrated at least once
    flash_calibrated.write(true);

    weight_calib = val_c; 
    weight_offset = val_o; 
    return;
}

void get_Weight_calib(float* val_c, float* val_o){
    SerialUSB.begin(9600);

    // Read the content of "flash_c" and assign it to "weight_calib"
    bool calib = flash_calibrated.read();

    //needs to be calibrated at least once
    if(calib == true){
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
    }
    else{
        set_Weight_calib(1.0, 1.0);
        // Read the content of "flash_c" and assign it to "weight_calib"
        weight_calib = flash_c.read();
        // Read the content of "flash_o" and assign it to "weight_offset"
        weight_offset = flash_o.read();
    }

    SerialUSB.print("Offset: ");
    SerialUSB.println(weight_offset);
    SerialUSB.print("Calibration: ");
    SerialUSB.println(weight_calib);

    SerialUSB.print("Calibration done?: ");
    SerialUSB.println(calib);

    *val_c = weight_calib; 
    *val_o = weight_offset; 
    return; 
}

float get_Temp(void){
    // call sensors.requestTemperatures() to issue a global temperature 
    // request to all devices on the bus
    //SerialUSB.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    //SerialUSB.println("DONE");
    // After we got the temperatures, we can print them here.
    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    //SerialUSB.print("Temperature for the device 1 (index 0) is: ");
    //SerialUSB.println(sensors.getTempCByIndex(0));  
    return sensors.getTempCByIndex(0); 
}

long get_Weight_raw(void){
    long val;
    if(adc.getValue(val))
        return val; 
}

float get_Weight(void){

    //Datasheet ADS1232 page 8 -> w = m * val + wzs - calibration_weight

    //m = (calibration_weight / (weight_calib - weight_offset)) 
    //wzs = -m * weight_offset

    long val;
    if(adc.getValue(val)){        //this call blocks until a sample is ready!
    //  SerialUSB.print(millis());
    //  SerialUSB.print(",");
        //SerialUSB.println((4.2/8388607.0)*val, 10);  //23 bits of accuracy (24th is sign) with 4.2 volt (measured) AVDD. 2^23 = 8388607 !
    } 
    else{
        //SerialUSB.println("Failed to get data");
    }
    //delay(500);
    
    unsigned long w_tare = 0;

    float result = 0; 
    float m = (calibration_weight / (weight_calib - weight_offset));
    float wzs = (-m) * weight_offset;
    result = m * val + wzs - tare_weight;

    return result; 
}

/******************* NBIOT Functions ********************/

void checkMessage(const char msg[STDSTRINGLEN])
{
	String data = msg;
	
    if(data == "OK!") // OK!
    {
      DEBUG_STREAM.println("OK! nothing todo");
    }
    else if(data.length() > 0) // something was transmitted..
    {
      if(data.indexOf('t') >= 0) // Tare command?
      {
       DEBUG_STREAM.println("tare!");
      }
      if(data.indexOf('W') >= 0) // Weight definition command?
      {
       DEBUG_STREAM.println("Weight");
       uint8_t i = data.indexOf('W')+1;
       int j = 0;
       char weightbuffer[10];
       while(isDigit(data[i]))
       {
        weightbuffer[j] = data[i];
        i++;
        j++;
       }
       weightbuffer[j] = '\0';
       j = atoi(weightbuffer);
       DEBUG_STREAM.println(weightbuffer);
       DEBUG_STREAM.println(j);
       DEBUG_STREAM.println("g");
      }
    }
    else
    {
      
      DEBUG_STREAM.println("not OK!");
    }
}

const char* sendMessageThroughUDP(const char param[STDSTRINGLEN])
{
    DEBUG_STREAM.println();
    DEBUG_STREAM.println("Sending message through UDP");

    int localPort = 16666;
    int socketID = nbiot.createSocket(localPort);

    if (socketID >= 7 || socketID < 0) {
        DEBUG_STREAM.println("Failed to create socket");
        return "SOCKETERROR";
    }

    DEBUG_STREAM.println("Created socket!");

    // const char* strBuffer = "1,50,49";
    const char* strBuffer = param;
    size_t size = strlen(strBuffer);

    // send data
    int lengthSent = nbiot.socketSend(socketID, "18.184.113.47", 2222, strBuffer); // "195.34.89.241" : 7 is the ublox echo service

    DEBUG_STREAM.print("String length vs sent: ");
    DEBUG_STREAM.print(size);
    DEBUG_STREAM.print(" vs ");
    DEBUG_STREAM.println(lengthSent);
	
	String msg = "";
    // wait for data
    if (nbiot.waitForUDPResponse()) {
        DEBUG_STREAM.println("Received response!");
        while (nbiot.hasPendingUDPBytes()) {
            char data[10];
            
            
            // read two bytes at a time
            SaraN2UDPPacketMetadata p;
            int size = nbiot.socketReceiveHex(data, 2, &p);

            if (size) {
                DEBUG_STREAM.write(HEX_PAIR_TO_BYTE(data[0],data[1]));
                // p is a pointer to memory that is owned by nbiot class
                DEBUG_STREAM.println(char(HEX_PAIR_TO_BYTE(data[0],data[1])));
                DEBUG_STREAM.println(p.socketID);
                DEBUG_STREAM.println(p.ip);
                DEBUG_STREAM.println(p.port);
                DEBUG_STREAM.println(p.length);
                DEBUG_STREAM.println(p.remainingLength);
                
                msg += char(HEX_PAIR_TO_BYTE(data[0],data[1]));
            }
            else {
                DEBUG_STREAM.println("Receive failed!");
            }
        }
       DEBUG_STREAM.println("Done checking message");
    }
    else {
        DEBUG_STREAM.println("Timed-out!");
    }

    nbiot.closeSocket(socketID);
    DEBUG_STREAM.println();
    return msg.c_str();
}
