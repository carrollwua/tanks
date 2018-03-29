/* Mesh network endpoint main source file
* University of Arizona ENG 498 A&B Team 17023
* Team TANKS
*
* Main file for firmware for the arduino portion of the Mesh Network Endpoint
*  subsystem. Due to the raspberry pi's lack of hardwre interrupts an arduino
*  will handl the radio communication and transfer the data to the raspberry
*  pi microcomputer by way of USB serial.
*
* The actual hardware to be deployed on the SRER will use an Adafruit Feather
*   M0 LoRa like the rest of the mesh network but for design day we'll be using
*   some prototype hardware, an Arduino Mega driving an RF95 LoRa breakout board
*   hence the two sets of pin IDs.
*/
#include <Arduino.h>
#include <SPI.h>
#include <RHMesh.h>
#include <RH_RF95.h>

//For Mega 2560
#define RF95_CS 53
#define RF95_RST 48
#define RF95_INT 2

//For Feather M0
/*
#define LORA_RESET 4
#define LORA_IRQ_PIN 3
#define LORA_CS_PIN 8
*/
#define RF95_FREQ 915.0
#define MESH_ADDRESS 0
#define MESH_LISTEN_DURATION 500   //Length to listen for incoming mesh traffic
#define RH_MESH_MAX_MESSAGE_LEN 50

RH_RF95 rf95(RF95_CS, RF95_INT);
RHMesh mesh(rf95, MESH_ADDRESS);

String txString;
uint8_t txBuf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t txLength;
uint8_t rxBuf[RH_MESH_MAX_MESSAGE_LEN];
uint8_t sender;
String rxString;
uint8_t rxLength;

uint8_t serialBytesAvailable;
String serialString;
char serialBuffer[64];        //The serial buffer has a max length of 64
uint8_t serialBufferLength;
unsigned int targetID;        //Command target parsed from serial

void setup()
{
  //Power the radio up
  pinMode(RF95_RST, OUTPUT);
  digitalWrite(RF95_RST, HIGH);

  Serial.begin(9600);

  //manually reset the radio
   digitalWrite(RF95_RST, LOW);
   delay(10);
   digitalWrite(RF95_RST, HIGH);
   delay(10);

  //try initializing the mesh
  if (!mesh.init())
  {
    Serial.println("Radio initialization failed!");
    while(1);
  }

  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("Failed to set frequency!");
    while(1);
  }
}

void loop()
{
  rxLength = RH_MESH_MAX_MESSAGE_LEN;
  if (mesh.recvfromAckTimeout(rxBuf, &rxLength, MESH_LISTEN_DURATION, &sender))
  {
    if (rxLength >= 3)
    {
      rxString = String((char*)rxBuf);
      Serial.println(rxString + " " + String(sender));

      //If it's a route request, reply with an acknowledgement.
      if (rxString.startsWith("RR"))
      {
        txString = "EA";
        txLength = txString.length() + 1;
        txString.getBytes(txBuf, txLength);
        mesh.sendtoWait(txBuf, txLength, sender);
      }
    }
  }

  //Check for incoming serial traffic and push commands to the network
  serialBytesAvailable = Serial.available();
  if (serialBytesAvailable > 0)
  {
    serialBufferLength = Serial.readBytes(serialBuffer, serialBytesAvailable);
    serialBuffer[serialBufferLength] = 0;//Add a terminal null character
    serialString = String(serialBuffer);
    serialString.trim();

    //OK command is a status request. The target should reply with "AOK"
    //and a millis() output
    if (serialString.startsWith("OK"))
    {//Command must be formatted "OK ###" where ### is a radio ID
      if (serialString.length() >= 6)
      {//Make sure it's long enough
        targetID = serialString.substring(3,6).toInt();
        if (targetID > 0)
        {//toInt returns 0 if the parse failed
          txString = "OK";
          txLength = txString.length() + 1;
          txString.getBytes(txBuf, txLength);
          mesh.sendtoWait(txBuf, txLength, targetID);
        }
      }
    }

    //OK command is a status request. The target should reply with "AOK"
    //and a millis() output
    if (serialString.startsWith("SF"))
    {//Command must be formatted "SF ###" where ### is a radio ID
      if (serialString.length() >= 6)
      {//Make sure it's long enough
        targetID = serialString.substring(3,6).toInt();
        if (targetID > 0)
        {//toInt returns 0 if the parse failed
          txString = "SF";
          txLength = txString.length() + 1;
          txString.getBytes(txBuf, txLength);
          mesh.sendtoWait(txBuf, txLength, targetID);
        }
      }
    }
  }

  delay(5);
}
