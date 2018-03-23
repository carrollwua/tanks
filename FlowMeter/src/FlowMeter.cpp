/* Flow meter main source file
* University of Arizona ENG 498 A&B Team 17023
* Team TANKS
*
* Main file for firmware for the Flow Monitor subsystem
* Designed for the Adafruit Feather M0 LoRa
* Monitors and transmits data from a Signet 2536 paddlewheel flow meter
* Flow meter is powered from an Adafruit Verter Buck/Boost controller
* Requests and receives data from the Camera Subsystem via NRF24L01+ ISM radio
*
* Pins are as follows:
* Board Pin#     Description
* D4 (Internal)  LoRa Reset
* D3 (Internal)  LoRa IRQ
* D8 (Internal)  LoRa chip select
* A1/D15         Voltage booster enable
* A2/D16         Flow meter signal
* More as components are included
*/
#define RH_MESH_MAX_MESSAGE_LEN 50

#include <arduino.h>
#include <RHMesh.h>
#include <RH_RF95.h>
#include <SPI.h>

//Pin macros
#define LORA_RESET 4
#define LORA_IRQ_PIN 3
#define LORA_CS_PIN 8
#define METER_ENABLE_PIN 16
#define METER_INPUT_PIN 15

//Constants and settings
#define SAMPLE_FREQUENCY 1UL           //in minutes
#define SAMPLE_DURATION 15UL           //in seconds
#define K_FACTOR_1_25_INCH 47.2        //Pulses per liter
#define RF95_FREQ 915.0                //In MHz, License free band
#define MESH_ADDRESS 1                 //TODO: Addressing by switches
#define MESH_ENDPOINT_ADDRESS 0        //Mesh network endpoint address
#define MESH_ACK_TIMEOUT 4U            //Time to wait for an acknowledgement
#define MESH_CONNECT_FLASH_SECONDS 5   //Number of seconds to flash after power
                                       //if mesh network connected
#define ULONG_MAX 4294967295           //2^32 - 1

//Function prototype
void meterPinISR();                    //Interrupt for meter signal

//Libraries instantiated
RH_RF95 rf95(LORA_CS_PIN, LORA_IRQ_PIN);
RHMesh mesh(rf95, MESH_ADDRESS);

//Some gloabls for time and record keeping
volatile unsigned long pulses;         //Count of pulses during each sample
unsigned long currentTick;
unsigned long msDifference;
unsigned long sampleStart;
unsigned long lastSample;
float flowRate;
uint8_t txBuf[RH_MESH_MAX_MESSAGE_LEN];
String txString;
uint8_t txResult;
unsigned int txLength;
uint8_t rxBuf[RH_MESH_MAX_MESSAGE_LEN];
String rxString;
uint8_t rxLength;
uint8_t rxSender;

/* States for FSM type operations, allowing for multiple simultaneous
* functions (mostly) without blocking flow.
* State            Function
* -----            --------
* idle             Wait for the next sample period and listen for radio packets
* beginSampling    Enable the voltage booster and start sampling pulses
* sampling         Continue sampling pulses until time has elapsed
* stopSampling     Disable the voltage booster
* calculateResults Use the k-value from the datasheet to calculate flow volume
* requestIR        Send a request packet to the Camera
* waitForIR        Wait for response from the Camera
* makePacket       Upon response received from the Camera encode a mesh packet
* transmitPacket   Push the packet to the mesh network and return to idle
*/
enum FlowMeterState { routeRequest, routeRequestRetry, waitForRouteAck,
                      meterIdle, beginSampling, sampling, stopSampling,
                      transmitPacket};
volatile FlowMeterState meterState = routeRequest;

void setup()
  {
    //Set up pins
    pinMode(METER_ENABLE_PIN, OUTPUT);
    pinMode(METER_INPUT_PIN, INPUT_PULLUP);
    pinMode(LORA_RESET, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    //Enable radio
    digitalWrite(LORA_RESET, HIGH);
    //TODO:Take serial out later
    Serial.begin(9600);
    //while (!Serial);

    //Manual radio reset
    delay(100);
    digitalWrite(LORA_RESET, LOW);
    delay(10);
    digitalWrite(LORA_RESET, HIGH);
    delay(10);

    //Setup functions
    //Initialize variables
    currentTick = millis();
    msDifference = 0;
    sampleStart = 0;
    flowRate = 0;
    pulses = 0;


    //Attach ISR for meter pin
    //attachInterrupt(digitalPinToInterrupt(METER_INPUT_PIN), meterPinISR, FALLING);

    //Start mesh network radio
    if (!mesh.init())
    {
      Serial.println("Radio initialization failed!");
      while(1);
    }
    Serial.println("Radio initialized.");

    if (!rf95.setFrequency(RF95_FREQ))
    {
      Serial.println("Failed to set frequency!");
      while(1);
    }
    Serial.println("Frequency set");
  }

  void loop()
  {
    //Store current timestamp
    currentTick = millis();
    if (currentTick < lastSample)
    {//Account for millis() overflow;
      msDifference = currentTick + (ULONG_MAX - lastSample);
    }
    else
    {
      msDifference = currentTick - lastSample;
    }
    //Check for millis() overflow, should happen once every 50 days or so
    switch (meterState)
    {
      case routeRequest:
        /*Try to contact the endpoint and display some connection status.
         *This state should only be encounterd on power on or reset.
         *If a route error occurs, move to a single retry state.
         *If no error occurs, move to a receive state.
         *If any other error occurs, move to active and try establishing route
         *  later.
        */
        Serial.print("Sending route request 1: ");
        txString = "RR";
        txLength = txString.length()+1;
        txString.getBytes(txBuf, txLength);
        txResult = mesh.sendtoWait(txBuf, txLength, MESH_ENDPOINT_ADDRESS);
        if (txResult == RH_ROUTER_ERROR_NONE)
        {
          Serial.println("Sent.");
          meterState = waitForRouteAck;
        }
        else if (txResult == RH_ROUTER_ERROR_NO_ROUTE)
        {
          Serial.println("Not sent, retrying.");
          meterState = routeRequestRetry;
        }
        else
        {
          Serial.println("Not sent, error.");
          meterState = meterIdle;
        }
      break;
      case routeRequestRetry:
      /*
       * An attempt at routing to the base station has been made and a route
       * error occurred. This retries sending the ack request once. If it's
       * successfully sent, move to a receive state. if not, move to idle and
       * try routing later.
      */
        Serial.print("Sending route retry: ");
        txString = "RR";
        txLength = txString.length()+1;
        txString.getBytes(txBuf, txLength);
        txResult = mesh.sendtoWait(txBuf, txLength, MESH_ENDPOINT_ADDRESS);
        if (txResult == RH_ROUTER_ERROR_NONE)
        {
          Serial.println("Sent.");
          meterState = waitForRouteAck;
        }
        else
        {
          Serial.println("Not sent, error.");
          meterState = meterIdle;
        }
      break;
      case waitForRouteAck:
        /*
         * Wait for a reply to the route request. If one is received flash the
         * onboard LED to let the user know connection established
        */
        Serial.print("Waiting for ack: ");
        rxLength = RH_MESH_MAX_MESSAGE_LEN;
        if (mesh.recvfromAckTimeout(rxBuf, &rxLength, MESH_ACK_TIMEOUT * 1000, &rxSender))
        {
          Serial.println("Message received.");
          Serial.print("Length: "); Serial.println(rxLength);
          Serial.print("Contents: ");
          rxString = String((char*)rxBuf);
          Serial.println(rxString);
          if (rxString.startsWith("EA"))
          {
            for (int i = 0; i < MESH_CONNECT_FLASH_SECONDS; i++)
            {
              digitalWrite(LED_BUILTIN, HIGH);
              delay (500);
              digitalWrite(LED_BUILTIN, LOW);
              delay (500);
            }
          }
        }
        else
        {
          Serial.println("Wait for ack timed out.");
        }
        meterState = meterIdle;
      break;
      case meterIdle:
        if (msDifference >= SAMPLE_FREQUENCY * 60 * 1000)
        {// If it's time to sample, do so
            meterState = beginSampling;
        }
        else
        {// If not, relay mesh network messages until it is time
          rxLength = RH_MESH_MAX_MESSAGE_LEN;

          if (mesh.recvfromAckTimeout(rxBuf, &rxLength, (SAMPLE_FREQUENCY * 60 * 1000) - msDifference))
          {
            rxString = String((char*)rxBuf);
            //TODO: Handle possible actions like sample request here
          }
        }
      break;
      case beginSampling:
      Serial.println("Beginning a sample...");
      lastSample = currentTick;
      sampleStart = currentTick;            //store start time
      pulses = 0;                           //reset the pulse count
      flowRate = 0;
      digitalWrite(METER_ENABLE_PIN, HIGH); //Enable the voltage boost
      meterState = sampling;
      attachInterrupt(digitalPinToInterrupt(METER_INPUT_PIN), meterPinISR, FALLING);
      break;
      case sampling:
      if (currentTick - sampleStart > SAMPLE_DURATION * 1000)
      {
         meterState = stopSampling;
      }
      break;
      case stopSampling:
      Serial.println("Stopping sampling");
      digitalWrite(METER_ENABLE_PIN, LOW);  //Disable voltage boost
      detachInterrupt(digitalPinToInterrupt(METER_INPUT_PIN));
      flowRate = ((pulses / K_FACTOR_1_25_INCH) / SAMPLE_DURATION);
      Serial.print("Sample rate: ");
      Serial.print(flowRate);
      Serial.println(" l/s.");
      meterState = transmitPacket;
      break;
      case transmitPacket:
      Serial.println("Building packet...");
      txString = String(MESH_ADDRESS) + " " + String(flowRate,2);
      Serial.print("Packet: ");
      Serial.println(txString);
      txLength = txString.length() + 1;
      Serial.print("Length: "); Serial.println(txLength);
      txString.getBytes(txBuf,  txLength);
      Serial.println("Sending packet:");
      txResult = mesh.sendtoWait(txBuf, txLength, MESH_ENDPOINT_ADDRESS);
      if(txResult == RH_ROUTER_ERROR_NONE)
      {
        Serial.println("Packet transmitted successfully.");
      }
      else if (txResult == RH_ROUTER_ERROR_NO_ROUTE)
      {
        Serial.println("Packet transmission failed, no route.");
        //Try again?
      }
      else if (txResult ==  RH_ROUTER_ERROR_UNABLE_TO_DELIVER)
      {
        Serial.println("Packet transmission failed, unable to deliver.");
      }
      else
      {
        Serial.println("Unknown packet transmission error.");
      }
      Serial.println("Done sending.");
      meterState = meterIdle;
      break;
    }

    delay(5);
  }

  //ISR for falling signal on meter input pin. Increments pulse count.
  void meterPinISR()
  {
    pulses++;
  }
