/* Mesh network relay node main source file
* University of Arizona ENG 498 A&B Team 17023
* Team TANKS
*
* Main file for firmware for the Mesh Network Relay Node subsystem
* Designed for the Adafruit Feather M0 LoRa, simply a relay node that
* relays mesh network packets or responds to status requests.
*
* Pins are as follows:
* Board Pin#     Description
* D4 (Internal)  LoRa Reset
* D3 (Internal)  LoRa IRQ
* D8 (Internal)  LoRa chip select
* More as components are included
* TODO: DIP siwtches for ID when PCB is fabbed
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

//Constant and settings
#define RF95_FREQ 915.0                //In MHz, License free band
#define MESH_ADDRESS 2
#define MESH_ENDPOINT_ADDRESS 0        //Mesh network endpoint address
#define MESH_ACK_TIMEOUT 4U            //Time to wait for an acknowledgement
#define MESH_CONNECT_FLASH_SECONDS 5   //Number of seconds to flash after power
                                       //if mesh network connected
#define ULONG_MAX 4294967295           //2^32 - 1

//Libraries instantiated
RH_RF95 rf95(LORA_CS_PIN, LORA_IRQ_PIN);
RHMesh mesh(rf95, MESH_ADDRESS);

//globals for buffer handling
uint8_t txBuf[RH_MESH_MAX_MESSAGE_LEN];
String txString;
uint8_t txResult;
uint8_t txLength;
uint8_t rxBuf[RH_MESH_MAX_MESSAGE_LEN];
String rxString;
uint8_t rxLength;
uint8_t rxSender;

/* States for FSM type operations, allowing for multiple simultaneous
* functions (mostly) without blocking flow.
* State            Function
* -----            --------
* routeRequest     Send a route request to the mesh network endpoint to show if
*                  this node is in range of a contiguous mesh to the endpoint
* routeRequestRetryTry again if the initial request fails.
* waitForRouteAck  Wait to receive an acknowledgement from the endpoint.
* idle             Relay radio packets unless this particular node receives an
*                  addressed packet and act on it.
*/
enum MeshNetworkNodeState { routeRequest, routeRequestRetry, waitForRouteAck, idle};
MeshNetworkNodeState mnrnState = routeRequest;

void setup()
{
    // Set up pins
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

void loop() {
  switch (mnrnState)
  {
    case routeRequest:
      Serial.print("Sending route request 1: ");
      txString = "RR";
      txLength = txString.length()+1;
      txString.getBytes(txBuf, txLength);
      txResult = mesh.sendtoWait(txBuf, txLength, MESH_ENDPOINT_ADDRESS);
      if (txResult == RH_ROUTER_ERROR_NONE)
      {
        Serial.println("Sent.");
        mnrnState = waitForRouteAck;
      }
      else if (txResult == RH_ROUTER_ERROR_NO_ROUTE)
      {
        Serial.println("Not sent, retrying.");
        mnrnState = routeRequestRetry;
      }
      else
      {
        Serial.println("Not sent, error.");
        mnrnState = idle;
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
        mnrnState = waitForRouteAck;
      }
      else
      {
        Serial.println("Not sent, error.");
        mnrnState = idle;
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
      mnrnState = idle;
    break;
    case idle:
    if (mesh.recvfromAck(rxBuf, &rxLength))
    {
      rxString = String((char*)rxBuf);
      if (rxString.startsWith("OK"))
      {//Status check
        txString = "AOK " + String(millis());
        txLength = txString.length()+1;
        txString.getBytes(txBuf, txLength);
        txResult = mesh.sendtoWait(txBuf, txLength, MESH_ENDPOINT_ADDRESS);
      }
    }
    break;
  }
  delay(5);
}
