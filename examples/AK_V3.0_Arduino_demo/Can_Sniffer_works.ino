/*
  Exercise the MCP2515 CAN Bus Module (Receiving Module Code)
  Requires 2 Arduino boards and 2 CAN bus modules to create a link
  Module connects to power/ground and SPI bus.  Pin 10 used for CS.
*/

#include <SPI.h>          //SPI is used to talk to the CAN Controller
#include <mcp_can.h>

MCP_CAN CAN(10);          //set CAN Bus Chip Select to pin 10

unsigned char len = 0;    // length of received buffer
unsigned char buf[8];     // Buffer to hold up to 8 bytes of data
long unsigned int canID;       // Can message ID
//===============================================================================
//  Initialization
//===============================================================================
void setup()
{
  Serial.begin(115200);   // Initialize communication with Serial monitor

  Serial.println("CAN receiver test");
 // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s
  if(CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) 
    Serial.println("MCP2515 Initialized Successfully!");
  else 
    Serial.println("Error Initializing MCP2515...");

  CAN.setMode(MCP_NORMAL);   // Normal mode to allow messages to be received
}

//===============================================================================
//  Main
//===============================================================================
void loop()
{
    if(CAN_MSGAVAIL == CAN.checkReceive())    //check if data is coming in
    {
        CAN.readMsgBuf(&canID, &len, buf);    // Read data,  len: data length, buf: data buffer
 
        Serial.print("CAN ID: ");
        Serial.print(canID, HEX);     // print the CAN ID in HEX

        Serial.print("    Data Length: "); // Print the length of the received data
        Serial.print(len);
        Serial.print("    ");
        
        for(int i = 0; i<len; i++)    //loop on the incoming data to print each byte
        {
            Serial.print(buf[i]);     
            if(i<len-1) Serial.print(",");  // Separate the numbers for readability
        }
        Serial.println();
    }
}