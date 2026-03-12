#include <mcp_can.h>
#include <SPI.h>

const int SPI_CS_PIN = 10;   // try 9 too if needed
MCP_CAN CAN(SPI_CS_PIN);

const uint32_t EXT_ID = 0x868;   // MIT mode (8) + motor ID 0x68

void setup() {
  Serial.begin(500000);
  delay(1000);

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("CAN init fail");
    delay(200);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN init ok");
}

void loop() {
  // Exact packet from manual:
  // 00000868 0006667FFF8F57FF
  byte buf[8] = {0x00, 0x06, 0x66, 0x7F, 0xFF, 0x8F, 0x57, 0xFF};

  byte s = CAN.sendMsgBuf(EXT_ID, 1, 8, buf);

  Serial.print("send status = ");
  Serial.println(s);

  Serial.print("TX: ");
  for (int i = 0; i < 8; i++) {
    if (buf[i] < 16) Serial.print("0");
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    unsigned long rxId;
    byte len = 0;
    byte rbuf[8];
    CAN.readMsgBuf(&rxId, &len, rbuf);

    Serial.print("RX ID: 0x");
    Serial.println(rxId, HEX);

    Serial.print("RX DATA: ");
    for (int i = 0; i < len; i++) {
      if (rbuf[i] < 16) Serial.print("0");
      Serial.print(rbuf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  } else {
    Serial.println("No reply");
  }

  delay(200);
}