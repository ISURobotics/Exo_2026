/*
CamRo
Cameron Coward cameroncoward.com
Serial Hobbyism serialhobbyism.com

Works with both Arduino Uno R4 Minima and Arduino Uno R4 WiFi
(solder the corresponding bridge on the shield)
*/

#include <FastLED.h>
#include <Arduino_CAN.h>
#include <arduino-timer.h>

#define START_BUTTON    6       // Start button pin on Arduino
#define HOME_BUTTON     7       // Home button pin on Arduino
#define ARM_ANGLE_POT   A5      // Pin for arm angle potentiometer
#define ARM_SPEED_POT   A4      // Pin for arm speed potentiometer
#define TABLE_SPEED_POT A3      // Pin for table speed potentiometer
#define NEOPIXEL_PIN    2       // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT     1       // Number of NeoPixels
#define ACCEL           5000    // acceleration value to use for shoulder, elbow, and table movements
#define POS_TOL         0.25    // how close (in degrees, plus or minus) the reported position needs to be to the desired position to consider it "reached"
#define HOM_SPD         1000    // this is the speed at which the arm motor moves when homing
#define HOM_CUR_THRESH  4.0     // if arm motor draws more current than this, it is hitting stop and that is home
#define HOM_BACK        8.0     // how many degrees the arm moves back after touching the stop
#define BRAKE_CUR       4.0     // how much current to use when braking arm motor

CRGB leds[PIXEL_COUNT];                 // for the status LED

auto timer = timer_create_default();    // to update motors at regular intervals

// store button states
bool startButtonStateOld = HIGH;
bool startButtonState = HIGH;
bool homeButtonStateOld = HIGH;
bool homeButtonState = HIGH;

// has the arm been homed?
bool homed = false;

// motor IDsset these in the CubeMars tool! Will not work unless these match!
const int armMotorID = 106;
const int tableMotorID = 107;

// for motor updates when moving
int motorToUpdate = 0;
int armAngle = 0;
int armSpeed = 0;
int tableSpeed = 0;

// values received from Arm motor via CAN updates
float reportedArmPos = 0.1;
float reportedArmSpd = 0.1;
float reportedArmCur = 0.1;
int reportedArmTmp = 1;
int reportedArmErr = 1;

// values received from Table motor via CAN updates
float reportedTablePos = 1.0;
float reportedTableSpd = 1.0;
float reportedTableCur = 1.0;
int reportedTableTmp = 1;
int reportedTableErr = 1;

// required for motor control
enum AKMode {
  AK_PWM = 0,
  AK_CURRENT,
  AK_CURRENT_BRAKE,
  AK_VELOCITY,
  AK_POSITION,
  AK_ORIGIN,
  AK_POSITION_VELOCITY,
};

void setup() {
  Serial.begin(115200);
  FastLED.addLeds<NEOPIXEL, NEOPIXEL_PIN>(leds, PIXEL_COUNT);
  pinMode(START_BUTTON, INPUT_PULLUP);
  pinMode(HOME_BUTTON, INPUT_PULLUP);
  leds[0] = CRGB::Red;
  FastLED.show();
  if (!CAN.begin(CanBitRate::BR_1000k))
  {

  }
  Serial.println("CAN initialized.");
  home(); // get ready to home arm after startup
}

void loop() {
  timer.tick();   // advance the timer

  // check for CAN messages. If they exist, process them.
  if (CAN.available()) {
    processCAN();
  }

  // can start movement if arm is homed.
  if (homed == true) {
    startButtonState = digitalRead(START_BUTTON);
    if (startButtonState == LOW) {
      startButtonStateOld = LOW;
      run();
    }
  }
}

void home(){

  // check for button press
  while (homeButtonState == HIGH) {
    homeButtonState = digitalRead(HOME_BUTTON);
  }

  // make status LED yellow to indicate homing
  leds[0] = CRGB::Orange;
  FastLED.show();

  // check CAN to clear buffer
  if (CAN.available()) {
    processCAN();
  }

  Serial.println("Homing.");

  // begin moving arm until motor current exceeds threshold, indicating it has hit the stop
  while (reportedArmCur < HOM_CUR_THRESH) {
    comm_can_set_rpm(armMotorID, HOM_SPD);

    if (CAN.available()) {
      processCAN();
    }
  }

  Serial.println("Found home.");

  // set arm arngle back slightly, so it isn't pushed up against the stop
  armAngle = reportedArmPos - HOM_BACK;

  // factor in tolerance in reported position
  float armAngleDiff = abs(reportedArmPos - armAngle);

  // move arm to new postion until it falls within that tolerance
  while (armAngleDiff > POS_TOL) {
    if (CAN.available()) {
      processCAN();
    }

    comm_can_set_pos_spd(armMotorID, armAngle, HOM_SPD, ACCEL);

    armAngleDiff = abs(reportedArmPos - armAngle);

    if (armAngleDiff < POS_TOL) {
      break;
    }
  }

  if (CAN.available()) {
      processCAN();
  }

  delay(5);

  // set origin at current angles and change desired angles to 0 to prevent movment

  comm_can_set_origin(armMotorID, 0);
  armAngle = 0.0;
  armSpeed = 0.0;
  tableSpeed = 0.0;

  if (CAN.available()) {
      processCAN();
  }
  
  delay (5);

  comm_can_set_origin(tableMotorID, 0);

  // update motors so they don't try to move to old set positions
  if (CAN.available()) {
      processCAN();
  }

  Serial.print("Arm position: ");
  Serial.println(reportedArmPos);
  Serial.print("Table position: ");
  Serial.println(reportedTablePos);

  // set status LED to green to indicate readiness
  leds[0] = CRGB::Green;
  FastLED.show();
  homed = true;

  // begin updating motors every 20 milliseconds
  timer.every(20, updateMotors);
}

void run(){

  // not in home position after this
  homed = false;

  leds[0] = CRGB::Blue;
  FastLED.show();

  // check to see how to move motors
  armAngle = analogRead(ARM_ANGLE_POT);
  armSpeed = analogRead(ARM_SPEED_POT);
  tableSpeed = analogRead(TABLE_SPEED_POT);
  
  // map potentiometer inputs to valid ranges
  armAngle = map(armAngle, 0, 1023, 0, -173);
  armSpeed = map(armSpeed, 0, 1023, 0, 3000);
  tableSpeed = map(tableSpeed, 0, 1023, 1000, -1000);

  // see if current position is close to the desired position
  float armAngleDiff = abs(reportedArmPos - armAngle);

  // if that is further than the specified tolerance
  while (armAngleDiff > POS_TOL) {
    timer.tick();

    if (CAN.available()) {
      processCAN();
    }

    armAngleDiff = abs(reportedArmPos - armAngle);

    if (armAngleDiff < POS_TOL) {
      break;
    }

    // user can push start button again to prematurely end the movement
    startButtonState = digitalRead(START_BUTTON);
    if (startButtonState == LOW && startButtonStateOld == HIGH) {
      startButtonStateOld = LOW;
      break;
    }
    startButtonStateOld = startButtonState;
  }

  armAngle = 0;

  armAngleDiff = abs(reportedArmPos - armAngle);

  while (armAngleDiff > POS_TOL) {
    timer.tick();

    if (CAN.available()) {
      processCAN();
    }

    armAngleDiff = abs(reportedArmPos - armAngle);

    if (armAngleDiff < POS_TOL) {
      break;
    }
  }

  armSpeed = 0;
  tableSpeed = 0;

  leds[0] = CRGB::Green;
  FastLED.show();  

  homed = true;
}

bool updateMotors(void *){

  // each time the timer triggers this function, update one of the motors (but not the other). This prevents overloading the CAN bus.
  switch (motorToUpdate){
    case 0:
      updateArm();
      break;
    case 1:
      updateTable();
      break;
    default:
      break;
  }

  motorToUpdate = motorToUpdate + 1;
  if (motorToUpdate > 1){motorToUpdate = 0;}
}

void updateArm(){
  // move the arm motor
  comm_can_set_pos_spd(armMotorID, armAngle, armSpeed, ACCEL);
}

void updateTable(){
  // move the table motor
  comm_can_set_rpm(tableMotorID, tableSpeed);
}

// ||||||||||||-----------CAN Bus Functions-----------||||||||||||

void processCAN() {
  // looks at all messages on the CAN bus and parses/stores the relevant info from each motor

  CanMsg const msg = CAN.read();

  /* CAN ID reporting is very weird.

    it is the motor ID in Hex (like 6A), but with a '1' in front (like 16A)
    and THAT value is then converted to decimal (like 362)

    that's because it is sending hex address bytes as Little Endian: 6A-01
    or Big Endian: 01-6A

    so, we just want the relevant byte (6A) and to ignore th other (01)

    like this:

    byte motor_id = lowByte(msg.getStandardId());
  
    Serial.print("ID: ");
    Serial.print(lowByte(motor_id), HEX);

  */

  byte motor_id = lowByte(msg.getStandardId());
  int16_t pos_int = msg.data[0] << 8 | msg.data[1];
  int16_t spd_int = msg.data[2] << 8 | msg.data[3];
  int16_t cur_int = msg.data[4] << 8 | msg.data[5];
  
  if (motor_id == byte(armMotorID)) {
    reportedArmPos = (float)(pos_int * 0.1f);
    reportedArmSpd = (float)(spd_int * 10.0f);
    reportedArmCur = (float)(cur_int * 0.01f);
    reportedArmTmp = msg.data[6];
    reportedArmErr = msg.data[7];
  } else if (motor_id == byte(tableMotorID)) {
    reportedTablePos = (float)(pos_int * 0.1f);
    reportedTableSpd = (float)(spd_int * 10.0f);
    reportedTableCur = (float)(cur_int * 0.01f);
    reportedTableTmp = msg.data[6];
    reportedTableErr = msg.data[7];
  } else {
  }
}

// ||||||||||||-----------CubeMars Motor Functions-----------||||||||||||

/*

Motors:

Shoulder: 105 (69 in hex)
Elbow: 106 (6A in hex)
Wrist: 107 (6B in hex)

Address them by hex, like this:

comm_can_set_origin(0x69, 0);

Or this:

comm_can_set_pos_spd(0x6C, 1800000, 20000, 30000);

The position value is of type int32, and the range is -360,000,000-360,000,000, representing
-36000-36000.

So position is as follows:

Number of degrees, up to 100 turns in either direction.

That, multiplied by 10,000.0, which should yield a number suitable to convert to a 32-bit integer

*/

uint32_t canId(int id, AKMode Mode_set) {
  uint32_t mode;
  mode = Mode_set;
  return uint32_t(id | mode << 8);
}

void comm_can_transmit_eid(uint32_t id, const uint8_t* data, uint8_t len) {
  uint8_t i = 0;
  if (len > 8) {
    len = 8;
  }
  uint8_t buf[len];
  for (i = 0; i < len; i++) {
    buf[i] = data[i];
  }
  //CAN.sendMsgBuf(id, 1, len, buf); //Note that data frame is extended so I did write "1" in here
  CanMsg const msg(CanExtendedId(id), len, buf);
  CAN.write(msg);

}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t* index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t* index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}


void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_PWM), buffer, send_index);
}

void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT), buffer, send_index);
}

void comm_can_set_cb(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_CURRENT_BRAKE), buffer, send_index);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_VELOCITY), buffer, send_index);
}

void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  int32_t setPosition = pos * 10000;
  buffer_append_int32(buffer, setPosition, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION), buffer, send_index);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA) {
  int32_t send_index = 0;
  int16_t send_index1 = 4;
  uint8_t buffer[8];
  int32_t setPosition = pos * 10000;
  buffer_append_int32(buffer, setPosition, &send_index);
  buffer_append_int16(buffer, spd/10.0, &send_index1);
  buffer_append_int16(buffer, RPA/10.0, &send_index1);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_POSITION_VELOCITY), buffer, send_index1);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)set_origin_mode, &send_index);
  comm_can_transmit_eid(canId(controller_id, AKMode::AK_ORIGIN), buffer, send_index);
}

void motor_receive(float* motor_pos, float* motor_spd, float* motor_cur, int8_t* motor_temp, int8_t* motor_error, uint8_t* rx_message) {
  byte len = 0;
  byte buf[8];
  unsigned long canId;

  //CAN.read(&len, buf);
  int16_t pos_int = buf[0] << 8 | buf[1];
  int16_t spd_int = buf[2] << 8 | buf[3];
  int16_t cur_int = buf[4] << 8 | buf[5];
  *motor_pos = (float)(pos_int * 0.1f);
  *motor_spd = (float)(spd_int * 10.0f);
  *motor_cur = (float)(cur_int * 0.01f);
  *motor_temp = buf[6];
  *motor_error = buf[7];
}