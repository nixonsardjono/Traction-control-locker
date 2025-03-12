#include <SPI.h>
#include <mcp_can.h>
#include <PID_v1.h>

#define CAN_CS_PIN 10  // Chip Select pin for MCP2515
MCP_CAN CAN(CAN_CS_PIN);

const int brakeActuator1 = 5;  // Brake actuator (Relay/Motor)
const int brakeActuator2 = 6;  
const int manualLock1 = 7;  // Manual lock button
const int manualLock2 = 8;  

double setpoint = 0;  // Ideal wheel speed difference
double input, output; // PID input/output values
double Kp = 2.0, Ki = 0.5, Kd = 0.1;  // PID tuning values

PID tractionControl(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  pinMode(brakeActuator1, OUTPUT);
  pinMode(brakeActuator2, OUTPUT);
  pinMode(manualLock1, INPUT_PULLUP);
  pinMode(manualLock2, INPUT_PULLUP);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("CAN Bus Initialized Successfully!");
  } else {
    Serial.println("CAN Bus Initialization Failed!");
    while (1);
  }
  CAN.setMode(MCP_NORMAL);
  
  tractionControl.SetMode(AUTOMATIC);
}

void loop() {
  long unsigned int canId;
  unsigned char len = 0;
  unsigned char buf[8];

  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&canId, &len, buf);
    
    if (canId == 0x200) {  // Example CAN ID for wheel speed sensors
      int wheelSpeed1 = buf[0] << 8 | buf[1];  // Front-left wheel
      int wheelSpeed2 = buf[2] << 8 | buf[3];  // Front-right wheel
      
      input = abs(wheelSpeed1 - wheelSpeed2);  // Difference in wheel speeds
      tractionControl.Compute();
      
      if (input > 10) {  // If wheel slip is detected
        Serial.println("Wheel slip detected! Engaging brakes...");
        digitalWrite(brakeActuator1, output > 50 ? HIGH : LOW);
        digitalWrite(brakeActuator2, output > 50 ? HIGH : LOW);
      } else {
        digitalWrite(brakeActuator1, LOW);
        digitalWrite(brakeActuator2, LOW);
      }
    }
  }

  // Manual locker activation
  if (digitalRead(manualLock1) == LOW) {
    digitalWrite(brakeActuator1, HIGH);
    Serial.println("Manual lock engaged for left wheel.");
  }
  if (digitalRead(manualLock2) == LOW) {
    digitalWrite(brakeActuator2, HIGH);
    Serial.println("Manual lock engaged for right wheel.");
  }
}
