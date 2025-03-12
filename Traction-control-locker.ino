#include <SPI.h>
#include <mcp_can.h>
#include <PID_v1.h>

#define CAN_CS_PIN 10  // Chip Select pin for MCP2515
MCP_CAN CAN(CAN_CS_PIN);

// Actuator and Control Pins
#define BRAKE_ACTUATOR_LEFT 5  
#define BRAKE_ACTUATOR_RIGHT 6  
#define MANUAL_LOCK_LEFT 7  
#define MANUAL_LOCK_RIGHT 8  
#define LOCKER_SYSTEM_TOGGLE 9  

// PID Control
double setpoint = 0;  
double input, output;  
double Kp = 2.0, Ki = 0.5, Kd = 0.1;  

PID tractionControl(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// System Status
bool lockerSystemEnabled = true;  

void setup() {
    Serial.begin(115200);
    
    pinMode(BRAKE_ACTUATOR_LEFT, OUTPUT);
    pinMode(BRAKE_ACTUATOR_RIGHT, OUTPUT);
    pinMode(MANUAL_LOCK_LEFT, INPUT_PULLUP);
    pinMode(MANUAL_LOCK_RIGHT, INPUT_PULLUP);
    pinMode(LOCKER_SYSTEM_TOGGLE, INPUT_PULLUP);

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

    // Read locker system toggle switch
    lockerSystemEnabled = digitalRead(LOCKER_SYSTEM_TOGGLE) == LOW;

    if (CAN_MSGAVAIL == CAN.checkReceive() && lockerSystemEnabled) {
        CAN.readMsgBuf(&canId, &len, buf);
        
        if (canId == 0x200) {  // Example CAN ID for wheel speed sensors
            int wheelSpeedLeft = buf[0] << 8 | buf[1];  
            int wheelSpeedRight = buf[2] << 8 | buf[3];  

            input = abs(wheelSpeedLeft - wheelSpeedRight);  
            tractionControl.Compute();

            // If one wheel is slipping (spinning 5km/h+ faster than the other)
            if (abs(wheelSpeedLeft - wheelSpeedRight) > 5) {
                Serial.println("Wheel slip detected! Engaging traction control...");
                
                if (wheelSpeedLeft > wheelSpeedRight) {
                    Serial.println("Applying 57% brake force to left wheel.");
                    analogWrite(BRAKE_ACTUATOR_LEFT, 145);  // 57% braking power
                    analogWrite(BRAKE_ACTUATOR_RIGHT, 0);
                } else {
                    Serial.println("Applying 57% brake force to right wheel.");
                    analogWrite(BRAKE_ACTUATOR_RIGHT, 145);
                    analogWrite(BRAKE_ACTUATOR_LEFT, 0);
                }
            }
            // If one wheel is stuck (0 speed) and the other is spinning
            else if (wheelSpeedLeft == 0 && wheelSpeedRight > 0) {
                Serial.println("Left wheel stuck! Applying 75% brake force to right.");
                analogWrite(BRAKE_ACTUATOR_RIGHT, 190);  
            }
            else if (wheelSpeedRight == 0 && wheelSpeedLeft > 0) {
                Serial.println("Right wheel stuck! Applying 75% brake force to left.");
                analogWrite(BRAKE_ACTUATOR_LEFT, 190);
            }
            // If both wheels are moving within range, release brakes
            else {
                analogWrite(BRAKE_ACTUATOR_LEFT, 0);
                analogWrite(BRAKE_ACTUATOR_RIGHT, 0);
                Serial.println("Wheels balanced. Releasing brakes.");
            }
        }
    }

    // Manual Lock Activation
    if (digitalRead(MANUAL_LOCK_LEFT) == LOW) {
        analogWrite(BRAKE_ACTUATOR_LEFT, 255);
        Serial.println("Manual lock engaged for left wheel.");
    }
    if (digitalRead(MANUAL_LOCK_RIGHT) == LOW) {
        analogWrite(BRAKE_ACTUATOR_RIGHT, 255);
        Serial.println("Manual lock engaged for right wheel.");
    }

    // If locker system is OFF, ensure no brakes are applied
    if (!lockerSystemEnabled) {
        analogWrite(BRAKE_ACTUATOR_LEFT, 0);
        analogWrite(BRAKE_ACTUATOR_RIGHT, 0);
        Serial.println("Locker system OFF. Brakes disengaged.");
    }
}
