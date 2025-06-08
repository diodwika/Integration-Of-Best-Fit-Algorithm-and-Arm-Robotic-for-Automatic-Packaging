#include <WiFi.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

// WiFi Configuration
const char* ssid = "Sepsal";
const char* password = "salsa.279";

// MQTT Configuration
const char* mqtt_broker = "192.168.207.178";
const int mqtt_port = 1883;
const char* mqtt_topic_send = "esp32/commands";
const char* mqtt_topic_receive = "esp32/coordinates";
const char* mqtt_topic_error = "esp32/error";
const char* mqtt_topic_request_correction = "esp32/request_correction";
const char* mqtt_topic_acknowledge_status = "esp32/acknowledge_status";

WiFiClient espClient;
PubSubClient client(espClient);

// Pin Definitions
#define PUL_PIN 19
#define DIR_PIN 18
#define SERVO_RIGHT_PIN 26
#define SERVO_LEFT_PIN 14
#define SERVO_ELBOW1_PIN 32
#define SERVO_ELBOW2_PIN 13
#define SERVO_GRIPPER 12

// Constants
const int STEPS_PER_REV = 8000;
const float DEGREE_PER_STEP = 360.0 / STEPS_PER_REV;
const float GEAR_RATIO = 3.47222222;
const float ROBOT_X = 46.0;
const float ROBOT_Y = 0.0;
const float ROBOT_Z = 13;
const float DEGREE_PER_CM_ERROR = 2.0;

// Arm Lengths
float l1 = 19.5;
float l2 = 12.0;
float l3 = 24.0;

// Servo Objects
Servo servoRight;
Servo servoLeft;
Servo servoElbow1;
Servo servoElbow2;
Servo servoGripper;

// Variables
float targetX, targetY, targetZ;
float targetAngle_1, targetAngle_2, targetAngle_3, targetAngle_4;
float thetaAngle_1, thetaAngle_3, theta_3a, thetaAngle_4, theta_4a;
float theta_1, theta_2, theta_3, theta_4;
float lastTheta = 0;
int direction;
long stepsToMove;
float currentAngle = 0;

// Servo positions
float currentServoRightPos = 90;
float currentServoLeftPos = 90;
float currentServoElbow1Pos = 150;
float currentServoElbow2Pos = 180;
float currentServoGripperPos = 180;

// Error correction
float errorX = 0;
float errorY = 0;
bool receivedError = false;
bool receivedOkSignal = false;
bool correctionCompleted = false;

// Correction State Machine
enum CorrectionState {
  CORRECTION_IDLE,
  RETURNING_SHOULDER,
  CORRECTING_STEPPER,
  RESTORING_SHOULDER,
  CORRECTION_COMPLETE
};
CorrectionState correctionState = CORRECTION_IDLE;

// Main State Machine
enum State { 
  WAITING_FOR_CAN, 
  MOVING_TO_CAN, 
  CORRECTING_CAN_POSITION,
  WAITING_FOR_PLACEMENT, 
  MOVING_TO_PLACEMENT,
  CORRECTING_PLACEMENT_POSITION
};
State currentState = WAITING_FOR_CAN;
String received_can_coordinates = "";
String received_placement_coordinates = "";

// Variables to store shoulder position before correction
float shoulderRightPosBeforeCorrection = 0;
float shoulderLeftPosBeforeCorrection = 0;

void setup() {
  Serial.begin(115200);
  
  // Setup WiFi and MQTT
  setup_wifi();
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);
  
  // Setup Stepper and Servos
  pinMode(PUL_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  servoRight.attach(SERVO_RIGHT_PIN);
  servoLeft.attach(SERVO_LEFT_PIN);
  servoElbow1.attach(SERVO_ELBOW1_PIN);
  servoElbow2.attach(SERVO_ELBOW2_PIN);
  servoGripper.attach(SERVO_GRIPPER);
  
  // Set initial servo positions
  servoRight.write(currentServoRightPos);
  servoLeft.write(currentServoLeftPos);
  servoElbow1.write(currentServoElbow1Pos);
  servoElbow2.write(currentServoElbow2Pos);
  servoGripper.write(currentServoGripperPos);

  Serial.println("System Initialized. Waiting for coordinates...");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  switch (currentState) {
    case WAITING_FOR_CAN:
      if (received_can_coordinates != "") {
        processCoordinates(received_can_coordinates, "Can:");
        currentState = MOVING_TO_CAN;
        received_can_coordinates = "";
      }
      break;
      
    case MOVING_TO_CAN:
      if (moveToTarget()) {
        currentState = CORRECTING_CAN_POSITION;
        Serial.println("Initial movement to can completed, starting error correction");
        client.publish(mqtt_topic_request_correction, "error handling");
        Serial.println("Sent initial 'error handling' signal");
      }
      break;
      
    case CORRECTING_CAN_POSITION:
      if (receivedError) {
        correctPosition();
      }
      
      if (receivedOkSignal) {
        Serial.println("Received ok 1 signal, can position corrected");
        smoothMoveServo(servoGripper, currentServoGripperPos, 60, 20);
        client.publish(mqtt_topic_send, "1");
        currentState = WAITING_FOR_PLACEMENT;
        receivedOkSignal = false;
        correctionCompleted = false;
      }
      break;
      
    case WAITING_FOR_PLACEMENT:
      if (received_placement_coordinates != "") {
        processCoordinates(received_placement_coordinates, "Placement:");
        currentState = MOVING_TO_PLACEMENT;
        received_placement_coordinates = "";
      }
      break;
      
    case MOVING_TO_PLACEMENT:
      if (moveToTarget()) {
        currentState = CORRECTING_PLACEMENT_POSITION;
        Serial.println("Initial movement to placement completed, starting error correction");
        client.publish(mqtt_topic_request_correction, "error handling");
        Serial.println("Sent initial 'error handling' signal");
      }
      break;
      
    case CORRECTING_PLACEMENT_POSITION:
      if (receivedError) {
        correctPosition();
      }
      
      if (receivedOkSignal) {
        Serial.println("Received ok 2 signal, placement position corrected");
        smoothMoveServo(servoGripper, currentServoGripperPos, 180, 20);
        client.publish(mqtt_topic_send, "2");
        currentState = WAITING_FOR_CAN;
        receivedOkSignal = false;
        correctionCompleted = false;
      }
      break;
  }
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char message[length + 1];
  for (int i = 0; i < length; i++) {
    message[i] = (char)payload[i];
  }
  message[length] = '\0';
  Serial.println(message);

  if (strstr(topic, "coordinates")) {
    if (strstr(message, "Can:") && currentState == WAITING_FOR_CAN) {
      received_can_coordinates = message;
      Serial.println("Received can coordinates");
    } else if (strstr(message, "Placement:") && currentState == WAITING_FOR_PLACEMENT) {
      received_placement_coordinates = message;
      Serial.println("Received placement coordinates");
    }
  } else if (strstr(topic, "error")) {
    if (sscanf(message, "Error: %f, %f", &errorX, &errorY) == 2) {
      Serial.print("Received error values - X: ");
      Serial.print(errorX);
      Serial.print(" mm, Y: ");
      Serial.print(errorY);
      Serial.println(" mm");
      
      receivedError = true;
    }
  } else if (strstr(topic, "status")) {
    if (strcmp(message, "ok 1") == 0 && currentState == CORRECTING_CAN_POSITION) {
      receivedOkSignal = true;
    } else if (strcmp(message, "ok 2") == 0 && currentState == CORRECTING_PLACEMENT_POSITION) {
      receivedOkSignal = true;
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_topic_receive);
      client.subscribe(mqtt_topic_error);
      client.subscribe(mqtt_topic_acknowledge_status);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void processCoordinates(String coordinates, const char* prefix) {
  int prefixLen = strlen(prefix);
  int openParenPos = coordinates.indexOf('(', prefixLen);
  if (openParenPos == -1) {
    Serial.println("No opening parenthesis found");
    return;
  }

  int firstCommaPos = coordinates.indexOf(',', openParenPos);
  int secondCommaPos = coordinates.indexOf(',', firstCommaPos + 1);
  int closeParenPos = coordinates.indexOf(')', secondCommaPos);

  if (firstCommaPos != -1 && secondCommaPos != -1 && closeParenPos != -1) {
    targetX = coordinates.substring(openParenPos + 1, firstCommaPos).toFloat();
    targetY = coordinates.substring(firstCommaPos + 1, secondCommaPos).toFloat();
    targetZ = coordinates.substring(secondCommaPos + 1, closeParenPos).toFloat();
    
    Serial.print("Processing coordinates - X: ");
    Serial.print(targetX);
    Serial.print(", Y: ");
    Serial.print(targetY);
    Serial.print(", Z: ");
    Serial.println(targetZ);
    
    calculateAngles(targetX, targetY, targetZ);
  } else {
    Serial.println("Invalid coordinate format");
  }
}

void calculateAngles(float x, float y, float z) {
  // Calculate theta_1
  targetAngle_1 = atan2(y - ROBOT_Y, x - ROBOT_X) * 180.0 / PI;
  thetaAngle_1 = fmod((targetAngle_1 + 180.0), 180.0);
  theta_1 = 90 - (thetaAngle_1 + 5);

  direction = (theta_1 >= 0) ? 1 : -1;
  stepsToMove = fabs(theta_1) * GEAR_RATIO / DEGREE_PER_STEP;

  Serial.print("Theta 1: ");
  Serial.print(theta_1);
  Serial.println("°");

  float a = sqrt(pow(x - ROBOT_X, 2) + pow(y - ROBOT_Y, 2));
  float b = a - l3;
  float z_offset = z - ROBOT_Z;
  float c = sqrt(pow(b, 2) + pow(z_offset, 2));

  if (c > (l1 + l2) || c < fabs(l1 - l2)) {
    Serial.println("Target out of reach");
  } else {
    // Calculate theta_2
    float alpha_1 = atan2(z_offset, b);
    float alpha_2 = acos((pow(l1, 2) + pow(c, 2) - pow(l2, 2)) / (2 * l1 * c));
    targetAngle_2 = (alpha_1 + alpha_2) * 180.0 / PI;
    theta_2 = targetAngle_2;

    // Calculate theta_3
    targetAngle_3 = acos((pow(l1, 2) + pow(l2, 2) - pow(c, 2)) / (2.0 * l1 * l2)) + PI;
    thetaAngle_3 = targetAngle_3 * 180.0 / PI;
    theta_3a = 180 - (fmod(thetaAngle_3 + 180.0, 360.0));
    theta_3 = fabs(theta_3a) + 35;

    float beta_1 = acos((pow(l2,2) + pow(c,2) - pow(l1,2)) / (2 * l2 * c));
    float beta_2 = atan2(b, z_offset);
    targetAngle_4 = beta_1 + beta_2 + radians(90) + PI;
    thetaAngle_4 = targetAngle_4 * 180.0 / PI;
    theta_4a = fmod(thetaAngle_4 + 180.0, 360.0) - 180.0;
    theta_4 = (theta_4a + 15) + 90;

    Serial.print("Theta 2: ");
    Serial.println(theta_2);
    Serial.print("Theta 3: ");
    Serial.println(theta_3);
    Serial.print("Theta 4: ");
    Serial.println(theta_4);
    Serial.println("°");
  }
}

bool moveToTarget() {
  static bool stepperDone = false;
  static bool servosDone = false;
  static bool errorHandlingSent = false;
  
  if (!stepperDone) {
    returnServoToOrigin();
    returnStepperToOrigin();
    
    if (stepsToMove > 0) {
      Serial.print("Moving Stepper Motor, Steps: ");
      Serial.println(stepsToMove);
      moveStepper(stepsToMove, direction);
    }
    stepperDone = true;
    return false;
  }
  
  if (!servosDone) {
    moveServosToTarget();
    servosDone = true;
    return false;
  }
  
  if (!errorHandlingSent) {
    client.publish(mqtt_topic_request_correction, "error handling");
    Serial.println("Sent 'error handling' after completing IK movement");
    errorHandlingSent = true;
    return false;
  }

  stepperDone = false;
  servosDone = false;
  errorHandlingSent = false;
  return true;
}

void correctPosition() {
  static bool initialShoulderReturned = false;
  static bool stepperCorrected = false;
  static bool shoulderRestored = false;
  static bool errorHandlingSent = false;

  switch (correctionState) {
    case CORRECTION_IDLE:
      Serial.println("Starting correction process");
      // Store current shoulder positions before correction
      shoulderRightPosBeforeCorrection = currentServoRightPos;
      shoulderLeftPosBeforeCorrection = currentServoLeftPos;
      
      correctionState = RETURNING_SHOULDER;
      initialShoulderReturned = false;
      stepperCorrected = false;
      shoulderRestored = false;
      errorHandlingSent = false;
      break;

    case RETURNING_SHOULDER:
      if (!initialShoulderReturned) {
        Serial.println("Returning shoulder to origin");
        smoothMoveServo(servoRight, currentServoRightPos, 90, 20);
        smoothMoveServo(servoLeft, currentServoLeftPos, 90, 20);
        initialShoulderReturned = true;
      } else {
        correctionState = CORRECTING_STEPPER;
      }
      break;
      
    case CORRECTING_STEPPER:
      if (!stepperCorrected) {
        if (errorX != 0) {
          Serial.print("Correcting X position, Error: ");
          Serial.println(errorX);
          float angleCorrection = errorX * DEGREE_PER_CM_ERROR;
          long correctionSteps = fabs(angleCorrection) * GEAR_RATIO / DEGREE_PER_STEP;
          int correctionDirection = (errorX > 0) ? 1 : -1;
          moveStepper(correctionSteps, correctionDirection);
        }
        stepperCorrected = true;
        correctionState = RESTORING_SHOULDER;
      }
      break;
      
    case RESTORING_SHOULDER:
      if (!shoulderRestored) {
        Serial.println("Restoring shoulder to previous position");
        smoothMoveServo(servoRight, currentServoRightPos, shoulderRightPosBeforeCorrection, 20);
        smoothMoveServo(servoLeft, currentServoLeftPos, shoulderLeftPosBeforeCorrection, 20);
        shoulderRestored = true;
        correctionState = CORRECTION_COMPLETE;
      }
      break;
      
    case CORRECTION_COMPLETE:
      if (!errorHandlingSent) {
        Serial.println("Correction process completed");
        client.publish(mqtt_topic_request_correction, "error handling");
        errorHandlingSent = true;
        correctionCompleted = true;
        receivedError = false;
        
        // Reset state machine
        correctionState = CORRECTION_IDLE;
      }
      break;
  }
}

void returnServoToOrigin() {
  smoothMoveServo(servoRight, currentServoRightPos, 90, 20);
  smoothMoveServo(servoLeft, currentServoLeftPos, 90, 20);
  smoothMoveServo(servoElbow1, currentServoElbow1Pos, 150, 20);
  smoothMoveServo(servoElbow2, currentServoElbow2Pos, 150, 20);
}

void returnStepperToOrigin() {
  if (lastTheta != 0) {
    long returnSteps = abs(lastTheta * GEAR_RATIO / DEGREE_PER_STEP);
    int returnDirection = (lastTheta >= 0) ? -1 : 1;

    Serial.print("Returning to Origin, Steps: ");
    Serial.println(returnSteps);
    moveStepper(returnSteps, returnDirection);
    lastTheta = 0;
  }
}

void moveStepper(long steps, int dir) {
  digitalWrite(DIR_PIN, dir > 0 ? HIGH : LOW);
  delay(10);

  for (long i = 0; i < steps; i++) {
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(500);
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(500);
  }
  
  lastTheta += (dir > 0 ? 1 : -1) * steps * DEGREE_PER_STEP / GEAR_RATIO;
}

void moveServosToTarget() {
  smoothMoveServo(servoElbow2, currentServoElbow2Pos, theta_4, 20);
  smoothMoveServo(servoElbow1, currentServoElbow1Pos, theta_3, 20);
  smoothMoveServo(servoRight, currentServoRightPos, theta_2, 20);
  smoothMoveServo(servoLeft, currentServoLeftPos, 180 - theta_2, 20);
}

void smoothMoveServo(Servo &servo, float &currentPos, float targetPos, int stepDelay) {
  float stepSize = 1.0;
  if (currentPos < targetPos) {
    while (currentPos < targetPos) {
      currentPos += stepSize;
      if (currentPos > targetPos) currentPos = targetPos;
      servo.write(currentPos);
      delay(stepDelay);
    }
  } else if (currentPos > targetPos) {
    while (currentPos > targetPos) {
      currentPos -= stepSize;
      if (currentPos < targetPos) currentPos = targetPos;
      servo.write(currentPos);
      delay(stepDelay);
    }
  }
}