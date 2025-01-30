/*
Made by team 5 MCTA
*/

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <esp_now.h>
#include <WiFi.h>
HardwareSerial mySerial(1); // Use UART1
// Motor and Encoder Pins
#define PWM_PIN 1         // PWM pin for motor speed control
#define DIR_PIN1 3        // Direction pin 1
#define DIR_PIN2 22       // Direction pin 2
#define ENCODER_PIN_A 2   // Encoder A channel
#define ENCODER_PIN_B 15  // Encoder B channel
#define LIMIT_SWITCH_PIN 23 // Limit switch pin
#define STEP_PIN 33        // Connect to the STEP pin on the driver
#define DIR_PIN 32         // Connect to the DIR pin on the driver
#define LIMIT_SWITCH_PIN_X 35 // Limit switch pin
#define dirPin1 25 // Limit switch pin
#define pulsePin1 26 // Limit switch pin
#define RX 16
#define TX 17
#define suction 5 


int Homing_counter = 0; 
int To_Product_counter = 0; 
int Cuboidcount = 1;
int Cylindercount = 1;     
int receivedInt; 
int body_detected_Number = 0 ; 
String receivedMessage;
unsigned long long cuboid_move[2][4] = {{13,16,8,16},{-32,-32,-24,-24}};    
unsigned long long cylinder_move[2][4] = {{10,16,8,16},{-10,-13.5,-5.5}}; //{{8,16,8,16},{-13.5,-13.5,-5.5}};
#define X_axis 0
#define Y_axis 1

SemaphoreHandle_t HomingFinishedSemaphore;
SemaphoreHandle_t to_product_finished_semaphore;
SemaphoreHandle_t sorting_semaphore;
SemaphoreHandle_t suction_to_product_finished_semaphore;
SemaphoreHandle_t start_robotic_arm_smphr;

TaskHandle_t homing_task_handle;
TaskHandle_t PID_Task_handle; 
TaskHandle_t To_product_handle; 
TaskHandle_t sorting_handle; 
TaskHandle_t Adjustyhandler;
TaskHandle_t SortHandler;
TaskHandle_t suction_to_product_handle;
TaskHandle_t Wheel_feeding_handle;
TaskHandle_t Store_product_handle;
TaskHandle_t SortingTaskHandle;


// Wheel Feeding 

// Wheel Feeding


// Pin definitions
const int enablePin = 2;
const int dirPin = 14;
const int pulsePin = 12;
const int irSensor1Pin = 36; // Adjust to your IR sensor 1 pin
const int irSensor2Pin = 39; // Adjust to your IR sensor 2 pin
int mahmoud = 0;

// Motor speed and acceleration parameters
float stepsPerSecond = 200.0;        // Initial speed (steps per second)
float maxStepsPerSecond = 400.0;    // Lower maximum speed for high torque
float acceleration = 100.0;         // Slow acceleration for smooth operation
unsigned long waitTime;
unsigned long startTime; // Add this to fix undeclared variable

// Task handle
TaskHandle_t motorControlTaskHandle;


// Motor Z axis 

void rotateMotor(bool direction, int divisor) {
  pinMode(dirPin1, OUTPUT);
  pinMode(pulsePin1, OUTPUT);

  digitalWrite(dirPin1, direction); // Set direction based on the parameter
  digitalWrite(pulsePin1, LOW);

  float stepsPerSecond = 5.0;        // Start at a low speed
  float maxStepsPerSecond = 25.0;    // Max speed
  float acceleration = 1;           // Smaller step-up for smooth acceleration
  int waitTime;

  const int stepsPerRevolution = 200;  // Total steps for one full revolution
  const int targetSteps = stepsPerRevolution / divisor;  // Calculate steps based on divisor

  for (int step = 0; step < targetSteps; step++) {
    if (stepsPerSecond < maxStepsPerSecond) {
      stepsPerSecond += acceleration;
    }

    waitTime = int(1000000.0 / stepsPerSecond);

    if (waitTime > 10) {
      digitalWrite(pulsePin1, HIGH);
      delayMicroseconds(15);
      digitalWrite(pulsePin1, LOW);
      delayMicroseconds(waitTime - 15);
    }
  }

  digitalWrite(pulsePin1, LOW);
}


// X axis 
const float STEPS_PER_REV = 200;       // Steps per revolution (e.g., 200 for a 1.8° step angle)
const float MM_PER_REV = 8.0;          // Linear distance per revolution in mm (depends on lead screw or belt pitch)

// State variables
bool isMotorStopped = false; // Flag to indicate motor is permanently stopped

int calculateSteps(float distance_cm) {
    float distance_mm = distance_cm * 10; // Convert cm to mm
    return (distance_mm / MM_PER_REV) * STEPS_PER_REV / 8; // Adjusted calculation
}

void moveStepper(float position) {
    int steps = calculateSteps(position);

    if (steps > 0) {
        digitalWrite(DIR_PIN, HIGH); // Set direction forward
    } else {
        digitalWrite(DIR_PIN, LOW);  // Set direction backward
        steps = -steps;              // Convert steps to positive for backward movement
    }

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(2000);    // Adjust delay to control motor speed
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(2000);    // Same as above
    }
}

// PID Control Task
ESP32Encoder encoder;         // Encoder object
long lastEncoderPulse = 0;    // Last encoder count
float pos = 0;                // Current position
long prevT = 0;               // Previous time
float eprev = 0;              // Previous error
float eintegral = 0;          // Integral term

int targetPosition_cm_actual = 50; // Default target position in cm
float integralMin = -255;    // Minimum limit for integral term
float integralMax = 255;     // Maximum limit for integral term
float RADIUS = 1.1;          // in cm
float circumference = 2 * 3.14159 * RADIUS; // Calculate circumference
float PPR = 374;             // Pulses per revolution
float distancePerPulse = circumference / PPR; // Distance per pulse

void PID_Task(void *parameter) {
  while (true) {
    int targetPosition_cm = 4 * targetPosition_cm_actual;
    int targetPosition = targetPosition_cm * (1 / distancePerPulse);

    float Kp = 0.5;            // Proportional constant
    float Ki = Kp * 0.3;       // Integral constant
    float Kd = 0.05;           // Derivative constant

    long currentTime = millis();
    float dt = (currentTime - prevT) * 0.001; // Convert to seconds

    long encoderDiff = encoder.getCount() - lastEncoderPulse;
    pos += encoderDiff;

    int error = targetPosition - pos;
    float dedt = (error - eprev) / dt;

    eintegral += error * dt;
    if (eintegral > integralMax) {
      eintegral = integralMax;
    } else if (eintegral < integralMin) {
      eintegral = integralMin;
    }

    float u = Kp * error + Kd * dedt + Ki * eintegral;

    float pwr = fabs(u);
    if (pwr > 255) {
      pwr = 255; // Limit power to 255
    }

    if (u > 0) {
      digitalWrite(DIR_PIN1, HIGH);
      digitalWrite(DIR_PIN2, LOW);
    } else if (u < 0) {
      digitalWrite(DIR_PIN1, LOW);
      digitalWrite(DIR_PIN2, HIGH);
    }

    if (error != 0) {
      analogWrite(PWM_PIN, pwr);
    }

    eprev = error;
    prevT = currentTime;
    lastEncoderPulse = encoder.getCount();

    vTaskDelay(100 / portTICK_PERIOD_MS);
    vTaskResume(homing_task_handle);
  }
}

void homingTask(void *parameter) {
//  xSemaphoreTake(start_robotic_arm_smphr, portMAX_DELAY);
    while (true) {
        int limitSwitchState;
mySerial.println("Homing Robotic Arm");
        switch (Homing_counter) {
            case 0:
                rotateMotor(1,2);
                Homing_counter++;
                break;

            case 1:
                if (digitalRead(LIMIT_SWITCH_PIN) == 1) {
                    encoder.clearCount();
                    targetPosition_cm_actual = -10;
                    vTaskSuspend(homing_task_handle);
                    Homing_counter++;
                } else if (digitalRead(LIMIT_SWITCH_PIN) == 0) {
                    targetPosition_cm_actual = 50;
                    vTaskSuspend(homing_task_handle);
                }
                break;

            case 2:
                limitSwitchState = digitalRead(LIMIT_SWITCH_PIN_X);

                if (limitSwitchState == HIGH) {
                    moveStepper(-10);
                    isMotorStopped = true;
                    Homing_counter++; 
                    To_Product_counter = 0;
                } else if (limitSwitchState == LOW) { 
                    moveStepper(1);
                }
                break;

            case 3:
            delay(2000);
                xSemaphoreGive(HomingFinishedSemaphore);
                
                vTaskSuspend(homing_task_handle);
                break;

            default:
                break;
        }
    }
}

void To_Product_Task(void *parameter) {
  xSemaphoreTake(HomingFinishedSemaphore, portMAX_DELAY);
  mySerial.println("Going To Product");
  vTaskSuspend(homing_task_handle) ; 
  targetPosition_cm_actual = -24;
  delay(1000);
  moveStepper(-10);
  xSemaphoreGive(to_product_finished_semaphore);
  vTaskSuspend(To_product_handle);
  
}

void Suction_Task(void *parameter) {
  xSemaphoreTake(sorting_semaphore, portMAX_DELAY);
  targetPosition_cm_actual = -32;
  mySerial.println("Suction The product");
  delay(1000);
  moveStepper(-6);
  delay(1000);
  
  rotateMotor(0,7); 
  delay(1000);
  digitalWrite(suction, LOW);
  delay(2000); 
  rotateMotor(1,7); 
  delay(2000);
  
  xSemaphoreGive(suction_to_product_finished_semaphore);
  vTaskSuspend(suction_to_product_handle);

}

void Sorting(void *pvParameters) {
    xSemaphoreTake(to_product_finished_semaphore, portMAX_DELAY);
    mySerial.println("Sorting");
    delay(6000);
    xSemaphoreGive(sorting_semaphore);
   vTaskSuspend(SortingTaskHandle);

        // Check for timeout
    }

   

void Store_product_task(void *parameter) {
  xSemaphoreTake(suction_to_product_finished_semaphore, portMAX_DELAY);
while(1)
{
  mySerial.println("Storing Product");
  
  switch (receivedInt) {    // receivedInt
    case 1:
      moveStepper(cuboid_move[X_axis][Cuboidcount - 1]);
      targetPosition_cm_actual = cuboid_move[Y_axis][Cuboidcount - 1];
      Cuboidcount++; 
      delay(2000);
      rotateMotor(0,3); 
      digitalWrite(suction, HIGH);
      vTaskResume(Wheel_feeding_handle);
      delay(5000);
    

      Homing_counter = 0; 
      //vTaskSuspend(Store_product_handle);
      

      break;
    case 2:
      moveStepper(cylinder_move[X_axis][Cylindercount - 1]);
      targetPosition_cm_actual = cylinder_move[Y_axis][Cylindercount - 1];
      Cylindercount++;
      delay(2000);
      rotateMotor(0,3); 
      digitalWrite(suction, HIGH);
      vTaskResume(Wheel_feeding_handle);
      delay(5000);
      //vTaskSuspend(Store_product_handle);
      Homing_counter = 0; 
      break;
    
  }
  vTaskResume(homing_task_handle) ; 
   vTaskResume(To_product_handle) ; 
   vTaskResume(suction_to_product_handle) ; 
   
}
}

// Wheel Task 
void motorControlTask(void *parameter) {
  // Initialize motor control pins
  pinMode(enablePin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(pulsePin, OUTPUT);
  pinMode(irSensor1Pin, INPUT);
  pinMode(irSensor2Pin, INPUT);

  digitalWrite(enablePin, LOW); // Enable motor driver
  digitalWrite(dirPin, HIGH);   // Set initial direction
  digitalWrite(pulsePin, LOW);

  // Motor control loop
  while (true) {
    // Limit speed to the defined maximum
    if (stepsPerSecond < maxStepsPerSecond) {
      waitTime = (unsigned long)(1000000.0 / stepsPerSecond); // Calculate delay between steps
      stepsPerSecond += acceleration * (1.0 / stepsPerSecond); // Gradual acceleration
    }

    // Ensure valid waitTime to avoid negative or invalid delay
    if (waitTime > 10) {
      if (digitalRead(irSensor1Pin) == LOW) {
        mahmoud = 1;
        startTime = millis(); // Record the start time
      }

      while (mahmoud == 1) {
        // Exit the loop after 4000 ms
        if (digitalRead(irSensor2Pin) == LOW) {
          delay(3000) ; 
          mahmoud = 0; // Reset mahmoud to exit the loop
          
        }

        // Generate step pulses
        digitalWrite(pulsePin, HIGH);
        delayMicroseconds(10);          // Pulse HIGH for 10 µs
        digitalWrite(pulsePin, LOW);
        delayMicroseconds(waitTime - 10); // Remaining wait time
      }
    }
    xSemaphoreGive(start_robotic_arm_smphr);
 
    // Yield control to other tasks
    vTaskDelay(1 / portTICK_PERIOD_MS); // Avoid busy-waiting
  }
}


void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, -1, TX); // TX pin is GPIO17
    mySerial.begin(9600, SERIAL_8N1, RX, -1); // RX pin is GPIO16
    HomingFinishedSemaphore = xSemaphoreCreateBinary();
    to_product_finished_semaphore = xSemaphoreCreateBinary();
    sorting_semaphore = xSemaphoreCreateBinary();
    suction_to_product_finished_semaphore = xSemaphoreCreateBinary();
    start_robotic_arm_smphr = xSemaphoreCreateBinary();
    
    pinMode(LIMIT_SWITCH_PIN, INPUT);
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN1, OUTPUT);
    pinMode(DIR_PIN2, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LIMIT_SWITCH_PIN_X, INPUT);
    pinMode(suction, OUTPUT);
    
    
    encoder.attachFullQuad(ENCODER_PIN_A, ENCODER_PIN_B);
    encoder.clearCount();
    
    xTaskCreatePinnedToCore(PID_Task, "Motor Control Task", 2048, NULL, 1, &PID_Task_handle, 1);
    xTaskCreatePinnedToCore(homingTask, "Homing Task", 1000, NULL, 1, &homing_task_handle, 0);
    xTaskCreatePinnedToCore(To_Product_Task, "To product Task", 1000, NULL, 1, &To_product_handle, 0);
    xTaskCreatePinnedToCore(Suction_Task, "Suction_Task", 1000, NULL, 1, &suction_to_product_handle, 0);
    xTaskCreatePinnedToCore(Store_product_task, "Store_product_task", 1000, NULL, 1, &Store_product_handle, 0);
    xTaskCreatePinnedToCore(Sorting, "Sorting Camera Recieve", 1000, NULL, 1, &SortingTaskHandle, 0);
    

  xTaskCreate(
    motorControlTask,          // Task function
    "Wheel Control Feeding Task",      // Task name (for debugging)
    1000,                      // Stack size (in words)
    NULL,                      // Task parameters
    1,                         // Task priority
    &motorControlTaskHandle    // Task handle
  );


}

void loop()
{
   if (mySerial.available()) {
            receivedMessage = mySerial.readStringUntil('\n');
            receivedInt = receivedMessage.toInt();
            body_detected_Number = receivedInt ; 
   }       

}
