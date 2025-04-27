// Teensy 4.1 Motor Control over CAN for Formula Student Car
// Features: Torque Vectoring, Traction Control, Launch Control, ABS (motor-only)
// Incremental torque adjustment logic to preserve prior control effects

#include <FlexCAN_T4.h>
#include <ADC.h>
#include <NXPMotionSense.h>

// === Global Variables and Constants ===
// CAN Bus
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;

// ADC and Wheel Speed
ADC adc;
const int pinFL = A0, pinFR = A1, pinRL = A2, pinRR = A3;

// Motor CAN IDs
const int motorFL_ID = 0x01;
const int motorFR_ID = 0x02;
const int motorRL_ID = 0x03;
const int motorRR_ID = 0x04;

// VCU CAN Input ID
const int vcuInputID = 0x100;

// Constants
const float maxFrontTorque = 0.5;
const float maxRearTorque = 1.0;
const float maxSlipRatio = 0.15;
const float launchSpeedThreshold = 2.0;

// Inputs
float throttleInput = 0.0;
float brakeInput = 0.0;
bool launchRequested = false;
float steeringAngle = 0.0;

// Wheel Speeds
float wheelSpeedFL = 0.0, wheelSpeedFR = 0.0;
float wheelSpeedRL = 0.0, wheelSpeedRR = 0.0;
float vehicleSpeed = 0.0;

// Previous Values for Delta Calculations
float wheelSpeedFLprev = 0.0, wheelSpeedFRprev = 0.0;
float wheelSpeedRLprev = 0.0, wheelSpeedRRprev = 0.0;
float vehicleSpeedprev = 0.0;

// Slip Ratios
float slipFL = 0.0, slipFR = 0.0, slipRL = 0.0, slipRR = 0.0;
float slipFLprev = 0.0, slipFRprev = 0.0, slipRLprev = 0.0, slipRRprev = 0.0;

// Torque Targets
float targetTorqueFL = 0.0, targetTorqueFR = 0.0;
float targetTorqueRL = 0.0, targetTorqueRR = 0.0;

// IMU Data
NXPMotionSense imu;
float imuVehicleSpeed = 0.0;
float yawRate = 0.0;

// Adaptive Slip Ratio
float adaptiveMaxSlipRatio = maxSlipRatio;

// === Initialization Functions ===
void initializeIMU() {
  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

// === Helper Functions ===
void readWheelSpeeds() {
  wheelSpeedFL = adc.analogRead(pinFL) * 0.01;
  wheelSpeedFR = adc.analogRead(pinFR) * 0.01;
  wheelSpeedRL = adc.analogRead(pinRL) * 0.01;
  wheelSpeedRR = adc.analogRead(pinRR) * 0.01;
}

void readIMUData() {
  float ax, ay, az, gx, gy, gz;
  if (imu.available()) {
    imu.readMotionSensor(ax, ay, az, gx, gy, gz);
    imuVehicleSpeed = sqrt(ax * ax + ay * ay); // Simplified speed calculation
    yawRate = gz;                              // Gyroscope Z-axis for yaw rate
  }
}

void sendMotorCommand(int motorID, float torquePercent) {
  CAN_message_t msg;
  msg.id = motorID;
  msg.len = 2;
  msg.buf[0] = constrain(torquePercent * 255.0, 0, 255);
  msg.buf[1] = 0;
  can1.write(msg);
}

void handleCANInput(const CAN_message_t &msg) {
  if (msg.id == vcuInputID && msg.len >= 2) {
    brakeInput = msg.buf[1] / 255.0;

    if (brakeInput > 0)
      throttleInput = 0.0;
    else
      throttleInput = msg.buf[0] / 255.0;

    launchRequested = msg.len >= 3 ? (msg.buf[2] > 127) : false;
  }
}

void updateAdaptiveSlipRatio() {
  if (yawRate > 0.5) { // Example threshold for high yaw rate
    adaptiveMaxSlipRatio = maxSlipRatio * 0.8; // Reduce slip ratio for stability
  } else {
    adaptiveMaxSlipRatio = maxSlipRatio;
  }
}

float computeSlip(float drivenSpeed, float actualSpeed) {
  if (actualSpeed == 0)
    return 0.0;
  return (drivenSpeed - actualSpeed) / actualSpeed;
}

void adjustTorque(float &target, float requested, float slip, float deltaSlip) {
  const float rampUpRate = 0.02;
  const float rampDownHard = 0.3;
  const float rampDownSoft = 0.1;

  if (slip > adaptiveMaxSlipRatio) {
    target -= (deltaSlip > 0) ? rampDownHard : rampDownSoft;
  } else if (target < requested) {
    target += rampUpRate;
  }
  target = constrain(target, 0.0, requested);
}

// === Main Loop Helper Functions ===
void handleCANEvents() {
  can1.events(); // Handle CAN input
}

void updateWheelSpeeds() {
  readWheelSpeeds();
}

void calculateSlipRatios() {
  vehicleSpeed = (wheelSpeedRL + wheelSpeedRR) / 2.0;

  slipFL = computeSlip(vehicleSpeed, wheelSpeedFL);
  slipFR = computeSlip(vehicleSpeed, wheelSpeedFR);
  slipRL = computeSlip(vehicleSpeed, wheelSpeedRL);
  slipRR = computeSlip(vehicleSpeed, wheelSpeedRR);
}

void calculateSlipDeltas(float &deltaSlipFL, float &deltaSlipFR, float &deltaSlipRL, float &deltaSlipRR) {
  deltaSlipFL = slipFL - slipFLprev;
  deltaSlipFR = slipFR - slipFRprev;
  deltaSlipRL = slipRL - slipRLprev;
  deltaSlipRR = slipRR - slipRRprev;
}

void computeTorqueRequests(float &requestedTorqueFL, float &requestedTorqueFR, float &requestedTorqueRL, float &requestedTorqueRR) {
  requestedTorqueFL = throttleInput * maxFrontTorque;
  requestedTorqueFR = throttleInput * maxFrontTorque;
  requestedTorqueRL = throttleInput * maxRearTorque;
  requestedTorqueRR = throttleInput * maxRearTorque;
}

void applyTorqueAdjustments(float requestedTorqueFL, float requestedTorqueFR, float requestedTorqueRL, float requestedTorqueRR,
                            float deltaSlipFL, float deltaSlipFR, float deltaSlipRL, float deltaSlipRR) {
  adjustTorque(targetTorqueFL, requestedTorqueFL, slipFL, deltaSlipFL);
  adjustTorque(targetTorqueFR, requestedTorqueFR, slipFR, deltaSlipFR);
  adjustTorque(targetTorqueRL, requestedTorqueRL, slipRL, deltaSlipRL);
  adjustTorque(targetTorqueRR, requestedTorqueRR, slipRR, deltaSlipRR);
}

void handleLaunchControl() {
  if (launchRequested && vehicleSpeed < launchSpeedThreshold) {
    targetTorqueRL = maxRearTorque;
    targetTorqueRR = maxRearTorque;
    targetTorqueFL = maxFrontTorque;
    targetTorqueFR = maxFrontTorque;
  }
}

void applyABSLogic() {
  if (brakeInput > 0.0) {
    if (wheelSpeedFL < vehicleSpeed * 0.8)
      targetTorqueFL = 0.0;
    if (wheelSpeedFR < vehicleSpeed * 0.8)
      targetTorqueFR = 0.0;
    if (wheelSpeedRL < vehicleSpeed * 0.8)
      targetTorqueRL = 0.0;
    if (wheelSpeedRR < vehicleSpeed * 0.8)
      targetTorqueRR = 0.0;
  }
}

void sendTorqueCommands() {
  sendMotorCommand(motorFL_ID, targetTorqueFL);
  sendMotorCommand(motorFR_ID, targetTorqueFR);
  sendMotorCommand(motorRL_ID, targetTorqueRL);
  sendMotorCommand(motorRR_ID, targetTorqueRR);
}

void savePreviousValues() {
  slipFLprev = slipFL;
  slipFRprev = slipFR;
  slipRLprev = slipRL;
  slipRRprev = slipRR;

  wheelSpeedFLprev = wheelSpeedFL;
  wheelSpeedFRprev = wheelSpeedFR;
  wheelSpeedRLprev = wheelSpeedRL;
  wheelSpeedRRprev = wheelSpeedRR;
  vehicleSpeedprev = vehicleSpeed;
}

// === Main Functions ===
void setup() {
  Serial.begin(115200);
  can1.begin();
  can1.setBaudRate(500000);
  can1.setMaxMB(16);
  can1.enableMBInterrupts();
  can1.onReceive(handleCANInput);

  adc.adc0->setAveraging(8);
  adc.adc0->setResolution(10);
  adc.adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc.adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  initializeIMU(); // Initialize the IMU
}

void loop() {
  handleCANEvents();
  updateWheelSpeeds();
  readIMUData();             // Read IMU data
  updateAdaptiveSlipRatio(); // Update adaptive slip ratio
  calculateSlipRatios();

  float deltaSlipFL, deltaSlipFR, deltaSlipRL, deltaSlipRR;
  calculateSlipDeltas(deltaSlipFL, deltaSlipFR, deltaSlipRL, deltaSlipRR);

  float requestedTorqueFL, requestedTorqueFR, requestedTorqueRL, requestedTorqueRR;
  computeTorqueRequests(requestedTorqueFL, requestedTorqueFR, requestedTorqueRL, requestedTorqueRR);

  applyTorqueAdjustments(requestedTorqueFL, requestedTorqueFR, requestedTorqueRL, requestedTorqueRR,
                         deltaSlipFL, deltaSlipFR, deltaSlipRL, deltaSlipRR);

  handleLaunchControl();
  applyABSLogic();
  sendTorqueCommands();
  savePreviousValues();

  delay(2); // 500 Hz loop
}