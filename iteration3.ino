#include <Stepper.h>

// Constants
const int voltagePin = A0;          // Voltage feedback
const int currentPin = A1;          // Current feedback
const int feedRatePin = A2;         // Target wire feed rate
const int burnFudgePin = A3;        // Wire burn pulse fudge factor
const int pwmPin = 9;               // PWM output for buck converter
const int stepPin = 2;              // Stepper motor step pin
const int dirPin = 3;               // Stepper motor direction pin
const int buttonPin = 7;            // Button for initiating weld
const int photoSensorPin = A4;      // Photoelectric sensor feedback
const int fineTunePin = A5;         // Fine-tuning knob
const int oscFreqKnobPin = A6;      // Oscillation frequency knob

// Parameters
int targetFeedRate = 0;
int burnFudgeFactor = 0;
int pwmDutyCycle = 0;
int stepperPosition = 0;
int stepperMaxPosition = 200;       // Adjust as per actual requirements

// Short circuit thresholds
const int shortCircuitVoltageThreshold = 10;   // Voltage below this indicates a possible short circuit
const int shortCircuitCurrentThreshold = 800;  // Current above this indicates a possible short circuit

// Frequency search parameters
float minFrequency = 500.0;         // Minimum PWM frequency in Hz
float maxFrequency = 2000.0;        // Maximum PWM frequency in Hz
float tolerance = 0.1;              // Tolerance for frequency search

// Moving average filter parameters
const int numSamples = 10;          // Number of samples for moving average
float voltageSamples[numSamples];   // Voltage samples for moving average
float currentSamples[numSamples];   // Current samples for moving average
float photoSensorSamples[numSamples]; // Photoelectric sensor samples
int sampleIndex = 0;

// Stepper motor configuration
Stepper stepper(200, stepPin, dirPin);  // 200 steps per revolution

// Oscillation frequency control
float baseOscillationFreq = 2.0;    // Base oscillation frequency in Hz
float maxOscillationFreq = 10.0;    // Max oscillation frequency in Hz
float oscFreqFactor = 1.0;          // Oscillation frequency adjustment factor

void setup() {
  pinMode(voltagePin, INPUT);
  pinMode(currentPin, INPUT);
  pinMode(feedRatePin, INPUT);
  pinMode(burnFudgePin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(photoSensorPin, INPUT);
  pinMode(fineTunePin, INPUT);
  pinMode(oscFreqKnobPin, INPUT);

  stepper.setSpeed(60);  // Initial speed setting
  initializeSamples();
}

void loop() {
  // Read inputs
  targetFeedRate = analogRead(feedRatePin);
  burnFudgeFactor = analogRead(burnFudgePin);
  oscFreqFactor = analogRead(oscFreqKnobPin) / 1023.0;  // Normalize to 0-1

  // Wait for button press to start the process
  if (digitalRead(buttonPin) == LOW) {
    // Pre-contact feed
    feedWireForward();

    // Oscillate to detect contact
    while (!detectContact()) {
      oscillateWireFeed();
    }

    // Arc initiation with binary search for resonant frequency
    initiateArc();

    // Enter welding loop
    while (weldingInProgress()) {
      burnWirePulse();
      adjustWireFeed();

      // Check for short circuit condition
      if (checkForShortCircuit()) {
        stopWelding();  // Stop the welding process immediately
        break;          // Exit the welding loop
      }
    }
  }
}

// Functions for various stages of the welding process
void feedWireForward() {
  // Feed forward a few mm
  moveStepper(stepperMaxPosition / 10, true); // Example: feed forward 10% of max range
}

bool detectContact() {
  int voltage = analogRead(voltagePin);
  return (voltage < 10);  // Example threshold for contact detection
}

void oscillateWireFeed() {
  // Calculate adjusted oscillation frequency based on arc quality
  float arcQualityScore = evaluateArcQuality();
  float adjustedOscillationFreq = baseOscillationFreq + (maxOscillationFreq - baseOscillationFreq) * arcQualityScore * oscFreqFactor;

  // Calculate delay time between oscillations (in milliseconds)
  int oscillationDelay = 1000 / adjustedOscillationFreq;

  // Oscillate the stepper back and forth
  moveStepper(-20, false);
  delay(oscillationDelay);
  moveStepper(20, true);
  delay(oscillationDelay);
}

void initiateArc() {
  // Use binary search to find the resonant frequency
  float resonantFrequency = findResonantFrequency(minFrequency, maxFrequency, tolerance);

  // Set PWM to the resonant frequency
  adjustPwmFrequency(resonantFrequency);

  // Start arc with the resonant frequency
  pwmDutyCycle = 50;  // Start with a low duty cycle
  analogWrite(pwmPin, pwmDutyCycle);

  while (!arcStable()) {
    // Adjust PWM duty cycle if necessary during arc initiation
    pwmDutyCycle++;
    analogWrite(pwmPin, pwmDutyCycle);
  }
}

bool arcStable() {
  // Update the moving average samples
  updateSamples();

  // Calculate the moving averages
  float avgVoltage = calculateMovingAverage(voltageSamples);
  float avgCurrent = calculateMovingAverage(currentSamples);
  float avgPhotoSensor = calculateMovingAverage(photoSensorSamples);

  // Fine-tuning knob input
  float fineTune = analogRead(fineTunePin) / 1023.0; // Normalize to 0-1

  // Scoring system for arc quality (weights can be fine-tuned)
  float voltageWeight = 0.4 + 0.3 * fineTune;  // Adjust weight based on fine-tuning knob
  float currentWeight = 0.4 - 0.3 * fineTune;
  float photoSensorWeight = 0.2;

  // Calculate the weighted score
  float score = voltageWeight * avgVoltage + currentWeight * (100 - avgCurrent) + photoSensorWeight * avgPhotoSensor;

  // Threshold for determining arc stability
  return (score > 50);  // Example threshold (adjust as needed)
}

float evaluateArcQuality() {
  // Calculate arc quality score (same as arcStable but returns a normalized score)
  updateSamples();
  float avgVoltage = calculateMovingAverage(voltageSamples);
  float avgCurrent = calculateMovingAverage(currentSamples);
  float avgPhotoSensor = calculateMovingAverage(photoSensorSamples);

  float fineTune = analogRead(fineTunePin) / 1023.0;

  float voltageWeight = 0.4 + 0.3 * fineTune;
  float currentWeight = 0.4 - 0.3 * fineTune;
  float photoSensorWeight = 0.2;

  float score = voltageWeight * avgVoltage + currentWeight * (100 - avgCurrent) + photoSensorWeight * avgPhotoSensor;

  return constrain(score / 100.0, 0.0, 1.0);  // Normalize score to 0-1 range
}

void burnWirePulse() {
  // Short burst of high current to burn wire
  pwmDutyCycle += burnFudgeFactor;
  analogWrite(pwmPin, pwmDutyCycle);
  delay(100);  // Pulse duration
  pwmDutyCycle -= burnFudgeFactor;
  analogWrite(pwmPin, pwmDutyCycle);
}

void adjustWireFeed() {
  // Adjust the wire feed based on the length of the wire burnt
  int burnCorrection = burnFudgeFactor / 2;  // Example correction factor
  moveStepper(burnCorrection, true);
}

void adjustPwmFrequency(float frequency) {
  // Adjust PWM frequency by modifying timer settings
  // Implementation will depend on the specific microcontroller and PWM configuration
}

bool weldingInProgress() {
  // Condition to check if welding is still in progress
  return digitalRead(buttonPin) == LOW;
}

// Function to find the resonant frequency using binary search
float findResonantFrequency(float minFreq, float maxFreq, float tol) {
  float midFreq;
  while ((maxFreq - minFreq) > tol) {
    midFreq = (minFreq + maxFreq) / 2.0;
    adjustPwmFrequency(midFreq);

    if (arcQuality(midFreq)) {
      maxFreq = midFreq;  // Narrow down to the lower half
    } else {
      minFreq = midFreq;  // Narrow down to the upper half
    }
  }
  return midFreq;
}

// Function to assess arc quality at a given frequency (improved with advanced evaluation)
bool arcQuality(float frequency) {
  return arcStable();
}

// Initialize the sample arrays for moving average
void initializeSamples() {
  for (int i = 0; i < numSamples; i++) {
    voltageSamples[i] = analogRead(voltagePin);
    currentSamples[i] = analogRead(currentPin);
    photoSensorSamples[i] = analogRead(photoSensorPin);
  }
  sampleIndex = 0;
}

// Update the samples with new readings for moving average
void updateSamples() {
  voltageSamples[sampleIndex] = analogRead(voltagePin);
  currentSamples[sampleIndex] = analogRead(currentPin);
  photoSensorSamples[sampleIndex] = analogRead(photoSensorPin);

  sampleIndex = (sampleIndex + 1) % numSamples;
}

// Calculate the moving average of an array of samples
float calculateMovingAverage(float samples[]) {
  float sum = 0.0;
  for (int i = 0; i < numSamples; i++) {
    sum += samples[i];
  }
  return sum / numSamples;
}

// Function to move the stepper motor with acceleration and jerk control
void moveStepper(int steps, bool accelerate) {
  int direction = (steps > 0) ? 1 : -1;
  steps = abs(steps);

  static int currentStepperSpeed = 0;
  static long lastStepTime = 0;
  static int currentAccel = 0;
  static int currentJerk = 0;

  for (int i = 0; i < steps; i++) {
    long currentTime = micros();
    long timeDiff = currentTime - lastStepTime;

    // Calculate speed based on acceleration and jerk
    if (accelerate) {
      currentAccel += maxJerk * timeDiff / 1000000;  // Change acceleration with jerk
      currentAccel = constrain(currentAccel, -maxAccel, maxAccel);
      currentStepperSpeed += currentAccel * timeDiff / 1000000;  // Change speed with acceleration
      currentStepperSpeed = constrain(currentStepperSpeed, minStepperSpeed, maxStepperSpeed);
    } else {
      // Decelerate smoothly
      currentStepperSpeed -= maxJerk * timeDiff / 1000000;
      currentStepperSpeed = max(currentStepperSpeed, minStepperSpeed);
    }

    // Move the stepper by one step
    digitalWrite(dirPin, direction > 0 ? HIGH : LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000000 / currentStepperSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000000 / currentStepperSpeed);

    lastStepTime = currentTime;
  }
}

// Check for a short circuit condition based on voltage and current feedback
bool checkForShortCircuit() {
  int voltage = analogRead(voltagePin);
  int current = analogRead(currentPin);
  return (voltage < shortCircuitVoltageThreshold && current > shortCircuitCurrentThreshold);
}

// Stop the welding process immediately in case of a short circuit
void stopWelding() {
  analogWrite(pwmPin, 0);  // Stop PWM
  moveStepper(-stepperPosition, true);  // Retract wire
  stepperPosition = 0;  // Reset stepper position
}
