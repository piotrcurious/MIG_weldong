void initiateArc() {
    // Use binary search to find the resonant frequency
    float resonantFrequency = findResonantFrequency(minFrequency, maxFrequency, tolerance);

    // Set PWM to the resonant frequency
    adjustPwmFrequency(resonantFrequency);

    // Start arc with the resonant frequency
    pwmDutyCycle = 5;  // Start with a very low duty cycle
    analogWrite(pwmPin, pwmDutyCycle);

    // PID control parameters
    float Kp = 1.5;  // Proportional gain
    float Ki = 0.1;  // Integral gain
    float Kd = 0.01; // Derivative gain

    float previousError = 0;
    float integral = 0;

    unsigned long lastTime = millis();
    unsigned long currentTime;
    float deltaTime;

    // Stage 1: Pre-Ignition
    while (!detectContact()) {
        oscillateWireFeed(); // Oscillate wire to detect contact
    }

    // Stage 2: Soft Start
    while (!arcStable()) {
        currentTime = millis();
        deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

        // Read feedback from voltage and current
        int voltage = analogRead(voltagePin);
        int current = analogRead(currentPin);

        // Calculate the error
        float setPoint = 10; // Target initial arc voltage (example value)
        float error = setPoint - voltage;

        // Calculate PID terms
        integral += error * deltaTime;
        float derivative = (error - previousError) / deltaTime;

        // Calculate the PID output
        float output = Kp * error + Ki * integral + Kd * derivative;
        pwmDutyCycle += output;

        // Constrain the PWM duty cycle within limits
        pwmDutyCycle = constrain(pwmDutyCycle, 0, 255);

        // Apply the new PWM duty cycle
        analogWrite(pwmPin, pwmDutyCycle);

        // Update previous error and time
        previousError = error;
        lastTime = currentTime;

        // Additional safety check for short circuit
        if (checkForShortCircuit()) {
            stopWelding();
            return;
        }
    }

    // Stage 3: Full Ignition
    while (!arcFullyStable()) {
        currentTime = millis();
        deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds

        // Fine-tune PID parameters for full ignition
        float fineTuneKp = 2.0;
        float fineTuneKi = 0.2;
        float fineTuneKd = 0.02;

        // Read feedback from voltage and current
        int voltage = analogRead(voltagePin);
        int current = analogRead(currentPin);

        // Calculate the error
        float fullIgnitionSetPoint = 20; // Target full ignition voltage (example value)
        float error = fullIgnitionSetPoint - voltage;

        // Calculate PID terms
        integral += error * deltaTime;
        float derivative = (error - previousError) / deltaTime;

        // Calculate the PID output
        float output = fineTuneKp * error + fineTuneKi * integral + fineTuneKd * derivative;
        pwmDutyCycle += output;

        // Constrain the PWM duty cycle within limits
        pwmDutyCycle = constrain(pwmDutyCycle, 0, 255);

        // Apply the new PWM duty cycle
        analogWrite(pwmPin, pwmDutyCycle);

        // Update previous error and time
        previousError = error;
        lastTime = currentTime;

        // Additional safety check for short circuit
        if (checkForShortCircuit()) {
            stopWelding();
            return;
        }
    }

    // Welding process continues...
}

// Function to assess if the arc is fully stable during full ignition
bool arcFullyStable() {
    // Similar to arcStable(), but with stricter criteria for stability
    updateSamples();
    float avgVoltage = calculateMovingAverage(voltageSamples);
    float avgCurrent = calculateMovingAverage(currentSamples);
    float avgPhotoSensor = calculateMovingAverage(photoSensorSamples);

    float fineTune = analogRead(fineTunePin) / 1023.0;

    float voltageWeight = 0.5 + 0.3 * fineTune;  // Higher emphasis on voltage stability
    float currentWeight = 0.3 - 0.2 * fineTune;
    float photoSensorWeight = 0.2;

    float score = voltageWeight * avgVoltage + currentWeight * (100 - avgCurrent) + photoSensorWeight * avgPhotoSensor;

    return (score > 70);  // Stricter threshold for full arc stability
}
