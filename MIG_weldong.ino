
// Define the pins for the PWM output, the step and dir signals for the stepper motor, and the analog inputs for the controls and feedback
#define PWM_PIN 3
#define STEP_PIN 4
#define DIR_PIN 5
#define WIRE_FEED_RATE_PIN A0
#define WIRE_BURN_PULSE_PIN A1
#define VOLTAGE_PIN A2
#define CURRENT_PIN A3

// Define some constants for the PWM frequency, the stepper motor steps per revolution, and the wire diameter
#define PWM_FREQ 1000 // Hz
#define STEPS_PER_REV 200
#define WIRE_DIA 0.8 // mm

// Define some variables for the wire feed rate, the wire burn pulse factor, the PWM duty cycle, and the stepper motor direction and speed
float wire_feed_rate; // mm/s
float wire_burn_pulse; // fraction of wire feed rate
int pwm_duty; // 0-255
bool dir; // true for forward, false for backward
int step_delay; // microseconds

// Define some variables for the voltage and current feedback, the contact detection, and the arc initiation and stabilization
float voltage; // V
float current; // A
bool contact; // true if contact detected, false otherwise
bool arc; // true if arc initiated and stable, false otherwise
int arc_freq; // Hz

// Define some variables for the wire burn pulse cycle, the wire feed correction, and the error margin
bool pulse; // true if in pulse mode, false otherwise
float wire_burnt; // mm
float wire_feed_error; // mm
float margin; // mm

void setup() {
  // Set the PWM pin as output and set the PWM frequency
  pinMode(PWM_PIN, OUTPUT);
  analogWriteFrequency(PWM_PIN, PWM_FREQ);

  // Set the step and dir pins as output and initialize them to low
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // Set the analog input pins as input and enable pullup resistors
  pinMode(WIRE_FEED_RATE_PIN, INPUT_PULLUP);
  pinMode(WIRE_BURN_PULSE_PIN, INPUT_PULLUP);
  pinMode(VOLTAGE_PIN, INPUT_PULLUP);
  pinMode(CURRENT_PIN, INPUT_PULLUP);

  // Initialize the variables to default values
  wire_feed_rate = 10.0; // mm/s
  wire_burn_pulse = 0.5; // fraction of wire feed rate
  pwm_duty = 0; // 0-255
  dir = true; // true for forward, false for backward
  step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); // microseconds
  voltage = 0.0; // V
  current = 0.0; // A
  contact = false; // true if contact detected, false otherwise
  arc = false; // true if arc initiated and stable, false otherwise
  arc_freq = PWM_FREQ / 2; // Hz
  pulse = false; // true if in pulse mode, false otherwise
  wire_burnt = 0.0; // mm
  wire_feed_error = 0.0; // mm
  margin = WIRE_DIA / 10.0; // mm

}

void loop() {
  
   if (digitalRead(2) == HIGH) { 
    startWelding(); 
   } else { 
    stopWelding(); 
   }
}

void startWelding() {
  
   readControls(); 
   readFeedback(); 
   detectContact(); 
   initiateArc(); 
   stabilizeArc(); 
   performPulse(); 
   correctFeed(); 
   outputPWM(); 
   outputStep();
}

void stopWelding() {
  
   pwm_duty = 0;
   analogWrite(PWM_PIN,pwm_duty); 
   digitalWrite(STEP_PIN,LOW); 
}

void readControls() {
  
   wire_feed_rate = map(analogRead(WIRE_FEED_RATE_PIN),0,1023,5.0,15.0); 
   wire_burn_pulse = map(analogRead(WIRE_BURN_PULSE_PIN),0,1023,0.1,0.9); 
   step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); 
}

void readFeedback() {
  
   voltage = map(analogRead(VOLTAGE_PIN),0,1023,0.0,50.0); 
   current = map(analogRead(CURRENT_PIN),0,1023,0.0,200.0); 
}

void detectContact() {
  
   if (voltage < 1.0) { 
    contact = true; 
   } else { 
    contact = false; 
   }
}

void initiateArc() {
  
   if (contact && !arc) { 
    pwm_duty = 1; 
    arc_freq = PWM_FREQ / 2; 
    while (!arc) { 
      arc_freq++; 
      analogWriteFrequency(PWM_PIN,arc_freq); 
      readFeedback(); 
      if (current > 10.0) { 
        arc = true; 
      }
    }
   }
}

void stabilizeArc() {
  
   if (arc) { 
    pwm_duty = map(current,10.0,200.0,1,255); 
    analogWrite(PWM_PIN,pwm_duty); 
   }
}

void performPulse() {
  
   if (arc) { 
    pulse = !pulse; 
    if (pulse) { 
      wire_feed_rate *= wire_burn_pulse; 
      step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); 
      wire_burnt = 0.0; 
    } else { 
      wire_feed_rate /= wire_burn_pulse; 
      step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); 
      wire_burnt = WIRE_DIA * pwm_duty / 255.0; // estimate of wire burnt in one pulse
    }
   }
}

void correctFeed() {
  
   if (arc) { 
    wire_feed_error = wire_burnt - wire_feed_rate / arc_freq; // difference between wire burnt and wire fed in one cycle
    if (wire_feed_error > margin) { // too much wire burnt
      dir = false; // reverse direction
      wire_feed_rate += wire_feed_error; // increase feed rate
      step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); // adjust step delay
    } else if (wire_feed_error < -margin) { // too little wire burnt
      dir = true; // forward direction
      wire_feed_rate -= wire_feed_error; // decrease feed rate
      step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); // adjust step delay
    } else { // within margin
      dir = true; // forward direction
      wire_feed_rate = map(analogRead(WIRE_FEED_RATE_PIN),0,1023,5.0,15.0); // reset feed rate to control input
      step_delay = (60 * 1000000) / (wire_feed_rate * STEPS_PER_REV); // reset step delay
    }
   }
}

void outputPWM() {
  
   analogWrite(PWM_PIN,pwm_duty); 
}

void outputStep() {
  
   digitalWrite(DIR_PIN,dir); 
   digitalWrite(STEP_PIN,HIGH); 
   delayMicroseconds(step_delay / 2); 
   digitalWrite(STEP_PIN,LOW); 
   delayMicroseconds(step_delay / 2); 
}
