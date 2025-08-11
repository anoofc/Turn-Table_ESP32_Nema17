#include <Arduino.h>

#define DEBUG       1     // Set to 1 to enable debug output, 0 to disable

// Pins
#define PULSE_PIN   25
#define DIR_PIN     26
// #define ENABLE_PIN 32

// Motion constants (adjust to your mechanics/driver)
#define PULSE_PER_REV   18
#define REV_PER_CM      1

// Linear motion control variables
float currentVelocity   = 0.0;              // Current velocity in steps/second
float maxVelocity       = 10000.0;          // Maximum velocity in steps/second
float accelerationRate  = 500.0;            // Acceleration in steps/second²
float decelerationRate  = 500.0;            // Deceleration in steps/second²
uint32_t velocityUpdateInterval = 1000;     // Microseconds between velocity updates
uint32_t lastVelocityUpdate = 0;
uint32_t stepInterval   = 0;                // Microseconds between step pulses
uint32_t lastStepTime   = 0;

// Motion phases
enum MotionPhase {
  PHASE_IDLE,
  PHASE_ACCELERATING,
  PHASE_CONSTANT_VELOCITY,
  PHASE_DECELERATING,
};

static MotionPhase currentPhase = PHASE_IDLE;

// Positions in "cm" units (as per REV_PER_CM)
uint32_t currentPosition = 0;
uint32_t targetPosition = 0;

// Limits and safety clamps
const uint32_t MAX_POS_CM = 100000;       // optional clamp
const float MIN_VELOCITY = 50.0;          // Minimum velocity in steps/second
const float MAX_VELOCITY_LIMIT = 5000.0;  // Maximum allowed velocity
const float MIN_ACCELERATION = 10.0;      // Minimum acceleration
const float MAX_ACCELERATION = 10000.0;   // Maximum acceleration

// Calculate motion profile phases
static void calculateMotionProfile(uint32_t totalSteps, uint32_t &accelSteps, 
                                 uint32_t &constantSteps, uint32_t &decelSteps) {
  if (totalSteps == 0) {
    accelSteps = constantSteps = decelSteps = 0;
    return;
  }
  
  // Calculate time and steps needed to reach max velocity
  float accelTime = maxVelocity / accelerationRate;
  float decelTime = maxVelocity / decelerationRate;
  
  // Calculate steps for each phase using kinematic equations
  // Distance = 0.5 * acceleration * time²
  accelSteps = (uint32_t)(0.5 * accelerationRate * accelTime * accelTime);
  decelSteps = (uint32_t)(0.5 * decelerationRate * decelTime * decelTime);
  
  // Check if we have enough distance for full acceleration/deceleration
  if (accelSteps + decelSteps > totalSteps) {
    // Triangular profile - no constant velocity phase
    // Calculate time to reach peak velocity for triangular profile
    float totalAccelDecelRate = accelerationRate + decelerationRate;
    float timeToMid = sqrt((2.0 * totalSteps) / totalAccelDecelRate);
    
    // Recalculate steps for triangular profile
    accelSteps = (uint32_t)(0.5 * accelerationRate * timeToMid * timeToMid);
    decelSteps = totalSteps - accelSteps;
    constantSteps = 0;
    
    // Adjust max velocity for this move only
    maxVelocity = accelerationRate * timeToMid;
    
    if (DEBUG) {
      Serial.print("Triangular profile: peak velocity = ");
      Serial.print(maxVelocity);
      Serial.println(" sps");
    }
  } else {
    // Trapezoidal profile - has constant velocity phase
    constantSteps = totalSteps - accelSteps - decelSteps;
  }
}

// Enhanced linear acceleration motor control
static void linearMotorControlHandler() {
  static int32_t  stepsRemaining = 0;
  static int32_t  stepsDone      = 0;
  static int8_t   direction      = 1;
  static bool     initialized    = false;
  static bool     pulseLevel     = LOW;
  static uint32_t accelSteps     = 0;
  static uint32_t constantSteps  = 0;
  static uint32_t decelSteps     = 0;
  static float    moveMaxVelocity = 0; // Store original max velocity for move
  
  // Initialize new move
  if (!initialized) {
    int32_t delta = (int32_t)targetPosition - (int32_t)currentPosition;
    if (delta == 0) return;
    
    // Set direction
    direction = (delta > 0) ? 1 : -1;
    digitalWrite(DIR_PIN, (direction > 0) ? HIGH : LOW);
    
    // Calculate total steps needed
    stepsRemaining = abs(delta) * PULSE_PER_REV * REV_PER_CM;
    stepsDone = 0;
    currentVelocity = 0.0;
    currentPhase = PHASE_ACCELERATING;
    
    // Store original max velocity and calculate motion profile
    moveMaxVelocity = maxVelocity;
    calculateMotionProfile(stepsRemaining, accelSteps, constantSteps, decelSteps);
    
    // Initialize timing
    lastVelocityUpdate = micros();
    lastStepTime = micros();
    pulseLevel = LOW;
    digitalWrite(PULSE_PIN, LOW);
    initialized = true;
    
    if (DEBUG) {
      Serial.print("Linear motion start: total="); Serial.print(stepsRemaining);
      Serial.print(", accel="); Serial.print(accelSteps);
      Serial.print(", const="); Serial.print(constantSteps);
      Serial.print(", decel="); Serial.print(decelSteps);
      Serial.print(", maxVel="); Serial.print(maxVelocity);
      Serial.println(" sps");
    }
  }
  
  if (!initialized) return;
  
  uint32_t now = micros();
  
  // Update velocity at regular intervals
  if ((now - lastVelocityUpdate) >= velocityUpdateInterval) {
    float deltaTime = (now - lastVelocityUpdate) / 1000000.0; // Convert to seconds
    
    // Determine current phase and update velocity accordingly
    if (stepsDone < accelSteps && currentPhase == PHASE_ACCELERATING) {
      // Acceleration phase: v = v0 + a*t
      currentVelocity += accelerationRate * deltaTime;
      if (currentVelocity >= maxVelocity) {
        currentVelocity = maxVelocity;
        currentPhase = PHASE_CONSTANT_VELOCITY;
        if (DEBUG) Serial.println("Phase: Constant velocity");
      }
    } 
    else if (stepsDone >= accelSteps && stepsDone < (accelSteps + constantSteps)) {
      // Constant velocity phase
      if (currentPhase != PHASE_CONSTANT_VELOCITY) {
        currentPhase = PHASE_CONSTANT_VELOCITY;
        if (DEBUG) Serial.println("Phase: Constant velocity");
      }
      currentVelocity = maxVelocity;
    }
    else if (stepsDone >= (accelSteps + constantSteps)) {
      // Deceleration phase
      if (currentPhase != PHASE_DECELERATING) {
        currentPhase = PHASE_DECELERATING;
        if (DEBUG) Serial.println("Phase: Decelerating");
      }
      currentVelocity -= decelerationRate * deltaTime;
      if (currentVelocity < MIN_VELOCITY) {
        currentVelocity = MIN_VELOCITY;
      }
    }
    
    // Calculate step interval from current velocity
    // Step interval is the time for half a pulse cycle (microseconds)
    if (currentVelocity > 0) {
      stepInterval = (uint32_t)(500000.0 / currentVelocity); // 500000 = 1000000/2
    } else {
      stepInterval = 50000; // Very slow fallback (10 Hz)
    }
    
    // Ensure minimum step interval for hardware limitations
    if (stepInterval < 50) stepInterval = 50; // 10 kHz maximum
    
    lastVelocityUpdate = now;
  }
  
  // Generate step pulses based on current velocity
  if ((now - lastStepTime) >= stepInterval && stepsRemaining > 0) {
    pulseLevel = !pulseLevel;
    digitalWrite(PULSE_PIN, pulseLevel);
    lastStepTime = now;
    
    // Count steps on falling edge (complete pulse cycle)
    if (pulseLevel == LOW) {
      stepsRemaining--;
      stepsDone++;
      
      // Debug output every 1000 steps
      if (DEBUG && (stepsDone % 1000 == 0)) {
        Serial.print("Progress: "); Serial.print(stepsDone);
        Serial.print("/"); Serial.print(stepsDone + stepsRemaining);
        Serial.print(", vel="); Serial.print(currentVelocity);
        Serial.print(" sps, phase="); Serial.println((int)currentPhase);
      }
    }
  }
  
  // Move complete
  if (stepsRemaining <= 0) {
    initialized       = false;
    currentPosition   = targetPosition;
    currentVelocity   = 0.0;
    currentPhase      = PHASE_IDLE; // Corrected assignment operator
    maxVelocity       = moveMaxVelocity; // Restore original max velocity
    digitalWrite(PULSE_PIN, LOW);
    
    if (DEBUG) {
      Serial.println("Linear motion complete.");
      Serial.print("Final position: "); Serial.print(currentPosition); Serial.println(" cm");
    }
  }
}

// Enhanced serial command parser for linear motion
static void enhancedReadSerial() {
  if (!Serial.available()) return;
  
  String incoming = Serial.readStringUntil('\n');
  incoming.trim();
  if (incoming.length() == 0) return;
  
  // Status query command
  if (incoming.equals("?")) {
    Serial.println("=== LINEAR MOTION CONTROLLER STATUS ===");
    Serial.print("Position: current="); Serial.print(currentPosition);
    Serial.print(" cm, target="); Serial.print(targetPosition); Serial.println(" cm");
    Serial.print("Velocity: current="); Serial.print(currentVelocity);
    Serial.print(" sps, max="); Serial.print(maxVelocity); Serial.println(" sps");
    Serial.print("Acceleration="); Serial.print(accelerationRate);
    Serial.print(" sps², Deceleration="); Serial.print(decelerationRate); Serial.println(" sps²");
    Serial.print("Update interval="); Serial.print(velocityUpdateInterval); Serial.println(" µs");
    Serial.print("Motion phase: ");
    switch(currentPhase) {
      case PHASE_IDLE: Serial.println("IDLE"); break;
      case PHASE_ACCELERATING: Serial.println("ACCELERATING"); break;
      case PHASE_CONSTANT_VELOCITY: Serial.println("CONSTANT VELOCITY"); break;
      case PHASE_DECELERATING: Serial.println("DECELERATING"); break;
    }
    Serial.println("Commands: P(pos), V(maxvel), A(accel), D(decel), U(update), S(stop), H(help)");
    return;
  }
  
  // Help command
  if (incoming.equals("H") || incoming.equals("h")) {
    Serial.println("=== COMMAND HELP ===");
    Serial.println("P<value> - Set target position in cm (e.g., P50)");
    Serial.println("V<value> - Set max velocity in steps/sec (e.g., V1000)");
    Serial.println("A<value> - Set acceleration in steps/sec² (e.g., A500)");
    Serial.println("D<value> - Set deceleration in steps/sec² (e.g., D500)");
    Serial.println("U<value> - Set velocity update interval in µs (e.g., U1000)");
    Serial.println("S - Emergency stop");
    Serial.println("? - Show status");
    Serial.println("H - Show this help");
    return;
  }
  
  // Emergency stop
  if (incoming.equals("S") || incoming.equals("s")) {
    targetPosition  = currentPosition; // Stop immediately
    currentVelocity = 0.0;
    currentPhase    = PHASE_IDLE;
    digitalWrite(PULSE_PIN, LOW);
    Serial.println("EMERGENCY STOP - Motion halted");
    return;
  }
  
  if (incoming.length() < 2) {
    if (DEBUG) { 
      Serial.print("Invalid command: "); 
      Serial.println(incoming); 
      Serial.println("Type 'H' for help");
    }
    return;
  }
  
  char cmd = incoming.charAt(0);
  float val = incoming.substring(1).toFloat();
  
  switch (cmd) {
    case 'P': case 'p':
      // Position command
      if (val < 0) val = 0;
      if (val > MAX_POS_CM) val = MAX_POS_CM;
      targetPosition = (uint32_t)val;
      if (DEBUG) {
        Serial.print("Target position set: "); 
        Serial.print(val); 
        Serial.print(" cm (current: ");
        Serial.print(currentPosition);
        Serial.println(" cm)");
      }
      break;
      
    case 'V': case 'v':
      // Max velocity command
      if (val >= MIN_VELOCITY && val <= MAX_VELOCITY_LIMIT) {
        maxVelocity = val;
        if (DEBUG) {
          Serial.print("Max velocity set: "); 
          Serial.print(val); 
          Serial.println(" steps/sec");
        }
      } else {
        if (DEBUG) {
          Serial.print("Velocity out of range (");
          Serial.print(MIN_VELOCITY);
          Serial.print("-");
          Serial.print(MAX_VELOCITY_LIMIT);
          Serial.println(" sps)");
        }
      }
      break;
      
    case 'A': case 'a':
      // Acceleration command
      if (val >= MIN_ACCELERATION && val <= MAX_ACCELERATION) {
        accelerationRate = val;
        if (DEBUG) {
          Serial.print("Acceleration set: "); 
          Serial.print(val); 
          Serial.println(" steps/sec²");
        }
      } else {
        if (DEBUG) {
          Serial.print("Acceleration out of range (");
          Serial.print(MIN_ACCELERATION);
          Serial.print("-");
          Serial.print(MAX_ACCELERATION);
          Serial.println(" sps²)");
        }
      }
      break;
      
    case 'D': case 'd':
      // Deceleration command
      if (val >= MIN_ACCELERATION && val <= MAX_ACCELERATION) {
        decelerationRate = val;
        if (DEBUG) {
          Serial.print("Deceleration set: "); 
          Serial.print(val); 
          Serial.println(" steps/sec²");
        }
      } else {
        if (DEBUG) {
          Serial.print("Deceleration out of range (");
          Serial.print(MIN_ACCELERATION);
          Serial.print("-");
          Serial.print(MAX_ACCELERATION);
          Serial.println(" sps²)");
        }
      }
      break;
      
    case 'U': case 'u':
      // Update interval command
      if (val >= 100 && val <= 10000) {
        velocityUpdateInterval = (uint32_t)val;
        if (DEBUG) {
          Serial.print("Velocity update interval set: "); 
          Serial.print(val); 
          Serial.println(" µs");
        }
      } else {
        if (DEBUG) {
          Serial.println("Update interval out of range (100-10000 µs)");
        }
      }
      break;
      
    default:
      if (DEBUG) {
        Serial.print("Unknown command: "); 
        Serial.println(incoming);
        Serial.println("Type 'H' for help");
      }
      break;
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  
  // Configure pins
  pinMode(PULSE_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  
  // Uncomment if using enable pin
  // pinMode(ENABLE_PIN, OUTPUT);
  // digitalWrite(ENABLE_PIN, HIGH); // Enable the motor driver
  
  // Initialize timing
  lastVelocityUpdate = micros();
  lastStepTime = micros();
  
  if (DEBUG) {
    Serial.println("=====================================");
    Serial.println("LINEAR ACCELERATION STEPPER CONTROLLER");
    Serial.println("=====================================");
    Serial.println("Ready for commands!");
    Serial.println("Type 'H' for help or '?' for status");
    Serial.print("Default settings: MaxVel=");
    Serial.print(maxVelocity);
    Serial.print("sps, Accel=");
    Serial.print(accelerationRate);
    Serial.print("sps², Decel=");
    Serial.print(decelerationRate);
    Serial.println("sps²");
  }
}

// Main loop
void loop() {
  enhancedReadSerial();
  linearMotorControlHandler();
}
