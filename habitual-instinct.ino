#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include <SimplexNoise.h>

// ---------------------------------------------------------------------------
// Habitual Instinct
// By: Jordan Shaw
// Refactored and Enhanced
// ---------------------------------------------------------------------------

// Commands
// Start: 'g' (103)
// Stop: 's' (115)
// Next: 'n' (110)
// Previous: 'p' (112)
// Configure: 'c' (99)

// Constants and Definitions
#define DEBUG true
#define MAX_DISTANCE 400       // Maximum distance (in cm) to ping.
#define PING_INTERVAL 33       // Milliseconds between sensor pings.
#define UPDATE_INTERVAL 40     // Update interval in ms
#define CONTROL_INCREMENT 10
#define NUM_SENSORS 20
#define NUM_READINGS 5         // For smoothing sensor data
#define DATA_BUFFER_SIZE 256

// Enumerations for modes
enum Mode {
  MODE_STOP,
  MODE_SWEEP,
  MODE_SWEEP_REACT,
  MODE_SWEEP_REACT_PAUSE,
  MODE_NOISE,
  MODE_NOISE_REACT,
  MODE_PATTERN_WAVE_SMALL_V2,
  MODE_MEASURE,
  MODE_MEASURE_REACT
};

// Struct for configuration
struct Config {
  uint8_t servoPins[NUM_SENSORS];
  uint8_t triggerPins[NUM_SENSORS];
  uint8_t echoPins[NUM_SENSORS];
  uint8_t panel;
  Mode currentMode;
  unsigned long pingIntervals[NUM_SENSORS];
};

Config config = {
  // Servo pins
  {
    A0, A1, A2, A3, A4, A5, A6, A7, A8, A9,
    40, 38, 36, 34, 32, 30, 28, 26, 24, 22
  },
  // Trigger pins
  {
    A11, A12, A13, A14, A15, 52, 50, 48, 46, 44,
    31, 29, 27, 25, 23, 9, 10, 11, 12, 13
  },
  // Echo pins
  {
    45, 47, 49, 51, 53, 33, 35, 37, 39, 41,
    7, 6, 5, 4, 3, 2, 18, 19, 20, 21
  },
  // Panel number
  1,
  // Current mode
  MODE_STOP,
  // Ping intervals (initialized later)
  {0}
};

// Global Variables
unsigned long pingTimers[NUM_SENSORS];
unsigned int distances[NUM_SENSORS];
uint8_t currentSensor = 0;
int incomingByte = -1;

// Function Prototypes
void setupSweepers();
void handleSerialInput();
void attachAllServos();
void detachAllServos();
void setPatternWavePosition();
void echoCheck();
void initializeSensors();
void logDebug(const char* message);
void logDebug(const String& message);

// Class Definitions
class Sweeper {
  public:
    Sweeper(uint8_t id, uint8_t servoPin, uint8_t triggerPin, uint8_t echoPin);
    void attachServo();
    void detachServo();
    void update();
    void setMode(Mode mode);
    void reset();
    void setPosition(int position);
    void switchIncrementDirection();
    void setPatternPosition(int position);
    void handleDistanceMeasurement(unsigned int distance);

    NewPing sonar; // Made public for access in echoCheck

  private:
    void updateSweep();
    void updateSweepReact();
    void updateSweepReactPause();
    void updateNoise();
    void updateNoiseReact();
    void updateMeasure();
    void updateMeasureReact();
    void sendData();
    void storeData(unsigned int distance);
    void resetSmoothing();

    uint8_t id;
    Servo servo;
    SimplexNoise noiseGenerator;
    Mode mode;
    uint8_t servoPin;
    uint8_t triggerPin;
    uint8_t echoPin;
    int position;
    int increment;
    unsigned long lastUpdate;
    unsigned long pausedMillis;
    unsigned long pauseInterval;
    bool paused;
    bool publishData;
    char dataBuffer[DATA_BUFFER_SIZE];
    size_t dataBufferIndex;
    int readings[NUM_READINGS];
    int readIndex;
    int total;
    int average;
    float noiseX;
    float noiseIncrement;
    // For reaction modes
    bool reactionTriggered;
    unsigned long reactionStartTime;
    unsigned long reactionDuration;
};

Sweeper* sweepers[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect.
  }
  setupSweepers();
  initializeSensors();
  attachAllServos();
  logDebug("Setup complete.");
}

void loop() {
  handleSerialInput();

  // Update all sweepers
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sweepers[i]->update();
  }

  // Handle sensor pings
  unsigned long currentMillis = millis();
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    if (currentMillis >= pingTimers[i]) {
      pingTimers[i] += PING_INTERVAL * NUM_SENSORS;
      currentSensor = i;
      distances[currentSensor] = 0;
      sweepers[currentSensor]->sonar.ping_timer(echoCheck);
    }
  }
}

// Implementation of Sweeper class methods
Sweeper::Sweeper(uint8_t id, uint8_t servoPin, uint8_t triggerPin, uint8_t echoPin)
  : id(id), servoPin(servoPin), triggerPin(triggerPin), echoPin(echoPin),
    sonar(triggerPin, echoPin, MAX_DISTANCE), mode(MODE_STOP), position(90),
    increment(2), lastUpdate(0), pausedMillis(0), pauseInterval(0), paused(false),
    publishData(false), dataBufferIndex(0), readIndex(0), total(0), average(0),
    noiseX(random(0, 1000)/100.0), noiseIncrement(0.01), reactionTriggered(false),
    reactionStartTime(0), reactionDuration(0) {

  resetSmoothing();
  memset(dataBuffer, 0, DATA_BUFFER_SIZE);
}

void Sweeper::attachServo() {
  if (!servo.attached()) {
    servo.attach(servoPin);
    logDebug("Servo attached on pin " + String(servoPin));
  }
}

void Sweeper::detachServo() {
  if (servo.attached()) {
    servo.detach();
    logDebug("Servo detached from pin " + String(servoPin));
  }
}

void Sweeper::update() {
  switch (mode) {
    case MODE_SWEEP:
      updateSweep();
      break;
    case MODE_SWEEP_REACT:
      updateSweepReact();
      break;
    case MODE_SWEEP_REACT_PAUSE:
      updateSweepReactPause();
      break;
    case MODE_NOISE:
      updateNoise();
      break;
    case MODE_NOISE_REACT:
      updateNoiseReact();
      break;
    case MODE_MEASURE:
      updateMeasure();
      break;
    case MODE_MEASURE_REACT:
      updateMeasureReact();
      break;
    case MODE_PATTERN_WAVE_SMALL_V2:
      updateSweep(); // Use sweep behavior for pattern wave
      break;
    case MODE_STOP:
    default:
      // Do nothing
      break;
  }
}

void Sweeper::setMode(Mode newMode) {
  mode = newMode;
  reset();
}

void Sweeper::reset() {
  position = 90;
  increment = 2;
  lastUpdate = millis();
  paused = false;
  pausedMillis = millis();
  publishData = false;
  dataBufferIndex = 0;
  memset(dataBuffer, 0, DATA_BUFFER_SIZE);
  resetSmoothing();
}

void Sweeper::setPosition(int pos) {
  position = constrain(pos, 0, 180);
  servo.write(position);
}

void Sweeper::switchIncrementDirection() {
  increment = -increment;
}

void Sweeper::setPatternPosition(int pos) {
  setPosition(pos);
}

void Sweeper::handleDistanceMeasurement(unsigned int distance) {
  // Smooth sensor readings
  total -= readings[readIndex];
  readings[readIndex] = distance;
  total += readings[readIndex];
  readIndex = (readIndex + 1) % NUM_READINGS;
  average = total / NUM_READINGS;

  storeData(distance);
}

void Sweeper::storeData(unsigned int distance) {
  // Build data string
  int n = snprintf(dataBuffer + dataBufferIndex, DATA_BUFFER_SIZE - dataBufferIndex,
                   "%d:%d:%d/", id, position, distance);
  if (n > 0) {
    dataBufferIndex += n;
    if (dataBufferIndex >= DATA_BUFFER_SIZE - 20) {
      sendData();
    }
  } else {
    logDebug("Error in snprintf");
  }
}

void Sweeper::sendData() {
  if (dataBufferIndex > 0) {
    // Remove trailing '/'
    if (dataBuffer[dataBufferIndex - 1] == '/') {
      dataBuffer[dataBufferIndex - 1] = '\0';
    }
    Serial.print(config.panel);
    Serial.print("_");
    Serial.println(dataBuffer);
    dataBufferIndex = 0;
    memset(dataBuffer, 0, DATA_BUFFER_SIZE);
  }
}

void Sweeper::resetSmoothing() {
  total = 0;
  for (int i = 0; i < NUM_READINGS; i++) {
    readings[i] = 0;
  }
}

void Sweeper::updateSweep() {
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    if (!paused) {
      position += increment;
      if (position <= 0 || position >= 180) {
        increment = -increment;
      }
      attachServo();
      servo.write(position);
    }
  }
}

void Sweeper::updateSweepReact() {
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    if (!paused) {
      if (position > 70 && position < 110) {
        if (average > 10 && average < 120) {
          // Object detected, react
          if (position > 90) {
            position = 170;
          } else {
            position = 10;
          }
          paused = true;
          pauseInterval = 50;
          pausedMillis = currentMillis;
        } else {
          position += increment;
        }
      } else {
        position += increment;
      }

      if (position <= 0 || position >= 180) {
        increment = -increment;
      }
      attachServo();
      servo.write(position);
    } else {
      if (currentMillis - pausedMillis >= pauseInterval) {
        paused = false;
      }
    }
  }
}

void Sweeper::updateSweepReactPause() {
  // Implement similar to updateSweepReact with additional pauses
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    if (!paused) {
      if (!reactionTriggered && position > 40 && position < 150 && average > 10 && average < 120) {
        // Trigger reaction
        reactionTriggered = true;
        reactionStartTime = currentMillis;
        reactionDuration = 5000; // 5 seconds
        if (position > 90) {
          position = 170;
        } else {
          position = 10;
        }
        attachServo();
        servo.write(position);
        paused = true;
        pauseInterval = 50;
        pausedMillis = currentMillis;
      } else if (reactionTriggered) {
        if (currentMillis - reactionStartTime >= reactionDuration) {
          // Reaction over
          reactionTriggered = false;
        }
        // Keep paused during reaction
      } else {
        position += increment;
        if (position <= 0 || position >= 180) {
          increment = -increment;
        }
        attachServo();
        servo.write(position);
      }
    } else {
      if (currentMillis - pausedMillis >= pauseInterval) {
        paused = false;
      }
    }
  }
}

void Sweeper::updateNoise() {
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    float noiseValue = noiseGenerator.noise(noiseX, 0.0);
    noiseX += noiseIncrement;
    position = map(noiseValue * 100, -100, 100, 0, 180);
    attachServo();
    servo.write(position);
  }
}

void Sweeper::updateNoiseReact() {
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    if (!reactionTriggered && position > 50 && position < 130 && average > 10 && average < 120) {
      // Trigger reaction
      reactionTriggered = true;
      reactionStartTime = currentMillis;
      reactionDuration = 4000; // 4 seconds
      attachServo();
      servo.write(position + 20);
      paused = true;
      pauseInterval = 150;
      pausedMillis = currentMillis;
    } else if (reactionTriggered) {
      if (currentMillis - reactionStartTime < reactionDuration) {
        // Continue reaction sequence
        if (currentMillis - pausedMillis >= pauseInterval) {
          pausedMillis = currentMillis;
          if (position >= 160) {
            position = 90;
          } else {
            position = 160;
          }
          attachServo();
          servo.write(position);
        }
      } else {
        // Reaction over
        reactionTriggered = false;
        paused = false;
      }
    } else {
      float noiseValue = noiseGenerator.noise(noiseX, 0.0);
      noiseX += noiseIncrement;
      position = map(noiseValue * 100, -100, 100, 0, 180);
      attachServo();
      servo.write(position);
    }
  }
}

void Sweeper::updateMeasure() {
  // Keep servo at 90 degrees and send distance data
  attachServo();
  servo.write(90);
}

void Sweeper::updateMeasureReact() {
  unsigned long currentMillis = millis();
  if ((currentMillis - lastUpdate) > UPDATE_INTERVAL) {
    lastUpdate = currentMillis;

    if (average > 10 && average < 120 && !reactionTriggered) {
      // Object detected, start reaction
      reactionTriggered = true;
      reactionStartTime = currentMillis;
      reactionDuration = 500; // 0.5 seconds
      attachServo();
      servo.write(10);
      paused = true;
      pauseInterval = 500;
      pausedMillis = currentMillis;
    } else if (reactionTriggered) {
      if (currentMillis - reactionStartTime >= reactionDuration) {
        // Reaction over
        reactionTriggered = false;
        paused = false;
        detachServo();
      }
    }
  }
}

// Setup functions
void setupSweepers() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sweepers[i] = new Sweeper(i, config.servoPins[i], config.triggerPins[i], config.echoPins[i]);
  }
}

void initializeSensors() {
  unsigned long startTime = millis() + 75;
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pingTimers[i] = startTime + i * PING_INTERVAL;
  }
}

void attachAllServos() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sweepers[i]->attachServo();
  }
}

void detachAllServos() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    sweepers[i]->detachServo();
  }
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    switch (incomingByte) {
      case 'g': // Start
        config.currentMode = MODE_SWEEP;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case 's': // Stop
        config.currentMode = MODE_STOP;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
          sweepers[i]->detachServo();
        }
        break;
      case '1':
        config.currentMode = MODE_SWEEP;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '2':
        config.currentMode = MODE_SWEEP_REACT;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '3':
        config.currentMode = MODE_SWEEP_REACT_PAUSE;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '4':
        config.currentMode = MODE_NOISE;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '5':
        config.currentMode = MODE_NOISE_REACT;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '6':
        config.currentMode = MODE_PATTERN_WAVE_SMALL_V2;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        setPatternWavePosition();
        break;
      case '7':
        config.currentMode = MODE_MEASURE;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case '8':
        config.currentMode = MODE_MEASURE_REACT;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setMode(config.currentMode);
        }
        break;
      case 'c':
        // Configure: Reset positions to 90
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
          sweepers[i]->setPosition(90);
          sweepers[i]->reset();
        }
        break;
      default:
        break;
    }
  }
}

void setPatternWavePosition() {
  // Implement pattern wave positions based on panel number
  if (config.panel == 1) {
    // Example pattern
    sweepers[0]->setPatternPosition(10);
    sweepers[1]->setPatternPosition(30);
    sweepers[2]->setPatternPosition(50);
    sweepers[3]->setPatternPosition(70);
    // Continue setting positions for other sweepers
  }
  // Implement for other panels as needed
}

void echoCheck() {
  if (sweepers[currentSensor]->sonar.check_timer()) {
    unsigned int distance = sweepers[currentSensor]->sonar.ping_result / US_ROUNDTRIP_CM;
    sweepers[currentSensor]->handleDistanceMeasurement(distance);
  }
}

void logDebug(const char* message) {
  if (DEBUG) {
    Serial.println(message);
  }
}

void logDebug(const String& message) {
  if (DEBUG) {
    Serial.println(message);
  }
}
