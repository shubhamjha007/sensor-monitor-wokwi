/*
 * Simulated Sensor Monitoring System (ESP32, Wokwi)
 *
 *  - NTC temperature sensor read on ADC pin 34
 *  - Push button on GPIO 25 handled by a GPIO interrupt (flag set in ISR,
 *    work done in loop)
 *  - Finite state machine: IDLE -> SAMPLING -> ALERT
 *  - UART command-line interface (115200 baud) for runtime control
 *  - Red LED on GPIO 26 (via resistor) signals ALERT
 */
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

// ---------- Pins ----------
constexpr uint8_t PIN_NTC    = 34;
constexpr uint8_t PIN_BUTTON = 25;
constexpr uint8_t PIN_LED    = 26;

// ---------- Configuration ----------
constexpr float    BETA             = 3950.0f;  // NTC beta coefficient
constexpr uint32_t SAMPLE_PERIOD_MS = 1000;
constexpr uint32_t DEBOUNCE_MS      = 200;
constexpr float    HYSTERESIS_C     = 2.0f;     // avoids flapping near threshold
constexpr size_t   LINE_MAX_LEN     = 48;

// ---------- State machine ----------
enum class State : uint8_t { IDLE, SAMPLING, ALERT };

static State    g_state        = State::IDLE;
static float    g_thresholdC   = 30.0f;
static float    g_lastTempC    = NAN;
static uint32_t g_lastSampleMs = 0;

// ---------- ISR shared data ----------
// Written in the ISR, read in loop(). volatile so the compiler re-reads it.
static volatile bool     g_buttonPressed = false;
static volatile uint32_t g_lastIsrMs     = 0;

// ---------- UART line buffer ----------
static char   g_line[LINE_MAX_LEN];
static size_t g_lineLen = 0;

// Keep the ISR tiny: debounce, set a flag, return.
void IRAM_ATTR onButtonPress() {
  uint32_t now = millis();
  if (now - g_lastIsrMs > DEBOUNCE_MS) {
    g_lastIsrMs = now;
    g_buttonPressed = true;
  }
}

const char* stateName(State s) {
  switch (s) {
    case State::IDLE:     return "IDLE";
    case State::SAMPLING: return "SAMPLING";
    case State::ALERT:    return "ALERT";
  }
  return "?";
}

void setState(State next) {
  if (next == g_state) return;
  Serial.printf("[%lu ms] state %s -> %s\n",
                (unsigned long)millis(), stateName(g_state), stateName(next));
  g_state = next;
  digitalWrite(PIN_LED, next == State::ALERT ? HIGH : LOW);
  if (next == State::SAMPLING) {
    g_lastSampleMs = millis() - SAMPLE_PERIOD_MS;  // sample immediately
  }
}

// Convert 12-bit ADC reading of the NTC divider to degrees Celsius.
float readTemperatureC() {
  int raw = analogRead(PIN_NTC);
  if (raw <= 0 || raw >= 4095) return NAN;  // open/short, avoid log(0)
  return 1.0f / (logf(1.0f / (4095.0f / raw - 1.0f)) / BETA + 1.0f / 298.15f) - 273.15f;
}

void logReading(float tC) {
  Serial.printf("[%lu ms] T=%.1f C  threshold=%.1f C  state=%s\n",
                (unsigned long)millis(), tC, g_thresholdC, stateName(g_state));
}

// ---------- Button handling (runs in loop, not in the ISR) ----------
void handleButton() {
  if (!g_buttonPressed) return;
  g_buttonPressed = false;
  Serial.println("button pressed");
  switch (g_state) {
    case State::IDLE:     setState(State::SAMPLING); break;
    case State::SAMPLING: setState(State::IDLE);     break;
    case State::ALERT:    setState(State::IDLE);     break;  // acknowledge
  }
}

// ---------- State machine step ----------
void runStateMachine() {
  if (g_state == State::IDLE) return;

  uint32_t now = millis();
  if (now - g_lastSampleMs < SAMPLE_PERIOD_MS) return;
  g_lastSampleMs = now;

  float t = readTemperatureC();
  if (isnan(t)) {
    Serial.println("sensor error: ADC out of range");
    return;
  }
  g_lastTempC = t;
  logReading(t);

  if (g_state == State::SAMPLING && t > g_thresholdC) {
    setState(State::ALERT);
  } else if (g_state == State::ALERT && t < g_thresholdC - HYSTERESIS_C) {
    setState(State::SAMPLING);
  }
}

// ---------- UART command-line interface ----------
void printHelp() {
  Serial.println("commands: help | status | read | set <degC> | start | stop");
}

void executeCommand(char* line) {
  char* cmd = strtok(line, " ");
  if (cmd == nullptr) return;

  if (strcmp(cmd, "help") == 0) {
    printHelp();
  } else if (strcmp(cmd, "status") == 0) {
    Serial.printf("state=%s threshold=%.1f C last=%.1f C\n",
                  stateName(g_state), g_thresholdC, g_lastTempC);
  } else if (strcmp(cmd, "read") == 0) {
    float t = readTemperatureC();
    if (isnan(t)) Serial.println("sensor error: ADC out of range");
    else          logReading(t);
  } else if (strcmp(cmd, "set") == 0) {
    char* arg = strtok(nullptr, " ");
    if (arg == nullptr) {
      Serial.println("usage: set <degC>");
      return;
    }
    char* end = nullptr;
    float v = strtof(arg, &end);
    if (end == arg || *end != '\0' || v < -40.0f || v > 125.0f) {
      Serial.println("invalid value (range -40..125)");
      return;
    }
    g_thresholdC = v;
    Serial.printf("threshold set to %.1f C\n", g_thresholdC);
  } else if (strcmp(cmd, "start") == 0) {
    setState(State::SAMPLING);
  } else if (strcmp(cmd, "stop") == 0) {
    setState(State::IDLE);
  } else {
    Serial.println("unknown command (type 'help')");
  }
}

// Non-blocking: collect characters until newline, then run the command.
void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      g_line[g_lineLen] = '\0';
      executeCommand(g_line);
      g_lineLen = 0;
    } else if (g_lineLen < LINE_MAX_LEN - 1) {
      g_line[g_lineLen++] = c;
    } else {
      g_lineLen = 0;  // line too long: discard
      Serial.println("line too long");
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  analogReadResolution(12);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), onButtonPress, FALLING);

  Serial.println("Sensor monitor ready. Press the button or type 'start'.");
  printHelp();
}

void loop() {
  handleButton();
  handleSerial();
  runStateMachine();
}
