#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <esp_timer.h>
#include "Supervisory.h"

constexpr uint8_t SERVO = 22, TRIG = 18, ECHO = 19;
constexpr uint8_t LEFT_ENABLE = 13, LEFT_FWD = 26, LEFT_REV = 25;
constexpr uint8_t RIGHT_ENABLE = 4, RIGHT_FWD = 33, RIGHT_REV = 32;
constexpr uint8_t PWM_LEFT = 175, PWM_RIGHT = 180;
constexpr uint32_t MOTOR_TEST_MS = 1000, MOTOR_SETTLE_MS = 750;
constexpr uint32_t ADVANCE_LIMIT_MS = 5000, ADVANCE_WATCHDOG_MS = 450;
constexpr float OBSTACLE_CM = 45.0f;

// Calibre os extremos para o seu SG90 e inverta se 0 graus apontar para a direita.
constexpr uint16_t SERVO_MIN_US = 1000, SERVO_MAX_US = 2000;
// Nesta montagem o pulso menor apontava para a direita. O mapa sempre
// recebe o ângulo lógico: 0° à esquerda, 90° à frente, 180° à direita.
constexpr bool REVERSE_SERVO = true;
constexpr uint32_t AUTO_MAX_MS = 350, AUTO_REVERSE_MS = 120, AUTO_LEASE_MS = 2500;
constexpr uint16_t MAX_CM = 400;
constexpr uint8_t ANGLE_STEP = 10, LEFT_ANGLE = 0, LAST_ANGLE = 180;
constexpr uint8_t CENTER_ANGLE = LAST_ANGLE / 2;
constexpr uint8_t HOME_ANGLE = CENTER_ANGLE;  // Posição de repouso do suporte da câmera e do sensor.
static_assert(HOME_ANGLE <= LAST_ANGLE);
static_assert(ANGLE_STEP <= 10 && LAST_ANGLE % ANGLE_STEP == 0);
constexpr uint8_t POINTS = LAST_ANGLE / ANGLE_STEP + 1;
constexpr uint32_t FIRST_SETTLE_MS = 700, SERVO_STEP_MS = 45, STEP_SETTLE_MS = 300, PING_MS = 70;
constexpr uint8_t SAMPLES_PER_POINT = 5, MIN_AGREEING_SAMPLES = 3;
constexpr char AP_NAME[] = "Robo-Sonar";
constexpr char AP_PASSWORD[] = "sonar-2026";

WebServer server(80);
SemaphoreHandle_t motorLock = nullptr;
esp_timer_handle_t motorTimer = nullptr;
int64_t motorDeadline = 0;
uint32_t motorStoppedAt = 0;
bool motorReady = false;
bool servoReady = false;

// Os tempos usam o relógio da ESP32, pois a latência USB não representa
// tempo de tração. O callback de corte e o loop compartilham o mutex.
int64_t autoStartedUs = 0;
uint32_t autoElapsedMs = 0;
uint32_t autoPingAt = 0;
char autoDirection = 0;
bool autoActive = false;
uint32_t autoLastContact = 0;
char autoEnd = 'i';  // i: inativo, t: tempo, o: obstáculo, e: sem eco, s: Parar.

enum class ScanState { idle, checking, advancing, settling, homing, moving, running, returning, stopped, done, error };
enum class AdvanceEnd { none, obstacle, noEcho, timeLimit, motorFailure };

struct Reading {
  uint8_t angle;
  float distance;
};

ScanState scanState = ScanState::idle;
Reading readings[POINTS];
uint16_t rawPulses[POINTS][SAMPLES_PER_POINT];
uint8_t count = 0;
uint8_t angle = HOME_ANGLE;
uint8_t tries = 0, validCount = 0;
float validSamples[SAMPLES_PER_POINT];
uint8_t scanTarget = HOME_ANGLE;
uint8_t scanEchoes = 0, scanValidEchoes = 0;
uint32_t pingAt = 0;
uint32_t advanceStartedAt = 0;
AdvanceEnd advanceEnd = AdvanceEnd::none;
float frontCm = -1.0f;

bool activityRunning() {
  return scanState == ScanState::checking || scanState == ScanState::advancing ||
         scanState == ScanState::settling || scanState == ScanState::homing ||
         scanState == ScanState::moving || scanState == ScanState::running ||
         scanState == ScanState::returning;
}

void stopMotorsLocked() {
  ledcWrite(LEFT_ENABLE, 0);
  ledcWrite(RIGHT_ENABLE, 0);
  digitalWrite(LEFT_FWD, LOW);
  digitalWrite(RIGHT_FWD, LOW);
  digitalWrite(LEFT_REV, LOW);
  digitalWrite(RIGHT_REV, LOW);
  if (autoStartedUs != 0) {
    const int64_t elapsed = esp_timer_get_time() - autoStartedUs;
    autoElapsedMs = static_cast<uint32_t>(elapsed / 1000);
    autoStartedUs = 0;
  }
  if (motorDeadline != 0) {
    motorDeadline = 0;
    motorStoppedAt = millis();
  }
}

void motorTimeout(void*) {
  xSemaphoreTake(motorLock, portMAX_DELAY);
  if (motorDeadline != 0 && esp_timer_get_time() >= motorDeadline) {
    stopMotorsLocked();
  }
  xSemaphoreGive(motorLock);
}

void stopMotors() {
  if (!motorReady) return;
  xSemaphoreTake(motorLock, portMAX_DELAY);
  esp_timer_stop(motorTimer);
  stopMotorsLocked();
  xSemaphoreGive(motorLock);
}

bool motorBusy(bool includeSettling) {
  if (!motorReady) return false;
  xSemaphoreTake(motorLock, portMAX_DELAY);
  const bool busy = motorDeadline != 0 ||
                    (includeSettling && millis() - motorStoppedAt < MOTOR_SETTLE_MS);
  xSemaphoreGive(motorLock);
  return busy;
}

bool startMotion(uint32_t durationMs, char direction) {
  if (!motorReady || motorBusy(true)) return false;
  xSemaphoreTake(motorLock, portMAX_DELAY);
  // Num giro no lugar as duas rodas têm sentidos opostos.
  // A ré inverte as duas rodas e usa apenas o temporizador. O sonar
  // está na frente; a autorização do trecho já percorrido cabe ao painel.
  const bool leftReverse = direction == 'l' || direction == 'b';
  const bool rightReverse = direction == 'r' || direction == 'b';
  digitalWrite(LEFT_REV, leftReverse ? HIGH : LOW);
  digitalWrite(RIGHT_REV, rightReverse ? HIGH : LOW);
  digitalWrite(LEFT_FWD, leftReverse ? LOW : HIGH);
  digitalWrite(RIGHT_FWD, rightReverse ? LOW : HIGH);
  const bool powered = ledcWrite(LEFT_ENABLE, PWM_LEFT) && ledcWrite(RIGHT_ENABLE, PWM_RIGHT);
  const int64_t now = esp_timer_get_time();
  autoStartedUs = direction && powered ? now : 0;
  motorDeadline = now + durationMs * 1000ULL;
  const bool timed = powered && esp_timer_start_once(motorTimer, durationMs * 1000ULL) == ESP_OK;
  if (!timed) stopMotorsLocked();
  xSemaphoreGive(motorLock);
  return timed;
}

bool renewAdvance(uint32_t durationMs) {
  xSemaphoreTake(motorLock, portMAX_DELAY);
  const int64_t now = esp_timer_get_time();
  const bool armed = motorDeadline > now &&
                     esp_timer_restart(motorTimer, durationMs * 1000ULL) == ESP_OK;
  if (armed) motorDeadline = now + durationMs * 1000ULL;
  xSemaphoreGive(motorLock);
  return armed;
}

bool setAngle(uint8_t requested) {
  const uint8_t physical = REVERSE_SERVO ? LAST_ANGLE - requested : requested;
  const uint32_t pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * physical / LAST_ANGLE;
  const uint32_t duty = (pulse * (1UL << 14) + 10000) / 20000;
  if (!servoReady || !ledcWrite(SERVO, duty)) {
    servoReady = false;
    scanState = ScanState::error;
    return false;
  }
  angle = requested;
  return true;
}

void prepareScanMove(uint8_t target) {
  scanTarget = target;
  tries = 0;
  validCount = 0;
  pingAt = millis() + (angle == target ? STEP_SETTLE_MS : SERVO_STEP_MS);
}

unsigned long measureEchoUs() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, 30000);
}

float echoDistanceCm(unsigned long pulseUs) {
  return pulseUs * 0.0343f / 2.0f;
}

bool checkAutoFront() {
  frontCm = echoDistanceCm(measureEchoUs());
  if (frontCm < 2.0f || frontCm > MAX_CM) {
    frontCm = -1.0f;
    autoEnd = 'e';
    return false;
  }
  if (frontCm <= OBSTACLE_CM) {
    autoEnd = 'o';
    return false;
  }
  return true;
}

int startAuto(const char* action, const char*& error) {
  const char direction = action[5];
  char* end = nullptr;
  if ((direction != 'f' && direction != 'l' && direction != 'r' && direction != 'b') || action[6] != '_') {
    error = "Direção autônoma inválida.";
    return 400;
  }
  const unsigned long duration = strtoul(action + 7, &end, 10);
  const uint32_t maxDuration = direction == 'b' ? AUTO_REVERSE_MS : AUTO_MAX_MS;
  if (*end || duration < 40 || duration > maxDuration) {
    error = "Duração inválida para este sentido de tração.";
    return 400;
  }
  if (!autoActive || activityRunning() || motorBusy(true) || !servoReady || !motorReady || angle != CENTER_ANGLE) {
    error = "Aguarde as rodas pararem e centralize o sensor.";
    return 409;
  }

  autoElapsedMs = 0;
  autoEnd = 't';
  autoDirection = 0;
  if (direction == 'f' && !checkAutoFront()) return 200;
  if (!startMotion(duration, direction)) {
    error = "Falha ao iniciar movimento autônomo.";
    return 503;
  }
  autoDirection = direction;
  autoPingAt = millis() + PING_MS;
  return 200;
}

void updateAuto() {
  if (autoDirection == 0) return;
  if (!motorBusy(false)) {
    autoDirection = 0;
    return;
  }
  if (autoDirection != 'f' || static_cast<int32_t>(millis() - autoPingAt) < 0) return;
  autoPingAt = millis() + PING_MS;
  if (!checkAutoFront()) {
    stopMotors();
    autoDirection = 0;
  }
}

void startScan() {
  count = 0;
  scanEchoes = 0;
  scanValidEchoes = 0;
  scanState = ScanState::homing;
  prepareScanMove(LEFT_ANGLE);
}

void stopAndScan(AdvanceEnd reason) {
  stopMotors();
  advanceEnd = reason;
  scanState = ScanState::settling;
  pingAt = millis() + MOTOR_SETTLE_MS;
  if (reason == AdvanceEnd::obstacle) {
    Serial.printf("[AVANCO] Obstaculo a %.1f cm. Motores parados.\n", frontCm);
  } else {
    Serial.println("[AVANCO] Avanco encerrado. Preparando varredura.");
  }
}

void updateAdvance() {
  const uint32_t now = millis();
  if (scanState == ScanState::settling) {
    if (static_cast<int32_t>(now - pingAt) >= 0) startScan();
    return;
  }
  const bool moving = scanState == ScanState::advancing;
  if (!moving && scanState != ScanState::checking) return;
  if (moving && (now - advanceStartedAt >= ADVANCE_LIMIT_MS || !motorBusy(false))) {
    stopAndScan(AdvanceEnd::timeLimit);
    return;
  }
  if (static_cast<int32_t>(now - pingAt) < 0) return;
  pingAt = now + PING_MS;

  frontCm = echoDistanceCm(measureEchoUs());
  if (frontCm < 2.0f || frontCm > MAX_CM) {
    frontCm = -1.0f;
    stopAndScan(AdvanceEnd::noEcho);
  } else if (frontCm <= OBSTACLE_CM) {
    stopAndScan(AdvanceEnd::obstacle);
  } else if (!moving) {
    if (!startMotion(ADVANCE_WATCHDOG_MS, 0)) {
      stopAndScan(AdvanceEnd::motorFailure);
      return;
    }
    advanceStartedAt = millis();
    scanState = ScanState::advancing;
    Serial.println("[AVANCO] Motores ligados: PWM 175/180.");
  } else {
    const uint32_t elapsed = millis() - advanceStartedAt;
    if (elapsed >= ADVANCE_LIMIT_MS ||
        !renewAdvance(min(ADVANCE_WATCHDOG_MS, ADVANCE_LIMIT_MS - elapsed))) {
      stopAndScan(AdvanceEnd::timeLimit);
    }
  }
}

const char* advanceEndName() {
  switch (advanceEnd) {
    case AdvanceEnd::obstacle: return "obstacle";
    case AdvanceEnd::noEcho: return "noEcho";
    case AdvanceEnd::timeLimit: return "timeLimit";
    case AdvanceEnd::motorFailure: return "motorFailure";
    default: return "";
  }
}

float filteredDistance() {
  if (validCount < MIN_AGREEING_SAMPLES) return -1.0f;

  for (uint8_t i = 1; i < validCount; ++i) {
    const float value = validSamples[i];
    uint8_t j = i;
    while (j > 0 && validSamples[j - 1] > value) {
      validSamples[j] = validSamples[j - 1];
      --j;
    }
    validSamples[j] = value;
  }

  const float median = (validSamples[(validCount - 1) / 2] + validSamples[validCount / 2]) / 2.0f;
  const float tolerance = max(5.0f, median * 0.10f);
  uint8_t agreeing = 0;
  for (uint8_t i = 0; i < validCount; ++i) {
    if (fabsf(validSamples[i] - median) <= tolerance) ++agreeing;
  }
  return agreeing >= MIN_AGREEING_SAMPLES ? median : -1.0f;
}

void updateScan() {
  const bool moving = scanState == ScanState::homing || scanState == ScanState::moving ||
                      scanState == ScanState::returning;
  if ((!moving && scanState != ScanState::running) ||
      static_cast<int32_t>(millis() - pingAt) < 0) return;

  if (moving) {
    if (angle != scanTarget) {
      const uint8_t next = angle < scanTarget ? angle + 1 : angle - 1;
      if (setAngle(next)) {
        pingAt = millis() + (angle == scanTarget ? STEP_SETTLE_MS : SERVO_STEP_MS);
      }
      return;
    }
    if (scanState == ScanState::returning) {
      scanState = ScanState::done;
      Serial.printf("[SONAR] Varredura concluida. Repouso comandado em %u graus.\n", unsigned(HOME_ANGLE));
      return;
    }
    scanState = ScanState::running;
  }

  const uint32_t startedAt = millis();
  const unsigned long pulseUs = measureEchoUs();
  rawPulses[count][tries] = static_cast<uint16_t>(pulseUs);
  if (pulseUs != 0) ++scanEchoes;
  const float cm = echoDistanceCm(pulseUs);
  if (cm >= 2.0f && cm <= MAX_CM) {
    ++scanValidEchoes;
    validSamples[validCount++] = cm;
  }
  if (++tries < SAMPLES_PER_POINT) {
    pingAt = startedAt + PING_MS;
    return;
  }

  const float filtered = filteredDistance();
  readings[count++] = {angle, filtered};
  if (filtered >= 0) Serial.printf("[SONAR] %u graus | %.1f cm\n", unsigned(angle), filtered);
  else Serial.printf("[SONAR] %u graus | sem eco confiavel\n", unsigned(angle));

  if (angle == LAST_ANGLE) {
    scanState = ScanState::returning;
    prepareScanMove(HOME_ANGLE);
  } else {
    scanState = ScanState::moving;
    prepareScanMove(angle + ANGLE_STEP);
  }
}

const char* stateName() {
  switch (scanState) {
    case ScanState::checking: return "checking";
    case ScanState::advancing: return "advancing";
    case ScanState::settling: return "settling";
    case ScanState::homing: return "homing";
    case ScanState::moving: return "moving";
    case ScanState::running: return "running";
    case ScanState::returning: return "returning";
    case ScanState::stopped: return "stopped";
    case ScanState::done: return "done";
    case ScanState::error: return "error";
    default: return "idle";
  }
}

String buildState() {
  String json;
  json.reserve(1800);
  json += "{\"state\":\"";
  json += stateName();
  json += "\",\"angle\":";
  json += String(angle);
  json += ",\"frontCm\":";
  if (frontCm < 0) json += "null";
  else json += String(frontCm, 1);
  json += ",\"advanceEnd\":\"";
  json += advanceEndName();
  json += '"';
  json += ",\"servoReady\":";
  json += servoReady ? "true" : "false";
  json += ",\"motorBusy\":";
  json += motorBusy(true) ? "true" : "false";
  json += ",\"motorReady\":";
  json += motorReady ? "true" : "false";
  json += ",\"maxCm\":";
  json += String(MAX_CM);
  json += ",\"total\":";
  json += String(POINTS);
  uint32_t elapsed = 0;
  if (motorLock != nullptr) {
    xSemaphoreTake(motorLock, portMAX_DELAY);
    elapsed = autoElapsedMs;
    xSemaphoreGive(motorLock);
  }
  json += ",\"autoVersion\":1,\"autoElapsedMs\":";
  json += String(elapsed);
  json += ",\"reverseMaxMs\":";
  json += String(AUTO_REVERSE_MS);
  json += ",\"autoActive\":";
  json += autoActive ? "true" : "false";
  json += ",\"autoEnd\":\"";
  json += autoEnd;
  json += '"';
  json += ",\"scanEchoes\":";
  json += String(scanEchoes);
  json += ",\"scanValidEchoes\":";
  json += String(scanValidEchoes);
  json += ",\"points\":[";
  for (uint8_t i = 0; i < count; ++i) {
    if (i) json += ',';
    json += '[';
    json += String(readings[i].angle);
    json += ',';
    if (readings[i].distance < 0) json += "null";
    else json += String(readings[i].distance, 1);
    json += ']';
  }
  json += "],\"rawPulses\":[";
  for (uint8_t i = 0; i < count; ++i) {
    if (i) json += ',';
    json += '[';
    for (uint8_t j = 0; j < SAMPLES_PER_POINT; ++j) {
      if (j) json += ',';
      json += String(rawPulses[i][j]);
    }
    json += ']';
  }
  json += "]}";
  return json;
}

void sendState() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", buildState());
}

int applyCommand(const String& action, const char*& error) {
  if (action == "stop") {
    autoActive = false;
    if (autoDirection) {
      autoEnd = 's';
      autoDirection = 0;
    }
    if (activityRunning()) scanState = ScanState::stopped;
    stopMotors();
  } else if (action == "clear") {
    if (activityRunning()) {
      error = "Pare a atividade antes de limpar.";
      return 409;
    }
    count = 0;
    frontCm = -1.0f;
    advanceEnd = AdvanceEnd::none;
    if (scanState != ScanState::error) scanState = ScanState::idle;
  } else if (action == "start") {
    if (motorBusy(true) || activityRunning() || !servoReady) {
      error = "Aguarde os motores pararem e confira o servo.";
      return 409;
    }
    stopMotors();
    frontCm = -1.0f;
    advanceEnd = AdvanceEnd::none;
    startScan();
  } else if (action == "advance") {
    if (activityRunning() || motorBusy(true) || !motorReady || !servoReady) {
      error = "Avanço indisponível. Confira o servo e aguarde o robô parar.";
      return 409;
    }
    if (!setAngle(CENTER_ANGLE)) {
      error = "Falha ao direcionar o sensor para a frente.";
      return 503;
    }
    count = 0;
    frontCm = -1.0f;
    advanceEnd = AdvanceEnd::none;
    scanState = ScanState::checking;
    pingAt = millis() + FIRST_SETTLE_MS;
  } else if (action == "center") {
    if (activityRunning() || motorBusy(true)) {
      error = "Pare a varredura e aguarde os motores antes de centralizar.";
      return 409;
    }
    if (!setAngle(CENTER_ANGLE)) {
      error = "Falha ao centralizar o servo.";
      return 503;
    }
  } else if (action == "motor") {
    if (activityRunning() || !startMotion(MOTOR_TEST_MS, 0)) {
      error = "Teste indisponível durante a varredura ou a parada.";
      return 409;
    }
    count = 0;
    frontCm = -1.0f;
    advanceEnd = AdvanceEnd::none;
    if (scanState != ScanState::error) scanState = ScanState::idle;
  } else {
    error = "Comando inválido.";
    return 400;
  }
  return 200;
}

void command() {
  if (server.header("X-Robot-Command") != "local") {
    server.send(403, "text/plain", "Comando recusado.");
    return;
  }
  const String action = server.arg("action");
  if (autoActive && action != "stop") {
    server.send(409, "text/plain; charset=utf-8", "A exploração USB está em andamento.");
    return;
  }
  const char* error = nullptr;
  const int status = applyCommand(action, error);
  if (status == 200) sendState();
  else server.send(status, "text/plain; charset=utf-8", error);
}

void handleUsbRequest(const char* line) {
  if (*line != '@') return;
  char* action = nullptr;
  const unsigned long id = strtoul(line + 1, &action, 10);
  if (id == 0 || *action != ' ') return;
  ++action;

  if (strcmp(action, "echo") == 0) {
    if (activityRunning() || motorBusy(true)) {
      Serial.printf("@%lu 409 Pare o robo antes do teste de eco.\n", id);
      return;
    }
    const int echoIdle = digitalRead(ECHO);
    const unsigned long pulseUs = measureEchoUs();
    Serial.printf("@%lu 200 {\"pulseUs\":%lu,\"echoIdle\":%d,\"angle\":%u}\n",
                  id, pulseUs, echoIdle, unsigned(angle));
    return;
  }

  const char* error = nullptr;
  int status = 200;
  if (autoActive) autoLastContact = millis();
  if (strcmp(action, "auto_begin") == 0) {
    if (autoActive || activityRunning() || motorBusy(true) || !motorReady || !servoReady) {
      error = "A exploração não pode iniciar neste estado.";
      status = 409;
    } else {
      autoActive = true;
      autoLastContact = millis();
    }
  } else if (strncmp(action, "auto_", 5) == 0) {
    status = startAuto(action, error);
  } else if (strcmp(action, "state") != 0) {
    status = applyCommand(action, error);
  }
  Serial.printf("@%lu %d ", id, status);
  if (status == 200) Serial.println(buildState());
  else Serial.println(error);
}

void updateUsb() {
  static char line[48];
  static size_t length = 0;
  static bool overflow = false;
  for (uint8_t i = 0; i < sizeof(line) && Serial.available(); ++i) {
    const int byte = Serial.read();
    if (byte == '\n') {
      if (!overflow) {
        line[length] = '\0';
        handleUsbRequest(line);
      }
      length = 0;
      overflow = false;
    } else if (byte != '\r') {
      if (length < sizeof(line) - 1) line[length++] = static_cast<char>(byte);
      else overflow = true;
    }
  }
}

void setup() {
  Serial.begin(115200);
  const uint8_t motorPins[] = {LEFT_ENABLE, RIGHT_ENABLE, LEFT_FWD, LEFT_REV, RIGHT_FWD, RIGHT_REV};
  for (uint8_t pin : motorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  const bool leftReady = ledcAttach(LEFT_ENABLE, 2000, 8);
  const bool rightReady = ledcAttach(RIGHT_ENABLE, 2000, 8);
  if (leftReady) ledcWrite(LEFT_ENABLE, 0);
  if (rightReady) ledcWrite(RIGHT_ENABLE, 0);
  motorLock = xSemaphoreCreateMutex();
  esp_timer_create_args_t timerConfig = {};
  timerConfig.callback = motorTimeout;
  timerConfig.name = "motor-stop";
  motorReady = leftReady && rightReady && motorLock &&
               esp_timer_create(&timerConfig, &motorTimer) == ESP_OK;

  pinMode(TRIG, OUTPUT);
  digitalWrite(TRIG, LOW);
  pinMode(ECHO, INPUT);
  servoReady = ledcAttach(SERVO, 50, 14);
  setAngle(HOME_ANGLE);

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_NAME, AP_PASSWORD)) {
    Serial.println("[IHM] Falha ao iniciar a rede Wi-Fi.");
    return;
  }
  const char* headers[] = {"X-Robot-Command"};
  server.collectHeaders(headers, 1);
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html; charset=utf-8", SUPERVISORY_HTML); });
  server.on("/state", HTTP_GET, sendState);
  server.on("/command", HTTP_POST, command);
  server.begin();
  Serial.print("[IHM] http://");
  Serial.println(WiFi.softAPIP());
}

void loop() {
  // O corte de PWM é independente. A concessão USB também cancela a
  // varredura e libera o painel manual se o supervisório parar de responder.
  if (autoActive && millis() - autoLastContact > AUTO_LEASE_MS) {
    const char* error = nullptr;
    applyCommand("stop", error);
  }
  updateUsb();
  updateAuto();
  // Um cliente HTTP lento não pode atrasar a leitura frontal durante o pulso.
  if (!motorBusy(false)) server.handleClient();
  updateAdvance();
  updateScan();
  delay(1);
}
