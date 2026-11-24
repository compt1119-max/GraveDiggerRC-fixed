// =====================================================================
//  camion_rc_esp32.ino — Récepteur Bluetooth pour camion RC (ESP32 classique)
//
//  Téléphone --(Bluetooth Classic SPP)--> ESP32 --(PWM)--> ponts en H --> moteurs
//
//  PROTOCOLE (texte, une trame par ligne, ~50 trames/s envoyées par l'appli) :
//      J,<vitesse>,<direction>,<boutons>\n
//        vitesse    : -100 (arrière) .. 0 .. +100 (avant)
//        direction  : -100 (gauche)  .. 0 .. +100 (droite)
//        boutons    : masque binaire 0..15 (bit0 = bouton 1 ... bit3 = bouton 4)
//      Exemple : "J,65,-20,1\n"
//
//  SÉCURITÉ : si aucune trame valide n'arrive pendant FAILSAFE_MS, ou si le
//  téléphone se déconnecte, les moteurs s'arrêtent immédiatement.
//
//  CARTE : "ESP32 Dev Module" (ESP32 classique), Partition Scheme par défaut.
//  Fonctionne avec le paquet "esp32" d'Espressif version 2.x ou 3.x.
// =====================================================================
#include <Arduino.h>
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth Classic indisponible : choisir une carte ESP32 classique (ESP32 Dev Module)."
#endif

// ======================= ZONE AJUSTABLE PAR L'UTILISATEUR =======================

// --- Nom Bluetooth vu par le téléphone ---
const char* BT_NAME = "CamionRC";

// --- Broches des ponts en H (HYPOTHÈSE : 2 broches PWM par côté : avant / arrière) ---
// Évite les broches 0, 2, 12, 15 (démarrage), 6 à 11 (flash), 34 à 39 (entrée seule).
// Si tes ponts fonctionnent autrement (1 PWM + 1 DIR), adapte driveSide() plus bas.
const uint8_t PIN_L_FWD = 25;   // côté gauche, marche avant
const uint8_t PIN_L_REV = 26;   // côté gauche, marche arrière
const uint8_t PIN_R_FWD = 32;   // côté droit, marche avant
const uint8_t PIN_R_REV = 33;   // côté droit, marche arrière

// --- 4 sorties auxiliaires pilotées par les 4 boutons de l'appli ---
// Bouton 1 -> GPIO2 (LED bleue de la carte, pratique pour tester).
const uint8_t AUX_PINS[4] = {2, 16, 17, 18};

// --- PWM ---
const uint32_t PWM_FREQ_HZ  = 1000;  // 1 kHz (comme l'Arduino). Plus haut = moins de bruit audible
const uint8_t  PWM_RES_BITS = 8;     // résolution 8 bits (0..255)

// --- Orientation / mixage ---
const bool  INVERT_STEER = false;    // inverse gauche/droite de la direction
const bool  SWAP_SIDES   = false;    // échange les côtés gauche et droit
const bool  INVERT_LEFT  = false;    // inverse le sens du côté gauche
const bool  INVERT_RIGHT = false;    // inverse le sens du côté droit
const float STEER_GAIN   = 1.0f;     // sensibilité de la direction (1.0 = normal)

// --- Courbes de réponse (1.0 = linéaire ; >1 = plus doux autour du centre) ---
const float EXPO_SPEED = 1.5f;
const float EXPO_STEER = 1.3f;

// --- Moteurs ---
const float MOTOR_DEADZONE_PCT = 5.0f;   // en dessous de cette commande (%) : moteur à l'arrêt
const float MIN_DUTY_PCT       = 25.0f;  // PWM minimum quand le moteur tourne (vaincre le frottement)
const float MAX_DUTY_PCT       = 100.0f; // PWM maximum (plafond de vitesse)
const bool  BRAKE_WHEN_STOPPED = false;  // true : frein (2 entrées à 1) ; false : roue libre

// --- Rampes d'accélération (en % de pleine échelle par seconde) ---
const float ACCEL_PCT_PER_S = 400.0f;    // 0 -> 100 % en 0,25 s
const float DECEL_PCT_PER_S = 1200.0f;   // retour rapide vers 0

// --- Sécurité ---
const uint32_t FAILSAFE_MS = 300;        // arrêt si aucune trame pendant ce délai

// --- Télémétrie vers l'appli (capteurs futurs) ---
#define ENABLE_TELEMETRY 0
const uint32_t TELEMETRY_MS = 500;

// --- Débogage sur le moniteur série (115200) ---
#define DEBUG_SERIAL 1

// ======================= FIN DE LA ZONE AJUSTABLE =======================

BluetoothSerial SerialBT;

// ---------- Abstraction PWM (compatible paquet esp32 2.x et 3.x) ----------
const uint32_t PWM_MAX = (1u << PWM_RES_BITS) - 1;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static void pwmAttach(uint8_t pin, uint8_t /*ch*/) { ledcAttach(pin, PWM_FREQ_HZ, PWM_RES_BITS); }
  static void pwmWrite(uint8_t pin, uint8_t /*ch*/, uint32_t duty) { ledcWrite(pin, duty); }
#else
  static void pwmAttach(uint8_t pin, uint8_t ch) { ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(pin, ch); }
  static void pwmWrite(uint8_t /*pin*/, uint8_t ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// ---------- État ----------
static float    targetL = 0, targetR = 0;   // consignes (-100..100)
static float    curL = 0, curR = 0;         // valeurs réellement appliquées (après rampe)
static uint32_t lastFrameMs = 0;
static bool     everReceived = false;
static bool     prevLinkOk = false;
static bool     prevButtons[4] = {false, false, false, false};
static char     lineBuf[48];
static uint8_t  lineLen = 0;

// ---------- Boutons : ajoute ici tes actions (presets, klaxon, lecture de capteur...) ----------
static void onButton(uint8_t idx, bool on) {
  digitalWrite(AUX_PINS[idx], on ? HIGH : LOW);
  // EXEMPLES FUTURS :
  //   if (idx == 2 && on) { /* preset 1 : mode lent */ }
  //   if (idx == 3 && on) { /* preset 2 : mode rapide */ }
#if DEBUG_SERIAL
  Serial.printf("Bouton %u -> %s\n", idx + 1, on ? "ON" : "OFF");
#endif
}

static void applyButtons(uint8_t mask) {
  for (uint8_t i = 0; i < 4; i++) {
    bool on = (mask >> i) & 1;
    if (on != prevButtons[i]) { prevButtons[i] = on; onButton(i, on); }
  }
}

// ---------- Mixage ----------
static float expoCurve(float x, float e) {
  float a = fabsf(x) / 100.0f;
  float y = powf(a, e) * 100.0f;
  return (x < 0) ? -y : y;
}

// vitesse/direction (-100..100) -> consignes gauche/droite (-100..100), marche arrière incluse
static void computeTargets(int speed, int steer, float &l, float &r) {
  float t = expoCurve((float)speed, EXPO_SPEED);
  float s = expoCurve((float)steer, EXPO_STEER) * STEER_GAIN;
  if (INVERT_STEER) s = -s;
  l = t + s;
  r = t - s;
  float m = fmaxf(100.0f, fmaxf(fabsf(l), fabsf(r)));   // garde le rapport G/D si on dépasse 100
  l = l * 100.0f / m;
  r = r * 100.0f / m;
  if (SWAP_SIDES) { float tmp = l; l = r; r = tmp; }
  if (INVERT_LEFT)  l = -l;
  if (INVERT_RIGHT) r = -r;
}

// ---------- Commande d'un côté (-100..100) vers 2 broches PWM ----------
static void driveSide(uint8_t pinF, uint8_t chF, uint8_t pinR, uint8_t chR, float v) {
  float mag = fabsf(v);
  uint32_t duty = 0;
  if (mag >= MOTOR_DEADZONE_PCT) {
    float n   = (mag - MOTOR_DEADZONE_PCT) / (100.0f - MOTOR_DEADZONE_PCT);   // 0..1
    float pct = MIN_DUTY_PCT + (MAX_DUTY_PCT - MIN_DUTY_PCT) * n;
    duty = (uint32_t)(pct / 100.0f * PWM_MAX + 0.5f);
    if (duty > PWM_MAX) duty = PWM_MAX;
  }
  if (duty == 0) {
    uint32_t d = BRAKE_WHEN_STOPPED ? PWM_MAX : 0;
    pwmWrite(pinF, chF, d);
    pwmWrite(pinR, chR, d);
  } else if (v > 0) {
    pwmWrite(pinF, chF, duty);
    pwmWrite(pinR, chR, 0);
  } else {
    pwmWrite(pinF, chF, 0);
    pwmWrite(pinR, chR, duty);
  }
}

static void writeMotors() {
  driveSide(PIN_L_FWD, 0, PIN_L_REV, 1, curL);
  driveSide(PIN_R_FWD, 2, PIN_R_REV, 3, curR);
}

static float stepToward(float cur, float tgt, float dt) {
  bool speedingUp = (fabsf(tgt) > fabsf(cur)) && (tgt * cur >= 0);
  float rate = (speedingUp ? ACCEL_PCT_PER_S : DECEL_PCT_PER_S) * dt;
  float diff = tgt - cur;
  if (diff >  rate) return cur + rate;
  if (diff < -rate) return cur - rate;
  return tgt;
}

// ---------- Réception ----------
static void parseLine() {
  lineBuf[lineLen] = 0;
  int s, t, b;
  if (sscanf(lineBuf, "J,%d,%d,%d", &s, &t, &b) == 3) {
    s = constrain(s, -100, 100);
    t = constrain(t, -100, 100);
    b = constrain(b, 0, 15);
    computeTargets(s, t, targetL, targetR);
    applyButtons((uint8_t)b);
    lastFrameMs = millis();
    everReceived = true;
  }
}

static void readBluetooth() {
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n') {
      parseLine();
      lineLen = 0;
    } else if (c != '\r') {
      if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
      else lineLen = 0;   // ligne trop longue : on jette
    }
  }
}

// ---------- Télémétrie (optionnelle) ----------
static void sendTelemetry() {
#if ENABLE_TELEMETRY
  static uint32_t last = 0;
  if (millis() - last >= TELEMETRY_MS && SerialBT.hasClient()) {
    last = millis();
    // FORMAT : "T,<nom>,<valeur>\n" — remplace par la lecture d'un vrai capteur.
    SerialBT.printf("T,uptime,%lu\n", (unsigned long)(millis() / 1000));
  }
#endif
}

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < 4; i++) { pinMode(AUX_PINS[i], OUTPUT); digitalWrite(AUX_PINS[i], LOW); }

  pwmAttach(PIN_L_FWD, 0);
  pwmAttach(PIN_L_REV, 1);
  pwmAttach(PIN_R_FWD, 2);
  pwmAttach(PIN_R_REV, 3);
  writeMotors();   // tout à 0 au démarrage

  SerialBT.begin(BT_NAME);
  Serial.printf("Bluetooth prêt : appaire le téléphone avec \"%s\"\n", BT_NAME);
}

void loop() {
  readBluetooth();

  // Liaison valide = client connecté + trame récente
  bool linkOk = everReceived && SerialBT.hasClient() && (millis() - lastFrameMs <= FAILSAFE_MS);

  if (!linkOk) {
    // FAILSAFE : arrêt immédiat (sans rampe) et boutons relâchés
    targetL = targetR = 0;
    curL = curR = 0;
    if (prevLinkOk) applyButtons(0);
    if (!SerialBT.hasClient()) everReceived = false;
  }
  prevLinkOk = linkOk;

  // Mise à jour des moteurs toutes les 5 ms
  static uint32_t lastUs = 0;
  uint32_t nowUs = micros();
  if (nowUs - lastUs >= 5000) {
    float dt = (nowUs - lastUs) / 1000000.0f;
    lastUs = nowUs;
    curL = stepToward(curL, targetL, dt);
    curR = stepToward(curR, targetR, dt);
    writeMotors();
  }

  sendTelemetry();

#if DEBUG_SERIAL
  static uint32_t lastDbg = 0;
  if (millis() - lastDbg >= 200) {
    lastDbg = millis();
    if (linkOk) Serial.printf("L=%.0f R=%.0f\n", curL, curR);
  }
#endif

  delay(1);
}
