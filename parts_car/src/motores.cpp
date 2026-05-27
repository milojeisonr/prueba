#include "motores.h"
#include <Arduino.h>

// ==== Variables globales ====
int velocidadPWM        = 128;
int velocidadPorcentaje = 50;

// ==== Presets disponibles ====
static const int presetsManual[3]   = {30, 60, 90};
static const int presetsAutonoma[3] = {30, 50, 80};

// ==== Límites de velocidad ====
// Manual:   se fija cuando el usuario pulsa V·1 / V·2 / V·3
// Autónomo: se fija por el preset del modo autónomo
static int velocidadMaximaManual   = 100; // sin límite por defecto

// ======================================================
// CONFIGURACIÓN INICIAL
// ======================================================

void inicializarMotores() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);

  ledcAttachPin(ENA, PWM_CH_A);
  ledcAttachPin(ENB, PWM_CH_B);

  detenerMotores();
  Serial.println("✅ Motores inicializados");
}

// ======================================================
// CONTROL DE VELOCIDAD MANUAL
// ======================================================

void setVelocidad(int porcentaje) {
  // Respeta el límite máximo del preset manual
  porcentaje = constrain(porcentaje, 0, velocidadMaximaManual);

  velocidadPorcentaje = constrain(porcentaje, 0, 100);
  velocidadPWM        = map(velocidadPorcentaje, 0, 100, 0, 255);

  Serial.printf("⚙️ Velocidad manual: %d%% (%d PWM) | Límite: %d%%\n",
    velocidadPorcentaje, velocidadPWM, velocidadMaximaManual);
}

int getVelocidad() {
  return velocidadPorcentaje;
}

int getVelocidadPWM() {
  return velocidadPWM;
}

// ======================================================
// LÍMITE MANUAL
// ======================================================

void setVelocidadMaximaManual(int porcentaje) {
  velocidadMaximaManual = constrain(porcentaje, 0, 100);
  Serial.printf("🔒 Límite manual fijado en: %d%%\n", velocidadMaximaManual);
}

int getVelocidadMaximaManual() {
  return velocidadMaximaManual;
}

// ======================================================
// CONTROL DE VELOCIDAD AUTÓNOMA
// ======================================================


// ======================================================
// PRESETS
// ======================================================

// ✅ DEBE ESTAR en motores.cpp:
void setPresetSpeed(bool autonoma, int level) {
  if (level < 1) level = 1;
  if (level > 3) level = 3;

  if (!autonoma) {
    int porcentaje = presetsManual[level - 1];
    setVelocidadMaximaManual(porcentaje);
    setVelocidad(porcentaje);
    Serial.printf("🎚️ Preset manual nivel %d → %d%%\n", level, porcentaje);
  }
}
// ======================================================
// MOVIMIENTO ADELANTE
// ======================================================

void moverAdelante(int porcentaje) {
  if (porcentaje >= 0) {
    setVelocidad(porcentaje); // setVelocidad ya aplica el límite manual
  }

  ledcWrite(PWM_CH_A, velocidadPWM);
  ledcWrite(PWM_CH_B, velocidadPWM);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ======================================================
// MOVIMIENTO ATRÁS
// ======================================================

void moverAtras(int porcentaje) {
  if (porcentaje >= 0) {
    setVelocidad(porcentaje);
  }

  ledcWrite(PWM_CH_A, velocidadPWM);
  ledcWrite(PWM_CH_B, velocidadPWM);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ======================================================
// GIRO IZQUIERDA
// ======================================================

void girarIzquierda(int porcentaje) {
  if (porcentaje >= 0) {
    setVelocidad(porcentaje);
  }

  ledcWrite(PWM_CH_A, velocidadPWM);
  ledcWrite(PWM_CH_B, velocidadPWM);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ======================================================
// GIRO DERECHA
// ======================================================

void girarDerecha(int porcentaje) {
  if (porcentaje >= 0) {
    setVelocidad(porcentaje);
  }

  ledcWrite(PWM_CH_A, velocidadPWM);
  ledcWrite(PWM_CH_B, velocidadPWM);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

// ======================================================
// GIRO SUAVE IZQUIERDA
// ======================================================

void girarSuaveIzquierda() {
  int pwmSuave = (int)(velocidadPWM * 0.6);

  ledcWrite(PWM_CH_A, pwmSuave);
  ledcWrite(PWM_CH_B, velocidadPWM);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ======================================================
// GIRO SUAVE DERECHA
// ======================================================

void girarSuaveDerecha() {
  int pwmSuave = (int)(velocidadPWM * 0.6);

  ledcWrite(PWM_CH_A, velocidadPWM);
  ledcWrite(PWM_CH_B, pwmSuave);

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// ======================================================
// FRENADO SUAVE
// ======================================================

void frenarSuave() {
  for (int pwm = velocidadPWM; pwm >= 0; pwm -= 10) {
    ledcWrite(PWM_CH_A, pwm);
    ledcWrite(PWM_CH_B, pwm);
    delay(20);
  }
  detenerMotores();
}

// ======================================================
// DETENER MOTORES
// ======================================================

void detenerMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
}