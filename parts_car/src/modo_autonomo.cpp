#include "modo_autonomo.h"
#include "motores.h"
#include "sensores.h"
#include "webserver_setup.h"
#include "servo_rele.h"
#include "mapa_espacios.h"
#include <Arduino.h>

void avanzarEnMapa();

bool modoAutonomoActivo = false;

// ========== PARÁMETROS OPTIMIZADOS PARA CARRO 27×17cm ==========

// --- Distancias de decisión ---
const int DISTANCIA_SEGURA   = 45;   // > 45cm: TODO libre
const int DISTANCIA_ALERTA   = 30;   // 30-45cm: Reducir velocidad
const int DISTANCIA_CRITICA  = 15;   // 15-30cm: Crítico
const int DISTANCIA_BLOQUEADA = 8;   // < 8cm: Completamente bloqueado

// --- HISTÉRESIS: Banda muerta para evitar oscilaciones (cm) ---
const int HISTERESIS = 5;

// --- Tiempos de acción ---
const int RETARDO_GIRO      = 350;   // ms
const int RETARDO_DETECCION = 100;   // ms
const int BACKUP_TIME       = 400;   // ms cuando retrocede
const int ESCANEO_DELAY     = 220;   // ms espera tras mover servo
const int TIMEOUT_ALERTA    = 8000;  // ms máximo en ALERT/CRITICAL
const int TIMEOUT_GIRO      = 5000;  // ms máximo en TURNING

// --- PRESETS DE VELOCIDAD AUTÓNOMA (LÍMITES MÁXIMOS) ---
static const int PRESET_VELOCIDADES[3] = {30, 50, 80}; // Baja, Media, Alta
static int presetActualAutonomo        = 1;             // MEDIA por defecto
static int velocidadMaximaAutonoma     = PRESET_VELOCIDADES[1]; // 50%

// --- Velocidad dinámica adaptativa ---
int velocidadAutonomaPorcentaje = 50;

// ========== FILTRO EMA ==========
static long filtroFront = 100;
static long filtroLeft  = 100;
static long filtroRight = 100;
const float FACTOR_FILTRO = 0.3f;

long filtrarLectura(long nuevaLectura, long lecturaAnterior) {
  if (nuevaLectura <= 0 || nuevaLectura > 400) return lecturaAnterior;
  long filtrada = (nuevaLectura * FACTOR_FILTRO) + (lecturaAnterior * (1.0f - FACTOR_FILTRO));
  return constrain(filtrada, 0, 400);
}

bool sensorConfiable(long dist) {
  return (dist > 0 && dist < 400);
}

// ========== MÁQUINA DE ESTADOS ==========
enum AutonoState {
  AS_IDLE = 0,
  AS_NAVIGATE,
  AS_ALERT,
  AS_CRITICAL,
  AS_SCANNING,
  AS_TURNING,
  AS_BACKING,
  AS_ESCAPE,
  AS_MICRO_CORRECCION
};
static AutonoState autEstado = AS_IDLE;
static unsigned long estadoMillis  = 0;
static unsigned long accionMillis  = 0;
static unsigned long tiempoEnEstado = 0;

// Telemetría
static long   lastFront    = 999;
static long   lastLeft     = 999;
static long   lastCenter   = 999;
static long   lastRight    = 999;
static String lastAction   = "idle";
static String lastDecision = "INIT";

// Tracking de giros
static int           ultimoGiroDir       = -1;
static unsigned long tiempoUltimoGiro    = 0;
static int           conteoGirosContinuos = 0;
const  int           MAX_GIROS_CONTINUOS  = 3;

// Escaneo no bloqueante
enum ScanPhase {
  SP_INIT = 0, SP_SCAN_LEFT, SP_WAIT_LEFT,
  SP_SCAN_CENTER, SP_WAIT_CENTER,
  SP_SCAN_RIGHT,  SP_WAIT_RIGHT, SP_DONE
};
static ScanPhase     scanPhase    = SP_INIT;
static unsigned long scanMillis   = 0;
static long          scanLeftVal  = 999;
static long          scanCenterVal = 999;
static long          scanRightVal = 999;

// Timeouts
const  int           TIMEOUT_ESCANEO    = 3000;
const  int           TIMEOUT_CORRECCION = 2000;
static unsigned long scanStartTime      = 0;
static unsigned long ultimoMovimientoMapa = 0;
const  int           INTERVALO_MAPA     = 700;

// ========================================================
// FUNCIONES INTERNAS DE MOVIMIENTO (respetan límite autónomo)
// ========================================================
// Añade esto antes de girarSuaveIzquierda() o girarSuaveDerecha()


static void _adelanteAuto(int pct) {
  pct = constrain(pct, 0, velocidadMaximaAutonoma);
  int pwm = map(pct, 0, 100, 0, 255);
  ledcWrite(PWM_CH_A, pwm);
  ledcWrite(PWM_CH_B, pwm);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

static void _atrasAuto(int pct) {
  pct = constrain(pct, 0, velocidadMaximaAutonoma);
  int pwm = map(pct, 0, 100, 0, 255);
  ledcWrite(PWM_CH_A, pwm);
  ledcWrite(PWM_CH_B, pwm);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
}

static void _izquierdaAuto(int pct) {
  pct = constrain(pct, 0, velocidadMaximaAutonoma);
  int pwm = map(pct, 0, 100, 0, 255);
  ledcWrite(PWM_CH_A, pwm);
  ledcWrite(PWM_CH_B, pwm);
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

static void _derechaAuto(int pct) {
  pct = constrain(pct, 0, velocidadMaximaAutonoma);
  int pwm = map(pct, 0, 100, 0, 255);
  ledcWrite(PWM_CH_A, pwm);
  ledcWrite(PWM_CH_B, pwm);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

// ========================================================
// FUNCIONES DE VELOCIDAD AUTÓNOMA (declaradas en modo_autonomo.h)
// ========================================================


void setPresetVelocidadAutonoma(int preset) {
  if (preset < 1) preset = 1;
  if (preset > 3) preset = 3;
  presetActualAutonomo    = preset - 1;
  velocidadMaximaAutonoma = PRESET_VELOCIDADES[presetActualAutonomo];

  // Si la velocidad actual supera el nuevo límite, recortarla
  if (velocidadAutonomaPorcentaje > velocidadMaximaAutonoma)
    velocidadAutonomaPorcentaje = velocidadMaximaAutonoma;

  const char* nombres[] = {"🐢 BAJA", "🚗 MEDIA", "🚀 ALTA"};
  Serial.printf("⚙️ Preset autónomo: %s (%d%% máx)\n",
    nombres[presetActualAutonomo], velocidadMaximaAutonoma);
}

void setVelocidadAutonoma(int porcentaje) {
  porcentaje = constrain(porcentaje, 0, velocidadMaximaAutonoma);
  velocidadAutonomaPorcentaje = porcentaje;

  int pwm = map(porcentaje, 0, 100, 0, 255);
  ledcWrite(PWM_CH_A, pwm);
  ledcWrite(PWM_CH_B, pwm);

  Serial.printf("🤖 Vel autónoma: %d%% (%d PWM) | Máx: %d%%\n",
    porcentaje, pwm, velocidadMaximaAutonoma);
}

int getVelocidadAutonoma() {
  return velocidadAutonomaPorcentaje;
}

int getVelocidadMaximaAutonoma() {
  return velocidadMaximaAutonoma;
}

// ========================================================
// FUNCIONES DE DECISIÓN
// ========================================================

int calcularVelocidadAdaptativa(long distFront) {
  int base = velocidadMaximaAutonoma;
  if (distFront >= DISTANCIA_SEGURA)  return base;
  if (distFront >= DISTANCIA_ALERTA)  return max(15, (base * 80) / 100);
  if (distFront >= DISTANCIA_CRITICA) return max(15, (base * 50) / 100);
  return max(10, (base * 30) / 100);
}

int calcularEspacioDisponible(long distancia) {
  if (distancia >= DISTANCIA_SEGURA)   return 100;
  if (distancia >= DISTANCIA_ALERTA)   return 75;
  if (distancia >= DISTANCIA_CRITICA)  return 50;
  if (distancia >= DISTANCIA_BLOQUEADA) return 25;
  return 0;
}

bool estaAtrapado() {
  return (conteoGirosContinuos >= MAX_GIROS_CONTINUOS);
}

void registrarGiro(int direccion) {
  if (direccion != ultimoGiroDir) {
    ultimoGiroDir = direccion;
    conteoGirosContinuos = 1;
  } else {
    conteoGirosContinuos++;
  }
  tiempoUltimoGiro = millis();
  if (conteoGirosContinuos >= MAX_GIROS_CONTINUOS) {
    Serial.printf("⚠️ Atrapamiento: %d giros %s consecutivos\n",
      conteoGirosContinuos,
      (direccion == 0) ? "IZQUIERDA" : "DERECHA");
  }
}

void resetearGiros() {
  conteoGirosContinuos = 0;
  ultimoGiroDir = -1;
}

void guardarMemoriaGiro(int direccion) {
  memoriaGiros[indiceMemoria] = direccion;
  indiceMemoria++;
  if (indiceMemoria >= MEMORIA_GIROS_SIZE) indiceMemoria = 0;

  int iguales = 0;
  for (int i = 0; i < MEMORIA_GIROS_SIZE; i++)
    if (memoriaGiros[i] == direccion) iguales++;

  giroConsecutivos = iguales;
  Serial.printf("🧠 Memoria giro: %d | repetidos=%d\n", direccion, giroConsecutivos);
}

String getAutonomoDebug() {
  char json[512];
  snprintf(json, sizeof(json),
    "{\"activo\":%d,\"estado\":%d,\"front\":%ld,\"left\":%ld,\"center\":%ld,\"right\":%ld,"
    "\"velocidad_actual\":%d,\"velocidad_maxima\":%d,\"preset\":%d,"
    "\"accion\":\"%s\",\"decision\":\"%s\",\"espacio_disponible\":%d}",
    modoAutonomoActivo ? 1 : 0,
    (int)autEstado,
    lastFront, lastLeft, lastCenter, lastRight,
    velocidadAutonomaPorcentaje,
    velocidadMaximaAutonoma,
    presetActualAutonomo + 1,
    lastAction.c_str(),
    lastDecision.c_str(),
    calcularEspacioDisponible(lastFront)
  );
  return String(json);
}

// ========================================================
// MÁQUINA DE ESTADOS PRINCIPAL
// ========================================================

void ejecutarModoAutonomo(int modoActual) {
  if (modoActual != MODE_AUTO || !modoAutonomoActivo) return;

  // Leer y filtrar sensores
  filtroFront = filtrarLectura(medirDistanciaFront(), filtroFront);
  filtroLeft  = filtrarLectura(medirDistanciaLeft(),  filtroLeft);
  filtroRight = filtrarLectura(medirDistanciaRight(), filtroRight);

  long distFront = filtroFront;
  long distLeft  = filtroLeft;
  long distRight = filtroRight;

  lastFront = distFront;
  lastLeft  = distLeft;
  lastRight = distRight;

  unsigned long now = millis();
  tiempoEnEstado = now - estadoMillis;

  actualizarMapaDesdeDistancias(distFront, distLeft, distRight);

  int velocidadActual = calcularVelocidadAdaptativa(distFront);
  velocidadAutonomaPorcentaje = velocidadActual;

  switch (autEstado) {

    // ===== IDLE =====
    case AS_IDLE:
      moverServoCentro();
      _adelanteAuto(velocidadActual);
      lastAction   = "start_forward";
      lastDecision = "INICIAR";
      autEstado    = AS_NAVIGATE;
      estadoMillis = now;
      break;

    // ===== NAVIGATE =====
    case AS_NAVIGATE:
      resetearGiros();

      if (distFront > DISTANCIA_SEGURA) {
        _adelanteAuto(velocidadActual);

        // Corrección de trayectoria
        if (abs(distLeft - distRight) > 8) {
          if (distLeft > distRight + 5) {
            setVelocidad(velocidadActual);
            girarSuaveIzquierda();
            autEstado  = AS_MICRO_CORRECCION;
            accionMillis = now;
            lastAction = "micro_correction_left";
          } else if (distRight > distLeft + 5) {
            setVelocidad(velocidadActual);
            girarSuaveDerecha();
            autEstado  = AS_MICRO_CORRECCION;
            accionMillis = now;
            lastAction = "micro_correction_right";
          }
        }
        

        avanzarEnMapa();
        lastAction   = "navigate_free";
        lastDecision = "TODO_LIBRE";
      }
      else if (distFront > DISTANCIA_ALERTA + HISTERESIS) {
        _adelanteAuto(velocidadActual);
        lastAction   = "navigate_alert";
        lastDecision = "ALERTA_REDUCIR";
        autEstado    = AS_ALERT;
        estadoMillis = now;
      }
      else if (distFront > DISTANCIA_CRITICA + HISTERESIS) {
        autEstado    = AS_CRITICAL;
        estadoMillis = now;
      }
      else if (distFront <= DISTANCIA_CRITICA) {
        detenerMotores();
        lastAction   = "obstacle_detected";
        lastDecision = "OBSTACULO";
        autEstado    = AS_SCANNING;
        estadoMillis = now;
        scanPhase    = SP_INIT;
        scanStartTime = now;
      }
      break;

    // ===== ALERT =====
    case AS_ALERT:
      if (tiempoEnEstado > TIMEOUT_ALERTA) {
        Serial.println("⏱️ TIMEOUT ALERT - forzando escaneo");
        detenerMotores();
        autEstado     = AS_SCANNING;
        scanPhase     = SP_INIT;
        scanStartTime = now;
        lastDecision  = "TIMEOUT_SCAN";
        break;
      }
      if (distFront > DISTANCIA_SEGURA + HISTERESIS) {
        autEstado    = AS_NAVIGATE;
        lastDecision = "VOLVER_NAVEGAR";
      } else if (distFront <= DISTANCIA_CRITICA - HISTERESIS) {
        autEstado    = AS_CRITICAL;
        estadoMillis = now;
        lastDecision = "IR_CRITICA";
      } else {
        _adelanteAuto(velocidadActual);
        lastAction   = "alert_slow";
        lastDecision = "ALERTA";
      }
      break;

    // ===== CRITICAL =====
    case AS_CRITICAL:
      if (tiempoEnEstado > TIMEOUT_ALERTA) {
        Serial.println("⏱️ TIMEOUT CRITICAL - forzando escaneo");
        detenerMotores();
        autEstado     = AS_SCANNING;
        scanPhase     = SP_INIT;
        scanStartTime = now;
        lastDecision  = "TIMEOUT_SCAN_CRIT";
        break;
      }
      if (distFront > DISTANCIA_ALERTA + HISTERESIS) {
        autEstado    = AS_NAVIGATE;
        lastDecision = "VOLVER_LIBRE";
      } else if (distFront <= DISTANCIA_CRITICA - HISTERESIS) {
        detenerMotores();
        autEstado     = AS_SCANNING;
        estadoMillis  = now;
        scanPhase     = SP_INIT;
        scanStartTime = now;
        lastDecision  = "SCAN_URGENTE";
      } else {
        _adelanteAuto(max(10, velocidadActual / 2));
        lastAction   = "critical_slow";
        lastDecision = "CRITICA";
      }
      break;

    // ===== MICRO CORRECCIÓN =====
    case AS_MICRO_CORRECCION:
      if (now - accionMillis >= 200) {
        _adelanteAuto(velocidadActual);
        autEstado    = AS_NAVIGATE;
        estadoMillis = now;
        lastAction   = "correction_done";
      }
      break;

    // ===== SCANNING =====
    case AS_SCANNING: {
      if (now - scanStartTime > TIMEOUT_ESCANEO) {
        Serial.println("⏱️ TIMEOUT escaneo - forzar retroceso");
        detenerMotores();
        _atrasAuto(60);
        autEstado    = AS_BACKING;
        accionMillis = now;
        lastDecision = "TIMEOUT_ESCAPE";
        scanPhase    = SP_INIT;
        break;
      }

      switch (scanPhase) {
        case SP_INIT:
          detenerMotores();
          moverServoIzquierda();
          scanMillis = now;
          scanPhase  = SP_WAIT_LEFT;
          lastAction = "scan_left";
          break;

        case SP_WAIT_LEFT:
          if (now - scanMillis >= ESCANEO_DELAY) {
            scanLeftVal = medirDistanciaFront();
            lastLeft    = scanLeftVal;
            moverServoCentro();
            scanMillis = now;
            scanPhase  = SP_WAIT_CENTER;
            lastAction = "scan_center";
          }
          break;

        case SP_WAIT_CENTER:
          if (now - scanMillis >= ESCANEO_DELAY) {
            scanCenterVal = medirDistanciaFront();
            lastCenter    = scanCenterVal;
            moverServoDerecha();
            scanMillis = now;
            scanPhase  = SP_WAIT_RIGHT;
            lastAction = "scan_right";
          }
          break;

        case SP_WAIT_RIGHT:
          if (now - scanMillis >= ESCANEO_DELAY) {
            scanRightVal = medirDistanciaFront();
            lastRight    = scanRightVal;
            moverServoCentro();

            Serial.printf("📡 ESCANEO: L=%ld | C=%ld | R=%ld\n",
              scanLeftVal, scanCenterVal, scanRightVal);

            if (scanLeftVal  < DISTANCIA_BLOQUEADA &&
                scanCenterVal < DISTANCIA_BLOQUEADA &&
                scanRightVal  < DISTANCIA_BLOQUEADA) {
              lastAction   = "all_blocked";
              lastDecision = "BLOQUEADO_RETROCEDER";
              _atrasAuto(70);
              autEstado    = AS_BACKING;
              accionMillis = now;
              registrarGiro(2);
            }
            else if (scanCenterVal >= DISTANCIA_CRITICA) {
              lastAction   = "center_free";
              lastDecision = "CENTRO_LIBRE";
              _adelanteAuto(velocidadActual);
              autEstado = AS_NAVIGATE;
            }
            else if (scanLeftVal > scanRightVal + 15) {
              lastAction   = "turn_left";
              lastDecision = "GIRAR_IZQ";
              _izquierdaAuto(max(30, velocidadActual));
              rotarRobot(robotDireccion - 1);
              guardarMemoriaGiro(0);
              registrarGiro(0);
              autEstado    = AS_TURNING;
              accionMillis = now;
            }
            else if (scanRightVal > scanLeftVal + 15) {
              lastAction   = "turn_right";
              lastDecision = "GIRAR_DER";
              _derechaAuto(max(30, velocidadActual));
              rotarRobot(robotDireccion + 1);
              guardarMemoriaGiro(1);
              registrarGiro(1);
              autEstado    = AS_TURNING;
              accionMillis = now;
            }
            else {
              lastAction   = "turn_right_default";
              lastDecision = "GIRAR_DER_DEFAULT";
              _derechaAuto(max(30, velocidadActual));
              registrarGiro(1);
              autEstado    = AS_TURNING;
              accionMillis = now;
            }

            scanPhase = SP_INIT;
          }
          break;

        default:
          scanPhase = SP_INIT;
          break;
      }
      break;
    }

    // ===== TURNING =====
    case AS_TURNING:
      if (now - accionMillis >= RETARDO_GIRO) {
        detenerMotores();
        if (estaAtrapado()) {
          Serial.println("🚨 ATRAPADO - escape 180°");
          lastAction   = "escape_180";
          lastDecision = "ESCAPE_ATRAPAMIENTO";
          autEstado    = AS_ESCAPE;
          accionMillis = now;
        } else {
          _adelanteAuto(velocidadActual);
          lastAction   = "forward_after_turn";
          lastDecision = "NAVEGANDO";
          autEstado    = AS_NAVIGATE;
          estadoMillis = now;
        }
      }
      break;

    // ===== BACKING =====
    case AS_BACKING:
      if (now - accionMillis >= BACKUP_TIME) {
        detenerMotores();
        lastAction    = "backup_complete";
        lastDecision  = "ESCANEO_NUEVO";
        autEstado     = AS_SCANNING;
        estadoMillis  = now;
        scanPhase     = SP_INIT;
        scanStartTime = now;
      }
      break;

    // ===== ESCAPE =====
    case AS_ESCAPE:
      _derechaAuto(60);
      if (now - accionMillis >= (RETARDO_GIRO * 2)) {
        detenerMotores();
        rotarRobot(robotDireccion + 2);
        Serial.println("🔄 Escape 180° completado");
        lastAction   = "escape_complete";
        lastDecision = "REINICIAR";
        _adelanteAuto(velocidadActual);
        autEstado    = AS_NAVIGATE;
        estadoMillis = now;
        indiceMemoria    = 0;
        giroConsecutivos = 0;
        resetearGiros();
      }
      break;

  } // fin switch
}

// ========================================================
// INICIAR / DETENER
// ========================================================

void iniciarModoAutonomo() {
  detenerMotores();
  modoAutonomoActivo = true;
  moverServoCentro();
  autEstado    = AS_IDLE;
  lastAction   = "started";
  lastDecision = "INIT";

  memset(memoriaGiros, 0, sizeof(memoriaGiros));
  indiceMemoria    = 0;
  giroConsecutivos = 0;

  Serial.println("🤖 Modo autónomo activado (8 estados)");
  Serial.printf("📊 Velocidad máxima: %d%% (Preset %d)\n",
    velocidadMaximaAutonoma, presetActualAutonomo + 1);

  static unsigned long ultimoPrintMapa = 0;
  if (millis() - ultimoPrintMapa > 5000) {
    imprimirMapa();
    ultimoPrintMapa = millis();
  }
}

void detenerModoAutonomo() {
  modoAutonomoActivo = false;
  detenerMotores();
  moverServoCentro();
  autEstado    = AS_IDLE;
  lastAction   = "stopped";
  lastDecision = "STOP";
  Serial.println("🛑 Modo autónomo detenido");
}

bool estaAutonomoActivo() {
  return modoAutonomoActivo;
}

// ========================================================
// AVANZAR EN MAPA
// ========================================================

void avanzarEnMapa() {
  if (millis() - ultimoMovimientoMapa < INTERVALO_MAPA) return;
  ultimoMovimientoMapa = millis();

  int offsetX[] = {0, 1,  0, -1};
  int offsetY[] = {1, 0, -1,  0};

  actualizarPosicionRobot(
    robotX + offsetX[robotDireccion],
    robotY + offsetY[robotDireccion]
  );

  Serial.printf("📍 Robot mapa -> X:%d Y:%d DIR:%d\n",
    robotX, robotY, robotDireccion);
}