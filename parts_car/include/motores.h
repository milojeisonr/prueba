#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>

// ==== Pines de control ====
#define IN1 25
#define IN2 26
#define IN3 32
#define IN4 33
#define ENA 27
#define ENB 14

// ==== Configuración PWM ====
#define PWM_FREQ 5000
#define PWM_RES  8
#define PWM_CH_A 0
#define PWM_CH_B 1

// ==== Variables globales ====
extern int velocidadPWM;
extern int velocidadPorcentaje;

// ==== Inicialización ====
void inicializarMotores();

// ==== Movimiento ====
void moverAdelante(int vel = -1);
void moverAtras(int vel = -1);
void girarIzquierda(int vel = -1);
void girarDerecha(int vel = -1);
void detenerMotores();

// ==== Movimientos suaves ====
void girarSuaveIzquierda();
void girarSuaveDerecha();
void frenarSuave();

// ==== Velocidad manual ====
void setVelocidad(int porcentaje);
int  getVelocidad();
int  getVelocidadPWM();

// ==== Límite manual (presets V1/V2/V3 del joystick) ====
void setVelocidadMaximaManual(int porcentaje);
int  getVelocidadMaximaManual();

// ==== Preset combinado (solo usa caso manual=false) ====
void setPresetSpeed(bool autonoma, int level);

// NOTA: Las funciones autónomas están en modo_autonomo.h:
//   setPresetVelocidadAutonoma(int nivel)
//   setVelocidadAutonoma(int porcentaje)
//   getVelocidadAutonoma()
//   getVelocidadMaximaAutonoma()

#endif