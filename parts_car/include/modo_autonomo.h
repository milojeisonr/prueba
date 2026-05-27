#ifndef MODO_AUTONOMO_H
#define MODO_AUTONOMO_H

#include <Arduino.h>

// --- Modos de operación ---
#define MODE_MANUAL 0
#define MODE_AUTO   1

// --- Variable global ---
extern bool modoAutonomoActivo;

// --- Funciones principales ---
void   ejecutarModoAutonomo(int modoActual);
void   iniciarModoAutonomo();
void   detenerModoAutonomo();
bool   estaAutonomoActivo();

// --- Velocidad autónoma ---
void   setPresetVelocidadAutonoma(int nivel);   // límite máximo 1=30% 2=50% 3=80%
void   setVelocidadAutonoma(int porcentaje);    // velocidad actual (respeta límite)
int    getVelocidadAutonoma();                  // velocidad actual %
int    getVelocidadMaximaAutonoma();            // límite máximo del preset

// --- Debug ---
String getAutonomoDebug();

#endif