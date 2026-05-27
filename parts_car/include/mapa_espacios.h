#ifndef MAPA_ESPACIOS_H
#define MAPA_ESPACIOS_H

#include <Arduino.h>

// ========== SISTEMA DE MAPEO PARA NAVEGACIÓN INTELIGENTE ==========
// Mapea espacios visitados, zonas peligrosas y rutas libres

// ================================================================
// CONFIGURACIÓN DEL MAPA
// ================================================================

// Grid de mapeo (20x20 = 400cm x 400cm)
#define MAPA_GRID_SIZE 20

// Tamaño de cada celda en cm
#define MAPA_CELDA_TAMANO 20

// ================================================================
// TIPOS DE CELDA
// ================================================================

#define CELDA_DESCONOCIDA   0
#define CELDA_LIBRE         1
#define CELDA_PELIGROSA     2
#define CELDA_BLOQUEADA     3
#define CELDA_VISITADA      4

// ================================================================
// ESTRUCTURA DE CELDA
// ================================================================

typedef struct {
  uint8_t tipo;              // Tipo de celda
  uint8_t confianza;         // 0-100
  uint16_t visitasCount;     // Número de visitas
  unsigned long ultimaVez;   // Última actualización
} CeldaMapa;

// ================================================================
// VARIABLES GLOBALES DEL MAPA
// ================================================================

// Matriz principal del mapa
extern CeldaMapa mapa[MAPA_GRID_SIZE][MAPA_GRID_SIZE];

// Posición actual del robot
extern int robotX;
extern int robotY;

// Dirección actual
// 0 = Norte
// 1 = Este
// 2 = Sur
// 3 = Oeste
extern int robotDireccion;

// ================================================================
// MEMORIA DE GIROS Y NAVEGACIÓN
// ================================================================

// Historial de giros realizados
#define MEMORIA_GIROS_SIZE 10

// Memoria circular de giros
extern int memoriaGiros[MEMORIA_GIROS_SIZE];

// Índice actual de memoria
extern int indiceMemoria;

// Contador de giros repetidos
extern int giroConsecutivos;

// ================================================================
// FUNCIONES PRINCIPALES DEL MAPA
// ================================================================

// Inicializar mapa completo
void inicializarMapa();

// Actualizar mapa usando sensores
void actualizarMapaDesdeDistancias(
  long distFront,
  long distLeft,
  long distRight
);

// Marcar una celda
void marcarCeldaVisitada(
  int x,
  int y,
  uint8_t tipo,
  uint8_t confianza
);

// Actualizar posición robot
void actualizarPosicionRobot(
  int nuevoX,
  int nuevoY
);

// Rotar dirección robot
void rotarRobot(int nuevoDir);

// ================================================================
// MEMORIA Y ANTI-LOOPS
// ================================================================

// Guardar giro realizado
// 0 = izquierda
// 1 = derecha
// 2 = reversa/bloqueo
void guardarGiro(int direccion);

// Detectar patrones repetitivos
bool detectarLoopGiros();

// Resetear historial de giros
void resetearMemoriaGiros();

// ================================================================
// ANÁLISIS DEL MAPA
// ================================================================

// Contar celdas libres
int contarCeldasLibres();

// Contar celdas peligrosas
int contarCeldasPeligrosas();

// Verificar rutas alternativas
bool existeRutaAlternativa(
  int distFront,
  int distLeft,
  int distRight
);

// Calcular área libre estimada
int calcularAreaLibre(int distancia);

// Obtener confianza de celda
uint8_t getConfianzaCelda(
  int x,
  int y
);

// ================================================================
// DEBUG Y VISUALIZACIÓN
// ================================================================

// Imprimir mapa por Serial
void imprimirMapa();

// ================================================================
// LIMPIEZA Y MANTENIMIENTO
// ================================================================

// Limpiar mapa completo
void limpiarMapa();

// Reducir confianza de celdas antiguas
void resetearConfianza();

#endif