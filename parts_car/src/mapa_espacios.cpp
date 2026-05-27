#include "mapa_espacios.h"
#include <string.h>

// ================================================================
// VARIABLES GLOBALES DEL MAPA
// ================================================================

CeldaMapa mapa[MAPA_GRID_SIZE][MAPA_GRID_SIZE];

int robotX = 10;
int robotY = 10;
int robotDireccion = 0; // 0=norte

// ================================================================
// MEMORIA DE GIROS
// ================================================================

int memoriaGiros[MEMORIA_GIROS_SIZE] = {0};
int indiceMemoria = 0;
int giroConsecutivos = 0;

// ================================================================
// CONTROL DE CONFIANZA
// ================================================================

static unsigned long ultimaLimpiezaConfianza = 0;

const unsigned long TIEMPO_DECAY_CONFIANZA = 30000;

// ================================================================
// INICIALIZACIÓN
// ================================================================

void inicializarMapa() {

  for (int x = 0; x < MAPA_GRID_SIZE; x++) {

    for (int y = 0; y < MAPA_GRID_SIZE; y++) {

      mapa[x][y].tipo = CELDA_DESCONOCIDA;
      mapa[x][y].confianza = 0;
      mapa[x][y].visitasCount = 0;
      mapa[x][y].ultimaVez = 0;
    }
  }

  robotX = MAPA_GRID_SIZE / 2;
  robotY = MAPA_GRID_SIZE / 2;
  robotDireccion = 0;

  marcarCeldaVisitada(
    robotX,
    robotY,
    CELDA_VISITADA,
    100
  );

  resetearMemoriaGiros();

  Serial.println("[MAPA] Inicializado");
}

// ================================================================
// ACTUALIZAR MAPA DESDE DISTANCIAS
// ================================================================

void actualizarMapaDesdeDistancias(
  long distFront,
  long distLeft,
  long distRight
) {

  unsigned long now = millis();

  // ============================================================
  // DECAY DE CONFIANZA
  // ============================================================

  if (now - ultimaLimpiezaConfianza > TIEMPO_DECAY_CONFIANZA) {

    resetearConfianza();

    ultimaLimpiezaConfianza = now;
  }

  // ============================================================
  // DIRECCIONES RELATIVAS
  // ============================================================

  int dirAdelante = robotDireccion;
  int dirDerecha = (robotDireccion + 1) % 4;
  int dirIzquierda = (robotDireccion + 3) % 4;

  // ============================================================
  // OFFSETS
  // ============================================================

  int offsetX[] = {0, 1, 0, -1};
  int offsetY[] = {1, 0, -1, 0};

  // ============================================================
  // MAPEO FRONTAL
  // ============================================================

  if (distFront > 0 && distFront < 200) {

    int celdas = distFront / MAPA_CELDA_TAMANO;

    int celdaX = robotX +
                 (offsetX[dirAdelante] * celdas);

    int celdaY = robotY +
                 (offsetY[dirAdelante] * celdas);

    if (celdaX >= 0 &&
        celdaX < MAPA_GRID_SIZE &&
        celdaY >= 0 &&
        celdaY < MAPA_GRID_SIZE) {

      if (distFront < 15) {

        marcarCeldaVisitada(
          celdaX,
          celdaY,
          CELDA_BLOQUEADA,
          95
        );

      } else if (distFront < 30) {

        marcarCeldaVisitada(
          celdaX,
          celdaY,
          CELDA_PELIGROSA,
          80
        );

      } else {

        marcarCeldaVisitada(
          celdaX,
          celdaY,
          CELDA_LIBRE,
          70
        );
      }
    }
  }

  // ============================================================
  // MAPEO IZQUIERDO
  // ============================================================

  if (distLeft > 0 && distLeft < 200) {

    int celdas = distLeft / MAPA_CELDA_TAMANO;

    int celdaX = robotX +
                 (offsetX[dirIzquierda] * celdas);

    int celdaY = robotY +
                 (offsetY[dirIzquierda] * celdas);

    if (celdaX >= 0 &&
        celdaX < MAPA_GRID_SIZE &&
        celdaY >= 0 &&
        celdaY < MAPA_GRID_SIZE) {

      uint8_t tipo =
        (distLeft < 20)
        ? CELDA_BLOQUEADA
        : CELDA_LIBRE;

      marcarCeldaVisitada(
        celdaX,
        celdaY,
        tipo,
        60
      );
    }
  }

  // ============================================================
  // MAPEO DERECHO
  // ============================================================

  if (distRight > 0 && distRight < 200) {

    int celdas = distRight / MAPA_CELDA_TAMANO;

    int celdaX = robotX +
                 (offsetX[dirDerecha] * celdas);

    int celdaY = robotY +
                 (offsetY[dirDerecha] * celdas);

    if (celdaX >= 0 &&
        celdaX < MAPA_GRID_SIZE &&
        celdaY >= 0 &&
        celdaY < MAPA_GRID_SIZE) {

      uint8_t tipo =
        (distRight < 20)
        ? CELDA_BLOQUEADA
        : CELDA_LIBRE;

      marcarCeldaVisitada(
        celdaX,
        celdaY,
        tipo,
        60
      );
    }
  }
}

// ================================================================
// MARCAR CELDA
// ================================================================

void marcarCeldaVisitada(
  int x,
  int y,
  uint8_t tipo,
  uint8_t confianza
) {

  if (x < 0 ||
      x >= MAPA_GRID_SIZE ||
      y < 0 ||
      y >= MAPA_GRID_SIZE) {
    return;
  }

  if (mapa[x][y].confianza > 0) {

    mapa[x][y].confianza =
      (mapa[x][y].confianza + confianza) / 2;

  } else {

    mapa[x][y].confianza = confianza;
  }

  mapa[x][y].tipo = tipo;

  mapa[x][y].visitasCount++;

  mapa[x][y].ultimaVez = millis();
}

// ================================================================
// ACTUALIZAR POSICIÓN
// ================================================================

void actualizarPosicionRobot(
  int nuevoX,
  int nuevoY
) {

  if (nuevoX < 0) nuevoX = 0;
  if (nuevoY < 0) nuevoY = 0;

  if (nuevoX >= MAPA_GRID_SIZE) {
    nuevoX = MAPA_GRID_SIZE - 1;
  }

  if (nuevoY >= MAPA_GRID_SIZE) {
    nuevoY = MAPA_GRID_SIZE - 1;
  }

  robotX = nuevoX;
  robotY = nuevoY;

  marcarCeldaVisitada(
    robotX,
    robotY,
    CELDA_VISITADA,
    100
  );
}

// ================================================================
// ROTAR ROBOT
// ================================================================

void rotarRobot(int nuevoDir) {

  if (nuevoDir < 0) nuevoDir = 3;
  if (nuevoDir > 3) nuevoDir = 0;

  robotDireccion = nuevoDir;
}

// ================================================================
// MEMORIA DE GIROS
// ================================================================

void guardarGiro(int direccion) {

  memoriaGiros[indiceMemoria] = direccion;

  indiceMemoria++;

  if (indiceMemoria >= MEMORIA_GIROS_SIZE) {
    indiceMemoria = 0;
  }

  int ultimo =
    memoriaGiros[
      (indiceMemoria - 1 + MEMORIA_GIROS_SIZE)
      % MEMORIA_GIROS_SIZE
    ];

  int anterior =
    memoriaGiros[
      (indiceMemoria - 2 + MEMORIA_GIROS_SIZE)
      % MEMORIA_GIROS_SIZE
    ];

  if (ultimo == anterior) {

    giroConsecutivos++;

  } else {

    giroConsecutivos = 0;
  }
}

// ================================================================
// DETECTAR LOOP
// ================================================================

bool detectarLoopGiros() {

  return (giroConsecutivos >= 3);
}

// ================================================================
// RESETEAR MEMORIA
// ================================================================

void resetearMemoriaGiros() {

  memset(
    memoriaGiros,
    0,
    sizeof(memoriaGiros)
  );

  indiceMemoria = 0;

  giroConsecutivos = 0;
}

// ================================================================
// CONTAR CELDAS LIBRES
// ================================================================

int contarCeldasLibres() {

  int cuenta = 0;

  for (int x = 0; x < MAPA_GRID_SIZE; x++) {

    for (int y = 0; y < MAPA_GRID_SIZE; y++) {

      if (mapa[x][y].tipo == CELDA_LIBRE &&
          mapa[x][y].confianza > 50) {

        cuenta++;
      }
    }
  }

  return cuenta;
}

// ================================================================
// CONTAR CELDAS PELIGROSAS
// ================================================================

int contarCeldasPeligrosas() {

  int cuenta = 0;

  for (int x = 0; x < MAPA_GRID_SIZE; x++) {

    for (int y = 0; y < MAPA_GRID_SIZE; y++) {

      if (
        (mapa[x][y].tipo == CELDA_PELIGROSA ||
         mapa[x][y].tipo == CELDA_BLOQUEADA)
        &&
        mapa[x][y].confianza > 50
      ) {

        cuenta++;
      }
    }
  }

  return cuenta;
}

// ================================================================
// EXISTE RUTA ALTERNATIVA
// ================================================================

bool existeRutaAlternativa(
  int distFront,
  int distLeft,
  int distRight
) {

  int umbral = max(20, distFront / 2);

  return (
    distLeft > umbral ||
    distRight > umbral
  );
}

// ================================================================
// CALCULAR ÁREA LIBRE
// ================================================================

int calcularAreaLibre(int distancia) {

  int totalCeldas =
    MAPA_GRID_SIZE * MAPA_GRID_SIZE;

  int celdas =
    distancia / MAPA_CELDA_TAMANO;

  int circuloArea =
    celdas * celdas * 3;

  if (circuloArea > totalCeldas) {
    return 100;
  }

  return
    (circuloArea * 100)
    / totalCeldas;
}

// ================================================================
// OBTENER CONFIANZA
// ================================================================

uint8_t getConfianzaCelda(
  int x,
  int y
) {

  if (x < 0 ||
      x >= MAPA_GRID_SIZE ||
      y < 0 ||
      y >= MAPA_GRID_SIZE) {

    return 0;
  }

  return mapa[x][y].confianza;
}

// ================================================================
// IMPRIMIR MAPA
// ================================================================

void imprimirMapa() {

  Serial.println("\n===== MAPA =====");

  for (int y = MAPA_GRID_SIZE - 1; y >= 0; y--) {

    for (int x = 0; x < MAPA_GRID_SIZE; x++) {

      if (x == robotX &&
          y == robotY) {

        Serial.print("R");

      } else {

        char c;

        switch (mapa[x][y].tipo) {

          case CELDA_LIBRE:
            c = '.';
            break;

          case CELDA_PELIGROSA:
            c = '!';
            break;

          case CELDA_BLOQUEADA:
            c = '#';
            break;

          case CELDA_VISITADA:
            c = '*';
            break;

          default:
            c = ' ';
            break;
        }

        Serial.print(c);
      }
    }

    Serial.println();
  }

  Serial.printf(
    "[MAPA] Libres=%d | Peligrosas=%d\n",
    contarCeldasLibres(),
    contarCeldasPeligrosas()
  );
}

// ================================================================
// LIMPIAR MAPA
// ================================================================

void limpiarMapa() {

  for (int x = 0; x < MAPA_GRID_SIZE; x++) {

    for (int y = 0; y < MAPA_GRID_SIZE; y++) {

      mapa[x][y].tipo = CELDA_DESCONOCIDA;
      mapa[x][y].confianza = 0;
      mapa[x][y].visitasCount = 0;
      mapa[x][y].ultimaVez = 0;
    }
  }

  robotX = MAPA_GRID_SIZE / 2;
  robotY = MAPA_GRID_SIZE / 2;
  robotDireccion = 0;

  resetearMemoriaGiros();

  Serial.println("[MAPA] Limpiado");
}

// ================================================================
// RESETEAR CONFIANZA
// ================================================================

void resetearConfianza() {

  unsigned long now = millis();

  for (int x = 0; x < MAPA_GRID_SIZE; x++) {

    for (int y = 0; y < MAPA_GRID_SIZE; y++) {

      if (mapa[x][y].confianza > 0) {

        unsigned long tiempoTranscurrido =
          now - mapa[x][y].ultimaVez;

        int reduccion =
          (tiempoTranscurrido / 10000);

        int nuevaConfianza =
          (int)mapa[x][y].confianza
          - reduccion;

        mapa[x][y].confianza =
          max(0, nuevaConfianza);
      }
    }
  }
}