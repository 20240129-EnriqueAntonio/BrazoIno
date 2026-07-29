/*
 * ============================================================================
 *  BRAZO ROBOT TIPO SCARA - Guardado y ejecucion de posiciones
 * ============================================================================
 *  Componentes:
 *    - Motor a pasos 28BYJ-48 (tornillo sin fin, sube/baja el brazo)
 *        IN1=10, IN2=11, IN3=12, IN4=13
 *    - Servo1 (pin 6) controlado por Pot en A0
 *    - Servo2 (pin 7) controlado por Pot en A1
 *    - PushUp   (pin 4) -> sube el motor a pasos mientras se mantiene presionado
 *    - PushDown (pin 5) -> baja el motor a pasos mientras se mantiene presionado
 *    - Push1 (pin 8) -> pulsacion corta: GRABA posicion / mantener 4s: RESETEA
 *    - Push2 (pin 9) -> inicia ejecucion / segunda pulsacion: DETIENE
 *
 *  NOTA DE CABLEADO: los 4 botones estan con resistencia PULL-DOWN,
 *  es decir, PRESIONADO = HIGH (igual que el proyecto de referencia).
 * ============================================================================
 */

#include <Servo.h>

// ---------------------- PINES ----------------------
const int IN1 = 10;   // Bobinas del motor a pasos 28BYJ-48
const int IN2 = 11;
const int IN3 = 12;
const int IN4 = 13;

const int PIN_PUSH_UP   = 4;   // Sube motor a pasos (antihorario)
const int PIN_PUSH_DOWN = 5;   // Baja motor a pasos (horario)

const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 7;

const int PIN_POT1 = A0;       // Controla Servo1
const int PIN_POT2 = A1;       // Controla Servo2

const int PIN_PUSH1 = 8;       // Grabar / Resetear (mantener 4s)
const int PIN_PUSH2 = 9;       // Ejecutar / Detener

// ---------------------- OBJETOS Y CONSTANTES ----------------------
Servo servo1;
Servo servo2;

const int MAX_POSICIONES = 20;
const long TIEMPO_RESET_MS = 4000;   // 4 segundos para resetear
const unsigned long ANTIREBOTE_MS = 50;

// Secuencia de medio paso (half-step) para el 28BYJ-48.
// Da un movimiento mas suave y con mas par que el paso completo.
const int SECUENCIA[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// ---------------------- VARIABLES DE ESTADO ----------------------
// Cada posicion guarda: [0]=angulo servo1, [1]=angulo servo2, [2]=pasos del motor
int posiciones[MAX_POSICIONES][3];
int numPosiciones = 0;         // cuantas posiciones hay guardadas

int anguloServo1 = 0;          // angulo actual de cada servo
int anguloServo2 = 0;
long posicionPasos = 0;        // posicion actual del motor a pasos (num. de pasos)
int indiceSecuencia = 0;       // en que paso de la secuencia estamos

bool ejecutando = false;       // true mientras se reproduce el recorrido

// Para el antirebote y la deteccion de "mantener 4s" en Push1
bool push1Presionado = false;
unsigned long push1TiempoInicio = 0;
bool reseteoHecho = false;     // evita resetear varias veces en una sola pulsacion larga

// Para detectar el flanco de Push2 (inicio/paro)
bool push2Anterior = false;

// Para mostrar el estado por Serial sin saturar
unsigned long ultimoReporte = 0;
const unsigned long INTERVALO_REPORTE = 500;


// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(PIN_PUSH_UP, INPUT);
  pinMode(PIN_PUSH_DOWN, INPUT);
  pinMode(PIN_PUSH1, INPUT);
  pinMode(PIN_PUSH2, INPUT);

  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);

  // Leemos la posicion inicial de los servos desde los potenciometros
  anguloServo1 = leerAnguloPot(PIN_POT1);
  anguloServo2 = leerAnguloPot(PIN_POT2);
  servo1.write(anguloServo1);
  servo2.write(anguloServo2);

  Serial.println(F("========================================"));
  Serial.println(F("   BRAZO SCARA - Sistema iniciado"));
  Serial.println(F("========================================"));
  Serial.println(F("Posicion inicial de los motores:"));
  Serial.print(F("  Servo1: ")); Serial.print(anguloServo1); Serial.println(F(" grados"));
  Serial.print(F("  Servo2: ")); Serial.print(anguloServo2); Serial.println(F(" grados"));
  Serial.print(F("  Motor a pasos: ")); Serial.print(posicionPasos); Serial.println(F(" pasos"));
  Serial.println(F("----------------------------------------"));
  Serial.println(F("Push1 corto = grabar | Push1 4s = resetear"));
  Serial.println(F("Push2 = ejecutar / detener"));
  Serial.println(F("----------------------------------------"));
}


// ============================================================================
//  LOOP PRINCIPAL
// ============================================================================
void loop() {
  actualizarServos();     // los potenciometros mueven los servos en vivo
  controlarMotorPasos();  // PushUp / PushDown mueven el motor a pasos
  gestionarPush1();       // grabar / resetear
  gestionarPush2();       // iniciar ejecucion del recorrido
  mostrarEstado();        // reporte periodico por Serial
}


// ============================================================================
//  SERVOS: los potenciometros controlan los servos en tiempo real
// ============================================================================
void actualizarServos() {
  anguloServo1 = leerAnguloPot(PIN_POT1);
  anguloServo2 = leerAnguloPot(PIN_POT2);
  servo1.write(anguloServo1);
  servo2.write(anguloServo2);
}

// Lee un potenciometro y lo convierte a angulo evitando la vibracion:
// se recorta el rango del potenciometro (10-1013) y del servo (1-179),
// dejando fuera los extremos ruidosos. El constrain asegura que nunca
// se pase de 1-179 aunque la lectura salga de ese rango.
int leerAnguloPot(int pin) {
  int lectura = analogRead(pin);
  int angulo = map(lectura, 10, 1013, 1, 179);
  return constrain(angulo, 1, 179);
}


// ============================================================================
//  MOTOR A PASOS: control manual con PushUp / PushDown
// ============================================================================
// Aplica el patron de bobinas correspondiente a un paso de la secuencia
void aplicarPaso(int idx) {
  digitalWrite(IN1, SECUENCIA[idx][0]);
  digitalWrite(IN2, SECUENCIA[idx][1]);
  digitalWrite(IN3, SECUENCIA[idx][2]);
  digitalWrite(IN4, SECUENCIA[idx][3]);
}

// Da un paso en la direccion indicada: +1 = subir (antihorario), -1 = bajar (horario)
void darPaso(int direccion) {
  indiceSecuencia = (indiceSecuencia + direccion + 8) % 8;
  aplicarPaso(indiceSecuencia);
  posicionPasos += direccion;
  delay(2);   // el 28BYJ-48 necesita ~2ms entre pasos
}

void controlarMotorPasos() {
  if (digitalRead(PIN_PUSH_UP) == HIGH) {
    darPaso(+1);   // subir -> antihorario
  }
  else if (digitalRead(PIN_PUSH_DOWN) == HIGH) {
    darPaso(-1);   // bajar -> horario
  }
}


// ============================================================================
//  PUSH1: grabar posicion (corto) / resetear (mantener 4 segundos)
// ============================================================================
void gestionarPush1() {
  bool estado = (digitalRead(PIN_PUSH1) == HIGH);

  // --- Flanco de subida: se acaba de presionar ---
  if (estado && !push1Presionado) {
    push1Presionado = true;
    push1TiempoInicio = millis();
    reseteoHecho = false;
  }

  // --- Se mantiene presionado: comprobar si llego a los 4 segundos ---
  if (estado && push1Presionado && !reseteoHecho) {
    if (millis() - push1TiempoInicio >= TIEMPO_RESET_MS) {
      resetearPosiciones();
      reseteoHecho = true;   // ya se reseteo, no repetir hasta soltar
    }
  }

  // --- Flanco de bajada: se acaba de soltar ---
  if (!estado && push1Presionado) {
    unsigned long duracion = millis() - push1TiempoInicio;
    // Solo grabamos si fue una pulsacion corta (antirebote) y no fue un reset
    if (!reseteoHecho && duracion >= ANTIREBOTE_MS) {
      grabarPosicion();
    }
    push1Presionado = false;
  }
}

void grabarPosicion() {
  if (numPosiciones >= MAX_POSICIONES) {
    Serial.println(F(">> Maximo de 20 posiciones alcanzado. Resetea para grabar mas."));
    return;
  }

  posiciones[numPosiciones][0] = anguloServo1;
  posiciones[numPosiciones][1] = anguloServo2;
  posiciones[numPosiciones][2] = (int)posicionPasos;
  numPosiciones++;

  Serial.print(F(">> Posicion GRABADA #")); Serial.print(numPosiciones);
  Serial.print(F("  |  Servo1: ")); Serial.print(anguloServo1);
  Serial.print(F("  Servo2: ")); Serial.print(anguloServo2);
  Serial.print(F("  Pasos: ")); Serial.println(posicionPasos);
}

void resetearPosiciones() {
  for (int i = 0; i < MAX_POSICIONES; i++) {
    posiciones[i][0] = 0;
    posiciones[i][1] = 0;
    posiciones[i][2] = 0;
  }
  numPosiciones = 0;
  Serial.println(F(">> POSICIONES RESETEADAS. Memoria vacia."));
}


// ============================================================================
//  PUSH2: iniciar el recorrido (con opcion una vez / en bucle)
// ============================================================================
void gestionarPush2() {
  bool estado = (digitalRead(PIN_PUSH2) == HIGH);

  // Detectamos solo el flanco de subida (recien presionado)
  if (estado && !push2Anterior) {
    if (numPosiciones == 0) {
      Serial.println(F(">> No hay posiciones guardadas para ejecutar."));
    } else {
      iniciarRecorrido();
    }
  }
  push2Anterior = estado;
}

void iniciarRecorrido() {
  // Preguntamos por el Monitor Serie: una vez o en bucle
  Serial.println(F("----------------------------------------"));
  Serial.println(F("Elige el modo de recorrido en el Monitor Serie:"));
  Serial.println(F("  Escribe 1 -> ejecutar UNA VEZ"));
  Serial.println(F("  Escribe 2 -> ejecutar EN BUCLE"));
  Serial.println(F("(pulsa Push2 de nuevo en cualquier momento para detener)"));

  // Vaciamos cualquier dato viejo del buffer serie
  while (Serial.available() > 0) Serial.read();

  int modo = 0;   // 1 = una vez, 2 = bucle
  while (modo == 0) {
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '1') modo = 1;
      else if (c == '2') modo = 2;
    }
  }

  ejecutando = true;
  if (modo == 1) Serial.println(F(">> Ejecutando recorrido UNA VEZ..."));
  else           Serial.println(F(">> Ejecutando recorrido EN BUCLE..."));

  do {
    reproducirSecuencia();
  } while (ejecutando && modo == 2);

  // Si termino solo (no fue detenido), avisamos
  if (ejecutando) {
    Serial.println(F(">> Recorrido FINALIZADO."));
  }
  ejecutando = false;
}

// Recorre las posiciones guardadas moviendo los 3 motores suavemente
void reproducirSecuencia() {
  for (int p = 0; p < numPosiciones; p++) {
    Serial.print(F("   Ejecutando posicion #")); Serial.println(p + 1);

    int objetivoServo1 = posiciones[p][0];
    int objetivoServo2 = posiciones[p][1];
    long objetivoPasos = posiciones[p][2];

    // Movemos los tres motores hacia su objetivo de forma gradual y coordinada
    bool enMovimiento = true;
    while (enMovimiento) {
      enMovimiento = false;

      // --- Servo1 ---
      if (anguloServo1 < objetivoServo1) { anguloServo1++; enMovimiento = true; }
      else if (anguloServo1 > objetivoServo1) { anguloServo1--; enMovimiento = true; }
      servo1.write(anguloServo1);

      // --- Servo2 ---
      if (anguloServo2 < objetivoServo2) { anguloServo2++; enMovimiento = true; }
      else if (anguloServo2 > objetivoServo2) { anguloServo2--; enMovimiento = true; }
      servo2.write(anguloServo2);

      // --- Motor a pasos ---
      if (posicionPasos < objetivoPasos) { darPaso(+1); enMovimiento = true; }
      else if (posicionPasos > objetivoPasos) { darPaso(-1); enMovimiento = true; }

      delay(10);   // velocidad del movimiento de los servos

      // Permite DETENER la ejecucion pulsando Push2 de nuevo
      if (digitalRead(PIN_PUSH2) == HIGH) {
        Serial.println(F(">> Ejecucion DETENIDA por el usuario."));
        ejecutando = false;
        // esperamos a que se suelte el boton para no reactivarlo
        while (digitalRead(PIN_PUSH2) == HIGH) { delay(10); }
        push2Anterior = false;
        return;
      }
    }
    delay(400);   // pequena pausa entre posiciones
  }
}


// ============================================================================
//  REPORTE POR SERIAL: muestra la posicion actual de cada motor
// ============================================================================
void mostrarEstado() {
  if (millis() - ultimoReporte >= INTERVALO_REPORTE) {
    ultimoReporte = millis();
    Serial.print(F("Servo1: ")); Serial.print(anguloServo1);
    Serial.print(F(" | Servo2: ")); Serial.print(anguloServo2);
    Serial.print(F(" | Pasos: ")); Serial.print(posicionPasos);
    Serial.print(F(" | Guardadas: ")); Serial.println(numPosiciones);
  }
}
