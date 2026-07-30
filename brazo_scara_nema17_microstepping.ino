/*
 * ============================================================================
 *  BRAZO ROBOT TIPO SCARA - Version NEMA 17 (17HS4401) + DRV8825 + AccelStepper
 *  >>> MICROSTEPPING CONFIGURABLE CON UNA SOLA VARIABLE <<<
 * ============================================================================
 *  Componentes:
 *    - Motor a pasos NEMA 17 17HS4401 (tornillo sin fin, sube/baja el brazo)
 *      controlado por un driver DRV8825:
 *        STEP = pin 2   (paso)
 *        DIR  = pin 3   (direccion)
 *        M0   = pin 10  (seleccion de microstepping)
 *        M1   = pin 11
 *        M2   = pin 12
 *    - Servo1 (pin 6) controlado por Pot en A0
 *    - Servo2 (pin 7) controlado por Pot en A1
 *    - PushUp   (pin 4) -> sube el motor a pasos mientras se mantiene presionado
 *    - PushDown (pin 5) -> baja el motor a pasos mientras se mantiene presionado
 *    - Push1 (pin 8) -> pulsacion corta: GRABA posicion / mantener 4s: RESETEA
 *    - Push2 (pin 9) -> inicia ejecucion / segunda pulsacion: DETIENE
 *
 *  NOTA DE CABLEADO: los 4 botones estan con resistencia PULL-DOWN,
 *  es decir, PRESIONADO = HIGH (igual que las versiones anteriores).
 *
 *  DIFERENCIA CON brazo_scara_nema17.ino:
 *    - El nivel de microstepping se elige cambiando UNA sola variable
 *      (MICROSTEPPING). El codigo pone solo los pines M0/M1/M2 en el nivel
 *      correcto segun la tabla del DRV8825, y ademas ajusta la velocidad para
 *      que el motor se mueva a la MISMA velocidad fisica sin importar el nivel.
 * ============================================================================
 */

#include <Servo.h>
#include <AccelStepper.h>

// ---------------------- PINES ----------------------
// Pines de control del driver DRV8825 para el NEMA 17
const int PIN_STEP = 2;    // STEP del DRV8825
const int PIN_DIR  = 3;    // DIR  del DRV8825

// Pines de seleccion de microstepping del DRV8825
const int PIN_M0 = 10;
const int PIN_M1 = 11;
const int PIN_M2 = 12;

const int PIN_PUSH_UP   = 4;   // Sube motor a pasos (antihorario)
const int PIN_PUSH_DOWN = 5;   // Baja motor a pasos (horario)

const int PIN_SERVO1 = 6;
const int PIN_SERVO2 = 7;

const int PIN_POT1 = A0;       // Controla Servo1
const int PIN_POT2 = A1;       // Controla Servo2

const int PIN_PUSH1 = 8;       // Grabar / Resetear (mantener 4s)
const int PIN_PUSH2 = 9;       // Ejecutar / Detener

// ---------------------- CONFIGURACION DEL MOTOR A PASOS ----------------------
// El NEMA 17 17HS4401 es un motor de 1.8 grados => 200 pasos por vuelta (paso completo).
const int PASOS_POR_VUELTA = 200;

// >>> UNICA VARIABLE PARA EL MICROSTEPPING <<<
// Cambia SOLO este numero. Valores validos del DRV8825: 1, 2, 4, 8, 16 o 32.
//   1  = paso completo   (mas par, menos suave)
//   2  = 1/2 paso
//   4  = 1/4 de paso
//   8  = 1/8 de paso
//   16 = 1/16 de paso
//   32 = 1/32 de paso    (mas suave, menos par)
// El codigo se encarga de poner M0/M1/M2 en el nivel correcto automaticamente.
const int MICROSTEPPING = 1;

// Con el microstepping activo, una vuelta necesita mas pasos. Este valor se usa
// para mantener la velocidad fisica constante sin importar el nivel elegido.
const long MICROPASOS_POR_VUELTA = (long)PASOS_POR_VUELTA * MICROSTEPPING;

// >>> CONTROL DE VELOCIDAD (en RPM) <<<
// La velocidad se expresa en RPM (revoluciones por minuto) del eje del motor,
// asi es independiente del microstepping: internamente se convierte a pasos/seg
// segun el nivel elegido, y el motor gira igual de rapido con 1 o con 32.
// Cambia SOLO estos tres numeros para ajustar la velocidad del brazo:
//
//   RPM_JOG   -> velocidad al subir/bajar con PushUp / PushDown.
//   RPM_MAX   -> tope de velocidad permitido (debe ser >= RPM_JOG).
//   RPM_ACEL  -> aceleracion, en RPM ganados por segundo.
//
// Orientacion para el NEMA 17 con el DRV8825: 60-300 RPM funciona bien; a mas
// RPM necesitas mas voltaje (VMOT) y buen ajuste de corriente (Vref) o pierde pasos.
const float RPM_JOG  = 180.0;
const float RPM_MAX  = 300.0;
const float RPM_ACEL = 600.0;

// Conversion de RPM a pasos/seg (lo que entiende AccelStepper), ya con el
// microstepping incluido. RPM / 60 = vueltas/seg; x micropasos = pasos/seg.
const float VELOCIDAD_JOG = RPM_JOG  / 60.0 * MICROPASOS_POR_VUELTA;
const float VELOCIDAD_MAX = RPM_MAX  / 60.0 * MICROPASOS_POR_VUELTA;
const float ACELERACION   = RPM_ACEL / 60.0 * MICROPASOS_POR_VUELTA;

// AccelStepper en modo DRIVER: solo controla los pines STEP y DIR del DRV8825.
AccelStepper motorPasos(AccelStepper::DRIVER, PIN_STEP, PIN_DIR);

// ---------------------- OBJETOS Y CONSTANTES ----------------------
Servo servo1;
Servo servo2;

const int MAX_POSICIONES = 20;
const long TIEMPO_RESET_MS = 4000;   // 4 segundos para resetear
const unsigned long ANTIREBOTE_MS = 50;

// ---------------------- VARIABLES DE ESTADO ----------------------
// Cada posicion guarda: [0]=angulo servo1, [1]=angulo servo2, [2]=pasos del motor
int posiciones[MAX_POSICIONES][3];
int numPosiciones = 0;         // cuantas posiciones hay guardadas

int anguloServo1 = 0;          // angulo actual de cada servo
int anguloServo2 = 0;
long posicionPasos = 0;        // posicion actual del motor a pasos (num. de pasos)

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
//  CONFIGURACION DEL MICROSTEPPING (una sola variable -> pines M0/M1/M2)
// ============================================================================
// Traduce el valor de MICROSTEPPING a los niveles de M0/M1/M2 segun la tabla
// del DRV8825:
//        M2 M1 M0
//   1  =  L  L  L
//   2  =  L  L  H
//   4  =  L  H  L
//   8  =  L  H  H
//   16 =  H  L  L
//   32 =  H  H  H   (la hoja de datos admite H H X; usamos H H H)
void configurarMicrostepping(int nivel) {
  bool m0, m1, m2;
  switch (nivel) {
    case 1:  m2 = LOW;  m1 = LOW;  m0 = LOW;  break;
    case 2:  m2 = LOW;  m1 = LOW;  m0 = HIGH; break;
    case 4:  m2 = LOW;  m1 = HIGH; m0 = LOW;  break;
    case 8:  m2 = LOW;  m1 = HIGH; m0 = HIGH; break;
    case 16: m2 = HIGH; m1 = LOW;  m0 = LOW;  break;
    case 32: m2 = HIGH; m1 = HIGH; m0 = HIGH; break;
    default:                                        // valor no valido -> paso completo
      m2 = LOW; m1 = LOW; m0 = LOW;
      Serial.println(F(">> MICROSTEPPING no valido. Usando paso completo (1)."));
      break;
  }
  digitalWrite(PIN_M0, m0);
  digitalWrite(PIN_M1, m1);
  digitalWrite(PIN_M2, m2);
}


// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(9600);

  // --- Configuracion del microstepping del DRV8825 ---
  pinMode(PIN_M0, OUTPUT);
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  configurarMicrostepping(MICROSTEPPING);   // pone M0/M1/M2 segun la variable

  // --- Configuracion del motor a pasos con AccelStepper ---
  // (AccelStepper configura solo los pines STEP y DIR como salida)
  motorPasos.setMaxSpeed(VELOCIDAD_MAX);
  motorPasos.setAcceleration(ACELERACION);
  motorPasos.setCurrentPosition(0);   // el origen es la posicion inicial

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
  Serial.println(F("   (NEMA 17 + DRV8825 + AccelStepper)"));
  Serial.println(F("========================================"));
  Serial.println(F("Posicion inicial de los motores:"));
  Serial.print(F("  Servo1: ")); Serial.print(anguloServo1); Serial.println(F(" grados"));
  Serial.print(F("  Servo2: ")); Serial.print(anguloServo2); Serial.println(F(" grados"));
  Serial.print(F("  Motor a pasos: ")); Serial.print(posicionPasos); Serial.println(F(" pasos"));
  Serial.print(F("  Microstepping: 1/")); Serial.print(MICROSTEPPING);
  Serial.print(F("  (")); Serial.print(MICROPASOS_POR_VUELTA); Serial.println(F(" pasos/vuelta)"));
  Serial.print(F("  Velocidad jog: ")); Serial.print(RPM_JOG); Serial.println(F(" RPM"));
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
//  MOTOR A PASOS: control con la libreria AccelStepper (driver DRV8825)
// ============================================================================
// Da UN paso en la direccion indicada: +1 = subir (antihorario), -1 = bajar (horario).
// Usamos setSpeed() + runSpeed() para dar un unico paso a velocidad constante
// (VELOCIDAD_JOG). runSpeed() genera el pulso STEP cuando toca segun esa velocidad,
// por eso repetimos hasta que la posicion interna del driver avance ese paso.
// Asi mantenemos exactamente la misma logica coordinada que la version anterior.
void darPaso(int direccion) {
  motorPasos.setSpeed(direccion > 0 ? VELOCIDAD_JOG : -VELOCIDAD_JOG);
  long objetivo = motorPasos.currentPosition() + direccion;
  while (motorPasos.currentPosition() != objetivo) {
    motorPasos.runSpeed();   // avanza un paso cuando corresponde por tiempo
  }
  posicionPasos += direccion;
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
