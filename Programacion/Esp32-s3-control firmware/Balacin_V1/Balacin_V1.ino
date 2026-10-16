/*
  ############################################################################
  BALACIN V1  -  Robot balancin de dos ruedas (Control Inteligente 2)
  Placa: ESP32-S3 (Arduino core 3.x)  |  MPU6050 + TB6612FNG + 2 encoders
  ############################################################################

  COMO ESTA ORGANIZADO ESTE ARCHIVO (en el orden en que aparece):

    0. UTILIDAD COMPARTIDA ........ Out: buffer de texto seguro entre nucleos
    PARTE 1  CONTROL EMBEBIDO ..... Todo lo ESENCIAL para balancear. Corre en el
                                    NUCLEO 1 y no depende del WiFi ni del HMI.
        1.1  Configuracion y pines (motores, encoders, MPU, limites)
        1.2  Parametros del control (PID de angulo, PID de rpm, filtros)
        1.3  Variables globales del control
        1.4  Tarea de control (reloj de 10 ms)
        1.5  Lazo de control en cascada (PID de rpm -> PID de angulo)
        1.6  Salida a los motores (u -> PWM)
        1.7  Encoders (cuentas y velocidad en rpm)
        1.8  Sensor MPU6050 y filtro complementario (angulo)
    PARTE 2  HMI, COMUNICACION Y SECUNDARIO ... Corre en el NUCLEO 0. Si falla
                                    el WiFi, el control sigue funcionando.
        2.1  Configuracion del WiFi, pulsador y LED RGB
        2.2  Variables de comunicacion
        2.3  Tarea de comunicaciones
        2.4  Salida de texto hacia Serial y WebSocket
        2.5  WiFi y WebSocket (conexion con el HMI)
        2.6  Comandos (Serial / HMI)
        2.7  Pulsador de arranque y LED de estado
        2.8  Telemetria (lo que se envia al HMI)
    ARRANQUE ........................ setup() y loop(): une las dos partes

  FLUJO RESUMIDO (cada 10 ms, nucleo 1):
    leer MPU -> angulo filtrado -> leer encoders -> rpm de cada rueda
      -> PID de rpm (por motor)  -> theta_sp del motor
      -> PID de angulo (por motor) -> u del motor -> PWM -> TB6612 -> motor

  COMANDOS (Monitor Serial 115200 con "Nueva linea", o desde el HMI):
    m = control ON/OFF      s = parar        c = recalibrar MPU     z = fijar cero
    PULSADOR (GPIO 40) + LED RGB: 1a pulsacion = c, 2a = z, 3a = m (control ON), otra = parar.
                            LED: azul -> (c) -> amarillo -> (z) -> verde -> (m) -> morado
    kp<n> ki<n> kd<n>       PID del angulo (iguales para los dos motores)
    tol<n>                  tolerancia en grados alrededor del setpoint
    vp<n> vi<n> vd<n>       PID de rpm de los DOS motores   (v1p.. motor 1, v2p.. motor 2)
    tf<n>                   constante de tiempo del filtro del angulo (segundos)
    pa<n> pb<n>             PWM minimo del motor A (izq) / B (der)
    te                      prueba de encoders (ruedas en el aire)    e = encoders a cero
    i2c<kHz>                velocidad del bus I2C (50 a 400)

  PRUEBA SEGURA: primero con las RUEDAS EN EL AIRE. Si el balancin pasa de
  CORTE_GRADOS (45) el control se apaga solo.
*/

// ============================================================================
// LIBRERIAS
// ============================================================================
#include <Wire.h>              // I2C para el MPU6050
#include <WiFi.h>              // WiFi del ESP32 (viene con el core)
#include <ESPmDNS.h>           // nombre balancin.local (viene con el core)
#include <WebSocketsServer.h>  // Libreria "WebSockets" de Markus Sattler (Gestor de bibliotecas)
#include "secretos.h"          // WIFI_SSID y WIFI_PASS. Si no existe, copia secretos_ejemplo.h como secretos.h

// ##############################################################################
// 0. UTILIDAD COMPARTIDA: OUT (TEXTO SEGURO ENTRE NUCLEOS)
// Out se usa como Serial: Out.print(...), Out.println(...). Las dos partes lo
// usan para mensajes. Guarda el texto en un buffer circular; SOLO la tarea de
// comunicaciones (Parte 2) lo manda a Serial y WebSocket. Asi el control nunca
// se detiene esperando a un puerto lento.
// ##############################################################################

// Out: igual que Serial, pero cada linea completa tambien se manda por WebSocket.
// Es seguro usarlo desde cualquier tarea: escribe en un buffer circular y SOLO la tarea de
// comunicaciones lo vacia hacia Serial y WebSocket (vaciarSalida). Asi un envio lento por
// WiFi o por USB nunca frena a la tarea de control.
const int SALIDA_N = 4096;
char bufSalida[SALIDA_N];
volatile int salidaLeer = 0, salidaEscribir = 0;
portMUX_TYPE muxSalida = portMUX_INITIALIZER_UNLOCKED;

class SalidaDual : public Print {
 public:
  size_t write(uint8_t c) override { return write(&c, 1); }
  size_t write(const uint8_t* b, size_t n) override {
    portENTER_CRITICAL(&muxSalida);
    for (size_t i = 0; i < n; i++) {
      int sig = (salidaEscribir + 1) % SALIDA_N;
      if (sig == salidaLeer) break;              // buffer lleno: se descarta lo que sobra
      bufSalida[salidaEscribir] = (char)b[i];
      salidaEscribir = sig;
    }
    portEXIT_CRITICAL(&muxSalida);
    return n;
  }
};
SalidaDual Out;

// ---------- Puente entre los dos nucleos (comandos que necesitan I2C o motores) ----------
// La tarea de comunicaciones recibe los comandos c, z, te e i2c, pero los
// ejecuta la tarea de control (porque ella es la unica que usa el bus I2C y
// los motores).
// Comandos pesados: los recibe la tarea de comunicaciones pero los ejecuta la tarea de control
TaskHandle_t hTareaControl = NULL;
String comandoPesado;
volatile bool hayComandoPesado = false;   // true = hay un comando pesado esperando a que lo tome la tarea de control
volatile bool ocupadoPesado = false;      // true = la tarea de control lo esta EJECUTANDO ahora (calibrando, fijando cero...)
// Etapa del arranque: 0 = encendido, 1 = giroscopio calibrado (c), 2 = cero fijado (z).
// La actualizan los comandos c y z cuando terminan bien, vengan del pulsador, del Serial o del HMI.
// El LED RGB y el pulsador (seccion 2.7) leen esta variable para saber en que paso van.
volatile int etapaRobot = 0;

// ============================================================================
// PROTOTIPOS (lista de todas las funciones, agrupadas por parte)
// ============================================================================
// ---- Parte 1: control embebido ----
void tareaControl(void* arg);
void lazoControl();
void reiniciarIntegrales();
void aplicarSalidaMotor(int c, float u);
void motorA(bool dirForward, int pwm);
void motorB(bool dirForward, int pwm);
void aplicarMovimiento();
void detenerMotores();
void IRAM_ATTR ISR_encIzq();
void IRAM_ATTR ISR_encDer();
void iniciarEncoders();
void leerCuentas(int32_t &izq, int32_t &der);
void actualizarEncoders(float dt);
void pruebaEncoders();
void atenderMPU();
bool mpuEscribir(uint8_t registro, uint8_t valor);
bool mpuLeerCrudo();
void escanearI2C();
bool calibrarGiroscopio();
float anguloDesdeAcelerometro();
bool iniciarMPU();
bool ponerCeroActual();
bool actualizarAngulo();
float anguloFinal();
// ---- Parte 2: HMI y comunicacion ----
void tareaComunicaciones(void* arg);
void vaciarSalida();
void conectarWiFi();
void atenderWiFi();
void eventoWebSocket(uint8_t num, WStype_t tipo, uint8_t* payload, size_t largo);
void procesarComandos();
bool esComandoPesado(const String& c);
void ejecutarComando(String entrada);
void ledColor(uint8_t r, uint8_t g, uint8_t b);
void actualizarLed();
void pulsadorAccion();
void atenderPulsador();
void imprimirEstado();

// ##############################################################################
// PARTE 1 - CONTROL EMBEBIDO (NUCLEO 1)
// Todo lo esencial para balancear: leer el MPU y los encoders, calcular los dos
// PID en cascada y mandar PWM a los motores. No depende del WiFi ni del HMI.
// ##############################################################################

// ==============================================================================
// 1.1 CONFIGURACION Y PINES
// Pines del driver TB6612FNG, de los encoders y del MPU6050, y los #define que
// dependen del montaje (sentido de los motores, signo del angulo, cero).
// ==============================================================================

// ---------- Pines del TB6612FNG (STBY fijo a 3.3V) ----------
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define BIN1 13
#define BIN2 12
#define PWMB 14

// ---------- Encoders de cuadratura (motores 25GA370, salida Hall A/B) ----------
// Se leen por interrupcion en los dos canales (decodificacion x4).
#define ENC_IZQ_A 18
#define ENC_IZQ_B 8
#define ENC_DER_A 41
#define ENC_DER_B 39
// Usa el pull-up interno por si las salidas son de colector abierto (si son push-pull no molesta)
#define ENC_PULLUP true
// Cambia a true si con el comando 'te' la cuenta de esa rueda sale NEGATIVA
#define ENC_INVERTIR_IZQ false     // medido: la rueda izquierda contaba negativo al girarla hacia adelante
#define ENC_INVERTIR_DER true

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

// ---------- MPU6050 ----------
#define MPU_ADDR     0x68
#define SDA_PIN      21
#define SCL_PIN      47
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75
#define ACCEL_XOUT_H 0x3B

// 1 = el balancin se inclina sobre el eje Y del sensor (usa ax, az, gy)
// 0 = el balancin se inclina sobre el eje X del sensor (usa ay, az, gx)
#define INCLINA_SOBRE_EJE_Y 1

// Cambia a true si el signo del angulo sale al reves de lo esperado
#define INVERTIR_ANGULO false

// Sentido de los motores respecto al signo del angulo:
//   true  -> angulo positivo = motores hacia ATRAS, angulo negativo = hacia ADELANTE
//   false -> angulo positivo = motores hacia ADELANTE, angulo negativo = hacia ATRAS
#define INVERTIR_SENTIDO_MOTORES true

// Cero inicial. Con 'z' lo fijas; para guardarlo al reiniciar, copia aqui
// el valor que imprime 'z'.
#define ANGULO_CERO -4.88

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
// Filtro complementario: alpha = TAU / (TAU + dt) con el dt REAL entre lecturas (micros).
// Antes era ALPHA = 0.98 fijo, que solo vale a 100 Hz; el MPU se lee a ~500 Hz y el
// acelerometro pesaba demasiado. Se cambia en vivo con 'tf'. Orden de magnitud: 1 a 2 s.
const float TAU_FILTRO_INICIAL = 1.5;

// ==============================================================================
// 1.2 PARAMETROS DEL CONTROL
// Valores iniciales de los PID, filtros y limites. Los PID se pueden cambiar en
// vivo con comandos (kp, ki, kd, vp, vi, vd, tol, tf...) sin recompilar; al
// reiniciar la ESP32 vuelven a estos valores.
// LAZO INTERNO  = PID de angulo (kp, ki, kd)       -> mismo para los dos motores
// LAZO EXTERNO  = PID de rpm   (kvp, kvi, kvd)     -> uno por motor (iguales)
// ==============================================================================

// ---------- Lazo de control ----------
const unsigned long TS_MS = 10;   // periodo de muestreo (Tsample) = 100 Hz
const float ANGULO_SP = 0.0;      // theta_sp: balancin vertical
// u esta en unidades de PWM y el error en grados: con Kp = 8, un error de 30
// grados da u = 240 (casi PWM maximo). Ajustalo en vivo con 'k'.
const float KP_INICIAL = 40.0;    // ganancia proporcional (valores tomados del HMI)
const float KI_INICIAL = 0.5;     // ganancia integral
const float KD_INICIAL = 1.1;     // ganancia derivativa
// true: de/dt del lazo de angulo = -(velocidad angular del giroscopio), sin retraso.
// false: derivada del error filtrada (como antes).
#define DERIVADA_DESDE_GIRO true
const float ALFA_D = 0.3;         // filtro de la derivada: peso de la muestra nueva (0-1). Menor = mas suave

// ---------- Lazo externo de velocidad (encoders): un PID por motor ----------
// Cuentas por vuelta de la RUEDA (con decodificacion x4). Midelas con el comando 'e':
// pon en cero, gira la rueda una vuelta a mano y lee la cuenta. Con 0 la velocidad
// se expresa en cuentas/s; con un valor > 0 se expresa en rpm.
// Medido a mano (3 vueltas, decodificacion x4): izq 10284/3 = 3428, der 10026/3 = 3342.
const float CUENTAS_POR_REV_IZQ = 3428;
const float CUENTAS_POR_REV_DER = 3342;
const float SP_VEL = 0.0;          // Sp_rpm = 0: el robot debe quedarse quieto
// Salida en grados. Arrancan en 0 = lazo apagado.
// En cuentas/s un orden de magnitud para empezar es Kvp ~ 0.0005; en rpm sera mucho mayor.
// Motor 1 (izquierdo) y motor 2 (derecho); valores tomados del HMI.
const float KVP_INICIAL = 0.08;      // Kp rpm motor 1
const float KVI_INICIAL = 0.0002;    // Ki rpm motor 1
const float KVD_INICIAL = 0.00001;   // Kd rpm motor 1
const float KVP_M2_INICIAL = KVP_INICIAL;   // motor 2 = igual que el motor 1
const float KVI_M2_INICIAL = KVI_INICIAL;
const float KVD_M2_INICIAL = KVD_INICIAL;
// true: el lazo de rpm de los DOS motores usa la velocidad PROMEDIO de las ruedas. Asi, si el
//       robot empieza a girar sobre si mismo (ruedas en sentidos opuestos), el lazo de rpm no
//       empuja ese giro. Cada motor sigue teniendo su propio PID y sus propias ganancias.
// false: cada motor usa solo la velocidad de su propia rueda (puede reforzar un giro).
#define LAZO_VEL_COMUN true
// true: si las ordenes de los dos motores salen con signos opuestos, se igualan (promedio).
//       Garantiza que las dos ruedas nunca giren en sentidos opuestos por el control.
#define FORZAR_MISMO_SENTIDO true
const float SP_MAX_GRADOS = 4.0;   // tope del setpoint de angulo que puede pedir el lazo externo
const float ALFA_V = 0.35;          // filtro de la velocidad: peso de la muestra nueva (0-1)
const float VEL_TEST_U = 60.0;     // u usada por el comando 'te'

// ---------- Proteccion de la integral (anti-windup) ----------
const float INTEGRAL_MAX_U = 50.0;        // el aporte de la integral nunca pasa de +-50 (unidades de PWM)
const float INTEGRAL_ZONA_GRADOS = 10.0;  // solo integra si |error| es menor que esto

// ---------- Tolerancia alrededor del setpoint ----------
// Si |error| <= tolGrados, P e I ven error 0 (el balancin "ya esta en el cero").
// Se cambia en vivo con 'tol' + numero (ej: tol0.5). tol0 = sin tolerancia.
const float TOL_INICIAL = 0.2;       // grados (+-)
const float TOL_MAX_GRADOS = 5.0;    // tope de seguridad para el comando 'tol'

// ---------- Salida a motores ----------
const float ZONA_MUERTA_U = 2.0;  // si |u| es menor, motores detenidos (evita temblor)
// PWM minimo con el que arranca cada motor (compensacion de zona muerta).
// Medidos a mano: el motor izquierdo arranca con 10 y el derecho con 19.
// Se asume Motor A = izquierdo y Motor B = derecho; si es al reves, intercambia los valores.
// Con el balancin en el suelo (con peso) suelen subir: vuelve a medirlos asi.
const int   PWM_MIN_A = 19;       // Motor A (izquierdo)
const int   PWM_MIN_B = 19;       // Motor B (derecho)
const int   PWM_MAX = 255;
const float CORTE_GRADOS = 45.0;  // si se inclina mas que esto, se apaga el control

// ---------- Tiempos del control ----------
const unsigned long REINTENTO_MPU_MS = 3000;
const int MAX_FALLOS_MPU = 5;     // lecturas seguidas fallidas antes de dar el MPU por perdido

// ==============================================================================
// 1.3 VARIABLES GLOBALES DEL CONTROL
// Estado del sensor, de los motores, de los dos PID y de los encoders.
// Las variables con [2] guardan un valor por motor: indice 0 = izquierdo,
// indice 1 = derecho.
// ==============================================================================

// ---------- MPU6050 ----------
int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
float gyroOffsetX = 0, gyroOffsetY = 0;
float anguloInclinacion = 0;      // grados (sin compensar)
float anguloCero = ANGULO_CERO;   // lo que marca el sensor en equilibrio
unsigned long ultimoTiempoMPU = 0;   // micros() de la ultima lectura
float tauFiltro = TAU_FILTRO_INICIAL;
float velGiroFinal = 0;           // velocidad angular con el signo final (grados/s)
unsigned long fallosI2CTotal = 0;   // lecturas del MPU que fallaron (diagnostico)
unsigned long tsMaxMs = 0;          // mayor periodo del lazo desde la ultima impresion (diagnostico)
int pwmMinA = PWM_MIN_A, pwmMinB = PWM_MIN_B;   // ajustables en vivo con 'pa' y 'pb'
bool mpuOk = false;
unsigned long ultimoReintento = 0;
int fallosMPU = 0;                // lecturas fallidas consecutivas

// ---------- Motores ----------
enum Modo { ADELANTE, REVERSA };
Modo modoActual = ADELANTE;
int velocidadActual = 0;

// ---------- Lazo de control ----------
bool controlActivo = false;       // arranca desactivado por seguridad
float anguloSetpoint = ANGULO_SP; // theta_sp
float anguloMedido = 0;           // theta_med
float errorAngulo = 0;            // e = theta_sp - theta_med
float kp = KP_INICIAL;            // ganancia proporcional
float ki = KI_INICIAL;            // ganancia integral
float tolGrados = TOL_INICIAL;    // tolerancia +- alrededor del setpoint (grados)
float kd = KD_INICIAL;            // ganancia derivativa
float terminoI = 0;               // (promedio de los dos motores, para imprimir)
float derivadaError = 0;          // (promedio de los dos motores, para imprimir)
// Estado por motor (indice 0 = izquierdo, 1 = derecho): cada uno con su propio lazo
float spTheta[2] = {0, 0};        // theta_sp que le pide el lazo de velocidad (grados)
float errorAng[2] = {0, 0};       // e = theta_sp - theta_med
float integralAng[2] = {0, 0};    // suma acumulada de e*Ts (lazo de angulo)
float terminoIAng[2] = {0, 0};    // aporte de la integral a u
float errorPrevioAng[2] = {0, 0}; // e_k-1
float derivadaAng[2] = {0, 0};    // de/dt filtrada (grados/s)
float uAng[2] = {0, 0};           // u de cada motor
int   pwmCanal[2] = {0, 0};       // PWM aplicado a cada motor
bool primeraMuestra = true;       // evita un pico de derivada al arrancar o tras fijar el cero
float salidaU = 0;                // u promedio de los dos motores (para imprimir)
unsigned long ultimoControlUs = 0;   // micros() de la ultima ejecucion del lazo
unsigned long tsReal = 0;         // ms reales entre dos ejecuciones del lazo

// ---------- Encoders ----------
volatile int32_t cuentasIzq = 0, cuentasDer = 0;   // se modifican en las interrupciones
volatile uint8_t estadoIzq = 0, estadoDer = 0;     // estado anterior (A<<1 | B)
portMUX_TYPE muxEnc = portMUX_INITIALIZER_UNLOCKED;
int32_t cuentasIzqPrev = 0, cuentasDerPrev = 0;
float vel[2] = {0, 0};            // velocidad de cada rueda (+ = sentido de u > 0), en UNIDAD_VEL
float kvp[2] = {KVP_INICIAL, KVP_M2_INICIAL};   // ganancia proporcional del lazo de velocidad (motor 1, motor 2)
float kvi[2] = {KVI_INICIAL, KVI_M2_INICIAL};   // ganancia integral del lazo de velocidad
float kvd[2] = {KVD_INICIAL, KVD_M2_INICIAL};   // ganancia derivativa del lazo de velocidad
float integralVel[2] = {0, 0};    // suma acumulada de e_vel*Ts por motor
float errorVelPrevio[2] = {0, 0};
float derivadaVel[2] = {0, 0};

// ==============================================================================
// 1.4 TAREA DE CONTROL (reloj exacto de 10 ms)
// Es la tarea que corre en el nucleo 1. Cada TS_MS (10 ms = 100 Hz):
//   1) atiende comandos pesados (c, z, te, i2c)   2) lee el MPU   3) calcula el lazo
// Usa vTaskDelayUntil para que el periodo sea exacto aunque el calculo tarde.
// ==============================================================================

// NUCLEO 1: lectura del MPU + lazo de control a periodo exacto de TS_MS
void tareaControl(void* arg) {
  TickType_t t0 = xTaskGetTickCount();
  for (;;) {
    if (hayComandoPesado) {                       // c, z, te, i2c (usan I2C o motores)
      String c = comandoPesado;
      ocupadoPesado = true;                       // primero "ocupado" y despues "ya no esta pendiente": nunca hay un hueco
      hayComandoPesado = false;
      if (controlActivo) {                        // estos comandos bloquean el lazo: paro de seguridad
        controlActivo = false;
        reiniciarIntegrales();
        detenerMotores();
        Out.println(">>> Control desactivado para ejecutar el comando");
      }
      ejecutarComando(c);
      ocupadoPesado = false;
      t0 = xTaskGetTickCount();                   // no intentes "recuperar" el tiempo que duro
    }
    atenderMPU();
    lazoControl();
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(TS_MS));
  }
}

// ==============================================================================
// 1.5 LAZO DE CONTROL EN CASCADA (el corazon del proyecto)
// Para cada motor (0 = izq, 1 = der):
//   LAZO EXTERNO: error rpm = SP_VEL(0) - rpm de la rueda -> PID -> theta_sp
//                 (saturado a +-SP_MAX_GRADOS). Si el robot se desplaza, pide
//                 inclinarse en contra para frenarse.
//   LAZO INTERNO: error angulo = theta_sp - theta_medido -> PID -> u del motor
// Opciones: LAZO_VEL_COMUN (rpm promedio) y FORZAR_MISMO_SENTIDO (ver 1.2).
// ==============================================================================

// CONTROL EN CASCADA, UN LAZO COMPLETO POR MOTOR (canal 0 = izquierdo, 1 = derecho).
// Se ejecuta solo cuando pasaron TS_MS (periodo de muestreo fijo).
//
//   SP_vel (0) -> e_vel = SP_vel - vel_i -> [PID velocidad] -> sat -> theta_sp_i
//   theta_sp_i -> e_theta = theta_sp_i - theta_med -> [PID angulo] -> u_i -> PWM_i -> motor_i
//
//   vel_i viene del encoder del motor i; theta_med es el MPU6050 (compartido).
//   El lazo del angulo esta DENTRO del lazo de velocidad.
void lazoControl() {
  // El periodo lo fija tareaControl (vTaskDelayUntil). Aqui solo se mide el real.
  unsigned long ahoraUs = micros();
  float dt = (ahoraUs - ultimoControlUs) / 1000000.0;
  ultimoControlUs = ahoraUs;
  if (dt <= 0.0 || dt > 0.1) dt = TS_MS / 1000.0;   // primera vez o pausa larga
  tsReal = (unsigned long)(dt * 1000.0 + 0.5);
  if (tsReal > tsMaxMs) tsMaxMs = tsReal;

  // Encoders: velocidad de cada rueda (se actualiza siempre, con o sin control)
  actualizarEncoders(dt);

  if (!mpuOk) return;

  anguloMedido = anguloFinal();                // theta_med (del MPU)

  for (int c = 0; c < 2; c++) {
    // ---------- LAZO EXTERNO: PID de velocidad -> theta_sp del motor c ----------
    if (controlActivo) {
#if LAZO_VEL_COMUN
      float eVel = SP_VEL - 0.5 * (vel[0] + vel[1]);   // velocidad promedio de las dos ruedas
#else
      float eVel = SP_VEL - vel[c];
#endif
      float dVelCruda = primeraMuestra ? 0.0 : (eVel - errorVelPrevio[c]) / dt;
      derivadaVel[c] = ALFA_D * dVelCruda + (1.0 - ALFA_D) * derivadaVel[c];
      errorVelPrevio[c] = eVel;

      integralVel[c] += eVel * dt;
      float tIv = kvi[c] * integralVel[c];
      if (tIv > SP_MAX_GRADOS)  tIv = SP_MAX_GRADOS;      // anti-windup del lazo externo
      if (tIv < -SP_MAX_GRADOS) tIv = -SP_MAX_GRADOS;
      if (kvi[c] != 0.0) integralVel[c] = tIv / kvi[c];

      // saturador: acota el angulo que puede pedir la velocidad.
      // El signo es NEGATIVO a proposito: vel es + en el sentido de u > 0, y u > 0 mueve las
      // ruedas hacia el lado de angulo NEGATIVO. Para frenar un movimiento hacia ese lado el
      // robot debe inclinarse hacia el lado contrario (angulo +), asi que theta_sp = -(PID).
      // (Con el signo contrario el lazo externo REALIMENTA POSITIVAMENTE y el robot se escapa:
      //  confirmado en la simulacion Programacion/simulacion.)
      spTheta[c] = constrain(-(kvp[c] * eVel + tIv + kvd[c] * derivadaVel[c]), -SP_MAX_GRADOS, SP_MAX_GRADOS);
    } else {
      spTheta[c] = ANGULO_SP;
      integralVel[c] = 0;
      errorVelPrevio[c] = 0;
      derivadaVel[c] = 0;
    }

    // ---------- LAZO INTERNO: PID de angulo del motor c ----------
    float e = spTheta[c] - anguloMedido;           // e = theta_sp - theta_med
    errorAng[c] = e;

    // TOLERANCIA: dentro de +-tolGrados P e I ven error 0 (la derivada usa el error real)
    float eControl = (fabs(e) <= tolGrados) ? 0.0f : e;

#if DERIVADA_DESDE_GIRO
    derivadaAng[c] = -velGiroFinal;                // de/dt = -(velocidad angular): sin retraso
#else
    float dCruda = primeraMuestra ? 0.0 : (e - errorPrevioAng[c]) / dt;
    derivadaAng[c] = ALFA_D * dCruda + (1.0 - ALFA_D) * derivadaAng[c];   // filtro suave
#endif
    errorPrevioAng[c] = e;

    if (controlActivo) {
      if (fabs(eControl) < INTEGRAL_ZONA_GRADOS) integralAng[c] += eControl * dt;  // anti-windup por zona
    } else {
      integralAng[c] = 0;
    }
    float tI = ki * integralAng[c];
    if (tI > INTEGRAL_MAX_U)  tI = INTEGRAL_MAX_U;                        // tope del aporte
    if (tI < -INTEGRAL_MAX_U) tI = -INTEGRAL_MAX_U;
    if (ki != 0.0) integralAng[c] = tI / ki;
    terminoIAng[c] = tI;

    uAng[c] = kp * eControl + tI + kd * derivadaAng[c];   // u = Kp*e + Ki*Int(e) + Kd*de/dt
  }
  primeraMuestra = false;

#if FORZAR_MISMO_SENTIDO
  // Las dos ruedas nunca deben recibir ordenes en sentidos opuestos: eso hace girar al robot
  if (uAng[0] * uAng[1] < 0) {
    float uMedia = 0.5 * (uAng[0] + uAng[1]);
    uAng[0] = uMedia;
    uAng[1] = uMedia;
  }
#endif

  // Promedios de los dos canales: lo que se imprime en la linea 1 de telemetria
  anguloSetpoint = 0.5 * (spTheta[0] + spTheta[1]);
  errorAngulo    = 0.5 * (errorAng[0] + errorAng[1]);
  derivadaError  = 0.5 * (derivadaAng[0] + derivadaAng[1]);
  terminoI       = 0.5 * (terminoIAng[0] + terminoIAng[1]);
  salidaU        = 0.5 * (uAng[0] + uAng[1]);

  // u -> sentido + PWM -> motores
  if (controlActivo) {
    if (fabs(anguloMedido) > CORTE_GRADOS) {    // el balancin cayo: apagar
      controlActivo = false;
      reiniciarIntegrales();
      detenerMotores();
      Out.println("!!! Angulo fuera de rango: control desactivado");
    } else {
      aplicarSalidaMotor(0, uAng[0]);
      aplicarSalidaMotor(1, uAng[1]);
      velocidadActual = (pwmCanal[0] + pwmCanal[1]) / 2;   // solo para imprimir
    }
  } else {
    detenerMotores();   // la tarea de control es la unica que manda a los motores: 's' o 'm' paran en <= 10 ms
  }
}

// Pone en cero todos los acumuladores de integral (angulo y velocidad de ambos motores)
void reiniciarIntegrales() {
  for (int c = 0; c < 2; c++) {
    integralAng[c] = 0;
    integralVel[c] = 0;
  }
}

// ==============================================================================
// 1.6 SALIDA A LOS MOTORES (u -> PWM)
// Convierte la orden u de cada motor en sentido de giro + PWM (0-255):
// zona muerta, PWM minimo de arranque por motor y limite maximo.
// ==============================================================================

// u -> motor del canal c. Signo de u = sentido; |u| = magnitud del PWM.
//   |u| < ZONA_MUERTA_U -> detenido
//   si no: PWM = PWM_MIN del motor + |u| reescalado hasta PWM_MAX
//          (cada motor tiene su propio minimo, asi arrancan a la vez)
//   Sentido: u > 0 -> adelante = INVERTIR_SENTIDO_MOTORES (u < 0 al reves)
void aplicarSalidaMotor(int c, float u) {
  float mag = fabs(u);
  if (mag < ZONA_MUERTA_U) {
    ledcWrite(c == 0 ? PWMA : PWMB, 0);
    pwmCanal[c] = 0;
    return;
  }

  mag = constrain(mag, 0.0f, (float)PWM_MAX);
  int pwmMin = (c == 0) ? pwmMinA : pwmMinB;
  int pwm = pwmMin + (int)(mag * (PWM_MAX - pwmMin) / PWM_MAX);
  pwmCanal[c] = pwm;

  bool adelante = INVERTIR_SENTIDO_MOTORES ? (u > 0) : !(u > 0);
  modoActual = adelante ? REVERSA : ADELANTE;   // solo informativo
  if (c == 0) motorA(adelante, pwm);
  else        motorB(adelante, pwm);
}

// ---------- Motores ----------
void motorA(bool dirForward, int pwm) {
  digitalWrite(AIN1, dirForward ? HIGH : LOW);
  digitalWrite(AIN2, dirForward ? LOW  : HIGH);
  ledcWrite(PWMA, pwm);
}

void motorB(bool dirForward, int pwm) {
  digitalWrite(BIN1, dirForward ? HIGH : LOW);
  digitalWrite(BIN2, dirForward ? LOW  : HIGH);
  ledcWrite(PWMB, pwm);
}

void aplicarMovimiento() {
  bool adelante = (modoActual == REVERSA); // invertido a proposito, confirmado en pruebas
  motorA(adelante, velocidadActual);
  motorB(adelante, velocidadActual);
}

void detenerMotores() {
  velocidadActual = 0;
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

// ==============================================================================
// 1.7 ENCODERS (cuentas y velocidad en rpm)
// Cada encoder tiene 2 canales (A y B). Una interrupcion en cada cambio cuenta
// pulsos hacia arriba o hacia abajo (decodificacion x4). Cada ciclo se calcula
// la velocidad de cada rueda en rpm con un filtro suave.
// ==============================================================================

// ---------- Encoders ----------
// Tabla de cuadratura x4: indice = (estado_anterior << 2) | estado_nuevo (cada estado = A<<1 | B)
static const int8_t TABLA_QUAD[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR ISR_encIzq() {
  uint8_t nuevo = (digitalRead(ENC_IZQ_A) << 1) | digitalRead(ENC_IZQ_B);
  int8_t d = TABLA_QUAD[(estadoIzq << 2) | nuevo];
  estadoIzq = nuevo;
  cuentasIzq += ENC_INVERTIR_IZQ ? -d : d;
}

void IRAM_ATTR ISR_encDer() {
  uint8_t nuevo = (digitalRead(ENC_DER_A) << 1) | digitalRead(ENC_DER_B);
  int8_t d = TABLA_QUAD[(estadoDer << 2) | nuevo];
  estadoDer = nuevo;
  cuentasDer += ENC_INVERTIR_DER ? -d : d;
}

void iniciarEncoders() {
  const int modo = ENC_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_IZQ_A, modo);
  pinMode(ENC_IZQ_B, modo);
  pinMode(ENC_DER_A, modo);
  pinMode(ENC_DER_B, modo);
  estadoIzq = (digitalRead(ENC_IZQ_A) << 1) | digitalRead(ENC_IZQ_B);
  estadoDer = (digitalRead(ENC_DER_A) << 1) | digitalRead(ENC_DER_B);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), ISR_encIzq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_B), ISR_encIzq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_A), ISR_encDer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_B), ISR_encDer, CHANGE);
}

// Copia atomica de las cuentas (las interrupciones las modifican en cualquier momento)
void leerCuentas(int32_t &izq, int32_t &der) {
  portENTER_CRITICAL(&muxEnc);
  izq = cuentasIzq;
  der = cuentasDer;
  portEXIT_CRITICAL(&muxEnc);
}

// Velocidad de cada rueda (cuentas/s) = cuentas nuevas / dt, con filtro de primer orden
void actualizarEncoders(float dt) {
  if (dt <= 0) return;
  int32_t cI, cD;
  leerCuentas(cI, cD);
  float vI = (cI - cuentasIzqPrev) / dt;
  float vD = (cD - cuentasDerPrev) / dt;
  cuentasIzqPrev = cI;
  cuentasDerPrev = cD;
  const float kI = (CUENTAS_POR_REV_IZQ > 0) ? 60.0 / CUENTAS_POR_REV_IZQ : 1.0;   // cuentas/s -> rpm
  const float kD = (CUENTAS_POR_REV_DER > 0) ? 60.0 / CUENTAS_POR_REV_DER : 1.0;
  vel[0] = ALFA_V * (vI * kI) + (1.0 - ALFA_V) * vel[0];
  vel[1] = ALFA_V * (vD * kD) + (1.0 - ALFA_V) * vel[1];
}

// Prueba: cada motor 1 s con u = +VEL_TEST_U. Las cuentas deben salir POSITIVAS.
// Bloquea ~2.5 s; solo con el control apagado y las ruedas en el aire.
void pruebaEncoders() {
  if (controlActivo) {
    Out.println("!!! Apaga el control ('s') antes de probar los encoders");
    return;
  }
  // Sentido de u > 0: aplicarSalidaMotor() usa adelante = INVERTIR_SENTIDO_MOTORES para u > 0
  const bool adelante = INVERTIR_SENTIDO_MOTORES;
  float mag = constrain(VEL_TEST_U, 0.0f, (float)PWM_MAX);
  int32_t i0, d0, i1, d1;

  Out.println(">>> Prueba de encoders (ruedas en el aire): motor izquierdo 1 s...");
  leerCuentas(i0, d0);
  motorA(adelante, pwmMinA + (int)(mag * (PWM_MAX - pwmMinA) / PWM_MAX));
  delay(1000);
  detenerMotores();
  delay(300);
  leerCuentas(i1, d1);
  Out.print("    Izquierda: ");
  Out.print(i1 - i0);
  Out.print(" cuentas (la derecha conto ");
  Out.print(d1 - d0);
  Out.println(")");

  Out.println(">>> motor derecho 1 s...");
  leerCuentas(i0, d0);
  motorB(adelante, pwmMinB + (int)(mag * (PWM_MAX - pwmMinB) / PWM_MAX));
  delay(1000);
  detenerMotores();
  delay(300);
  leerCuentas(i1, d1);
  Out.print("    Derecha: ");
  Out.print(d1 - d0);
  Out.print(" cuentas (la izquierda conto ");
  Out.print(i1 - i0);
  Out.println(")");
  Out.println(">>> Ambas deben ser POSITIVAS y la otra rueda ~0. Si no, ajusta ENC_INVERTIR_* o revisa el cableado A/B.");
  cuentasIzqPrev = i1;
  cuentasDerPrev = d1;
}

// ==============================================================================
// 1.8 SENSOR MPU6050 Y FILTRO COMPLEMENTARIO (angulo)
// Acelerometro: da el angulo sin deriva pero con ruido y errores al acelerar.
// Giroscopio:   da cambios rapidos y limpios pero se desvia con el tiempo.
// El filtro complementario mezcla los dos: alpha = tau / (tau + dt).
// El giroscopio tambien da la derivada del angulo (de/dt) sin retraso.
// ==============================================================================

// Actualiza el angulo; si se pierde el MPU, reintenta cada 3 s
void atenderMPU() {
  if (mpuOk) {
    if (actualizarAngulo()) {
      fallosMPU = 0;
    } else if (++fallosMPU >= MAX_FALLOS_MPU) {
      // Una lectura fallida suelta (ruido de los motores) se ignora: se conserva
      // el ultimo angulo. Solo si fallan varias seguidas se da el MPU por perdido.
      fallosMPU = 0;
      mpuOk = false;
      controlActivo = false;
      primeraMuestra = true;
      reiniciarIntegrales();
      detenerMotores();
      Out.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Out.println(">>> MPU6050 reconectado");
  }
}

bool mpuEscribir(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registro);
  Wire.write(valor);
  return Wire.endTransmission(true) == 0;
}

// Devuelve false si el sensor no responde (en vez de leer basura)
bool mpuLeerCrudo() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14, true) != 14) return false;

  axRaw = (Wire.read() << 8) | Wire.read();
  ayRaw = (Wire.read() << 8) | Wire.read();
  azRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // descarta temperatura
  gxRaw = (Wire.read() << 8) | Wire.read();
  gyRaw = (Wire.read() << 8) | Wire.read();
  gzRaw = (Wire.read() << 8) | Wire.read();
  return true;
}

void escanearI2C() {
  Out.println("Escaneando bus I2C...");
  int encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Out.print("  Dispositivo en 0x");
      if (dir < 16) Out.print("0");
      Out.println(dir, HEX);
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Out.println("  No se encontro NINGUN dispositivo I2C.");
  }
}

bool calibrarGiroscopio() {
  Out.println("Calibrando giroscopio, no muevas el sensor...");
  long sumX = 0, sumY = 0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    if (!mpuLeerCrudo()) return false;
    sumX += gxRaw;
    sumY += gyRaw;
    delay(3);
  }

  gyroOffsetX = sumX / (float)muestras;
  gyroOffsetY = sumY / (float)muestras;
  Out.println("Calibracion lista.");
  return true;
}

// Angulo calculado solo con el acelerometro (usa los ultimos datos crudos)
float anguloDesdeAcelerometro() {
  float ax = axRaw / ACCEL_SENS;
  float ay = ayRaw / ACCEL_SENS;
  float az = azRaw / ACCEL_SENS;
#if INCLINA_SOBRE_EJE_Y
  return atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
#else
  return atan2(ay, az) * 180.0 / PI;
#endif
}

bool iniciarMPU() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  if (!mpuEscribir(PWR_MGMT_1, 0x00)) return false; // despierta el sensor
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(WHO_AM_I);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, 1, true) == 1) {
    Out.print("WHO_AM_I = 0x");
    Out.println(Wire.read(), HEX); // esperado: 0x68
  }

  if (!calibrarGiroscopio()) return false;

  // Arranca el filtro desde el angulo real del acelerometro (no desde 0)
  if (!mpuLeerCrudo()) return false;
  anguloInclinacion = anguloDesdeAcelerometro();
  ultimoTiempoMPU = micros();
  return true;
}

// Fija la posicion actual como "angulo 0". Promedia 100 lecturas (~0.4 s).
bool ponerCeroActual() {
  float suma = 0;
  const int muestras = 100;
  for (int i = 0; i < muestras; i++) {
    if (!mpuLeerCrudo()) return false;
    suma += anguloDesdeAcelerometro();
    delay(3);
  }
  anguloCero = suma / muestras;
  anguloInclinacion = anguloCero; // sincroniza el filtro: ya marca 0
  ultimoTiempoMPU = micros();
  return true;
}

// Filtro complementario: giroscopio (rapido) + acelerometro (sin deriva)
bool actualizarAngulo() {
  if (!mpuLeerCrudo()) { fallosI2CTotal++; return false; }

  float anguloAccel = anguloDesdeAcelerometro();
#if INCLINA_SOBRE_EJE_Y
  float velGiro = (gyRaw - gyroOffsetY) / GYRO_SENS;
#else
  float velGiro = (gxRaw - gyroOffsetX) / GYRO_SENS;
#endif

  unsigned long ahora = micros();
  float dt = (ahora - ultimoTiempoMPU) / 1000000.0;
  ultimoTiempoMPU = ahora;
  if (dt > 0.05) dt = 0.05;                      // tras una pausa larga no integra de mas

  velGiroFinal = INVERTIR_ANGULO ? -velGiro : velGiro;

  float alpha = tauFiltro / (tauFiltro + dt);    // filtro por dt real
  anguloInclinacion = alpha * (anguloInclinacion + velGiro * dt) + (1.0 - alpha) * anguloAccel;
  return true;
}

// Angulo ya compensado (cero aplicado) y con el signo final
float anguloFinal() {
  float a = anguloInclinacion - anguloCero;
  return INVERTIR_ANGULO ? -a : a;
}

// ##############################################################################
// PARTE 2 - HMI, COMUNICACION Y SECUNDARIO (NUCLEO 0)
// Todo lo que NO es esencial para el equilibrio: WiFi, WebSocket (conexion con
// la pagina balancin_pid.html), comandos, pulsador, LED y telemetria. Si esta parte
// falla o se retrasa, el control del nucleo 1 sigue igual.
// ##############################################################################

// ==============================================================================
// 2.1 CONFIGURACION DEL WIFI, DEL PULSADOR Y DEL LED RGB
// El ESP32 se conecta a tu red (solo 2.4 GHz) y abre un servidor WebSocket.
// La pagina se conecta a  ws://<IP del ESP o balancin.local>:81
// La IP se imprime en el Monitor Serial al arrancar.
// La red y la clave estan en secretos.h (no se sube a git).
// ==============================================================================

const char* HOSTNAME  = "balancin";            // -> balancin.local
const uint16_t WS_PORT = 81;
const unsigned long WIFI_ESPERA_MS = 10000;    // espera maxima de conexion al arrancar
const unsigned long WIFI_REINTENTO_MS = 5000;  // cada cuanto reintenta si se cae la red
const unsigned long INTERVALO_MS = 100;        // periodo de la telemetria (10 Hz)

// ---------- Pulsador de arranque (secuencia de 3 pulsaciones, ver seccion 2.7) ----------
// Un lado del pulsador al GPIO 40 y el otro a GND (usa el pull-up interno).
//   1a pulsacion = comando 'c' (recalibrar)   2a = 'z' (fijar cero)   3a = 'm' (activar control)
//   Con el control activo, una pulsacion lo PARA ('s').
#define PIN_PULSADOR 40
#define PULSADOR_ACTIVO_BAJO true      // true: pulsador a GND. false: pulsador a 3.3 V (necesita pull-down externo)
#define PULSADOR_ANTIREBOTE_MS 50

// ---------- LED RGB de la placa (muestra en que paso va el arranque) ----------
// La DevKitC-1 trae un LED RGB tipo WS2812 en el GPIO 48 (placa v1.0) o en el GPIO 38 (placa v1.1).
// No sabemos cual trae la tuya, asi que se escriben los dos (en la PCB ninguno de los dos esta conectado).
// OJO: la DevKitC-1 tambien tiene un LED rojo de alimentacion que no se puede controlar. Si al encender
// el LED rojo no cambia de color con este firmware, es muy probable que sea ese y no el RGB.
#define PIN_LED_RGB_A 48
#define PIN_LED_RGB_B 38
#define LED_BRILLO 40                  // 0 a 255. Los WS2812 deslumbran: se deja bajo
// Colores (rojo, verde, azul):
//   AZUL     = encendido, esperando la 1a pulsacion
//   APAGADO  = trabajando (calibrando o fijando el cero)
//   AMARILLO = giroscopio calibrado, esperando la 2a pulsacion
//   VERDE    = cero fijado, esperando la 3a pulsacion (listo para arrancar)
//   MORADO   = control ACTIVO (motores funcionando)
//   ROJO     = error: el MPU6050 no responde

// ==============================================================================
// 2.2 VARIABLES DE COMUNICACION
// Estado del WiFi/WebSocket y cola de comandos recibidos desde el HMI.
// ==============================================================================

unsigned long ultimaImpresion = 0;

// ---------- WiFi / WebSocket ----------
WebSocketsServer webSocket(WS_PORT);
bool serviciosWiFiOk = false;          // true cuando ya arranco WebSocket + mDNS
int wsClientes = 0;                    // paginas conectadas ahora mismo
unsigned long ultimoReintentoWiFi = 0;

// Cola de comandos recibidos por WebSocket (se ejecutan en loop(), no dentro del callback)
const int COLA_N = 8;
String colaComandos[COLA_N];
int colaLeer = 0, colaEscribir = 0;

// ==============================================================================
// 2.3 TAREA DE COMUNICACIONES (nucleo 0)
// Repite sin parar: atender el WiFi, leer comandos, pulsador y LED, imprimir
// la telemetria y vaciar el buffer de texto. Nunca toca los motores ni el MPU.
// ==============================================================================

// NUCLEO 0: WiFi + WebSocket + comandos + impresion
void tareaComunicaciones(void* arg) {
  for (;;) {
    atenderWiFi();
    procesarComandos();
    atenderPulsador();
    imprimirEstado();
    vaciarSalida();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ==============================================================================
// 2.4 SALIDA DE TEXTO HACIA SERIAL Y WEBSOCKET
// Vacia el buffer de Out. Cada linea completa se envia tambien a la pagina.
// ==============================================================================

// Vacia el buffer hacia Serial y WebSocket. Solo la llama la tarea de comunicaciones (o setup).
void vaciarSalida() {
  static String linea;
  char tmp[96];
  for (;;) {
    int n = 0;
    portENTER_CRITICAL(&muxSalida);
    while (salidaLeer != salidaEscribir && n < (int)sizeof(tmp)) {
      tmp[n++] = bufSalida[salidaLeer];
      salidaLeer = (salidaLeer + 1) % SALIDA_N;
    }
    portEXIT_CRITICAL(&muxSalida);
    if (n == 0) break;
    Serial.write((const uint8_t*)tmp, n);
    for (int i = 0; i < n; i++) {
      char c = tmp[i];
      if (c == '\n') {
        if (wsClientes > 0 && linea.length() > 0) webSocket.broadcastTXT(linea.c_str(), linea.length());
        linea = "";
      } else if (c != '\r' && linea.length() < 400) {
        linea += c;
      }
    }
  }
}

// ==============================================================================
// 2.5 WIFI Y WEBSOCKET (conexion con el HMI)
// Conecta a la red, arranca el servidor WebSocket y el nombre balancin.local,
// y encola los comandos que llegan desde la pagina.
// ==============================================================================

// Conecta a la red WiFi (estacion). Espera hasta WIFI_ESPERA_MS; si no logra
// conectar, el balancin sigue funcionando por USB y reintenta desde loop().
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);            // sin ahorro de energia: menor latencia
  Out.print("Conectando a la red '");
  Out.print(WIFI_SSID);
  Out.print("'");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < WIFI_ESPERA_MS) {
    delay(300);
    Out.print(".");
    vaciarSalida();
  }
  Out.println();
  ultimoReintentoWiFi = millis();
}

// Arranca WebSocket + mDNS cuando hay red; reintenta la conexion si se cae
void atenderWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!serviciosWiFiOk) {
      serviciosWiFiOk = true;
      if (MDNS.begin(HOSTNAME)) MDNS.addService("ws", "tcp", WS_PORT);
      webSocket.begin();
      webSocket.onEvent(eventoWebSocket);
      webSocket.enableHeartbeat(15000, 3000, 2);   // detecta paginas que se cerraron sin avisar
      Out.print(">>> WiFi conectado. IP: ");
      Out.println(WiFi.localIP());
      Out.print("    En la pagina usa:  ");
      Out.print(WiFi.localIP());
      Out.print("   o   ");
      Out.print(HOSTNAME);
      Out.println(".local   (puerto 81)");
    }
    webSocket.loop();
  } else if (millis() - ultimoReintentoWiFi > WIFI_REINTENTO_MS) {
    ultimoReintentoWiFi = millis();
    WiFi.reconnect();
  }
}

// Eventos del WebSocket: solo encola el texto; se ejecuta en procesarComandos()
void eventoWebSocket(uint8_t num, WStype_t tipo, uint8_t* payload, size_t largo) {
  switch (tipo) {
    case WStype_CONNECTED:
      wsClientes++;
      webSocket.sendTXT(num, "=== Balancin WiFi conectado: telemetria y comandos activos ===");
      break;
    case WStype_DISCONNECTED:
      if (wsClientes > 0) wsClientes--;
      break;
    case WStype_TEXT: {
      int siguiente = (colaEscribir + 1) % COLA_N;
      if (siguiente != colaLeer && largo > 0 && largo < 64) {   // si la cola esta llena se descarta
        colaComandos[colaEscribir] = String((const char*)payload).substring(0, largo);
        colaEscribir = siguiente;
      }
      break;
    }
    default: break;
  }
}

// ==============================================================================
// 2.6 COMANDOS (Monitor Serial y HMI)
// Lee los comandos y los ejecuta. Casi todos solo cambian una variable del
// control (kp, ki, kd, tol, vp, vi, vd, tf, pa, pb, m, s). Los que usan el bus
// I2C o los motores (c, z, te, i2c) se pasan a la tarea de control.
// ==============================================================================

// Lee comandos del Monitor Serial y de la pagina (WebSocket)
void procesarComandos() {
  if (Serial.available() > 0) {
    ejecutarComando(Serial.readStringUntil('\n'));
  }
  while (colaLeer != colaEscribir) {
    String c = colaComandos[colaLeer];
    colaLeer = (colaLeer + 1) % COLA_N;
    ejecutarComando(c);
  }
}

bool esComandoPesado(const String& c) {
  return c.equalsIgnoreCase("c") || c.equalsIgnoreCase("z") || c.equalsIgnoreCase("te") ||
         (c.length() > 3 && c.substring(0, 3).equalsIgnoreCase("i2c"));
}

// Ejecuta un comando (viene del Serial o del WebSocket)
void ejecutarComando(String entrada) {
  entrada.trim();
  // kvp / kvi / kvd son sinonimos de vp / vi / vd (sin esto 'k' los leeria como Kp del angulo)
  if (entrada.length() > 2 && (entrada[0] == 'k' || entrada[0] == 'K') && (entrada[1] == 'v' || entrada[1] == 'V')) {
    entrada = entrada.substring(1);
  }
  // c, z, te e i2c usan el bus I2C o los motores: los ejecuta la tarea de control
  if (esComandoPesado(entrada) && xTaskGetCurrentTaskHandle() != hTareaControl) {
    if (hayComandoPesado) {
      Out.println("!!! Ocupado: espera a que termine el comando anterior");
    } else {
      comandoPesado = entrada;
      hayComandoPesado = true;
    }
    return;
  }

  if (entrada.equalsIgnoreCase("c")) {
    controlActivo = false;
    detenerMotores();
    primeraMuestra = true;
    reiniciarIntegrales();
    Out.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    Out.println(mpuOk ? ">>> Recalibracion completa"
                         : "!!! No se pudo recalibrar: el MPU6050 no responde");
    if (mpuOk && etapaRobot < 1) etapaRobot = 1;   // etapa 1: giroscopio calibrado (si el cero ya estaba fijado, se conserva)
  } else if (entrada.equalsIgnoreCase("z")) {
    primeraMuestra = true;
    reiniciarIntegrales();
    if (mpuOk && ponerCeroActual()) {
      etapaRobot = 2;                              // etapa 2: cero fijado, listo para arrancar
      Out.print(">>> CERO fijado. El sensor marcaba ");
      Out.print(anguloCero, 2);
      Out.println(" grados en equilibrio (ahora marca 0).");
      Out.print("    Para guardarlo al reiniciar: #define ANGULO_CERO ");
      Out.println(anguloCero, 2);
    } else {
      Out.println("!!! No se pudo fijar el cero: el MPU6050 no responde");
    }
  } else if (entrada.equalsIgnoreCase("m")) {
    if (!mpuOk) {
      Out.println(">>> No se puede: el MPU6050 no responde");
    } else {
      controlActivo = !controlActivo;
      primeraMuestra = true;
      reiniciarIntegrales();
      if (!controlActivo) detenerMotores();
      Out.println(controlActivo ? ">>> CONTROL ACTIVADO" : ">>> Control desactivado");
    }
  } else if (entrada.equalsIgnoreCase("s")) {
    controlActivo = false;
    reiniciarIntegrales();
    detenerMotores();
    Out.println(">>> Detenido");
  } else if (entrada.length() > 3 && entrada.substring(0, 3).equalsIgnoreCase("i2c")) {
    int khz = entrada.substring(3).toInt();
    if (khz >= 50 && khz <= 400) {
      Wire.setClock((uint32_t)khz * 1000);
      Out.print(">>> I2C a ");
      Out.print(khz);
      Out.println(" kHz");
    } else {
      Out.println("!!! Escribe i2c seguido de 50 a 400. Ej: i2c100");
    }
  } else if (entrada.length() > 2 && entrada.substring(0, 2).equalsIgnoreCase("tf")) {
    int i = 2;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if (i < (int)entrada.length()) {
      tauFiltro = constrain(entrada.substring(i).toFloat(), 0.05f, 20.0f);
      Out.print(">>> Tau del filtro = ");
      Out.print(tauFiltro, 2);
      Out.println(" s");
    } else {
      Out.println("!!! Escribe tf seguido de un numero. Ej: tf1.5");
    }
  } else if (entrada.length() > 2 && (entrada.substring(0, 2).equalsIgnoreCase("pa") || entrada.substring(0, 2).equalsIgnoreCase("pb"))) {
    bool esA = (entrada[1] == 'a' || entrada[1] == 'A');
    int i = 2;
    while (i < (int)entrada.length() && !isDigit(entrada[i])) i++;
    if (i < (int)entrada.length()) {
      int v = constrain((int)entrada.substring(i).toInt(), 0, 120);
      if (esA) pwmMinA = v; else pwmMinB = v;
      Out.print(">>> PWM minimo motor ");
      Out.print(esA ? "A (izq) = " : "B (der) = ");
      Out.println(v);
    } else {
      Out.println("!!! Escribe pa o pb seguido de un numero. Ej: pa30");
    }
  } else if (entrada.equalsIgnoreCase("e")) {
    int32_t cI, cD;
    portENTER_CRITICAL(&muxEnc);
    cuentasIzq = 0;
    cuentasDer = 0;
    portEXIT_CRITICAL(&muxEnc);
    leerCuentas(cI, cD);
    cuentasIzqPrev = cI;
    cuentasDerPrev = cD;
    Out.println(">>> Encoders en cero. Gira una rueda a mano 1 vuelta y mira la linea Enc para ver sus cuentas por vuelta.");
  } else if (entrada.equalsIgnoreCase("te")) {
    pruebaEncoders();
  } else if (entrada.length() > 2 && (entrada[0] == 'v' || entrada[0] == 'V')) {
    // Ganancias del lazo de velocidad (encoders):
    //   vp / vi / vd  + numero -> los DOS motores.   Ej: vp0.0005
    //   v1p v1i v1d / v2p v2i v2d + numero -> solo el motor 1 (izq) o el motor 2 (der). Ej: v2i0.0002
    int motor = -1, pos = 1;                          // -1 = ambos
    if (entrada[1] == '1' || entrada[1] == '2') { motor = entrada[1] - '1'; pos = 2; }
    char cual = (char)tolower(entrada[pos]);
    int i = pos + 1;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if ((cual == 'p' || cual == 'i' || cual == 'd') && i < (int)entrada.length()) {
      float valor = entrada.substring(i).toFloat();
      for (int c = 0; c < 2; c++) {
        if (motor >= 0 && motor != c) continue;
        if (cual == 'p') kvp[c] = valor;
        else if (cual == 'i') kvi[c] = valor;
        else kvd[c] = valor;
      }
      reiniciarIntegrales();                          // evita un salto al cambiar la ganancia
      Out.print(">>> Kv");
      Out.print(cual);
      Out.print(motor < 0 ? " (motores 1 y 2) = " : (motor == 0 ? " (motor 1) = " : " (motor 2) = "));
      Out.println(valor, 6);
    } else {
      Out.println("!!! Usa vp/vi/vd<num> (ambos motores) o v1p v1i v1d v2p v2i v2d <num>. Ej: vp0.0005  v2i0.0002");
    }
  } else if (entrada.length() > 3 && entrada.substring(0, 3).equalsIgnoreCase("tol")) {
    // Tolerancia alrededor del setpoint. Ej: tol0.5  tol=0.5  tol0 (sin tolerancia)
    int i = 3;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if (i < (int)entrada.length()) {
      tolGrados = constrain(entrada.substring(i).toFloat(), 0.0f, TOL_MAX_GRADOS);
      reiniciarIntegrales();                            // evita un salto al cambiar la tolerancia
      Out.print(">>> Tolerancia = +-");
      Out.print(tolGrados, 2);
      Out.println(" grados");
    } else {
      Out.println("!!! Escribe tol seguido de un numero. Ej: tol0.5");
    }
  } else if (entrada.length() > 1 && (entrada[0] == 'k' || entrada[0] == 'K')) {
    // Cambiar ganancias en vivo. Acepta: kp10  kp=10  k10  kd0.5  kd=0.5
    char cual = (char)tolower(entrada[1]);          // 'd' = Kd, 'i' = Ki, cualquier otra = Kp
    int i = 1;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.' && entrada[i] != '-') i++;
    if (i < (int)entrada.length()) {
      float valor = entrada.substring(i).toFloat();
      if (cual == 'd') {
        kd = valor;
        Out.print(">>> Kd = ");
        Out.println(kd, 3);
      } else if (cual == 'i') {
        ki = valor;
        reiniciarIntegrales();                          // evita un salto al cambiar Ki
        Out.print(">>> Ki = ");
        Out.println(ki, 3);
      } else {
        kp = valor;
        Out.print(">>> Kp = ");
        Out.println(kp, 3);
      }
    } else {
      Out.println("!!! Escribe kp, ki o kd seguido de un numero. Ej: kp10  ki0.5  kd0.5");
    }
  }
}

// ==============================================================================
// 2.7 PULSADOR DE ARRANQUE Y LED DE ESTADO
// Un solo boton lleva el arranque en orden y el LED RGB dice en que paso va:
//
//   LED azul     --pulsar-->  LED apagado (recalibra, comando c)   -->  LED amarillo
//   LED amarillo --pulsar-->  LED apagado (fija el cero, comando z) --> LED verde
//   LED verde    --pulsar-->  LED morado  (activa el control, comando m: arrancan los motores)
//   LED morado   --pulsar-->  para el control (comando s) y vuelve a verde
//
// Como usarlo: 1) robot QUIETO, pulsa y espera ~2 s. 2) sostenlo en su punto de equilibrio,
// pulsa y espera ~0.5 s. 3) sueltalo y pulsa: arranca. El LED se pone rojo si el MPU6050 no
// responde (al pulsar se reintenta calibrar). Mientras trabaja el LED esta apagado y las
// pulsaciones se ignoran.
// El paso en que va se guarda en etapaRobot, que tambien actualizan los comandos c y z escritos
// desde el Serial o el HMI, asi el LED siempre muestra la situacion real.
// ==============================================================================

// Enciende el LED RGB con un color (0-255 por canal, escalado por LED_BRILLO). Solo escribe si cambio.
void ledColor(uint8_t r, uint8_t g, uint8_t b) {
  static int ultimo = -1;
  int clave = (r << 16) | (g << 8) | b;
  if (clave == ultimo) return;
  ultimo = clave;
  uint8_t rr = (uint8_t)((uint16_t)r * LED_BRILLO / 255);
  uint8_t gg = (uint8_t)((uint16_t)g * LED_BRILLO / 255);
  uint8_t bb = (uint8_t)((uint16_t)b * LED_BRILLO / 255);
  neopixelWrite(PIN_LED_RGB_A, rr, gg, bb);
  neopixelWrite(PIN_LED_RGB_B, rr, gg, bb);
}

// Elige el color del LED segun la situacion real (se llama muy seguido, es barato).
void actualizarLed() {
  if (hayComandoPesado || ocupadoPesado) { ledColor(0, 0, 0); return; }   // calibrando o fijando cero: apagado
  if (!mpuOk)         { ledColor(255, 0, 0); return; }                    // error: el MPU no responde
  if (controlActivo)  { ledColor(160, 0, 255); return; }                  // control activo: morado
  if (etapaRobot <= 0)      ledColor(0, 0, 255);                          // azul: esperando la 1a pulsacion
  else if (etapaRobot == 1) ledColor(255, 160, 0);                        // amarillo: esperando la 2a
  else                      ledColor(0, 255, 0);                          // verde: esperando la 3a
}

// Que hace una pulsacion segun el paso en que va el arranque.
void pulsadorAccion() {
  if (hayComandoPesado || ocupadoPesado) return;     // esta calibrando o fijando el cero: ignora la pulsacion
  if (controlActivo) {                               // robot funcionando: la pulsacion es un PARO
    Out.println(">>> Pulsador: PARO");
    ejecutarComando("s");
    return;
  }
  if (!mpuOk) {                                      // error de sensor: reintenta calibrar
    Out.println(">>> Pulsador: el MPU6050 no responde, reintentando calibrar...");
    ejecutarComando("c");
    return;
  }
  if (etapaRobot <= 0) {
    Out.println(">>> Pulsador 1/3: recalibrando (mantenlo QUIETO)");
    ejecutarComando("c");
  } else if (etapaRobot == 1) {
    Out.println(">>> Pulsador 2/3: fijando el cero (sostenlo en su punto de equilibrio)");
    ejecutarComando("z");
  } else {
    Out.println(">>> Pulsador 3/3: control ACTIVADO");
    ejecutarComando("m");
  }
}

// Lee el pulsador (GPIO 40) con antirrebote, ejecuta la accion al PRESIONAR (no al soltar ni
// mientras se mantiene) y actualiza el LED.
void atenderPulsador() {
  static bool estadoEstable = false;           // true = pulsador presionado (ya filtrado)
  static bool lecturaPrevia = false;
  static unsigned long tCambio = 0;
  bool presionado = (digitalRead(PIN_PULSADOR) == (PULSADOR_ACTIVO_BAJO ? LOW : HIGH));
  if (presionado != lecturaPrevia) {           // la lectura cambio: reinicia el temporizador
    lecturaPrevia = presionado;
    tCambio = millis();
  }
  if ((millis() - tCambio) >= PULSADOR_ANTIREBOTE_MS && presionado != estadoEstable) {
    estadoEstable = presionado;
    if (estadoEstable) pulsadorAccion();
  }
  actualizarLed();
}

// ==============================================================================
// 2.8 TELEMETRIA (lo que se envia al HMI)
// Cada INTERVALO_MS imprime 3 lineas: (1) angulo, error, PID, u y PWM;
// (2) lecturas del MPU; (3) encoders, theta_sp de cada motor, ganancias de rpm
// y diagnostico. La pagina balancin_pid.html lee estas lineas.
// ==============================================================================

void imprimirEstado() {
  unsigned long ahora = millis();
  if (ahora - ultimaImpresion < INTERVALO_MS) return;
  ultimaImpresion = ahora;

  if (!mpuOk) {
    Out.println("Angulo: SIN SENSOR (revisa SDA/SCL)");
    return;
  }

  Out.print("Angulo: ");
  Out.print(anguloMedido, 2);
  Out.print(" | Error: ");
  Out.print(errorAngulo, 2);
  Out.print(" | de/dt: ");
  Out.print(derivadaError, 1);
  Out.print(" | Kp: ");
  Out.print(kp, 2);
  Out.print(" | Ki: ");
  Out.print(ki, 3);
  Out.print(" | Kd: ");
  Out.print(kd, 2);
  Out.print(" | I: ");
  Out.print(terminoI, 1);
  Out.print(" | Tol: ");
  Out.print(tolGrados, 2);
  Out.print(" | u: ");
  Out.print(salidaU, 2);
  Out.print(" | PWM: ");
  Out.print(controlActivo ? velocidadActual : 0);
  Out.print(" | Ts real: ");
  Out.print(tsReal);
  Out.println(controlActivo ? " ms [CONTROL]" : " ms");

  // Lo que lee el MPU6050 (acelerometro en g, giroscopio en grados/s)
  Out.print("   MPU -> Acel(g) X: ");
  Out.print(axRaw / ACCEL_SENS, 3);
  Out.print(" Y: ");
  Out.print(ayRaw / ACCEL_SENS, 3);
  Out.print(" Z: ");
  Out.print(azRaw / ACCEL_SENS, 3);
  Out.print(" | Giro(grados/s) X: ");
  Out.print((gxRaw - gyroOffsetX) / GYRO_SENS, 1);
  Out.print(" Y: ");
  Out.print((gyRaw - gyroOffsetY) / GYRO_SENS, 1);
  Out.print(" Z: ");
  Out.print(gzRaw / GYRO_SENS, 1);
  Out.print(" | Ang acel: ");
  Out.print(anguloDesdeAcelerometro(), 2);
  Out.print(" | Ang filtrado (sin cero): ");
  Out.println(anguloInclinacion, 2);

  // Encoders (cuentas acumuladas y cuentas/s por rueda) y lazo externo de velocidad
  int32_t cI, cD;
  leerCuentas(cI, cD);
  const char* uv = (CUENTAS_POR_REV_IZQ > 0 && CUENTAS_POR_REV_DER > 0) ? "rpm" : "c/s";
  Out.print("   Enc -> Izq: ");
  Out.print(cI);
  Out.print(" (");
  Out.print(vel[0], 0);
  Out.print(uv);
  Out.print(") Der: ");
  Out.print(cD);
  Out.print(" (");
  Out.print(vel[1], 0);
  Out.print(uv);
  Out.print(") | SP_theta Izq: ");
  Out.print(spTheta[0], 2);
  Out.print(" Der: ");
  Out.print(spTheta[1], 2);
  Out.print(" | u Izq: ");
  Out.print(uAng[0], 1);
  Out.print(" Der: ");
  Out.print(uAng[1], 1);
  Out.print(" | M1 Kp: ");
  Out.print(kvp[0], 6);
  Out.print(" Ki: ");
  Out.print(kvi[0], 6);
  Out.print(" Kd: ");
  Out.print(kvd[0], 6);
  Out.print(" | M2 Kp: ");
  Out.print(kvp[1], 6);
  Out.print(" Ki: ");
  Out.print(kvi[1], 6);
  Out.print(" Kd: ");
  Out.print(kvd[1], 6);
  Out.print(" | Tau: ");
  Out.print(tauFiltro, 2);
  Out.print(" | PWMmin A: ");
  Out.print(pwmMinA);
  Out.print(" B: ");
  Out.print(pwmMinB);
  Out.print(" | Diag fallosI2C: ");
  Out.print(fallosI2CTotal);
  Out.print(" Ts max: ");
  Out.print(tsMaxMs);
  Out.println(" ms");
  tsMaxMs = 0;
}

// ##############################################################################
// ARRANQUE DEL SISTEMA (UNE LAS DOS PARTES)
// setup(): configura motores, encoders, pulsador, WiFi y MPU, y crea las dos tareas:
//   control -> nucleo 1 (prioridad alta)      comunicaciones -> nucleo 0
// loop() queda vacio: todo corre en las tareas.
// ##############################################################################

void setup() {
  Serial.begin(115200);
  delay(1000);

  // ---- Motores ----
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  detenerMotores();
  iniciarEncoders();
  pinMode(PIN_PULSADOR, PULSADOR_ACTIVO_BAJO ? INPUT_PULLUP : INPUT);

  // ---- WiFi (antes del MPU: la espera no debe afectar la calibracion ni el filtro) ----
  conectarWiFi();

  // ---- MPU6050 ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);   // lectura del MPU ~4 veces mas rapida (si da errores, vuelve a 100000)
  Wire.setTimeOut(5);    // ms: evita que el bus se quede colgado esperando (el lazo es de 10 ms)
  escanearI2C();
  mpuOk = iniciarMPU();
  if (!mpuOk) {
    Out.println("!!! MPU6050 NO RESPONDE. Revisa:");
    Out.print("    SDA -> GPIO "); Out.println(SDA_PIN);
    Out.print("    SCL -> GPIO "); Out.println(SCL_PIN);
    Out.println("    VCC -> 3V3, GND -> GND, AD0 -> GND");
  }

  ultimoControlUs = micros();
  ultimaImpresion = millis();
  ultimoReintento = millis();

  // Motivo del ultimo reinicio: 1 = encendido, 3 = software, 4 = panic, 5/6/7 = watchdog, 9 = brownout
  Out.print("Motivo de reinicio (esp_reset_reason): ");
  Out.println((int)esp_reset_reason());
  Out.println("=== PID Paso 5 (PID) listo: c = recalibrar | z = fijar cero | kp<num> ki<num> kd<num> tol<num> | m = control ON/OFF | s = stop ===");

  vaciarSalida();   // muestra por Serial todo lo impreso durante setup

  // Control en el nucleo 1 (prioridad alta); WiFi, comandos e impresion en el nucleo 0
  xTaskCreatePinnedToCore(tareaControl, "control", 8192, NULL, 10, &hTareaControl, 1);
  xTaskCreatePinnedToCore(tareaComunicaciones, "comunic", 8192, NULL, 2, NULL, 0);
}

// loop() ya no hace nada: todo corre en las dos tareas
void loop() {
  vTaskDelete(NULL);
}
