/*
 * esqueleto_equilibrio.cpp — Cómo se arma un ciclo de 5 ms en SU firmware
 * Control Inteligente II (168418) · Universidad de Pamplona · Ing. Jeisson Martínez Flórez
 *
 * Esto es un ESQUELETO. Compila, pero no mueve el robot hasta que su grupo escriba las partes
 * marcadas con TODO, con el hardware y el código que ya tiene.
 *
 * Un ciclo, cada 5 ms (200 Hz), siempre en este orden:
 *   1. IMU → θ y θ̇
 *   2. Encoders → vueltas y RPM medida de cada rueda
 *   3. Seguridad y lazo de equilibrio → RPM deseada
 *   4. Lazo de RPM de cada rueda → PWM
 *   5. Copia para la telemetría
 *
 * El control corre en su propia tarea. loop() solo atiende comandos e imprime la telemetría,
 * para que imprimir nunca retrase el control.
 */

#include <Arduino.h>
#include <esp_arduino_version.h>
#include "soc/gpio_reg.h"
#include "soc/soc.h"
#include "config.h"             // pines, signos y cuentas por vuelta del Lab 1
#include "lazo_config.h"        // el que generó el notebook del Taller 2 para su grupo
#include "lazo_rpm_nucleo.h"    // ruedaL, ruedaR, disenar(), pasoLazo()
#include "estructura_estados.h" // Ganancias, Equilibrio, filtroComplementario(), pasoEquilibrio()

// ============================================================================
// 1. AJUSTES DEL GRUPO
// ============================================================================
#define SIGNO_THETA   -1    // que θ salga POSITIVO con el robot inclinado hacia ADELANTE
#define SIGNO_OMEGA   +1     // que θ̇ salga POSITIVA mientras se inclina hacia ADELANTE
#define TAU_FILTRO_S  0.5f   // constante de tiempo del filtro complementario
#define RPM_MAX       100.0f  // tope de la RPM deseada (deje margen bajo la RPM máxima del motor)

static Ganancias K = {64.32f, 9.31f, 3.66f, 147.49f}; // TODO: las de SU Taller 2 (unidades del firmware)
static float theta0 = 0.0f;                         // [°] ángulo de equilibrio (guía, paso B2)

// ============================================================================
// 2. HARDWARE (TODO: lo escribe cada grupo)
//    Si trabajan en src/ del repositorio, peguen aquí las secciones 1 (ENCODERS) y
//    2 (MOTORES) de control_rpm.cpp, con los struct Encoder y Motor y las variables
//    encL, encR, motL y motR. NO copien Recta, Rueda, ruedaL ni ruedaR: esos vienen
//    de lazo_rpm_nucleo.h. Vea la sección 4.4 de la guía.
// ============================================================================
// thetaAcc: ángulo de inclinación que da el ACELERÓMETRO [°] (lee la gravedad).
// omega:    velocidad de inclinación que da el GIROSCOPIO [°/s], menos el sesgo.
// Los dos sin corregir el signo: eso lo hacen SIGNO_THETA y SIGNO_OMEGA.


struct Encoder {
  int pinA, pinB;              // pinB = -1: modo de un canal
  int8_t signo;                // corrige el sentido de conteo
  float cuentasVuelta;         // cuentas por vuelta de rueda
  volatile int32_t cuentas;    // contador crudo (lo escribe solo la interrupción)
  volatile uint32_t flancosA, flancosB, errores;
  volatile uint8_t estado;     // último estado (A<<1)|B
  volatile int8_t sentido;     // modo de un canal: sentido del último PWM aplicado
  int32_t cero;                // valor crudo en el último "z"
};

struct Motor {
  int pinPwm, pinIn1, pinIn2;
  int8_t signo;
  uint8_t canal;               // canal LEDC (solo núcleo 2.x)
  Encoder *enc;
};


Encoder encL = {ENC_L_A, ENC_L_B, ENC_L_SIGNO, CUENTAS_VUELTA_L, 0, 0, 0, 0, 0, 1, 0};
Encoder encR = {ENC_R_A, ENC_R_B, ENC_R_SIGNO, CUENTAS_VUELTA_R, 0, 0, 0, 0, 0, 1, 0};
Motor motL = {MOTOR_L_PWM, MOTOR_L_IN1, MOTOR_L_IN2, MOTOR_L_SIGNO, 0, &encL};
Motor motR = {MOTOR_R_PWM, MOTOR_R_IN1, MOTOR_R_IN2, MOTOR_R_SIGNO, 1, &encR};


// ============================================================================
// 1. ENCODERS
// ============================================================================
// Un encoder en cuadratura tiene dos señales, A y B, desfasadas un cuarto de ciclo.
// El estado es el par (A, B): 00 → 01 → 11 → 10 → 00 en un sentido y al revés en el otro.
// Cada cambio de A o de B dispara una interrupción. La tabla dice cuánto sumar según el
// estado anterior y el actual: +1, −1, 0 (sin cambio) o 2 (salto imposible = ruido).
// Así se cuentan los cuatro flancos de cada ciclo (decodificación x4).
//
//   índice = estado_anterior·4 + estado_actual
static DRAM_ATTR const int8_t TABLA_CUADRATURA[16] = {
  //  actual: 00   01   10   11
  /* 00 */     0,  +1,  -1,   2,
  /* 01 */    -1,   0,   2,  +1,
  /* 10 */    +1,   2,   0,  -1,
  /* 11 */     2,  -1,  +1,   0,
};

// Lectura directa del registro de entradas: más rápida que digitalRead dentro de la interrupción.
static inline __attribute__((always_inline)) int leerPin(int pin)
{
  if (pin < 32)
    return (REG_READ(GPIO_IN_REG) >> pin) & 1;
  return (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

static void IRAM_ATTR pasoEncoder(Encoder &e)
{
  if (e.pinB < 0)
  {
    // Un solo canal: cada flanco de A vale 2 cuentas (x2) y el sentido lo da el PWM.
    e.cuentas += 2 * e.sentido;
    e.flancosA++;
    return;
  }
  const uint8_t actual = (leerPin(e.pinA) << 1) | leerPin(e.pinB);
  const uint8_t anterior = e.estado;
  if ((actual ^ anterior) & 0b10)
    e.flancosA++;
  if ((actual ^ anterior) & 0b01)
    e.flancosB++;
  const int8_t paso = TABLA_CUADRATURA[(anterior << 2) | actual];
  if (paso == 2)
    e.errores++;
  else
    e.cuentas += paso;
  e.estado = actual;
}

void IRAM_ATTR isrEncoderL() { pasoEncoder(encL); }
void IRAM_ATTR isrEncoderR() { pasoEncoder(encR); }

static void iniciarEncoder(Encoder &e, void (*isr)())
{
  pinMode(e.pinA, INPUT_PULLUP);
  if (e.pinB >= 0)
    pinMode(e.pinB, INPUT_PULLUP);
  e.estado = (leerPin(e.pinA) << 1) | (e.pinB >= 0 ? leerPin(e.pinB) : 0);
  attachInterrupt(digitalPinToInterrupt(e.pinA), isr, CHANGE);
  if (e.pinB >= 0)
    attachInterrupt(digitalPinToInterrupt(e.pinB), isr, CHANGE);
}

// Cuentas desde el último "z", con el signo corregido (avanzar = positivo).
static int32_t leerCuentas(const Encoder &e)
{
  const int8_t s = (e.pinB < 0) ? 1 : e.signo; // en un canal el sentido ya viene del PWM
  return s * (e.cuentas - e.cero);
}

static void ponerCero(Encoder &e)
{
  e.cero = e.cuentas;
  e.flancosA = e.flancosB = e.errores = 0;
}

// ============================================================================
// 2. MOTORES — TB6612FNG
// ============================================================================
// IN1/IN2 fijan el sentido y el PWM la magnitud. Con PWM = 0 y el driver habilitado el
// TB6612 frena en corto (el motor se frena con su propia fuerza contraelectromotriz).

// El generador de PWM (LEDC) cambió de interfaz en el núcleo 3.x.
static void pwmIniciar(Motor &m)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(m.pinPwm, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(m.canal, PWM_FREQ_HZ, PWM_BITS);
  ledcAttachPin(m.pinPwm, m.canal);
#endif
}

static void pwmEscribir(Motor &m, uint32_t duty)
{
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(m.pinPwm, duty);
#else
  ledcWrite(m.canal, duty);
#endif
}

static void iniciarMotor(Motor &m)
{
  pinMode(m.pinIn1, OUTPUT);
  pinMode(m.pinIn2, OUTPUT);
  digitalWrite(m.pinIn1, LOW);
  digitalWrite(m.pinIn2, LOW);
  pwmIniciar(m);
  pwmEscribir(m, 0);
}

// u en [-PWM_MAX, PWM_MAX]; positivo = adelante (después de corregir con MOTOR_x_SIGNO).
// casiQuieta: en el lazo, la rueda gira a menos de 2 RPM. El encoder de un canal toma el sentido
// del PWM, pero solo lo cambia con la rueda casi detenida: si el PWM cambia de signo mientras la
// rueda todavía gira en el sentido anterior, esas cuentas siguen siendo del sentido anterior.
static void aplicarPWM(Motor &m, int u, bool casiQuieta = true)
{
  u = constrain(u, -PWM_MAX, PWM_MAX);
  if (u != 0 && casiQuieta)
    m.enc->sentido = (u > 0) ? 1 : -1; // lo usa el encoder de un canal
  const int v = u * m.signo;           // sentido eléctrico
  if (v > 0)
  {
    digitalWrite(m.pinIn1, HIGH);
    digitalWrite(m.pinIn2, LOW);
  }
  else if (v < 0)
  {
    digitalWrite(m.pinIn1, LOW);
    digitalWrite(m.pinIn2, HIGH);
  }
  pwmEscribir(m, abs(v));
}

static void habilitarDriver(bool on)
{
  if (PIN_STBY >= 0)
    digitalWrite(PIN_STBY, on ? HIGH : LOW);
}

static void detenerMotores()
{
  aplicarPWM(motL, 0);
  aplicarPWM(motR, 0);
  habilitarDriver(false);
}



#include <Wire.h>
static float sesgoGy = 0;  

static bool leerIMU(float &thetaAcc, float &omega)
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                    // desde ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, 14) != 14)
    return false;
  int16_t d[7];                                        // ax, ay, az, temperatura, gx, gy, gz
  for (int i = 0; i < 7; i++)
    d[i] = (Wire.read() << 8) | Wire.read();
  thetaAcc = atan2f(d[0], d[2]) * RAD_TO_DEG;          // atan2(a_x, a_z) en grados
  omega = d[5] / 131.0f - sesgoGy;                     // gy en °/s (escala de ±250 °/s)
  return true;
}

// Vueltas de cada rueda desde que arrancó el programa, positivas al avanzar.
// En src/:  vueltasL = leerCuentas(encL) / encL.cuentasVuelta;  (igual con R)
static void leerVueltas(float &vueltasL, float &vueltasR)
{
  vueltasL = leerCuentas(encL) / encL.cuentasVuelta;   // cuentas → vueltas, positivas al avanzar
  vueltasR = leerCuentas(encR) / encR.cuentasVuelta;
}

static void moverMotores(int uL, int uR)
{
  aplicarPWM(motL, uL);                                // el signo de cada motor ya está corregido en el Lab 1
  aplicarPWM(motR, uR);
}

// ============================================================================
// 3. ESTADO COMPARTIDO ENTRE LA TAREA DE CONTROL Y loop()
// ============================================================================
enum Modo { PARADO, PRUEBA_RPM, EQUILIBRIO };
static volatile Modo modo = PARADO;
static volatile float rpmPrueba = 0; // consigna fija en el modo PRUEBA_RPM

struct Telemetria
{
  uint32_t t_ms;
  float theta, omega, rpmRef, yL, yR, x;
  int uL, uR;
  uint32_t ciclo_us; // lo que tardó el ciclo
};
static Telemetria tele;
static portMUX_TYPE muxTele = portMUX_INITIALIZER_UNLOCKED;

static Equilibrio eq;
static bool formatoCSV = true; // true = CSV con comas (por defecto, para los logs), false = tabla con "|"
static void imprimirEncabezado();

// ============================================================================
// 4. EL CICLO DE CONTROL (cada 5 ms)
// ============================================================================
static void tareaControl(void *)
{
  const float dt = TS_LAZO_US * 1e-6f;
  const int nV = VENTANA_MS * 1000 / TS_LAZO_US; // muestras de la ventana de RPM (2)
  float histL[8] = {0}, histR[8] = {0};          // vueltas de los últimos ciclos
  int k = 0;
  float theta = 0;
  TickType_t t0 = xTaskGetTickCount();
  for (;;)
  {
    vTaskDelayUntil(&t0, pdMS_TO_TICKS(TS_LAZO_US / 1000));
    const uint32_t inicio = micros();

    // 1. IMU → θ [°] y θ̇ [°/s], con la convención de signos del curso
    float thetaAcc, omega;
    const bool imuOk = leerIMU(thetaAcc, omega);
    omega *= SIGNO_OMEGA;
    theta = filtroComplementario(theta, SIGNO_THETA * thetaAcc, omega, dt, TAU_FILTRO_S);
    const float th = theta - theta0;

    // 2. Encoders → vueltas y RPM medida (cuentas de los últimos 10 ms)
    float vL, vR;
    leerVueltas(vL, vR);
    histL[k % 8] = vL;
    histR[k % 8] = vR;
    float yL = 0, yR = 0;
    if (k >= nV)
    {
      yL = (vL - histL[(k - nV) % 8]) / (nV * dt) * 60.0f;
      yR = (vR - histR[(k - nV) % 8]) / (nV * dt) * 60.0f;
    }
    k++;

    // 3. Consigna de RPM según el modo (r = 0 frena y deja los integradores en cero)
    float r = 0;
    if (modo != EQUILIBRIO && eq.armado)
      desarmar(eq);
    if (modo == PRUEBA_RPM)
      r = rpmPrueba;
    else if (modo == EQUILIBRIO && imuOk && revisarSeguridad(eq, th, vL, vR))
      r = pasoEquilibrio(eq, K, th, omega, vL, vR, dt, RPM_MAX);

    // 4. Lazo de RPM de cada rueda → PWM
    const int uL = (int)lroundf(pasoLazo(ruedaL, r, yL, dt));
    const int uR = (int)lroundf(pasoLazo(ruedaR, r, yR, dt));
    moverMotores(uL, uR);

    // 5. Telemetría
    portENTER_CRITICAL(&muxTele);
    const float xTele = eq.armado ? eq.x : 0.5f * (vL + vR); // sin armar: vueltas desde el arranque
    tele = {millis(), th, omega, r, yL, yR, xTele, uL, uR, micros() - inicio};
    portEXIT_CRITICAL(&muxTele);
  }
}

// ============================================================================
// 5. COMANDOS Y TELEMETRÍA (en loop, nunca dentro del ciclo de control)
// ============================================================================
//   e         equilibrio (se enciende solo cuando |θ| < 5°)
//   p         parar
//   r 20      prueba de los lazos de RPM con el robot en el soporte: las dos ruedas a 20 RPM
//   k Kθ Kω Kv Kx   cambiar ganancias, con el robot parado.  Ej.: k 80.8 10.9 3.21 129.4
//   o 1.5     θ0 en grados, con el robot parado
//   t         alternar la salida: tabla con "|" (para leer) o CSV con comas (para guardar y analizar)
static void procesar(String s)
{
  s.trim();
  if (s.length() == 0)
    return;
  float a, b, c, d;
  switch (s[0])
  {
  case 'e': modo = EQUILIBRIO; Serial.println("Equilibrio: se enciende con |θ| < 5°"); break;
  case 'p': modo = PARADO; Serial.println("Parado"); break;
  case 'r':
    if (sscanf(s.c_str() + 1, "%f", &a) == 1) { rpmPrueba = a; modo = PRUEBA_RPM; }
    break;
  case 'k':
    if (modo == PARADO && sscanf(s.c_str() + 1, "%f %f %f %f", &a, &b, &c, &d) == 4) K = {a, b, c, d};
    Serial.printf("K = %.2f %.2f %.3f %.2f\n", K.kTheta, K.kOmega, K.kV, K.kX);
    break;
  case 'o':
    if (modo == PARADO && sscanf(s.c_str() + 1, "%f", &a) == 1) theta0 = a;
    Serial.printf("θ0 = %.2f°\n", theta0);
    break;
  case 't':
    formatoCSV = !formatoCSV;
    Serial.println(formatoCSV ? "Formato: CSV" : "Formato: tabla");
    imprimirEncabezado();
    break;
  }
}

// Tabla de telemetría: columnas de ancho fijo, centradas y separadas por "|".
#define ANCHO_COL 9
#define N_COL 10

static void imprimirFila(const char *const campos[N_COL])
{
  char buf[N_COL * (ANCHO_COL + 1) + 1];
  int n = 0;
  for (int i = 0; i < N_COL; i++)
  {
    const int len = strlen(campos[i]);
    const int izq = (ANCHO_COL - len) / 2;
    n += snprintf(buf + n, sizeof(buf) - n, "%*s%s%*s%c", izq, "", campos[i], ANCHO_COL - len - izq, "",
                  i < N_COL - 1 ? '|' : '\0');
  }
  Serial.println(buf);
}

static void imprimirSeparador()
{
  for (int i = 0; i < N_COL * (ANCHO_COL + 1) - 1; i++)
    Serial.print('-');
  Serial.println();
}

static void imprimirEncabezado()
{
  static const char *const nombres[N_COL] = {"t_ms", "theta", "omega", "rpmRef", "yL", "yR", "uL", "uR", "x", "ciclo_us"};
  if (formatoCSV)
  {
    for (int i = 0; i < N_COL; i++)
    {
      Serial.print(nombres[i]);
      Serial.print(i < N_COL - 1 ? ',' : '\n');
    }
    return;
  }
  imprimirSeparador();
  imprimirFila(nombres);
  imprimirSeparador();
}

void setup()
{
  Serial.begin(115200);

  // IMU: arrancar la MPU6050 y medir el sesgo del giroscopio
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);              // I2C a 400 kHz
  // despertar la MPU6050 (0x6B = 0) y poner su filtro interno en unos 44 Hz (0x1A = 3)
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(3); Wire.endTransmission();
  delay(100);
  float suma = 0, a, w;
  int buenas = 0;
  for (int i = 0; i < 500; i++) { if (leerIMU(a, w)) { suma += w; buenas++; } delay(2); }
  if (buenas < 400)
    Serial.printf("ERROR: la IMU no responde (%d/500 lecturas). Revise SDA=%d SCL=%d.\n", buenas, SDA_PIN, SCL_PIN);
  sesgoGy = buenas ? suma / buenas : 0; // el robot NO se mueve durante este segundo

  // Encoders y motores: las mismas funciones de control_rpm.cpp
  if (PIN_STBY >= 0) pinMode(PIN_STBY, OUTPUT);
  iniciarMotor(motL);  iniciarMotor(motR);
  iniciarEncoder(encL, isrEncoderL);  iniciarEncoder(encR, isrEncoderR);
  habilitarDriver(true);

  // Esto ya viene en el esqueleto: no lo cambien
  disenar();                          // Kp y Ki de cada rueda desde el modelo
  xTaskCreatePinnedToCore(tareaControl, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 1);
  imprimirEncabezado();
}

void loop()
{
  static String linea;
  while (Serial.available())
  {
    const char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { procesar(linea); linea = ""; }
    else if (linea.length() < 64) linea += ch;
  }
  static uint32_t ultima = 0;
  if (millis() - ultima >= 20) // 50 Hz
  {
    ultima = millis();
    Telemetria t;
    portENTER_CRITICAL(&muxTele);
    t = tele;
    portEXIT_CRITICAL(&muxTele);
    char c[N_COL][16];
    snprintf(c[0], 16, "%lu", (unsigned long)t.t_ms);
    snprintf(c[1], 16, "%.2f", t.theta);
    snprintf(c[2], 16, "%.1f", t.omega);
    snprintf(c[3], 16, "%.1f", t.rpmRef);
    snprintf(c[4], 16, "%.1f", t.yL);
    snprintf(c[5], 16, "%.1f", t.yR);
    snprintf(c[6], 16, "%d", t.uL);
    snprintf(c[7], 16, "%d", t.uR);
    snprintf(c[8], 16, "%.3f", t.x);
    snprintf(c[9], 16, "%lu", (unsigned long)t.ciclo_us);
    const char *campos[N_COL];
    for (int i = 0; i < N_COL; i++)
      campos[i] = c[i];
    if (formatoCSV)
    {
      for (int i = 0; i < N_COL; i++)
      {
        Serial.print(campos[i]);
        Serial.print(i < N_COL - 1 ? ',' : '\n');
      }
    }
    else
      imprimirFila(campos);
  }
}
