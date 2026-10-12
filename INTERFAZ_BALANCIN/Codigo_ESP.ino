/*
  ============================================================================
  BALANCIN - Control Inteligente 2
  PID PASO 4: agrega la DERIVADA  ->  u = Kp*e + Kd*de/dt  (PD)
  Placa: ESP32-S3 (Arduino core 3.x)   |   MPU6050 + motores (sin encoders)
  ============================================================================

  Lazo interno de tu pizarra (primer bloque):

      theta_sp (0 grados) --> (+) --> e = theta_sp - theta_med
                               ^(-)
      theta_med (MPU6050) -----'

      e --> [ Kp*e + Kd*de/dt ] --> u --> [ zona muerta / limite ] --> PWM --> puente H --> motores
                          signo de u = sentido del giro
      de/dt = (e_k - e_k-1) / Ts      (Kp y Kd; falta Ki, viene en el paso 5)

  USO (Monitor Serial a 115200, con "Nueva linea" activado):
    - 'c' -> recalibra el MPU6050 (balancin quieto ~2 s)
    - 'z' -> fija el cero: la posicion actual pasa a marcar 0 grados
    - 'kp' + numero -> cambia Kp en vivo. Ej: kp10      (tambien vale k10)
    - 'kd' + numero -> cambia Kd en vivo. Ej: kd0.5
      (la derivada frena el balanceo: mide que tan rapido cambia el error)
    - 'm' -> ACTIVA/DESACTIVA el control (arranca DESACTIVADO por seguridad)
    - 's' -> desactiva el control y detiene los motores

  PRUEBA SEGURA: primero con las RUEDAS EN EL AIRE. Activa con 'm', inclina el
  balancin y mira que las ruedas giren hacia donde se inclina. Si el balancin
  pasa de CORTE_GRADOS el control se apaga solo.

  Salida cada 200 ms:  Angulo | Error | de/dt | Kp | Kd | u | PWM | Ts real

  QUE DEBES VER:
    Angulo +5  ->  Error -5  ->  u = Kp*(-5)  (negativo)
    Angulo -5  ->  Error +5  ->  u = Kp*(+5)  (positivo)
    u > 0 -> motores ADELANTE | u < 0 -> motores REVERSA (signo = direccion).
    Inclinar el balancin hacia adelante debe girar las ruedas hacia adelante.
    Ts real    ->  cerca de 10 ms (periodo de muestreo fijo)

  ============================================================================
  TABLA DE CONTENIDO
  ============================================================================
  Secciones: 1. DEPENDENCIAS  2. CONFIGURACION  3. VARIABLES GLOBALES
             4. FUNCIONES PRINCIPALES  5. FUNCIONES DE APOYO

  Nombramiento de variables:
    MAYUSCULAS        constantes y pines            TS_MS, ANGULO_SP
    camelCase         variables globales            errorAngulo
    sufijo Raw        dato crudo del sensor         axRaw
    sufijo Offset     correccion de calibracion     gyroOffsetY
    prefijo angulo... valores en grados             anguloCero

  Variables globales principales:
    anguloInclinacion   angulo filtrado sin compensar (grados)
    anguloCero          lo que marca el sensor en equilibrio (grados)
    anguloSetpoint      theta_sp, referencia (grados)
    anguloMedido        theta_med, angulo compensado (grados)
    errorAngulo         e = theta_sp - theta_med (grados)
    kp / kd             ganancias proporcional y derivativa (ajustables en vivo)
    errorPrevio         e_k-1, error de la muestra anterior
    derivadaError       de/dt filtrada (grados/segundo)
    salidaU             u = Kp * e (unidades de PWM, aun sin limitar)
    controlActivo       true si el lazo esta moviendo los motores
    modoActual          ADELANTE o REVERSA (sentido segun signo de u)
    velocidadActual     PWM aplicado 0-255
    tsReal              periodo real medido entre ejecuciones del lazo (ms)

  Funciones principales:
    setup(), loop(), procesarComandos(), atenderMPU(),
    lazoControl()  (a periodo fijo TS_MS: error, derivada, u = Kp*e + Kd*de/dt, PWM),
    imprimirEstado()

  Funciones de apoyo (motores):
    motorA(), motorB(), aplicarMovimiento(), detenerMotores(),
    aplicarSalida()  (u -> sentido + PWM con zona muerta y limite)

  Funciones de apoyo (MPU6050):
    mpuEscribir(), mpuLeerCrudo(), escanearI2C(), calibrarGiroscopio(),
    anguloDesdeAcelerometro(), iniciarMPU(), ponerCeroActual(),
    actualizarAngulo(), anguloFinal()
  ============================================================================
*/

// ============================================================================
// 1. DEPENDENCIAS
// ============================================================================
#include <Wire.h>   // I2C para el MPU6050

// ============================================================================
// 2. CONFIGURACION
// ============================================================================

// ---------- Pines del TB6612FNG (STBY fijo a 3.3V) ----------
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define BIN1 12
#define BIN2 13
#define PWMB 14

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

// Cero inicial. Con 'z' lo fijas; para guardarlo al reiniciar, copia aqui
// el valor que imprime 'z'.
#define ANGULO_CERO 0.0

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
const float ALPHA      = 0.98;    // filtro complementario

// ---------- Lazo de control ----------
const unsigned long TS_MS = 10;   // periodo de muestreo (Tsample) = 100 Hz
const float ANGULO_SP = 0.0;      // theta_sp: balancin vertical
// u esta en unidades de PWM y el error en grados: con Kp = 8, un error de 30
// grados da u = 240 (casi PWM maximo). Ajustalo en vivo con 'k'.
const float KP_INICIAL = 10.0;    // ganancia proporcional (la que elegiste)
const float KD_INICIAL = 0.0;     // ganancia derivativa: arranca en 0 (= solo P); subela con 'kd'
const float ALFA_D = 0.3;         // filtro de la derivada: peso de la muestra nueva (0-1). Menor = mas suave

// ---------- Salida a motores ----------
const float ZONA_MUERTA_U = 2.0;  // si |u| es menor, motores detenidos (evita temblor)
// PWM minimo con el que arranca cada motor (compensacion de zona muerta).
// Medidos a mano: el motor izquierdo arranca con 10 y el derecho con 19.
// Se asume Motor A = izquierdo y Motor B = derecho; si es al reves, intercambia los valores.
// Con el balancin en el suelo (con peso) suelen subir: vuelve a medirlos asi.
const int   PWM_MIN_A = 10;       // Motor A (izquierdo)
const int   PWM_MIN_B = 19;       // Motor B (derecho)
const int   PWM_MAX = 255;
const float CORTE_GRADOS = 45.0;  // si se inclina mas que esto, se apaga el control

// ---------- Tiempos ----------
const unsigned long INTERVALO_MS = 200;       // periodo de impresion
const unsigned long REINTENTO_MPU_MS = 3000;
const int MAX_FALLOS_MPU = 5;     // lecturas seguidas fallidas antes de dar el MPU por perdido

// ============================================================================
// 3. VARIABLES GLOBALES
// ============================================================================

// ---------- MPU6050 ----------
int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
float gyroOffsetX = 0, gyroOffsetY = 0;
float anguloInclinacion = 0;      // grados (sin compensar)
float anguloCero = ANGULO_CERO;   // lo que marca el sensor en equilibrio
unsigned long ultimoTiempoMPU = 0;
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
float kd = KD_INICIAL;            // ganancia derivativa
float errorPrevio = 0;            // e_k-1
float derivadaError = 0;          // de/dt filtrada (grados/s)
bool primeraMuestra = true;       // evita un pico de derivada al arrancar o tras fijar el cero
float salidaU = 0;                // u = Kp * e
unsigned long ultimoControl = 0;
unsigned long tsReal = 0;         // ms reales entre dos ejecuciones del lazo

unsigned long ultimaImpresion = 0;

// ---------- Prototipos ----------
void procesarComandos();
void atenderMPU();
void lazoControl();
void aplicarSalida(float u);
void motorA(bool dirForward, int pwm);
void motorB(bool dirForward, int pwm);
void aplicarMovimiento();
void detenerMotores();
void imprimirEstado();
bool mpuEscribir(uint8_t registro, uint8_t valor);
bool mpuLeerCrudo();
void escanearI2C();
bool calibrarGiroscopio();
float anguloDesdeAcelerometro();
bool iniciarMPU();
bool ponerCeroActual();
bool actualizarAngulo();
float anguloFinal();

// ============================================================================
// 4. FUNCIONES PRINCIPALES
// ============================================================================

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

  // ---- MPU6050 ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);   // ms: evita que el bus se quede colgado esperando
  escanearI2C();
  mpuOk = iniciarMPU();
  if (!mpuOk) {
    Serial.println("!!! MPU6050 NO RESPONDE. Revisa:");
    Serial.print("    SDA -> GPIO "); Serial.println(SDA_PIN);
    Serial.print("    SCL -> GPIO "); Serial.println(SCL_PIN);
    Serial.println("    VCC -> 3V3, GND -> GND, AD0 -> GND");
  }

  ultimoControl = millis();
  ultimaImpresion = millis();
  ultimoReintento = millis();

  Serial.println("=== PID Paso 4 (PD) listo: c = recalibrar | z = fijar cero | kp<num> kd<num> | m = control ON/OFF | s = stop ===");
}

void loop() {
  procesarComandos();
  atenderMPU();
  lazoControl();
  imprimirEstado();
}

// Lee una linea del Monitor Serial y ejecuta el comando
void procesarComandos() {
  if (Serial.available() <= 0) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();

  if (entrada.equalsIgnoreCase("c")) {
    controlActivo = false;
    detenerMotores();
    primeraMuestra = true;
    Serial.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    Serial.println(mpuOk ? ">>> Recalibracion completa"
                         : "!!! No se pudo recalibrar: el MPU6050 no responde");
  } else if (entrada.equalsIgnoreCase("z")) {
    primeraMuestra = true;
    if (mpuOk && ponerCeroActual()) {
      Serial.print(">>> CERO fijado. El sensor marcaba ");
      Serial.print(anguloCero, 2);
      Serial.println(" grados en equilibrio (ahora marca 0).");
      Serial.print("    Para guardarlo al reiniciar: #define ANGULO_CERO ");
      Serial.println(anguloCero, 2);
    } else {
      Serial.println("!!! No se pudo fijar el cero: el MPU6050 no responde");
    }
  } else if (entrada.equalsIgnoreCase("m")) {
    if (!mpuOk) {
      Serial.println(">>> No se puede: el MPU6050 no responde");
    } else {
      controlActivo = !controlActivo;
      primeraMuestra = true;
      if (!controlActivo) detenerMotores();
      Serial.println(controlActivo ? ">>> CONTROL ACTIVADO" : ">>> Control desactivado");
    }
  } else if (entrada.equalsIgnoreCase("s")) {
    controlActivo = false;
    detenerMotores();
    Serial.println(">>> Detenido");
  } else if (entrada.length() > 1 && (entrada[0] == 'k' || entrada[0] == 'K')) {
    // Cambiar ganancias en vivo. Acepta: kp10  kp=10  k10  kd0.5  kd=0.5
    char cual = (char)tolower(entrada[1]);          // 'd' = Kd, cualquier otra = Kp
    int i = 1;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.' && entrada[i] != '-') i++;
    if (i < (int)entrada.length()) {
      float valor = entrada.substring(i).toFloat();
      if (cual == 'd') {
        kd = valor;
        Serial.print(">>> Kd = ");
        Serial.println(kd, 3);
      } else {
        kp = valor;
        Serial.print(">>> Kp = ");
        Serial.println(kp, 3);
      }
    } else {
      Serial.println("!!! Escribe kp o kd seguido de un numero. Ej: kp10  kd0.5");
    }
  }
}

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
      detenerMotores();
      Serial.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Serial.println(">>> MPU6050 reconectado");
  }
}

// LAZO INTERNO (paso 4, PD): se ejecuta solo cuando pasaron TS_MS (periodo de muestreo fijo)
//   if (tiempoActual - tiempoUltimo >= Ts) { ... }
void lazoControl() {
  unsigned long ahora = millis();
  if (ahora - ultimoControl < TS_MS) return;

  tsReal = ahora - ultimoControl;
  ultimoControl = ahora;

  if (!mpuOk) return;

  anguloMedido = anguloFinal();                // theta_med (del MPU)
  errorAngulo  = anguloSetpoint - anguloMedido; // e = theta_sp - theta_med

  // PASO 4: derivada  de/dt = (e_k - e_k-1) / Ts   (grados/segundo)
  float dt = tsReal / 1000.0;
  float derivadaCruda = primeraMuestra ? 0.0 : (errorAngulo - errorPrevio) / dt;
  derivadaError = ALFA_D * derivadaCruda + (1.0 - ALFA_D) * derivadaError; // filtro suave
  errorPrevio = errorAngulo;
  primeraMuestra = false;

  salidaU = kp * errorAngulo + kd * derivadaError; // u = Kp*e + Kd*de/dt

  // PASO 3: u -> sentido + PWM -> motores
  if (controlActivo) {
    if (fabs(anguloMedido) > CORTE_GRADOS) {    // el balancin cayo: apagar
      controlActivo = false;
      detenerMotores();
      Serial.println("!!! Angulo fuera de rango: control desactivado");
    } else {
      aplicarSalida(salidaU);
    }
  }
}

// u -> motores. Signo de u = sentido; |u| = magnitud del PWM.
//   u > 0 -> ADELANTE      u < 0 -> REVERSA
//   |u| < ZONA_MUERTA_U    -> detenido
//   si no: PWM de cada motor = su PWM_MIN + |u| reescalado hasta PWM_MAX
//          (cada motor tiene su propio minimo, asi arrancan a la vez)
void aplicarSalida(float u) {
  float mag = fabs(u);

  if (mag < ZONA_MUERTA_U) {
    detenerMotores();
    return;
  }

  mag = constrain(mag, 0.0f, (float)PWM_MAX);
  int pwmA = PWM_MIN_A + (int)(mag * (PWM_MAX - PWM_MIN_A) / PWM_MAX);
  int pwmB = PWM_MIN_B + (int)(mag * (PWM_MAX - PWM_MIN_B) / PWM_MAX);
  velocidadActual = (pwmA + pwmB) / 2;          // solo para imprimir
  modoActual = (u > 0) ? ADELANTE : REVERSA;

  bool adelante = (modoActual == REVERSA);      // misma inversion que aplicarMovimiento()
  motorA(adelante, pwmA);
  motorB(adelante, pwmB);
}

void imprimirEstado() {
  unsigned long ahora = millis();
  if (ahora - ultimaImpresion < INTERVALO_MS) return;
  ultimaImpresion = ahora;

  if (!mpuOk) {
    Serial.println("Angulo: SIN SENSOR (revisa SDA/SCL)");
    return;
  }

  Serial.print("Angulo: ");
  Serial.print(anguloMedido, 2);
  Serial.print(" | Error: ");
  Serial.print(errorAngulo, 2);
  Serial.print(" | de/dt: ");
  Serial.print(derivadaError, 1);
  Serial.print(" | Kp: ");
  Serial.print(kp, 2);
  Serial.print(" | Kd: ");
  Serial.print(kd, 2);
  Serial.print(" | u: ");
  Serial.print(salidaU, 2);
  Serial.print(" | PWM: ");
  Serial.print(controlActivo ? velocidadActual : 0);
  Serial.print(" | Ts real: ");
  Serial.print(tsReal);
  Serial.println(controlActivo ? " ms [CONTROL]" : " ms");
}

// ============================================================================
// 5. FUNCIONES DE APOYO
// ============================================================================

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

// ---------- MPU6050 ----------

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
  Serial.println("Escaneando bus I2C...");
  int encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Dispositivo en 0x");
      if (dir < 16) Serial.print("0");
      Serial.println(dir, HEX);
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Serial.println("  No se encontro NINGUN dispositivo I2C.");
  }
}

bool calibrarGiroscopio() {
  Serial.println("Calibrando giroscopio, no muevas el sensor...");
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
  Serial.println("Calibracion lista.");
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
    Serial.print("WHO_AM_I = 0x");
    Serial.println(Wire.read(), HEX); // esperado: 0x68
  }

  if (!calibrarGiroscopio()) return false;

  // Arranca el filtro desde el angulo real del acelerometro (no desde 0)
  if (!mpuLeerCrudo()) return false;
  anguloInclinacion = anguloDesdeAcelerometro();
  ultimoTiempoMPU = millis();
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
  ultimoTiempoMPU = millis();
  return true;
}

// Filtro complementario: giroscopio (rapido) + acelerometro (sin deriva)
bool actualizarAngulo() {
  if (!mpuLeerCrudo()) return false;

  float anguloAccel = anguloDesdeAcelerometro();
#if INCLINA_SOBRE_EJE_Y
  float velGiro = (gyRaw - gyroOffsetY) / GYRO_SENS;
#else
  float velGiro = (gxRaw - gyroOffsetX) / GYRO_SENS;
#endif

  unsigned long ahora = millis();
  float dt = (ahora - ultimoTiempoMPU) / 1000.0;
  ultimoTiempoMPU = ahora;

  anguloInclinacion = ALPHA * (anguloInclinacion + velGiro * dt) + (1 - ALPHA) * anguloAccel;
  return true;
}

// Angulo ya compensado (cero aplicado) y con el signo final
float anguloFinal() {
  float a = anguloInclinacion - anguloCero;
  return INVERTIR_ANGULO ? -a : a;
}
