/*
  ============================================================================
  BALANCIN - Control Inteligente 2
  PID PASO 1: muestreo fijo (Ts) + calculo del error
  Placa: ESP32-S3 (Arduino core 3.x)   |   Solo MPU6050, SIN motores
  ============================================================================

  Lazo interno de tu pizarra (primer bloque):

      theta_sp (0 grados) --> (+) --> e = theta_sp - theta_med
                               ^(-)
      theta_med (MPU6050) -----'

  USO (Monitor Serial a 115200, con "Nueva linea" activado):
    - 'c' -> recalibra el MPU6050 (balancin quieto ~2 s)
    - 'z' -> fija el cero: la posicion actual pasa a marcar 0 grados

  Salida cada 200 ms:  Angulo (theta_med) | Setpoint | Error | Ts real

  QUE DEBES VER:
    Angulo +5  ->  Error -5      (si marca +5 grados, hay que corregir -5)
    Angulo -5  ->  Error +5
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
    tsReal              periodo real medido entre ejecuciones del lazo (ms)

  Funciones principales:
    setup(), loop(), procesarComandos(), atenderMPU(),
    lazoControl()  (a periodo fijo TS_MS: calcula el error),
    imprimirEstado()

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
#define ANGULO_CERO 0.2

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
const float ALPHA      = 0.98;    // filtro complementario

// ---------- Lazo de control ----------
const unsigned long TS_MS = 10;   // periodo de muestreo (Tsample) = 100 Hz
const float ANGULO_SP = 0.0;      // theta_sp: balancin vertical

// ---------- Tiempos ----------
const unsigned long INTERVALO_MS = 200;       // periodo de impresion
const unsigned long REINTENTO_MPU_MS = 3000;

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

// ---------- Lazo de control ----------
float anguloSetpoint = ANGULO_SP; // theta_sp
float anguloMedido = 0;           // theta_med
float errorAngulo = 0;            // e = theta_sp - theta_med
unsigned long ultimoControl = 0;
unsigned long tsReal = 0;         // ms reales entre dos ejecuciones del lazo

unsigned long ultimaImpresion = 0;

// ---------- Prototipos ----------
void procesarComandos();
void atenderMPU();
void lazoControl();
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

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

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

  Serial.println("=== PID Paso 1 listo: c = recalibrar | z = fijar cero ===");
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
    Serial.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    Serial.println(mpuOk ? ">>> Recalibracion completa"
                         : "!!! No se pudo recalibrar: el MPU6050 no responde");
  } else if (entrada.equalsIgnoreCase("z")) {
    if (mpuOk && ponerCeroActual()) {
      Serial.print(">>> CERO fijado. El sensor marcaba ");
      Serial.print(anguloCero, 2);
      Serial.println(" grados en equilibrio (ahora marca 0).");
      Serial.print("    Para guardarlo al reiniciar: #define ANGULO_CERO ");
      Serial.println(anguloCero, 2);
    } else {
      Serial.println("!!! No se pudo fijar el cero: el MPU6050 no responde");
    }
  }
}

// Actualiza el angulo; si se pierde el MPU, reintenta cada 3 s
void atenderMPU() {
  if (mpuOk) {
    if (!actualizarAngulo()) {
      mpuOk = false;
      Serial.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Serial.println(">>> MPU6050 reconectado");
  }
}

// LAZO INTERNO: se ejecuta solo cuando pasaron TS_MS (periodo de muestreo fijo)
//   if (tiempoActual - tiempoUltimo >= Ts) { ... }
void lazoControl() {
  unsigned long ahora = millis();
  if (ahora - ultimoControl < TS_MS) return;

  tsReal = ahora - ultimoControl;
  ultimoControl = ahora;

  if (!mpuOk) return;

  anguloMedido = anguloFinal();                // theta_med (del MPU)
  errorAngulo  = anguloSetpoint - anguloMedido; // e = theta_sp - theta_med
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
  Serial.print(" | Setpoint: ");
  Serial.print(anguloSetpoint, 1);
  Serial.print(" | Error: ");
  Serial.print(errorAngulo, 2);
  Serial.print(" | Ts real: ");
  Serial.print(tsReal);
  Serial.println(" ms");
}

// ============================================================================
// 5. FUNCIONES DE APOYO (MPU6050)
// ============================================================================

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
