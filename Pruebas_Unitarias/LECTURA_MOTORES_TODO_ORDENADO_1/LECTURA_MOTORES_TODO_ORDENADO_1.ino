/*
  ============================================================================
  BALANCIN AUTOBALANCEADO - Control Inteligente 2
  Motores + Encoders + MPU6050 con diagnostico I2C y modo automatico
  Placa: ESP32-S3 (Arduino core 3.x)
  ============================================================================

  USO (Monitor Serial a 115200, con "Nueva linea" activado):
    - Numero 0-255 -> fija la velocidad (PWM) en el modo actual
    - 'w'          -> ADELANTE      'r' -> REVERSA      's' -> detener
    - 'm'          -> activa/desactiva MODO AUTOMATICO (angulo -> motores)
    - 'c'          -> RECALIBRA el MPU6050 (balancin quieto ~2 s)
    - 'z'          -> FIJA EL CERO en la posicion de equilibrio actual
  Cada 200 ms imprime pulsos/RPM de cada motor y el angulo.

  ============================================================================
  TABLA DE CONTENIDO
  ============================================================================
  Secciones del archivo (en orden):
    1. DEPENDENCIAS
    2. CONFIGURACION  
    3. VARIABLES GLOBALES
    4. FUNCIONES PRINCIPALES
    5. FUNCIONES DE APOYO

  --------------------------------------------------------------------------
  NOMBRAMIENTO DE VARIABLES (convencion)
  --------------------------------------------------------------------------
    MAYUSCULAS          constantes y pines          AIN1, PWM_FREQ, ALPHA
    sufijo _A / _B      pertenece al motor A o B    PULSOS_POR_VUELTA_A
    prefijo ENC_        pines del encoder           ENC_A_FASEA
    camelCase           variables globales          velocidadActual
    sufijo Raw          dato crudo del sensor       axRaw, gyRaw
    sufijo Offset       correccion de calibracion   gyroOffsetY
    prefijo angulo...   valores en grados           anguloInclinacion
    prefijo modo...     banderas de estado          modoActual, modoAutomatico
    sufijo _ISR         rutinas de interrupcion     encoderA_ISR

  Variables globales principales:
    pulsosA / pulsosB      pulsos acumulados de cada encoder (volatile)
    modoActual             ADELANTE o REVERSA
    velocidadActual        PWM actual 0-255
    anguloInclinacion      angulo filtrado SIN compensar (grados)
    anguloCero             lo que marca el sensor en equilibrio (grados)
    mpuOk                  true si el MPU6050 responde
    modoAutomatico         true si el modo automatico esta activo

  --------------------------------------------------------------------------
  FUNCIONES PRINCIPALES (flujo del programa)
  --------------------------------------------------------------------------
    setup()                 inicia motores, encoders y MPU6050
    loop()                  ciclo: comandos -> sensor -> control -> impresion
    procesarComandos()      lee y ejecuta los comandos del Monitor Serial
    atenderMPU()            actualiza el angulo y reintenta si se pierde el MPU
    controlAutomatico()     angulo -> sentido y PWM de los motores
    imprimirEstado()        calcula RPM e imprime cada INTERVALO_MS

  --------------------------------------------------------------------------
  FUNCIONES DE APOYO
  --------------------------------------------------------------------------
    Encoders:   encoderA_ISR(), encoderB_ISR()
    Motores:    motorA(), motorB(), aplicarMovimiento(), detenerMotores()
    MPU6050:    mpuEscribir(), mpuLeerCrudo(), escanearI2C(),
                calibrarGiroscopio(), anguloDesdeAcelerometro(),
                iniciarMPU(), ponerCeroActual(), actualizarAngulo(),
                anguloFinal()
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

// ---------- Pines de los encoders ----------
#define ENC_A_FASEA 18
#define ENC_A_FASEB 8
#define ENC_B_FASEA 38
#define ENC_B_FASEB 39

// Calibrados a mano (5 vueltas): A = 4455/5, B = 4237/5
#define PULSOS_POR_VUELTA_A 891
#define PULSOS_POR_VUELTA_B 847

// ---------- MPU6050 ----------
#define MPU_ADDR     0x68
#define SDA_PIN      21
#define SCL_PIN      47
#define PWR_MGMT_1   0x6B
#define WHO_AM_I     0x75
#define ACCEL_XOUT_H 0x3B

// 1 = el balancin se inclina girando sobre el eje Y del sensor (usa ax, az, gy)
// 0 = el balancin se inclina girando sobre el eje X del sensor (usa ay, az, gx)
#define INCLINA_SOBRE_EJE_Y 1

// Cambia a true si el signo del angulo sale al reves de lo esperado
#define INVERTIR_ANGULO false

// Valor inicial del "cero" (equilibrio). Con el comando 'z' lo fijas en cualquier
// momento; si quieres que quede guardado al reiniciar, copia aqui el valor que imprime 'z'.
#define ANGULO_CERO 0.0

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
const float ALPHA      = 0.98;   // filtro complementario

// ---------- Modo automatico ----------
const float ZONA_MUERTA_GRADOS = 5.0;  // por debajo de esto, motores detenidos
const float ANGULO_MAX_GRADOS  = 30.0; // a partir de aqui, velocidad maxima
const int   VELOCIDAD_MIN      = 60;   // PWM minimo para vencer la friccion
const int   VELOCIDAD_MAX      = 255;

// ---------- Tiempos ----------
const unsigned long INTERVALO_MS = 200;   // periodo de impresion
const unsigned long REINTENTO_MPU_MS = 3000;

// ============================================================================
// 3. VARIABLES GLOBALES
// ============================================================================

// ---------- Encoders ----------
volatile long pulsosA = 0;
volatile long pulsosB = 0;
unsigned long ultimoCalculo = 0;

// ---------- Motores ----------
enum Modo { ADELANTE, REVERSA };
Modo modoActual = ADELANTE;
int velocidadActual = 0;

// ---------- MPU6050 ----------
int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
float gyroOffsetX = 0, gyroOffsetY = 0;
float anguloInclinacion = 0;       // grados (sin compensar)
float anguloCero = ANGULO_CERO;    // lo que marca el sensor en equilibrio
unsigned long ultimoTiempoMPU = 0;
bool mpuOk = false;
unsigned long ultimoReintento = 0;

// ---------- Modo automatico ----------
bool modoAutomatico = false;

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

  // ---- Encoders ----
  pinMode(ENC_A_FASEA, INPUT);
  pinMode(ENC_A_FASEB, INPUT);
  pinMode(ENC_B_FASEA, INPUT);
  pinMode(ENC_B_FASEB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_FASEA), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FASEA), encoderB_ISR, RISING);

  // ---- MPU6050 ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  escanearI2C();
  mpuOk = iniciarMPU();
  if (!mpuOk) {
    Serial.println("!!! MPU6050 NO RESPONDE. Revisa:");
    Serial.print("    SDA -> GPIO "); Serial.println(SDA_PIN);
    Serial.print("    SCL -> GPIO "); Serial.println(SCL_PIN);
    Serial.println("    VCC -> 3V3, GND -> GND, AD0 -> GND");
    Serial.println("    Los motores siguen funcionando por comandos.");
  }

  ultimoCalculo = millis();
  ultimoReintento = millis();

  Serial.println("=== Listo: numero 0-255 | w r s | m = automatico | c = recalibrar | z = fijar cero ===");
}

void loop() {
  procesarComandos();
  atenderMPU();
  if (modoAutomatico && mpuOk) controlAutomatico();
  imprimirEstado();
}

// Lee una linea del Monitor Serial y ejecuta el comando
void procesarComandos() {
  if (Serial.available() <= 0) return;

  String entrada = Serial.readStringUntil('\n');
  entrada.trim();

  if (entrada.equalsIgnoreCase("c")) {
    // Recalibrar el MPU sin desconectar nada (balancin quieto ~2 s)
    modoAutomatico = false;
    detenerMotores();
    Serial.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    if (mpuOk) {
      Serial.println(">>> Recalibracion completa");
    } else {
      Serial.println("!!! No se pudo recalibrar: el MPU6050 no responde");
    }
  } else if (entrada.equalsIgnoreCase("z")) {
    // Fijar el cero: el balancin debe estar quieto en su punto de equilibrio
    modoAutomatico = false;
    detenerMotores();
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
      modoAutomatico = !modoAutomatico;
      if (!modoAutomatico) detenerMotores();
      Serial.println(modoAutomatico ? ">>> MODO AUTOMATICO activado" : ">>> Modo automatico desactivado");
    }
  } else {
    modoAutomatico = false; // cualquier comando manual desactiva el automatico
    if (entrada.equalsIgnoreCase("w")) {
      modoActual = ADELANTE;
      aplicarMovimiento();
      Serial.println(">>> Modo: ADELANTE");
    } else if (entrada.equalsIgnoreCase("r")) {
      modoActual = REVERSA;
      aplicarMovimiento();
      Serial.println(">>> Modo: REVERSA");
    } else if (entrada.equalsIgnoreCase("s")) {
      detenerMotores();
      Serial.println(">>> Detenido");
    } else if (entrada.length() > 0) {
      velocidadActual = constrain(entrada.toInt(), 0, 255);
      aplicarMovimiento();
      Serial.print(">>> PWM: ");
      Serial.println(velocidadActual);
    }
  }
}

// Actualiza el angulo; si se pierde el MPU, detiene motores y reintenta cada 3 s
void atenderMPU() {
  if (mpuOk) {
    if (!actualizarAngulo()) {
      mpuOk = false;
      modoAutomatico = false;
      detenerMotores();
      Serial.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Serial.println(">>> MPU6050 reconectado");
  }
}

// Modo automatico provisional: angulo -> sentido y PWM proporcional.
// (Se reemplaza por el controlador PID en el siguiente paso.)
void controlAutomatico() {
  float ang = anguloFinal();
  float mag = fabs(ang);

  if (mag < ZONA_MUERTA_GRADOS) {
    detenerMotores();
    return;
  }

  // Invertido tras la prueba: inclinar adelante -> ruedas adelante
  modoActual = (ang > 0) ? REVERSA : ADELANTE;
  velocidadActual = map((long)(constrain(mag, ZONA_MUERTA_GRADOS, ANGULO_MAX_GRADOS) * 100),
                        (long)(ZONA_MUERTA_GRADOS * 100), (long)(ANGULO_MAX_GRADOS * 100),
                        VELOCIDAD_MIN, VELOCIDAD_MAX);
  aplicarMovimiento();
}

// Cada INTERVALO_MS calcula las RPM de cada motor e imprime el estado
void imprimirEstado() {
  unsigned long ahora = millis();
  if (ahora - ultimoCalculo < INTERVALO_MS) return;

  noInterrupts();
  long copiaA = pulsosA;
  long copiaB = pulsosB;
  pulsosA = 0;
  pulsosB = 0;
  interrupts();

  float minutos = (ahora - ultimoCalculo) / 60000.0;
  float rpmA = (copiaA / (float)PULSOS_POR_VUELTA_A) / minutos;
  float rpmB = (copiaB / (float)PULSOS_POR_VUELTA_B) / minutos;

  Serial.print("Motor A: ");
  Serial.print(copiaA);
  Serial.print(" pulsos, ");
  Serial.print(rpmA, 1);
  Serial.print(" RPM | Motor B: ");
  Serial.print(copiaB);
  Serial.print(" pulsos, ");
  Serial.print(rpmB, 1);
  if (mpuOk) {
    Serial.print(" RPM | Angulo: ");
    Serial.print(anguloFinal(), 2);
    Serial.println(modoAutomatico ? " grados [AUTO]" : " grados");
  } else {
    Serial.println(" RPM | Angulo: SIN SENSOR (revisa SDA/SCL)");
  }

  ultimoCalculo = ahora;
}

// ============================================================================
// 5. FUNCIONES DE APOYO
// ============================================================================

// ---------- Encoders (interrupciones) ----------
void IRAM_ATTR encoderA_ISR() {
  if (digitalRead(ENC_A_FASEB) == HIGH) pulsosA++; else pulsosA--;
}

void IRAM_ATTR encoderB_ISR() {
  if (digitalRead(ENC_B_FASEB) == HIGH) pulsosB++; else pulsosB--;
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

// Angulo calculado solo con el acelerometro (usa los ultimos datos crudos leidos)
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

// Fija la posicion actual como "angulo 0" (equilibrio). Promedia 100 lecturas
// del acelerometro (~0.4 s) para no depender de una sola muestra con ruido.
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
