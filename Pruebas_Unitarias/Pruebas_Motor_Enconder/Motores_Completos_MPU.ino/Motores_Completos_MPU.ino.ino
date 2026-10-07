/*
  Cada 200ms se imprime: pulsos/RPM de cada motor + angulo Y (roll) en grados.
  Mantén el sensor quieto y plano al iniciar: calibra el offset del giroscopio.
*/

#include <Wire.h>

#define AIN1 1
#define AIN2 2
#define PWMA 14
#define BIN1 15
#define BIN2 16
#define PWMB 17

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

#define ENC_A_FASEA 18
#define ENC_A_FASEB 21
#define ENC_B_FASEA 38
#define ENC_B_FASEB 39

// Motor A: 4455 pulsos / 5 vueltas = 891
// Motor B: 4237 pulsos / 5 vueltas = 847
#define PULSOS_POR_VUELTA_A 891
#define PULSOS_POR_VUELTA_B 847

volatile long pulsosA = 0;
volatile long pulsosB = 0;

unsigned long ultimoCalculo = 0;
const unsigned long INTERVALO_MS = 200;

enum Modo { ADELANTE, REVERSA };
Modo modoActual = ADELANTE;
int velocidadActual = 0;

// ---------- MPU6050 ----------
#define MPU_ADDR     0x68
#define SDA_PIN      47
#define SCL_PIN      48
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// Cambia a true si el signo del angulo Y sale al reves de lo esperado
#define INVERTIR_EJE_Y false

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;

int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
float gyroOffsetY = 0;
float anguloY = 0; // angulo de inclinacion sobre el eje Y (roll), en grados
unsigned long ultimoTiempoMPU;

// ================= Interrupciones de los encoders =================
void IRAM_ATTR encoderA_ISR() {
  if (digitalRead(ENC_A_FASEB) == HIGH) pulsosA++; else pulsosA--;
}

void IRAM_ATTR encoderB_ISR() {
  if (digitalRead(ENC_B_FASEB) == HIGH) pulsosB++; else pulsosB--;
}

// ================= Funciones del motor =================
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

// ================= Funciones del MPU6050 =================
void mpuEscribir(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registro);
  Wire.write(valor);
  Wire.endTransmission(true);
}

void mpuLeerCrudo() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  axRaw = (Wire.read() << 8) | Wire.read();
  ayRaw = (Wire.read() << 8) | Wire.read();
  azRaw = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // descarta temperatura
  gxRaw = (Wire.read() << 8) | Wire.read();
  gyRaw = (Wire.read() << 8) | Wire.read();
  gzRaw = (Wire.read() << 8) | Wire.read();
}

void calibrarGiroscopioY() {
  Serial.println("Calibrando giroscopio (eje Y), no muevas el sensor...");
  long sumY = 0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    mpuLeerCrudo();
    sumY += gyRaw;
    delay(3);
  }

  gyroOffsetY = sumY / (float)muestras;
  Serial.println("Calibracion lista.");
}

void actualizarAnguloY() {
  mpuLeerCrudo();

  float ay = ayRaw / ACCEL_SENS;
  float az = azRaw / ACCEL_SENS;
  float gy = (gyRaw - gyroOffsetY) / GYRO_SENS;

  float anguloAccelY = atan2(ay, az) * 180.0 / PI;

  unsigned long ahora = millis();
  float dt = (ahora - ultimoTiempoMPU) / 1000.0;
  ultimoTiempoMPU = ahora;

  const float ALPHA = 0.98;
  anguloY = ALPHA * (anguloY + gy * dt) + (1 - ALPHA) * anguloAccelY;

  if (INVERTIR_EJE_Y) anguloY = -anguloY;
}

void setup() {
  Serial.begin(115200);
  delay(300);

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
  Wire.setClock(100000); // 100kHz, confiable en protoboard (ya validado)

  mpuEscribir(PWR_MGMT_1, 0x00); // despierta el sensor
  delay(100);
  calibrarGiroscopioY();
  ultimoTiempoMPU = millis();

  ultimoCalculo = millis();

  Serial.println("=== Motores + Encoders + MPU6050 (eje Y) listo ===");
  Serial.println("Numero 0-255 = velocidad | w=adelante r=reversa s=stop");
}

void loop() {
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

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
      Serial.println(">>> Detenido (frenado corto, eje duro - normal)");
    } else if (entrada.length() > 0) {
      velocidadActual = constrain(entrada.toInt(), 0, 255);
      aplicarMovimiento();
      Serial.print(">>> PWM: ");
      Serial.println(velocidadActual);
    }
  }

  // El MPU se lee cada ciclo de loop (mas rapido) para que el filtro
  // complementario tenga buena resolucion temporal en el giroscopio.
  actualizarAnguloY();

  unsigned long ahora = millis();
  if (ahora - ultimoCalculo >= INTERVALO_MS) {
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
    Serial.print(" RPM | Angulo Y: ");
    Serial.print(anguloY, 2);
    Serial.println(" grados");

    ultimoCalculo = ahora;
  }
}
