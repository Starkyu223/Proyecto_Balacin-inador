/*
  PRUEBA UNITARIA - Acelerómetro/Giroscopio MPU6050 con ESP32
  Balancín - Control Inteligente 2

  Objetivo: leer el acelerómetro (y giroscopio) del MPU6050 por I2C
  y calcular el ángulo de inclinación (pitch), que es la señal que
  luego usará el controlador de balance.

  No usa librerías externas (solo Wire.h), para que puedas instalar
  esto en cualquier PC sin depender de gestores de librerías.

  =========================================================
  CONEXIONES EXACTAS
  =========================================================

  MPU6050          ESP32              Notas
  ------------------------------------------------------------
  VCC       -->    3V3                NUNCA a 5V si tu módulo es de 3.3V
  GND       -->    GND
  SDA       -->    GPIO 21
  SCL       -->    GPIO 22
  AD0       -->    GND                Fija la dirección I2C en 0x68
                                       (si lo dejas al aire o a 3.3V, será 0x69)
  INT       -->    (sin conectar, no se usa en esta prueba)

  =========================================================
  USO
  =========================================================
  Abre el Monitor Serial a 115200 baudios. Verás, cada 100 ms:
    AccelX, AccelY, AccelZ (en g) | GyroX, GyroY, GyroZ (en °/s) | Pitch, Roll (en °)

  Mantén el sensor quieto y plano al iniciar: los primeros segundos
  se usan para calibrar el offset del giroscopio.
*/

#include <Wire.h>

#define MPU_ADDR   0x68   // dirección I2C con AD0 a GND
#define SDA_PIN    21
#define SCL_PIN    22

#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B

// Sensibilidades por defecto del MPU6050 (rango ±2g y ±250°/s)
const float ACCEL_SENS = 16384.0; // LSB por g
const float GYRO_SENS  = 131.0;   // LSB por °/s

int16_t axRaw, ayRaw, azRaw;
int16_t gxRaw, gyRaw, gzRaw;
float gyroOffsetX = 0, gyroOffsetY = 0;

float pitch = 0, roll = 0;
unsigned long ultimoTiempo;

void mpuEscribir(uint8_t registro, uint8_t valor) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registro);
  Wire.write(valor);
  uint8_t error = Wire.endTransmission(true);
  if (error != 0) {
    Serial.print("ERROR escribiendo registro 0x");
    Serial.print(registro, HEX);
    Serial.print(" -> codigo: ");
    Serial.println(error);
  }
}

uint8_t mpuLeerRegistro(uint8_t registro) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(registro);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  if (Wire.available()) return Wire.read();
  return 0xFF; // no respondio
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

void calibrarGiroscopio() {
  Serial.println("Calibrando giroscopio, no muevas el sensor...");
  long sumX = 0, sumY = 0;
  const int muestras = 500;

  for (int i = 0; i < muestras; i++) {
    mpuLeerCrudo();
    sumX += gxRaw;
    sumY += gyRaw;
    delay(3);
  }

  gyroOffsetX = sumX / (float)muestras;
  gyroOffsetY = sumY / (float)muestras;
  Serial.println("Calibracion lista.");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000); // 100kHz, mas confiable en protoboard que 400kHz

  mpuEscribir(PWR_MGMT_1, 0x00); // saca al MPU6050 del modo sleep
  delay(100);

  // Verificacion: si el registro no quedo en 0x00, el sensor sigue dormido
  uint8_t pwrEstado = mpuLeerRegistro(PWR_MGMT_1);
  Serial.print("PWR_MGMT_1 leido = 0x");
  Serial.println(pwrEstado, HEX);
  if (pwrEstado & 0x40) {
    Serial.println("El sensor SIGUE en modo sleep. Revisa alimentacion/cableado.");
  } else {
    Serial.println("Sensor despierto correctamente.");
  }

  calibrarGiroscopio();
  ultimoTiempo = millis();

  Serial.println("=== Prueba MPU6050 lista ===");
  Serial.println("AccelX,AccelY,AccelZ (g) | GyroX,GyroY,GyroZ (dps) | Pitch,Roll (deg)");
}

void loop() {
  mpuLeerCrudo();

  float ax = axRaw / ACCEL_SENS;
  float ay = ayRaw / ACCEL_SENS;
  float az = azRaw / ACCEL_SENS;

  float gx = (gxRaw - gyroOffsetX) / GYRO_SENS;
  float gy = (gyRaw - gyroOffsetY) / GYRO_SENS;
  float gz = gzRaw / GYRO_SENS;

  // Ángulo estimado solo con el acelerómetro (en grados)
  float pitchAccel = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  float rollAccel  = atan2(ay, az) * 180.0 / PI;

  // Filtro complementario: combina giroscopio (rápido, deriva) y
  // acelerómetro (lento, estable) para un ángulo más limpio.
  unsigned long ahora = millis();
  float dt = (ahora - ultimoTiempo) / 1000.0;
  ultimoTiempo = ahora;

  const float ALPHA = 0.98;
  pitch = ALPHA * (pitch + gx * dt) + (1 - ALPHA) * pitchAccel;
  roll  = ALPHA * (roll  + gy * dt) + (1 - ALPHA) * rollAccel;

  Serial.print(ax, 2); Serial.print(",");
  Serial.print(ay, 2); Serial.print(",");
  Serial.print(az, 2); Serial.print(" | ");
  Serial.print(gx, 1); Serial.print(",");
  Serial.print(gy, 1); Serial.print(",");
  Serial.print(gz, 1); Serial.print(" | ");
  Serial.print(pitch, 2); Serial.print(",");
  Serial.println(roll, 2);

  delay(100);
}
