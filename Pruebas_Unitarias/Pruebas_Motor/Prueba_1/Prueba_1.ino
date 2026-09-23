/*
  PRUEBA UNITARIA - Puente H TB6612FNG con ESP32
  Balancín - Control Inteligente 2

  Objetivo: enviar un valor de PWM (0-255) por el Monitor Serial y
  verificar que ambos motores giran a esa velocidad.

  =========================================================
  CONEXIONES EXACTAS
  =========================================================

  TB6612FNG        ESP32              Notas
  ------------------------------------------------------------
  VM        -->    Salida del LM2596 (voltaje de motores, NO 3.3V)
  VCC       -->    3V3 (ESP32)        Alimentación lógica del driver
  GND       -->    GND (ESP32) y GND común con LM2596 y batería
  STBY      -->    GPIO 4             Debe estar en HIGH para habilitar el driver

  --- Motor A ---
  AIN1      -->    GPIO 16
  AIN2      -->    GPIO 17
  PWMA      -->    GPIO 25            (Canal LEDC 0)
  AO1/AO2   -->    Terminales del Motor A

  --- Motor B ---
  BIN1      -->    GPIO 18
  BIN2      -->    GPIO 19
  PWMB      -->    GPIO 26            (Canal LEDC 1)
  BO1/BO2   -->    Terminales del Motor B

  LM2596 (reductor DC):
  IN+/IN-   -->    Batería (voltaje de entrada)
  OUT+      -->    VM del TB6612FNG (ajustar salida al voltaje nominal del motor)
  OUT-/GND  -->    GND común con ESP32 y TB6612FNG

  MPU6050 (no usado en esta prueba, dejar conectado para pruebas futuras):
  VCC -> 3V3 | GND -> GND | SDA -> GPIO 21 | SCL -> GPIO 22

  =========================================================
  USO
  =========================================================
  Abrir el Monitor Serial a 115200 baudios y escribir:
    - Un número de 0 a 255  -> ambos motores giran adelante a esa velocidad
    - 'r'                   -> invierte el sentido de giro
    - 's' o 0               -> detiene los motores
*/

// ---------- Pines de control de dirección ----------
#define STBY 4

#define AIN1 17
#define AIN2 16
#define PWMA 25

#define BIN1 18
#define BIN2 19
#define PWMB 26

// ---------- Configuración PWM (LEDC) ----------
// API del ESP32 Arduino Core 3.x: se referencia por PIN, no por canal
#define PWM_FREQ       5000   // 5 kHz, adecuado para TB6612FNG
#define PWM_RESOLUTION 8      // 8 bits -> rango 0-255

bool reversa = false;

void setMotorA(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(AIN1, reversa ? LOW  : HIGH);
  digitalWrite(AIN2, reversa ? HIGH : LOW);
  ledcWrite(PWMA, pwm);
}

void setMotorB(int pwm) {
  pwm = constrain(pwm, 0, 255);
  digitalWrite(BIN1, reversa ? LOW  : HIGH);
  digitalWrite(BIN2, reversa ? HIGH : LOW);
  ledcWrite(PWMB, pwm);
}

void detenerMotores() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  digitalWrite(STBY, HIGH); // habilita el driver

  // Configuración PWM (API ESP32 Arduino Core 3.x, por pin)
  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);

  detenerMotores();

  Serial.println("=== Prueba TB6612FNG lista ===");
  Serial.println("Escribe un valor 0-255 para mover ambos motores.");
  Serial.println("Escribe 'r' para invertir sentido, 's' para detener.");
}

void loop() {
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    if (entrada.equalsIgnoreCase("r")) {
      reversa = !reversa;
      Serial.println(reversa ? "Sentido: REVERSA" : "Sentido: ADELANTE");
    } else if (entrada.equalsIgnoreCase("s")) {
      detenerMotores();
      Serial.println("Motores detenidos");
    } else if (entrada.length() > 0) {
      int pwm = entrada.toInt();
      pwm = constrain(pwm, 0, 255);
      setMotorA(pwm);
      setMotorB(pwm);
      Serial.print("PWM enviado: ");
      Serial.println(pwm);
    }
  }
}

/*
  NOTA sobre versión del núcleo ESP32 Arduino:
  Este código usa ledcAttach()/ledcWrite(pin, valor), válido para el
  ESP32 Arduino Core 3.x. Si en algún momento usas el Core 2.x,
  reemplaza en setup():

    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMA, 0);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PWMB, 1);

  y en setMotorX(), cambia ledcWrite(PWMA/PWMB, pwm) por
  ledcWrite(0, pwm) / ledcWrite(1, pwm) (por canal, no por pin).
*/
