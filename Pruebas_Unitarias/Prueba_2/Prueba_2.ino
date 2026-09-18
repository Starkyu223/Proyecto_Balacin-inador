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
  Abrir el Monitor Serial a 115200 baudios (con Nueva línea activado)
  y escribir:
    - Un número de 0 a 255  -> fija la velocidad en el modo actual
                                (por defecto arranca en ADELANTE)
    - 'w'                   -> modo ADELANTE (ambos motores igual sentido)
    - 'r'                   -> modo REVERSA  (ambos motores sentido opuesto)
    - 'a'                   -> gira a la IZQUIERDA (motor A reversa, motor B adelante)
    - 'd'                   -> gira a la DERECHA   (motor A adelante, motor B reversa)
    - 's' o 0               -> detiene los motores

  El cambio de modo (w/r/a/d) se aplica DE INMEDIATO con la última
  velocidad enviada, sin necesidad de volver a escribir el número.

  NOTA IMPORTANTE sobre el giro: si al pedir "izquierda" el robot en
  realidad gira a la derecha (o viceversa), es porque uno de los
  motores está físicamente montado en espejo respecto al otro.
  Soluciónalo invirtiendo el cableado de AO1/AO2 (o BO1/BO2) en las
  terminales del motor correspondiente, o cambiando en el código cuál
  IN1/IN2 se pone en HIGH para ese motor.
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

enum Modo { ADELANTE, REVERSA, IZQUIERDA, DERECHA };
Modo modoActual = ADELANTE;
int velocidadActual = 0;

// dirA=true -> AIN1=HIGH/AIN2=LOW (giro "adelante" del motor A)
void motorA(bool dirForward, int pwm) {
  digitalWrite(AIN1, dirForward ? HIGH : LOW);
  digitalWrite(AIN2, dirForward ? LOW  : HIGH);
  ledcWrite(PWMA, pwm);
}

// dirB=true -> BIN1=HIGH/BIN2=LOW (giro "adelante" del motor B)
void motorB(bool dirForward, int pwm) {
  digitalWrite(BIN1, dirForward ? HIGH : LOW);
  digitalWrite(BIN2, dirForward ? LOW  : HIGH);
  ledcWrite(PWMB, pwm);
}

// Aplica el modo actual con la velocidad actual. Se llama tanto al
// cambiar de modo como al cambiar de velocidad, así el cambio es
// inmediato en ambos casos.
void aplicarMovimiento() {
  switch (modoActual) {
    case ADELANTE:
      motorA(true,  velocidadActual);
      motorB(true,  velocidadActual);
      break;
    case REVERSA:
      motorA(false, velocidadActual);
      motorB(false, velocidadActual);
      break;
    case IZQUIERDA:
      motorA(false, velocidadActual); // rueda izquierda gira en reversa
      motorB(true,  velocidadActual); // rueda derecha gira adelante
      break;
    case DERECHA:
      motorA(true,  velocidadActual); // rueda izquierda gira adelante
      motorB(false, velocidadActual); // rueda derecha gira en reversa
      break;
  }
}

void detenerMotores() {
  velocidadActual = 0;
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
  Serial.println("Numero 0-255 = velocidad | w=adelante r=reversa a=izquierda d=derecha s=stop");
}

void loop() {
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    if (entrada.equalsIgnoreCase("w")) {
      modoActual = ADELANTE;
      aplicarMovimiento();
      Serial.println("Modo: ADELANTE");
    } else if (entrada.equalsIgnoreCase("r")) {
      modoActual = REVERSA;
      aplicarMovimiento();
      Serial.println("Modo: REVERSA");
    } else if (entrada.equalsIgnoreCase("a")) {
      modoActual = IZQUIERDA;
      aplicarMovimiento();
      Serial.println("Modo: IZQUIERDA");
    } else if (entrada.equalsIgnoreCase("d")) {
      modoActual = DERECHA;
      aplicarMovimiento();
      Serial.println("Modo: DERECHA");
    } else if (entrada.equalsIgnoreCase("s")) {
      detenerMotores();
      Serial.println("Motores detenidos");
    } else if (entrada.length() > 0) {
      velocidadActual = constrain(entrada.toInt(), 0, 255);
      aplicarMovimiento();
      Serial.print("PWM enviado: ");
      Serial.println(velocidadActual);
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
