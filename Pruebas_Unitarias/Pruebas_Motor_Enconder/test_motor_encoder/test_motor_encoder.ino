/*
  PRUEBA UNITARIA - Motores DC con encoder (25GA370-12V-100RPM) + TB6612FNG
  Balancin - Control Inteligente 2

  Hace girar los dos motores por PWM (como en la prueba anterior) y
  ademas cuenta los pulsos del encoder de cada uno para calcular su
  velocidad en RPM. Sirve para confirmar que el encoder si esta
  generando señal cuando el motor gira.

  =========================================================
  CONEXIONES
  =========================================================
  TB6612FNG (igual que las pruebas anteriores):
    STBY->4 | AIN1->16 AIN2->17 PWMA->25 | BIN1->18 BIN2->19 PWMB->26
    M1 Motor- / M1 Motor+ de cada motor -> AO1/AO2 (motor A) o BO1/BO2 (motor B)

  Encoder Motor A:
    GND Encoder      -> GND del ESP32
    3.3V/5V Encoder+ -> 3V3 del ESP32
    Fase A Encoder   -> GPIO 34
    Fase B Encoder   -> GPIO 35

  Encoder Motor B:
    GND Encoder      -> GND del ESP32
    3.3V/5V Encoder+ -> 3V3 del ESP32
    Fase A Encoder   -> GPIO 32
    Fase B Encoder   -> GPIO 33

  =========================================================
  USO
  =========================================================
  Monitor Serial a 115200 baudios. Escribe un numero de 0-255 para
  mover ambos motores adelante a esa velocidad, o 's' para detener.
  Cada 200ms se imprime el conteo de pulsos y las RPM de cada motor.

  AJUSTA PULSOS_POR_VUELTA mas abajo: gira el eje del motor a mano
  exactamente una vuelta completa (con PWM en 0, motor detenido) y
  cuenta cuantos pulsos aparecen en el conteo de esa rueda. No hay
  una ficha tecnica publica confiable para el numero exacto de
  pulsos por vuelta de esta referencia (25GA370-12V-100RPM), asi
  que la calibracion manual es la unica forma segura de saberlo.

  Dato util para verificar que la calibracion quedo bien: este
  motor esta especificado en 100 RPM en el eje de salida a 12V
  sin carga. Con PWM al maximo (255) y 12V en el TB6612FNG, las
  RPM que calcule el codigo deberian acercarse a ese valor (algo
  menos por friccion y la caida de voltaje del puente H). Si te
  da un numero muy distinto (el doble o la mitad), probablemente
  PULSOS_POR_VUELTA esta mal.
*/

// ---------- Pines del TB6612FNG ----------
#define STBY 4
#define AIN1 16
#define AIN2 17
#define PWMA 25
#define BIN1 18
#define BIN2 19
#define PWMB 26

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

// ---------- Pines de los encoders ----------
#define ENC_A_FASEA 34
#define ENC_A_FASEB 35
#define ENC_B_FASEA 32
#define ENC_B_FASEB 33

// AJUSTA este valor con tu motor real (ver instrucciones de calibracion arriba)
#define PULSOS_POR_VUELTA 390

volatile long pulsosA = 0;
volatile long pulsosB = 0;

unsigned long ultimoCalculo = 0;
const unsigned long INTERVALO_MS = 200;

// ================= Interrupciones de los encoders =================
void IRAM_ATTR encoderA_ISR() {
  // Compara Fase A y Fase B para saber el sentido de giro
  if (digitalRead(ENC_A_FASEB) == HIGH) {
    pulsosA++;
  } else {
    pulsosA--;
  }
}

void IRAM_ATTR encoderB_ISR() {
  if (digitalRead(ENC_B_FASEB) == HIGH) {
    pulsosB++;
  } else {
    pulsosB--;
  }
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
  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB, PWM_FREQ, PWM_RESOLUTION);
  detenerMotores();

  pinMode(ENC_A_FASEA, INPUT);
  pinMode(ENC_A_FASEB, INPUT);
  pinMode(ENC_B_FASEA, INPUT);
  pinMode(ENC_B_FASEB, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A_FASEA), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B_FASEA), encoderB_ISR, RISING);

  ultimoCalculo = millis();

  Serial.println("=== Prueba motor + encoder lista ===");
  Serial.println("Numero 0-255 = velocidad adelante | s = detener");
}

void loop() {
  // ---- Lectura de comandos por serial ----
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    if (entrada.equalsIgnoreCase("s")) {
      detenerMotores();
      Serial.println("Motores detenidos");
    } else if (entrada.length() > 0) {
      int vel = constrain(entrada.toInt(), 0, 255);
      motorA(true, vel);
      motorB(true, vel);
      Serial.print("PWM enviado: ");
      Serial.println(vel);
    }
  }

  // ---- Calculo de RPM cada INTERVALO_MS ----
  unsigned long ahora = millis();
  if (ahora - ultimoCalculo >= INTERVALO_MS) {
    noInterrupts();
    long copiaA = pulsosA;
    long copiaB = pulsosB;
    pulsosA = 0;
    pulsosB = 0;
    interrupts();

    float minutos = (ahora - ultimoCalculo) / 60000.0;
    float rpmA = (copiaA / (float)PULSOS_POR_VUELTA) / minutos;
    float rpmB = (copiaB / (float)PULSOS_POR_VUELTA) / minutos;

    Serial.print("Motor A: ");
    Serial.print(copiaA);
    Serial.print(" pulsos, ");
    Serial.print(rpmA, 1);
    Serial.print(" RPM | Motor B: ");
    Serial.print(copiaB);
    Serial.print(" pulsos, ");
    Serial.print(rpmB, 1);
    Serial.println(" RPM");

    ultimoCalculo = ahora;
  }
}
