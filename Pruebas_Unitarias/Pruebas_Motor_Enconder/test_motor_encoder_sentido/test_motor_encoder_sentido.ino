/*
  PRUEBA - Sentido de giro (adelante/reversa) + Encoder

  Cada 200ms se imprime el conteo de pulsos y las RPM (con signo) de
  cada motor. Revisa que:
    - En ADELANTE, las RPM de A y B salgan con el MISMO signo entre si
      (ambos positivos o ambos negativos, no uno de cada uno).
    - En REVERSA, el signo se invierta en ambos respecto a ADELANTE.
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

// Calibrados a mano girando el eje 5 vueltas completas y contando pulsos:
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

// Si en tus pruebas "adelante" movia el robot hacia atras, ya quedo
// corregido aqui invirtiendo el booleano - no hace falta tocar cables.
void aplicarMovimiento() {
  bool adelante = (modoActual == REVERSA); // <-- invertido a proposito
  motorA(adelante, velocidadActual);
  motorB(adelante, velocidadActual);
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

  Serial.println("=== Prueba sentido + encoder lista ===");
  Serial.println("Numero 0-255 = velocidad | w=adelante r=reversa s=stop");
}

void loop() {
  if (Serial.available() > 0) {
    String entrada = Serial.readStringUntil('\n');
    entrada.trim();

    if (entrada.equalsIgnoreCase("w")) {
      digitalWrite(STBY, HIGH);
      modoActual = ADELANTE;
      aplicarMovimiento();
      Serial.println(">>> Modo: ADELANTE");
    } else if (entrada.equalsIgnoreCase("r")) {
      digitalWrite(STBY, HIGH);
      modoActual = REVERSA;
      aplicarMovimiento();
      Serial.println(">>> Modo: REVERSA");
    } else if (entrada.equalsIgnoreCase("s")) {
      detenerMotores();
      digitalWrite(STBY, HIGH); // se mantiene activo -> frenado corto
      Serial.println(">>> Detenido (frenado corto, eje duro - normal)");
    } else if (entrada.equalsIgnoreCase("f")) {
      detenerMotores();
      digitalWrite(STBY, LOW); // desactiva el driver -> eje libre
      Serial.println(">>> LIBRE: puedes girar los ejes a mano sin resistencia");
    } else if (entrada.length() > 0) {
      digitalWrite(STBY, HIGH); // reactiva el driver si estaba en modo LIBRE
      velocidadActual = constrain(entrada.toInt(), 0, 255);
      aplicarMovimiento();
      Serial.print(">>> PWM: ");
      Serial.println(velocidadActual);
    }
  }

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
    Serial.println(" RPM");

    ultimoCalculo = ahora;
  }
}
