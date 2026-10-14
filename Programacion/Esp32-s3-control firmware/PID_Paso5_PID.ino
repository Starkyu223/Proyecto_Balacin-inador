/*
  ============================================================================
  BALANCIN - Control Inteligente 2
  PID PASO 5: agrega la INTEGRAL  ->  u = Kp*e + Ki*Integral(e) + Kd*de/dt  (PID)
  Placa: ESP32-S3 (Arduino core 3.x)   |   MPU6050 + motores (sin encoders)
  ============================================================================

  Lazo interno de tu pizarra (primer bloque):

      theta_sp (0 grados) --> (+) --> e = theta_sp - theta_med
                               ^(-)
      theta_med (MPU6050) -----'

      e --> [ Kp*e + Ki*Integral(e) + Kd*de/dt ] --> u --> [ zona muerta / limite ] --> PWM --> puente H --> motores
                          signo de u = sentido del giro
      de/dt = (e_k - e_k-1) / Ts        Integral(e) = suma de e * Ts

  USO (Monitor Serial a 115200, con "Nueva linea" activado):
    - 'c' -> recalibra el MPU6050 (balancin quieto ~2 s)
    - 'z' -> fija el cero: la posicion actual pasa a marcar 0 grados
    - 'kp' + numero -> cambia Kp en vivo. Ej: kp10      (tambien vale k10)
    - 'ki' + numero -> cambia Ki en vivo. Ej: ki0.5   (arranca en 0 = sin integral)
    - 'kd' + numero -> cambia Kd en vivo. Ej: kd0.5
      (la derivada frena el balanceo: mide que tan rapido cambia el error)
    - 'tol' + numero -> tolerancia +- alrededor del setpoint. Ej: tol0.5 (arranca en 0.5)
      Dentro de esa banda P e I ven error 0; tol0 la desactiva.
    - 'm' -> ACTIVA/DESACTIVA el control (arranca DESACTIVADO por seguridad)
    - 's' -> desactiva el control y detiene los motores

  PARA QUE SIRVE LA INTEGRAL: acumula el error en el tiempo. Si el balancin se
  queda con un error pequeno constante (por ejemplo el cero no coincide con el
  punto de equilibrio real), la integral crece hasta empujar lo necesario y
  eliminar ese error. Se ajusta DESPUES de Kp y Kd, con valores pequenos.
  Para que no se dispare ("windup") tiene dos protecciones:
    - INTEGRAL_MAX_U: tope del aporte de la integral (en unidades de PWM)
    - INTEGRAL_ZONA_GRADOS: solo integra cuando el error es menor que esto

  WIFI (comunicacion con balancin_pid.html):
    - Requiere la libreria "WebSockets" de Markus Sattler (Gestor de bibliotecas).
    - Red y clave estan en WIFI_SSID / WIFI_PASS (la red debe ser de 2.4 GHz).
    - Al arrancar imprime la IP. En la pagina escribe esa IP (o balancin.local)
      y pulsa "Conectar WiFi". Recibe el mismo texto del Monitor Serial y acepta
      los mismos comandos. El USB sigue funcionando igual.
    - Si se cierra la pagina o se cae la red, el control NO se apaga solo.

  PRUEBA SEGURA: primero con las RUEDAS EN EL AIRE. Activa con 'm', inclina el
  balancin y mira que las ruedas giren hacia donde se inclina. Si el balancin
  pasa de CORTE_GRADOS el control se apaga solo.

  Salida cada 200 ms (2 lineas):
    1) Angulo | Error | de/dt | Kp | Ki | Kd | I | u | PWM | Ts real
    2) Lectura del MPU: acelerometro X/Y/Z (g), giroscopio X/Y/Z (grados/s),
       angulo solo del acelerometro y angulo filtrado sin cero

  QUE DEBES VER:
    Angulo +5  ->  Error -5  ->  u = Kp*(-5)  (negativo)
    Angulo -5  ->  Error +5  ->  u = Kp*(+5)  (positivo)
    Con INVERTIR_SENTIDO_MOTORES = true (actual):
      angulo POSITIVO (u < 0) -> motores hacia ATRAS
      angulo NEGATIVO (u > 0) -> motores hacia ADELANTE
    Con false es al reves. Si el balancin cae mas rapido, cambia ese #define.
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
    kp / ki / kd        ganancias proporcional, integral y derivativa (ajustables en vivo)
    integralError       suma acumulada de e*Ts (grados*segundo)
    terminoI            aporte de la integral a u: Ki * integralError (limitado)
    errorPrevio         e_k-1, error de la muestra anterior
    derivadaError       de/dt filtrada (grados/segundo)
    salidaU             u = Kp * e (unidades de PWM, aun sin limitar)
    controlActivo       true si el lazo esta moviendo los motores
    modoActual          ADELANTE o REVERSA (sentido segun signo de u)
    velocidadActual     PWM aplicado 0-255
    tsReal              periodo real medido entre ejecuciones del lazo (ms)

  Funciones principales:
    setup(), loop(), procesarComandos(), atenderMPU(),
    lazoControl()  (a periodo fijo TS_MS: error, derivada, integral, u = Kp*e + Ki*Int(e) + Kd*de/dt, PWM),
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
#include <WiFi.h>              // WiFi del ESP32 (viene con el core)
#include <ESPmDNS.h>           // nombre balancin.local (viene con el core)
#include <WebSocketsServer.h>  // Libreria "WebSockets" de Markus Sattler (Gestor de bibliotecas)
// ============================================================================
// 2. CONFIGURACION
// ============================================================================

// ---------- WiFi + WebSocket (comunicacion con balancin_pid.html) ----------
// El ESP32 se conecta a tu red (solo 2.4 GHz) y abre un servidor WebSocket.
// La pagina se conecta a  ws://<IP del ESP o balancin.local>:81
// La IP se imprime en el Monitor Serial al arrancar.
const char* WIFI_SSID = "Engleyber";
const char* WIFI_PASS = "Ariadna11";
const char* HOSTNAME  = "balancin";            // -> balancin.local
const uint16_t WS_PORT = 81;
const unsigned long WIFI_ESPERA_MS = 10000;    // espera maxima de conexion al arrancar
const unsigned long WIFI_REINTENTO_MS = 5000;  // cada cuanto reintenta si se cae la red

// ---------- Pines del TB6612FNG (STBY fijo a 3.3V) ----------
#define AIN1 11
#define AIN2 10
#define PWMA 9
#define BIN1 13
#define BIN2 12
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

// Sentido de los motores respecto al signo del angulo:
//   true  -> angulo positivo = motores hacia ATRAS, angulo negativo = hacia ADELANTE
//   false -> angulo positivo = motores hacia ADELANTE, angulo negativo = hacia ATRAS
#define INVERTIR_SENTIDO_MOTORES true

// Cero inicial. Con 'z' lo fijas; para guardarlo al reiniciar, copia aqui
// el valor que imprime 'z'.
#define ANGULO_CERO 0.2

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
const float ALPHA      = 0.98;    // filtro complementario

// ---------- Lazo de control ----------
const unsigned long TS_MS = 10;   // periodo de muestreo (Tsample) = 100 Hz
const float ANGULO_SP = 0.0;      // theta_sp: balancin vertical
// u esta en unidades de PWM y el error en grados: con Kp = 8, un error de 30
// grados da u = 240 (casi PWM maximo). Ajustalo en vivo con 'k'.
const float KP_INICIAL = 10.0;    // ganancia proporcional (la que elegiste)
const float KI_INICIAL = 0.0;     // ganancia integral: arranca en 0 (= PD); subela con 'ki'
const float KD_INICIAL = 0.0;     // ganancia derivativa: arranca en 0 (= solo P); subela con 'kd'
const float ALFA_D = 0.3;         // filtro de la derivada: peso de la muestra nueva (0-1). Menor = mas suave

// ---------- Proteccion de la integral (anti-windup) ----------
const float INTEGRAL_MAX_U = 50.0;        // el aporte de la integral nunca pasa de +-50 (unidades de PWM)
const float INTEGRAL_ZONA_GRADOS = 10.0;  // solo integra si |error| es menor que esto

// ---------- Tolerancia alrededor del setpoint ----------
// Si |error| <= tolGrados, P e I ven error 0 (el balancin "ya esta en el cero").
// Se cambia en vivo con 'tol' + numero (ej: tol0.5). tol0 = sin tolerancia.
const float TOL_INICIAL = 0.5;       // grados (+-)
const float TOL_MAX_GRADOS = 5.0;    // tope de seguridad para el comando 'tol'

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
float anguloInclinacion = 0;      // grados (sin compensar)float anguloCero = ANGULO_CERO;   // lo que marca el sensor en equilibrio
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
float ki = KI_INICIAL;            // ganancia integral
float tolGrados = TOL_INICIAL;    // tolerancia +- alrededor del setpoint (grados)
float kd = KD_INICIAL;            // ganancia derivativa
float integralError = 0;          // suma acumulada de e*Ts
float terminoI = 0;               // aporte de la integral a u
float errorPrevio = 0;            // e_k-1
float derivadaError = 0;          // de/dt filtrada (grados/s)
bool primeraMuestra = true;       // evita un pico de derivada al arrancar o tras fijar el cero
float salidaU = 0;                // u = Kp * e
unsigned long ultimoControl = 0;
unsigned long tsReal = 0;         // ms reales entre dos ejecuciones del lazo

unsigned long ultimaImpresion = 0;

// ---------- WiFi / WebSocket ----------
WebSocketsServer webSocket(WS_PORT);
bool serviciosWiFiOk = false;          // true cuando ya arranco WebSocket + mDNS
int wsClientes = 0;                    // paginas conectadas ahora mismo
unsigned long ultimoReintentoWiFi = 0;

// Cola de comandos recibidos por WebSocket (se ejecutan en loop(), no dentro del callback)
const int COLA_N = 8;
String colaComandos[COLA_N];
int colaLeer = 0, colaEscribir = 0;

// Out: igual que Serial, pero cada linea completa tambien se manda por WebSocket.
// Asi la pagina recibe exactamente el mismo texto que ves en el Monitor Serial.
class SalidaDual : public Print {
  String linea;
  void acumular(uint8_t c) {
    if (c == '\n') {
      if (wsClientes > 0 && linea.length() > 0) webSocket.broadcastTXT(linea.c_str(), linea.length());
      linea = "";
    } else if (c != '\r' && linea.length() < 400) {
      linea += (char)c;
    }
  }
 public:
  size_t write(uint8_t c) override { Serial.write(c); acumular(c); return 1; }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
    for (size_t i = 0; i < n; i++) acumular(b[i]);
    return n;
  }
};
SalidaDual Out;

// ---------- Prototipos ----------
void eventoWebSocket(uint8_t num, WStype_t tipo, uint8_t* payload, size_t largo);
void conectarWiFi();
void atenderWiFi();
void ejecutarComando(String entrada);
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

  // ---- WiFi (antes del MPU: la espera no debe afectar la calibracion ni el filtro) ----
  conectarWiFi();

  // ---- MPU6050 ----
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);   // ms: evita que el bus se quede colgado esperando
  escanearI2C();
  mpuOk = iniciarMPU();
  if (!mpuOk) {
    Out.println("!!! MPU6050 NO RESPONDE. Revisa:");
    Out.print("    SDA -> GPIO "); Out.println(SDA_PIN);
    Out.print("    SCL -> GPIO "); Out.println(SCL_PIN);
    Out.println("    VCC -> 3V3, GND -> GND, AD0 -> GND");
  }

  ultimoControl = millis();
  ultimaImpresion = millis();
  ultimoReintento = millis();

  Out.println("=== PID Paso 5 (PID) listo: c = recalibrar | z = fijar cero | kp<num> ki<num> kd<num> tol<num> | m = control ON/OFF | s = stop ===");
}

void loop() {
  atenderWiFi();
  procesarComandos();
  atenderMPU();
  lazoControl();
  imprimirEstado();
}

// Conecta a la red WiFi (estacion). Espera hasta WIFI_ESPERA_MS; si no logra
// conectar, el balancin sigue funcionando por USB y reintenta desde loop().
void conectarWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);            // sin ahorro de energia: menor latencia
  Out.print("Conectando a la red '");
  Out.print(WIFI_SSID);
  Out.print("'");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < WIFI_ESPERA_MS) {
    delay(300);
    Out.print(".");
  }
  Out.println();
  ultimoReintentoWiFi = millis();
}

// Arranca WebSocket + mDNS cuando hay red; reintenta la conexion si se cae
void atenderWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!serviciosWiFiOk) {
      serviciosWiFiOk = true;
      if (MDNS.begin(HOSTNAME)) MDNS.addService("ws", "tcp", WS_PORT);
      webSocket.begin();
      webSocket.onEvent(eventoWebSocket);
      webSocket.enableHeartbeat(15000, 3000, 2);   // detecta paginas que se cerraron sin avisar
      Out.print(">>> WiFi conectado. IP: ");
      Out.println(WiFi.localIP());
      Out.print("    En la pagina usa:  ");
      Out.print(WiFi.localIP());
      Out.print("   o   ");
      Out.print(HOSTNAME);
      Out.println(".local   (puerto 81)");
    }
    webSocket.loop();
  } else if (millis() - ultimoReintentoWiFi > WIFI_REINTENTO_MS) {
    ultimoReintentoWiFi = millis();
    WiFi.reconnect();
  }
}

// Eventos del WebSocket: solo encola el texto; se ejecuta en procesarComandos()
void eventoWebSocket(uint8_t num, WStype_t tipo, uint8_t* payload, size_t largo) {
  switch (tipo) {
    case WStype_CONNECTED:
      wsClientes++;
      webSocket.sendTXT(num, "=== Balancin WiFi conectado: telemetria y comandos activos ===");
      break;
    case WStype_DISCONNECTED:
      if (wsClientes > 0) wsClientes--;
      break;
    case WStype_TEXT: {
      int siguiente = (colaEscribir + 1) % COLA_N;
      if (siguiente != colaLeer && largo > 0 && largo < 64) {   // si la cola esta llena se descarta
        colaComandos[colaEscribir] = String((const char*)payload).substring(0, largo);
        colaEscribir = siguiente;
      }
      break;
    }
    default: break;
  }
}

// Lee comandos del Monitor Serial y de la pagina (WebSocket)
void procesarComandos() {
  if (Serial.available() > 0) {
    ejecutarComando(Serial.readStringUntil('\n'));
  }
  while (colaLeer != colaEscribir) {
    String c = colaComandos[colaLeer];
    colaLeer = (colaLeer + 1) % COLA_N;
    ejecutarComando(c);
  }
}

// Ejecuta un comando (viene del Serial o del WebSocket)
void ejecutarComando(String entrada) {
  entrada.trim();

  if (entrada.equalsIgnoreCase("c")) {
    controlActivo = false;
    detenerMotores();
    primeraMuestra = true;
    integralError = 0;
    Out.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    Out.println(mpuOk ? ">>> Recalibracion completa"
                         : "!!! No se pudo recalibrar: el MPU6050 no responde");
  } else if (entrada.equalsIgnoreCase("z")) {
    primeraMuestra = true;
    integralError = 0;
    if (mpuOk && ponerCeroActual()) {
      Out.print(">>> CERO fijado. El sensor marcaba ");
      Out.print(anguloCero, 2);
      Out.println(" grados en equilibrio (ahora marca 0).");
      Out.print("    Para guardarlo al reiniciar: #define ANGULO_CERO ");
      Out.println(anguloCero, 2);
    } else {
      Out.println("!!! No se pudo fijar el cero: el MPU6050 no responde");
    }
  } else if (entrada.equalsIgnoreCase("m")) {
    if (!mpuOk) {
      Out.println(">>> No se puede: el MPU6050 no responde");
    } else {
      controlActivo = !controlActivo;
      primeraMuestra = true;
      integralError = 0;
      if (!controlActivo) detenerMotores();
      Out.println(controlActivo ? ">>> CONTROL ACTIVADO" : ">>> Control desactivado");
    }
  } else if (entrada.equalsIgnoreCase("s")) {
    controlActivo = false;
    integralError = 0;
    detenerMotores();
    Out.println(">>> Detenido");
  } else if (entrada.length() > 3 && entrada.substring(0, 3).equalsIgnoreCase("tol")) {
    // Tolerancia alrededor del setpoint. Ej: tol0.5  tol=0.5  tol0 (sin tolerancia)
    int i = 3;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if (i < (int)entrada.length()) {
      tolGrados = constrain(entrada.substring(i).toFloat(), 0.0f, TOL_MAX_GRADOS);
      integralError = 0;                            // evita un salto al cambiar la tolerancia
      Out.print(">>> Tolerancia = +-");
      Out.print(tolGrados, 2);
      Out.println(" grados");
    } else {
      Out.println("!!! Escribe tol seguido de un numero. Ej: tol0.5");
    }
  } else if (entrada.length() > 1 && (entrada[0] == 'k' || entrada[0] == 'K')) {
    // Cambiar ganancias en vivo. Acepta: kp10  kp=10  k10  kd0.5  kd=0.5
    char cual = (char)tolower(entrada[1]);          // 'd' = Kd, 'i' = Ki, cualquier otra = Kp
    int i = 1;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.' && entrada[i] != '-') i++;
    if (i < (int)entrada.length()) {
      float valor = entrada.substring(i).toFloat();
      if (cual == 'd') {
        kd = valor;
        Out.print(">>> Kd = ");
        Out.println(kd, 3);
      } else if (cual == 'i') {
        ki = valor;
        integralError = 0;                          // evita un salto al cambiar Ki
        Out.print(">>> Ki = ");
        Out.println(ki, 3);
      } else {
        kp = valor;
        Out.print(">>> Kp = ");
        Out.println(kp, 3);
      }
    } else {
      Out.println("!!! Escribe kp, ki o kd seguido de un numero. Ej: kp10  ki0.5  kd0.5");
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
      integralError = 0;
      detenerMotores();
      Out.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Out.println(">>> MPU6050 reconectado");
  }
}

// LAZO INTERNO (paso 5, PID): se ejecuta solo cuando pasaron TS_MS (periodo de muestreo fijo)
//   if (tiempoActual - tiempoUltimo >= Ts) { ... }
void lazoControl() {
  unsigned long ahora = millis();
  if (ahora - ultimoControl < TS_MS) return;

  tsReal = ahora - ultimoControl;
  ultimoControl = ahora;

  if (!mpuOk) return;

  anguloMedido = anguloFinal();                // theta_med (del MPU)
  errorAngulo  = anguloSetpoint - anguloMedido; // e = theta_sp - theta_med

  // TOLERANCIA: si el balancin esta a menos de +-tolGrados del setpoint se considera
  // "en el cero" y P e I no reaccionan (errorControl = 0). errorAngulo sigue siendo el
  // error real (es el que se imprime y el que usa la derivada).
  float errorControl = (fabs(errorAngulo) <= tolGrados) ? 0.0f : errorAngulo;

  // PASO 4: derivada  de/dt = (e_k - e_k-1) / Ts   (grados/segundo)
  float dt = tsReal / 1000.0;
  float derivadaCruda = primeraMuestra ? 0.0 : (errorAngulo - errorPrevio) / dt;
  derivadaError = ALFA_D * derivadaCruda + (1.0 - ALFA_D) * derivadaError; // filtro suave
  errorPrevio = errorAngulo;
  primeraMuestra = false;

  // PASO 5: integral  = suma de e*Ts, con anti-windup
  if (controlActivo) {
    if (fabs(errorControl) < INTEGRAL_ZONA_GRADOS) {
      integralError += errorControl * dt;          // solo integra con error moderado y fuera de la tolerancia
    }
  } else {
    integralError = 0;                             // control apagado: no acumula
  }
  terminoI = ki * integralError;
  if (terminoI > INTEGRAL_MAX_U)  terminoI = INTEGRAL_MAX_U;   // tope del aporte
  if (terminoI < -INTEGRAL_MAX_U) terminoI = -INTEGRAL_MAX_U;
  if (ki != 0.0) integralError = terminoI / ki;    // mantiene la suma dentro del tope

  salidaU = kp * errorControl + terminoI + kd * derivadaError; // u = Kp*e + Ki*Int(e) + Kd*de/dt

  // PASO 3: u -> sentido + PWM -> motores
  if (controlActivo) {
    if (fabs(anguloMedido) > CORTE_GRADOS) {    // el balancin cayo: apagar
      controlActivo = false;
      integralError = 0;
      detenerMotores();
      Out.println("!!! Angulo fuera de rango: control desactivado");
    } else {
      aplicarSalida(salidaU);
    }
  }
}

// u -> motores. Signo de u = sentido; |u| = magnitud del PWM.
//   u > 0 -> ADELANTE      u < 0 -> REVERSA   (se invierte si INVERTIR_SENTIDO_MOTORES)
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
  bool positivo = (u > 0);
  if (INVERTIR_SENTIDO_MOTORES) positivo = !positivo;
  modoActual = positivo ? ADELANTE : REVERSA;

  bool adelante = (modoActual == REVERSA);      // misma inversion que aplicarMovimiento()
  motorA(adelante, pwmA);
  motorB(adelante, pwmB);
}

void imprimirEstado() {
  unsigned long ahora = millis();
  if (ahora - ultimaImpresion < INTERVALO_MS) return;
  ultimaImpresion = ahora;

  if (!mpuOk) {
    Out.println("Angulo: SIN SENSOR (revisa SDA/SCL)");
    return;
  }

  Out.print("Angulo: ");
  Out.print(anguloMedido, 2);
  Out.print(" | Error: ");
  Out.print(errorAngulo, 2);
  Out.print(" | de/dt: ");
  Out.print(derivadaError, 1);
  Out.print(" | Kp: ");
  Out.print(kp, 2);
  Out.print(" | Ki: ");
  Out.print(ki, 3);
  Out.print(" | Kd: ");
  Out.print(kd, 2);
  Out.print(" | I: ");
  Out.print(terminoI, 1);
  Out.print(" | Tol: ");
  Out.print(tolGrados, 2);
  Out.print(" | u: ");
  Out.print(salidaU, 2);
  Out.print(" | PWM: ");
  Out.print(controlActivo ? velocidadActual : 0);
  Out.print(" | Ts real: ");
  Out.print(tsReal);
  Out.println(controlActivo ? " ms [CONTROL]" : " ms");

  // Lo que lee el MPU6050 (acelerometro en g, giroscopio en grados/s)
  Out.print("   MPU -> Acel(g) X: ");
  Out.print(axRaw / ACCEL_SENS, 3);
  Out.print(" Y: ");
  Out.print(ayRaw / ACCEL_SENS, 3);
  Out.print(" Z: ");
  Out.print(azRaw / ACCEL_SENS, 3);
  Out.print(" | Giro(grados/s) X: ");
  Out.print((gxRaw - gyroOffsetX) / GYRO_SENS, 1);
  Out.print(" Y: ");
  Out.print((gyRaw - gyroOffsetY) / GYRO_SENS, 1);
  Out.print(" Z: ");
  Out.print(gzRaw / GYRO_SENS, 1);
  Out.print(" | Ang acel: ");
  Out.print(anguloDesdeAcelerometro(), 2);
  Out.print(" | Ang filtrado (sin cero): ");
  Out.println(anguloInclinacion, 2);
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
  Out.println("Escaneando bus I2C...");
  int encontrados = 0;
  for (uint8_t dir = 1; dir < 127; dir++) {
    Wire.beginTransmission(dir);
    if (Wire.endTransmission() == 0) {
      Out.print("  Dispositivo en 0x");
      if (dir < 16) Out.print("0");
      Out.println(dir, HEX);
      encontrados++;
    }
  }
  if (encontrados == 0) {
    Out.println("  No se encontro NINGUN dispositivo I2C.");
  }
}

bool calibrarGiroscopio() {
  Out.println("Calibrando giroscopio, no muevas el sensor...");
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
  Out.println("Calibracion lista.");
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
    Out.print("WHO_AM_I = 0x");
    Out.println(Wire.read(), HEX); // esperado: 0x68
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
