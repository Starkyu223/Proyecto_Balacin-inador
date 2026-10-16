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
    - 'vp' + numero -> Kvp: ganancia proporcional del lazo EXTERNO de velocidad (encoders). Ej: vp0.0005
    - 'vi' + numero -> Kvi: ganancia integral del lazo externo de velocidad. Ej: vi0.0002
    - 'vd' + numero -> Kvd: ganancia derivativa del lazo externo de velocidad (suele quedar en 0)
      (tambien valen kvp / kvi / kvd. Cambian los DOS motores a la vez)
    - 'v1p' 'v1i' 'v1d' / 'v2p' 'v2i' 'v2d' + numero -> lo mismo pero solo el motor 1 (izq)
      o el motor 2 (der). Ej: v1p0.0004  v2i0.0001
      (con vp0, vi0 y vd0 el lazo externo esta apagado y el robot se comporta como antes)
    - 'e'  -> pone en cero las cuentas de los encoders (sirve para medir las
              cuentas por vuelta girando una rueda a mano y mirando la linea Enc)
    - 'te' -> PRUEBA de encoders: con el control apagado y las RUEDAS EN EL AIRE,
              mueve cada motor 1 s con u > 0 y muestra cuantas cuentas conto.
              Ambas deben salir POSITIVAS; si una sale negativa, cambia
              ENC_INVERTIR_IZQ / ENC_INVERTIR_DER.

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

  CONTROL EN CASCADA CON ENCODERS (un lazo completo POR MOTOR):

      Sp_vel (0) -> e_vel -> [PID velocidad] -> sat -> theta_sp_i
      theta_sp_i -> e_theta = theta_sp_i - theta_med -> [PID angulo] -> u_i -> PWM_i -> motor_i
      realimentacion: encoder_i (velocidad)  y  MPU6050 (angulo, compartido)

    Cada motor tiene su PID de velocidad, su PID de angulo y su encoder. El lazo del
    MPU esta DENTRO del lazo de los encoders. La velocidad es + en el sentido de u > 0.
    Las ganancias del angulo (kp, ki, kd) son las mismas para los dos motores. Las del
    lazo de velocidad se pueden fijar por separado (v1p..., v2p...) o juntas (vp...).
    Se ajusta DESPUES de tener el lazo de angulo estable: primero vp, luego vi.

  PRUEBA SEGURA: primero con las RUEDAS EN EL AIRE. Activa con 'm', inclina el
  balancin y mira que las ruedas giren hacia donde se inclina. Si el balancin
  pasa de CORTE_GRADOS el control se apaga solo.

  Salida cada 200 ms (3 lineas):
    1) Angulo | Error | de/dt | Kp | Ki | Kd | I | u | PWM | Ts real
    2) Lectura del MPU: acelerometro X/Y/Z (g), giroscopio X/Y/Z (grados/s),
       angulo solo del acelerometro y angulo filtrado sin cero
    3) Encoders: cuentas y velocidad de cada rueda, theta_sp y u de cada motor, ganancias de velocidad M1 y M2

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
    integralAng[2]      suma acumulada de e*Ts por motor (grados*segundo)
    terminoI            aporte de la integral a u: Ki * integralAng (limitado, promedio de los 2 motores)
    errorPrevioAng[2]   e_k-1 por motor, error de la muestra anterior
    spTheta[2]          theta_sp de cada motor, salida del PID de velocidad
    vel[2]              velocidad de cada rueda (encoders)
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
    aplicarSalidaMotor()  (u de un motor -> sentido + PWM con zona muerta y limite)

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
const char* WIFI_SSID = "HOME1";
const char* WIFI_PASS = "ISABELLA";
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

// ---------- Encoders de cuadratura (motores 25GA370, salida Hall A/B) ----------
// Se leen por interrupcion en los dos canales (decodificacion x4).
#define ENC_IZQ_A 18
#define ENC_IZQ_B 8
#define ENC_DER_A 41
#define ENC_DER_B 39
// Usa el pull-up interno por si las salidas son de colector abierto (si son push-pull no molesta)
#define ENC_PULLUP true
// Cambia a true si con el comando 'te' la cuenta de esa rueda sale NEGATIVA
#define ENC_INVERTIR_IZQ true
#define ENC_INVERTIR_DER false

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
#define ANGULO_CERO -4.92

const float ACCEL_SENS = 16384.0;
const float GYRO_SENS  = 131.0;
const float ALPHA      = 0.98;    // filtro complementario

// ---------- Lazo de control ----------
const unsigned long TS_MS = 10;   // periodo de muestreo (Tsample) = 100 Hz
const float ANGULO_SP = 0.0;      // theta_sp: balancin vertical
// u esta en unidades de PWM y el error en grados: con Kp = 8, un error de 30
// grados da u = 240 (casi PWM maximo). Ajustalo en vivo con 'k'.
const float KP_INICIAL = 37.5;    // ganancia proporcional (la que elegiste)
const float KI_INICIAL = 0.0;     // ganancia integral: arranca en 0 (= PD); subela con 'ki'
const float KD_INICIAL = 2;     // ganancia derivativa: arranca en 0 (= solo P); subela con 'kd'
const float ALFA_D = 0.3;         // filtro de la derivada: peso de la muestra nueva (0-1). Menor = mas suave

// ---------- Lazo externo de velocidad (encoders): un PID por motor ----------
// Cuentas por vuelta de la RUEDA (con decodificacion x4). Midelas con el comando 'e':
// pon en cero, gira la rueda una vuelta a mano y lee la cuenta. Con 0 la velocidad
// se expresa en cuentas/s; con un valor > 0 se expresa en rpm.
// Medido a mano (3 vueltas, decodificacion x4): izq 10284/3 = 3428, der 10026/3 = 3342.
const float CUENTAS_POR_REV_IZQ = 3428;
const float CUENTAS_POR_REV_DER = 3342;
const float SP_VEL = 0.0;          // Sp_rpm = 0: el robot debe quedarse quieto
// Salida en grados. Arrancan en 0 = lazo apagado.
// En cuentas/s un orden de magnitud para empezar es Kvp ~ 0.0005; en rpm sera mucho mayor.
const float KVP_INICIAL = 0.0;
const float KVI_INICIAL = 0.0;
const float KVD_INICIAL = 0.0;
const float SP_MAX_GRADOS = 4.0;   // tope del setpoint de angulo que puede pedir el lazo externo
const float ALFA_V = 0.2;          // filtro de la velocidad: peso de la muestra nueva (0-1)
const float VEL_TEST_U = 60.0;     // u usada por el comando 'te'

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
float ki = KI_INICIAL;            // ganancia integral
float tolGrados = TOL_INICIAL;    // tolerancia +- alrededor del setpoint (grados)
float kd = KD_INICIAL;            // ganancia derivativa
float terminoI = 0;               // (promedio de los dos motores, para imprimir)
float derivadaError = 0;          // (promedio de los dos motores, para imprimir)
// Estado por motor (indice 0 = izquierdo, 1 = derecho): cada uno con su propio lazo
float spTheta[2] = {0, 0};        // theta_sp que le pide el lazo de velocidad (grados)
float errorAng[2] = {0, 0};       // e = theta_sp - theta_med
float integralAng[2] = {0, 0};    // suma acumulada de e*Ts (lazo de angulo)
float terminoIAng[2] = {0, 0};    // aporte de la integral a u
float errorPrevioAng[2] = {0, 0}; // e_k-1
float derivadaAng[2] = {0, 0};    // de/dt filtrada (grados/s)
float uAng[2] = {0, 0};           // u de cada motor
int   pwmCanal[2] = {0, 0};       // PWM aplicado a cada motor
bool primeraMuestra = true;       // evita un pico de derivada al arrancar o tras fijar el cero
float salidaU = 0;                // u promedio de los dos motores (para imprimir)
unsigned long ultimoControl = 0;
unsigned long tsReal = 0;         // ms reales entre dos ejecuciones del lazo

// ---------- Encoders ----------
volatile int32_t cuentasIzq = 0, cuentasDer = 0;   // se modifican en las interrupciones
volatile uint8_t estadoIzq = 0, estadoDer = 0;     // estado anterior (A<<1 | B)
portMUX_TYPE muxEnc = portMUX_INITIALIZER_UNLOCKED;
int32_t cuentasIzqPrev = 0, cuentasDerPrev = 0;
float vel[2] = {0, 0};            // velocidad de cada rueda (+ = sentido de u > 0), en UNIDAD_VEL
float kvp[2] = {KVP_INICIAL, KVP_INICIAL};   // ganancia proporcional del lazo de velocidad (motor 1, motor 2)
float kvi[2] = {KVI_INICIAL, KVI_INICIAL};   // ganancia integral del lazo de velocidad
float kvd[2] = {KVD_INICIAL, KVD_INICIAL};   // ganancia derivativa del lazo de velocidad
float integralVel[2] = {0, 0};    // suma acumulada de e_vel*Ts por motor
float errorVelPrevio[2] = {0, 0};
float derivadaVel[2] = {0, 0};

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
void aplicarSalidaMotor(int c, float u);
void reiniciarIntegrales();
void motorA(bool dirForward, int pwm);
void motorB(bool dirForward, int pwm);
void aplicarMovimiento();
void detenerMotores();
void imprimirEstado();
void iniciarEncoders();
void actualizarEncoders(float dt);
void leerCuentas(int32_t &izq, int32_t &der);
void pruebaEncoders();
void ISR_encIzq();
void ISR_encDer();
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
  iniciarEncoders();

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
  // kvp / kvi / kvd son sinonimos de vp / vi / vd (sin esto 'k' los leeria como Kp del angulo)
  if (entrada.length() > 2 && (entrada[0] == 'k' || entrada[0] == 'K') && (entrada[1] == 'v' || entrada[1] == 'V')) {
    entrada = entrada.substring(1);
  }

  if (entrada.equalsIgnoreCase("c")) {
    controlActivo = false;
    detenerMotores();
    primeraMuestra = true;
    reiniciarIntegrales();
    Out.println(">>> Recalibrando MPU6050, mantenlo quieto...");
    mpuOk = iniciarMPU();
    Out.println(mpuOk ? ">>> Recalibracion completa"
                         : "!!! No se pudo recalibrar: el MPU6050 no responde");
  } else if (entrada.equalsIgnoreCase("z")) {
    primeraMuestra = true;
    reiniciarIntegrales();
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
      reiniciarIntegrales();
      reiniciarIntegrales();
      if (!controlActivo) detenerMotores();
      Out.println(controlActivo ? ">>> CONTROL ACTIVADO" : ">>> Control desactivado");
    }
  } else if (entrada.equalsIgnoreCase("s")) {
    controlActivo = false;
    reiniciarIntegrales();
    reiniciarIntegrales();
    detenerMotores();
    Out.println(">>> Detenido");
  } else if (entrada.equalsIgnoreCase("e")) {
    int32_t cI, cD;
    portENTER_CRITICAL(&muxEnc);
    cuentasIzq = 0;
    cuentasDer = 0;
    portEXIT_CRITICAL(&muxEnc);
    leerCuentas(cI, cD);
    cuentasIzqPrev = cI;
    cuentasDerPrev = cD;
    Out.println(">>> Encoders en cero. Gira una rueda a mano 1 vuelta y mira la linea Enc para ver sus cuentas por vuelta.");
  } else if (entrada.equalsIgnoreCase("te")) {
    pruebaEncoders();
  } else if (entrada.length() > 2 && (entrada[0] == 'v' || entrada[0] == 'V')) {
    // Ganancias del lazo de velocidad (encoders):
    //   vp / vi / vd  + numero -> los DOS motores.   Ej: vp0.0005
    //   v1p v1i v1d / v2p v2i v2d + numero -> solo el motor 1 (izq) o el motor 2 (der). Ej: v2i0.0002
    int motor = -1, pos = 1;                          // -1 = ambos
    if (entrada[1] == '1' || entrada[1] == '2') { motor = entrada[1] - '1'; pos = 2; }
    char cual = (char)tolower(entrada[pos]);
    int i = pos + 1;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if ((cual == 'p' || cual == 'i' || cual == 'd') && i < (int)entrada.length()) {
      float valor = entrada.substring(i).toFloat();
      for (int c = 0; c < 2; c++) {
        if (motor >= 0 && motor != c) continue;
        if (cual == 'p') kvp[c] = valor;
        else if (cual == 'i') kvi[c] = valor;
        else kvd[c] = valor;
      }
      reiniciarIntegrales();                          // evita un salto al cambiar la ganancia
      Out.print(">>> Kv");
      Out.print(cual);
      Out.print(motor < 0 ? " (motores 1 y 2) = " : (motor == 0 ? " (motor 1) = " : " (motor 2) = "));
      Out.println(valor, 6);
    } else {
      Out.println("!!! Usa vp/vi/vd<num> (ambos motores) o v1p v1i v1d v2p v2i v2d <num>. Ej: vp0.0005  v2i0.0002");
    }
  } else if (entrada.length() > 3 && entrada.substring(0, 3).equalsIgnoreCase("tol")) {
    // Tolerancia alrededor del setpoint. Ej: tol0.5  tol=0.5  tol0 (sin tolerancia)
    int i = 3;
    while (i < (int)entrada.length() && !isDigit(entrada[i]) && entrada[i] != '.') i++;
    if (i < (int)entrada.length()) {
      tolGrados = constrain(entrada.substring(i).toFloat(), 0.0f, TOL_MAX_GRADOS);
      reiniciarIntegrales();                            // evita un salto al cambiar la tolerancia
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
        reiniciarIntegrales();                          // evita un salto al cambiar Ki
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
      reiniciarIntegrales();
      detenerMotores();
      Out.println("!!! Se perdio la comunicacion con el MPU6050");
    }
  } else if (millis() - ultimoReintento > REINTENTO_MPU_MS) {
    ultimoReintento = millis();
    mpuOk = iniciarMPU();
    if (mpuOk) Out.println(">>> MPU6050 reconectado");
  }
}

// CONTROL EN CASCADA, UN LAZO COMPLETO POR MOTOR (canal 0 = izquierdo, 1 = derecho).
// Se ejecuta solo cuando pasaron TS_MS (periodo de muestreo fijo).
//
//   SP_vel (0) -> e_vel = SP_vel - vel_i -> [PID velocidad] -> sat -> theta_sp_i
//   theta_sp_i -> e_theta = theta_sp_i - theta_med -> [PID angulo] -> u_i -> PWM_i -> motor_i
//
//   vel_i viene del encoder del motor i; theta_med es el MPU6050 (compartido).
//   El lazo del angulo esta DENTRO del lazo de velocidad.
void lazoControl() {
  unsigned long ahora = millis();
  if (ahora - ultimoControl < TS_MS) return;

  tsReal = ahora - ultimoControl;
  ultimoControl = ahora;
  float dt = tsReal / 1000.0;

  // Encoders: velocidad de cada rueda (se actualiza siempre, con o sin control)
  actualizarEncoders(dt);

  if (!mpuOk) return;

  anguloMedido = anguloFinal();                // theta_med (del MPU)

  for (int c = 0; c < 2; c++) {
    // ---------- LAZO EXTERNO: PID de velocidad -> theta_sp del motor c ----------
    if (controlActivo) {
      float eVel = SP_VEL - vel[c];
      float dVelCruda = primeraMuestra ? 0.0 : (eVel - errorVelPrevio[c]) / dt;
      derivadaVel[c] = ALFA_D * dVelCruda + (1.0 - ALFA_D) * derivadaVel[c];
      errorVelPrevio[c] = eVel;

      integralVel[c] += eVel * dt;
      float tIv = kvi[c] * integralVel[c];
      if (tIv > SP_MAX_GRADOS)  tIv = SP_MAX_GRADOS;      // anti-windup del lazo externo
      if (tIv < -SP_MAX_GRADOS) tIv = -SP_MAX_GRADOS;
      if (kvi[c] != 0.0) integralVel[c] = tIv / kvi[c];

      // saturador: acota el angulo que puede pedir la velocidad.
      // El signo es NEGATIVO a proposito: vel es + en el sentido de u > 0, y u > 0 mueve las
      // ruedas hacia el lado de angulo NEGATIVO. Para frenar un movimiento hacia ese lado el
      // robot debe inclinarse hacia el lado contrario (angulo +), asi que theta_sp = -(PID).
      // (Con el signo contrario el lazo externo REALIMENTA POSITIVAMENTE y el robot se escapa:
      //  confirmado en la simulacion Programacion/simulacion.)
      spTheta[c] = constrain(-(kvp[c] * eVel + tIv + kvd[c] * derivadaVel[c]), -SP_MAX_GRADOS, SP_MAX_GRADOS);
    } else {
      spTheta[c] = ANGULO_SP;
      integralVel[c] = 0;
      errorVelPrevio[c] = 0;
      derivadaVel[c] = 0;
    }

    // ---------- LAZO INTERNO: PID de angulo del motor c ----------
    float e = spTheta[c] - anguloMedido;           // e = theta_sp - theta_med
    errorAng[c] = e;

    // TOLERANCIA: dentro de +-tolGrados P e I ven error 0 (la derivada usa el error real)
    float eControl = (fabs(e) <= tolGrados) ? 0.0f : e;

    float dCruda = primeraMuestra ? 0.0 : (e - errorPrevioAng[c]) / dt;
    derivadaAng[c] = ALFA_D * dCruda + (1.0 - ALFA_D) * derivadaAng[c];   // filtro suave
    errorPrevioAng[c] = e;

    if (controlActivo) {
      if (fabs(eControl) < INTEGRAL_ZONA_GRADOS) integralAng[c] += eControl * dt;  // anti-windup por zona
    } else {
      integralAng[c] = 0;
    }
    float tI = ki * integralAng[c];
    if (tI > INTEGRAL_MAX_U)  tI = INTEGRAL_MAX_U;                        // tope del aporte
    if (tI < -INTEGRAL_MAX_U) tI = -INTEGRAL_MAX_U;
    if (ki != 0.0) integralAng[c] = tI / ki;
    terminoIAng[c] = tI;

    uAng[c] = kp * eControl + tI + kd * derivadaAng[c];   // u = Kp*e + Ki*Int(e) + Kd*de/dt
  }
  primeraMuestra = false;

  // Promedios de los dos canales: lo que se imprime en la linea 1 de telemetria
  anguloSetpoint = 0.5 * (spTheta[0] + spTheta[1]);
  errorAngulo    = 0.5 * (errorAng[0] + errorAng[1]);
  derivadaError  = 0.5 * (derivadaAng[0] + derivadaAng[1]);
  terminoI       = 0.5 * (terminoIAng[0] + terminoIAng[1]);
  salidaU        = 0.5 * (uAng[0] + uAng[1]);

  // u -> sentido + PWM -> motores
  if (controlActivo) {
    if (fabs(anguloMedido) > CORTE_GRADOS) {    // el balancin cayo: apagar
      controlActivo = false;
      reiniciarIntegrales();
      detenerMotores();
      Out.println("!!! Angulo fuera de rango: control desactivado");
    } else {
      aplicarSalidaMotor(0, uAng[0]);
      aplicarSalidaMotor(1, uAng[1]);
      velocidadActual = (pwmCanal[0] + pwmCanal[1]) / 2;   // solo para imprimir
    }
  }
}

// Pone en cero todos los acumuladores de integral (angulo y velocidad de ambos motores)
void reiniciarIntegrales() {
  for (int c = 0; c < 2; c++) {
    integralAng[c] = 0;
    integralVel[c] = 0;
  }
}

// u -> motor del canal c. Signo de u = sentido; |u| = magnitud del PWM.
//   |u| < ZONA_MUERTA_U -> detenido
//   si no: PWM = PWM_MIN del motor + |u| reescalado hasta PWM_MAX
//          (cada motor tiene su propio minimo, asi arrancan a la vez)
//   Sentido: u > 0 -> adelante = INVERTIR_SENTIDO_MOTORES (u < 0 al reves)
void aplicarSalidaMotor(int c, float u) {
  float mag = fabs(u);
  if (mag < ZONA_MUERTA_U) {
    ledcWrite(c == 0 ? PWMA : PWMB, 0);
    pwmCanal[c] = 0;
    return;
  }

  mag = constrain(mag, 0.0f, (float)PWM_MAX);
  int pwmMin = (c == 0) ? PWM_MIN_A : PWM_MIN_B;
  int pwm = pwmMin + (int)(mag * (PWM_MAX - pwmMin) / PWM_MAX);
  pwmCanal[c] = pwm;

  bool adelante = INVERTIR_SENTIDO_MOTORES ? (u > 0) : !(u > 0);
  modoActual = adelante ? REVERSA : ADELANTE;   // solo informativo
  if (c == 0) motorA(adelante, pwm);
  else        motorB(adelante, pwm);
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

  // Encoders (cuentas acumuladas y cuentas/s por rueda) y lazo externo de velocidad
  int32_t cI, cD;
  leerCuentas(cI, cD);
  const char* uv = (CUENTAS_POR_REV_IZQ > 0 && CUENTAS_POR_REV_DER > 0) ? "rpm" : "c/s";
  Out.print("   Enc -> Izq: ");
  Out.print(cI);
  Out.print(" (");
  Out.print(vel[0], 0);
  Out.print(uv);
  Out.print(") Der: ");
  Out.print(cD);
  Out.print(" (");
  Out.print(vel[1], 0);
  Out.print(uv);
  Out.print(") | SP_theta Izq: ");
  Out.print(spTheta[0], 2);
  Out.print(" Der: ");
  Out.print(spTheta[1], 2);
  Out.print(" | u Izq: ");
  Out.print(uAng[0], 1);
  Out.print(" Der: ");
  Out.print(uAng[1], 1);
  Out.print(" | M1 Kp: ");
  Out.print(kvp[0], 6);
  Out.print(" Ki: ");
  Out.print(kvi[0], 6);
  Out.print(" Kd: ");
  Out.print(kvd[0], 6);
  Out.print(" | M2 Kp: ");
  Out.print(kvp[1], 6);
  Out.print(" Ki: ");
  Out.print(kvi[1], 6);
  Out.print(" Kd: ");
  Out.println(kvd[1], 6);
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

// ---------- Encoders ----------
// Tabla de cuadratura x4: indice = (estado_anterior << 2) | estado_nuevo (cada estado = A<<1 | B)
static const int8_t TABLA_QUAD[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR ISR_encIzq() {
  uint8_t nuevo = (digitalRead(ENC_IZQ_A) << 1) | digitalRead(ENC_IZQ_B);
  int8_t d = TABLA_QUAD[(estadoIzq << 2) | nuevo];
  estadoIzq = nuevo;
  cuentasIzq += ENC_INVERTIR_IZQ ? -d : d;
}

void IRAM_ATTR ISR_encDer() {
  uint8_t nuevo = (digitalRead(ENC_DER_A) << 1) | digitalRead(ENC_DER_B);
  int8_t d = TABLA_QUAD[(estadoDer << 2) | nuevo];
  estadoDer = nuevo;
  cuentasDer += ENC_INVERTIR_DER ? -d : d;
}

void iniciarEncoders() {
  const int modo = ENC_PULLUP ? INPUT_PULLUP : INPUT;
  pinMode(ENC_IZQ_A, modo);
  pinMode(ENC_IZQ_B, modo);
  pinMode(ENC_DER_A, modo);
  pinMode(ENC_DER_B, modo);
  estadoIzq = (digitalRead(ENC_IZQ_A) << 1) | digitalRead(ENC_IZQ_B);
  estadoDer = (digitalRead(ENC_DER_A) << 1) | digitalRead(ENC_DER_B);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_A), ISR_encIzq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_IZQ_B), ISR_encIzq, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_A), ISR_encDer, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_DER_B), ISR_encDer, CHANGE);
}

// Copia atomica de las cuentas (las interrupciones las modifican en cualquier momento)
void leerCuentas(int32_t &izq, int32_t &der) {
  portENTER_CRITICAL(&muxEnc);
  izq = cuentasIzq;
  der = cuentasDer;
  portEXIT_CRITICAL(&muxEnc);
}

// Velocidad de cada rueda (cuentas/s) = cuentas nuevas / dt, con filtro de primer orden
void actualizarEncoders(float dt) {
  if (dt <= 0) return;
  int32_t cI, cD;
  leerCuentas(cI, cD);
  float vI = (cI - cuentasIzqPrev) / dt;
  float vD = (cD - cuentasDerPrev) / dt;
  cuentasIzqPrev = cI;
  cuentasDerPrev = cD;
  const float kI = (CUENTAS_POR_REV_IZQ > 0) ? 60.0 / CUENTAS_POR_REV_IZQ : 1.0;   // cuentas/s -> rpm
  const float kD = (CUENTAS_POR_REV_DER > 0) ? 60.0 / CUENTAS_POR_REV_DER : 1.0;
  vel[0] = ALFA_V * (vI * kI) + (1.0 - ALFA_V) * vel[0];
  vel[1] = ALFA_V * (vD * kD) + (1.0 - ALFA_V) * vel[1];
}

// Prueba: cada motor 1 s con u = +VEL_TEST_U. Las cuentas deben salir POSITIVAS.
// Bloquea ~2.5 s; solo con el control apagado y las ruedas en el aire.
void pruebaEncoders() {
  if (controlActivo) {
    Out.println("!!! Apaga el control ('s') antes de probar los encoders");
    return;
  }
  // Sentido de u > 0: aplicarSalidaMotor() usa adelante = INVERTIR_SENTIDO_MOTORES para u > 0
  const bool adelante = INVERTIR_SENTIDO_MOTORES;
  float mag = constrain(VEL_TEST_U, 0.0f, (float)PWM_MAX);
  int32_t i0, d0, i1, d1;

  Out.println(">>> Prueba de encoders (ruedas en el aire): motor izquierdo 1 s...");
  leerCuentas(i0, d0);
  motorA(adelante, PWM_MIN_A + (int)(mag * (PWM_MAX - PWM_MIN_A) / PWM_MAX));
  delay(1000);
  detenerMotores();
  delay(300);
  leerCuentas(i1, d1);
  Out.print("    Izquierda: ");
  Out.print(i1 - i0);
  Out.print(" cuentas (la derecha conto ");
  Out.print(d1 - d0);
  Out.println(")");

  Out.println(">>> motor derecho 1 s...");
  leerCuentas(i0, d0);
  motorB(adelante, PWM_MIN_B + (int)(mag * (PWM_MAX - PWM_MIN_B) / PWM_MAX));
  delay(1000);
  detenerMotores();
  delay(300);
  leerCuentas(i1, d1);
  Out.print("    Derecha: ");
  Out.print(d1 - d0);
  Out.print(" cuentas (la izquierda conto ");
  Out.print(i1 - i0);
  Out.println(")");
  Out.println(">>> Ambas deben ser POSITIVAS y la otra rueda ~0. Si no, ajusta ENC_INVERTIR_* o revisa el cableado A/B.");
  cuentasIzqPrev = i1;
  cuentasDerPrev = d1;
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
