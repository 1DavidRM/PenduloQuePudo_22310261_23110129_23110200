/*
  PENDULO - PASO 2: Control PID de VELOCIDAD y POSICION + modo SIERRA (rampas de velocidad)
  Placa: ESP32 (Arduino IDE)  |  Puente H: L298N  |  Motor DC con encoder en cuadratura

  Conexiones (las de tu diagrama, sin cambios):
    GPIO25 -> ENA (PWM)      GPIO26 -> IN1      GPIO27 -> IN2
    GPIO32 -> Encoder canal A   GPIO33 -> Encoder canal B
    GPIO34 -> Cursor del potenciometro 10k (3V3 y GND en las patas)

  Comandos por el Monitor Serial (115200 baud, "Nueva linea"):
    v            -> modo VELOCIDAD
    p            -> modo POSICION
    sierra       -> modo SIERRA: acelera en rampa, se detiene, y repite
    sierra <rpm> <t_rampa_s> <t_pausa_s>  -> igual, con tus valores (ej: sierra 25 3 1.5)
    pot          -> el setpoint lo da el potenciometro
    s <valor>    -> setpoint por serial (RPM en modo v, grados en modo p)
    kp <x>  ki <x>  kd <x>   -> cambia ganancias del modo actual
    cero         -> HOME MANUAL: la posicion actual pasa a ser 0
    stop / go    -> deshabilita / habilita el motor (stop tambien sale del modo sierra)
    t 1..4       -> que curvas manda al Serial Plotter:
                      1 = SP, PV, U (para afinar el PID)
                      2 = Vel_RPM y Acel_rad_s2
                      3 = Pos_deg, Vel_RPM y Acel_rad_s2
                      4 = SP_RPM y Vel_RPM (la sierra: referencia contra velocidad real)
    ?            -> muestra estado y ganancias
*/

// ======================== PINES ========================
const int PIN_ENA = 25;
const int PIN_IN1 = 26;
const int PIN_IN2 = 27;
const int PIN_ENC_A = 32;
const int PIN_ENC_B = 33;
const int PIN_POT = 34;

// ======================== PARAMETROS A AJUSTAR ========================
const float ENC_CPR = 1497.6;      // cuentas por vuelta (medido a mano)
const bool  ENC_INVERTIR = false;  // true si al girar "hacia adelante" el conteo baja

const float RPM_MAX = 100.0;       // velocidad maxima con el pot (+/-)
const float POS_RANGO_DEG = 360.0; // recorrido del pot en modo posicion (0..este valor)
const float ZONA_MUERTA_POT = 0.05;// 5% alrededor del centro en modo velocidad
const float POS_TOLERANCIA_DEG = 1.0; // dentro de esto, el motor se apaga
const int   PWM_MIN_POS = 80;      // PWM minimo en modo posicion (vence la friccion)

const int   PWM_FREQ = 20000;      // 20 kHz (silencioso)
const int   PWM_BITS = 8;          // 0..255
const int   PWM_MAX = 255;

const unsigned long TS_US = 10000; // periodo del lazo: 10 ms (100 Hz)
const unsigned long TELEMETRIA_MS = 20; // 50 muestras por segundo para las curvas

// ---- Modo sierra (valores por defecto, se cambian con el comando 'sierra') ----
const float SIERRA_RPM = 25.0;     // velocidad a la que llega la rampa
const float SIERRA_T_RAMPA = 3.0;  // segundos que dura la aceleracion
const float SIERRA_T_PAUSA = 1.5;  // segundos detenido antes de la siguiente rampa
const bool  SIERRA_FRENAR = true;  // true: frena el motor (se detiene rapido); false: lo deja libre

// ---- Curvas ----
const int TEL_MODO_INICIAL = 4;    // 1 = SP/PV/U, 2 = Vel y Acel, 3 = Pos/Vel/Acel, 4 = SP y Vel
const int ACEL_N = 10;             // ventana de la aceleracion: 10 muestras = 100 ms
const float RPM_S_A_RAD_S2 = 0.10472f;  // 1 RPM/s = 0.10472 rad/s^2

// ======================== TIPO PID ========================
// IMPORTANTE: debe ir ANTES de cualquier funcion, si no el Arduino IDE
// genera prototipos que usan 'PID' antes de que exista y marca error.
struct PID {
  float kp, ki, kd;
  float integral;
  float prevMeas;
  float outMin, outMax;
};

// ======================== ENCODER ========================
volatile long encCount = 0;
volatile uint8_t encLast = 0;
// Tabla de decodificacion en cuadratura (estado anterior<<2 | estado actual)
const int8_t QEM[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};

void IRAM_ATTR isrEncoder() {
  uint8_t s = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  encCount += QEM[(encLast << 2) | s];
  encLast = s;
}

// ======================== PID ========================
PID pidVel = {0.8f, 4.0f, 0.0f, 0, 0, -PWM_MAX, PWM_MAX};
PID pidPos = {4.0f, 3.0f, 0.5f, 0, 0, -PWM_MAX, PWM_MAX};

float pidCalcular(PID &p, float sp, float pv, float dt) {
  float err = sp - pv;
  // Integral con anti-windup (se limita al rango de salida)
  p.integral += p.ki * err * dt;
  p.integral = constrain(p.integral, p.outMin, p.outMax);
  // Derivada sobre la medicion (evita picos al cambiar el setpoint)
  float dMeas = (pv - p.prevMeas) / dt;
  p.prevMeas = pv;
  float out = p.kp * err + p.integral - p.kd * dMeas;
  return constrain(out, p.outMin, p.outMax);
}

void pidReset(PID &p, float pv) {
  p.integral = 0;
  p.prevMeas = pv;
}

// ======================== ESTADO ========================
enum Modo { MODO_VEL, MODO_POS, MODO_SIERRA };
Modo modo = MODO_VEL;
bool setpointPot = true;
bool habilitado = true;
float setpointSerial = 0;

float setpoint = 0, medicion = 0, salida = 0;
float rpmFiltrada = 0;
float posActual = 0;               // posicion en grados (para las curvas)
float acelFiltrada = 0;            // aceleracion en RPM/s
float velHist[ACEL_N];             // historial de velocidad para calcular la aceleracion
int velIdx = 0;
int telModo = TEL_MODO_INICIAL;
long encPrev = 0;
float potFiltrado = 0;
unsigned long tPrev = 0, tTel = 0;

float sierraRpm = SIERRA_RPM;
float sierraTRampa = SIERRA_T_RAMPA;
float sierraTPausa = SIERRA_T_PAUSA;
unsigned long sierraT0 = 0;

// ======================== MOTOR ========================
void pwmEscribir(int duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_ENA, duty);
#else
  ledcWrite(0, duty);
#endif
}

void motorAplicar(float u) {
  int duty = constrain((int)fabs(u), 0, PWM_MAX);

  if (!habilitado || duty == 0) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    pwmEscribir(0);
    return;
  }
  if (u > 0) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  }
  pwmEscribir(duty);
}

// Freno: pone en corto las terminales del motor (se detiene rapido)
void motorFreno() {
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, HIGH);
  pwmEscribir(PWM_MAX);
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_ENA, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(0, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PIN_ENA, 0);
#endif
  motorAplicar(0);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_POT, ADC_11db); // rango completo 0-3.3 V

  encLast = (digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), isrEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), isrEncoder, CHANGE);

  potFiltrado = analogRead(PIN_POT);
  tPrev = micros();

  Serial.println("PENDULO - Paso 2 listo. Escribe '?' para ver comandos/estado.");
}

// ======================== LAZO DE CONTROL ========================
void lazoControl(float dt) {
  // --- Lectura del encoder ---
  long cuentas = encCount;
  if (ENC_INVERTIR) cuentas = -cuentas;
  long delta = cuentas - encPrev;
  encPrev = cuentas;

  float rpm = (delta / ENC_CPR) / dt * 60.0f;
  rpmFiltrada += 0.3f * (rpm - rpmFiltrada);       // filtro pasa-bajas
  float posDeg = cuentas * 360.0f / ENC_CPR;
  posActual = posDeg;

  // --- Aceleracion: diferencia de velocidad en una ventana de ACEL_N muestras ---
  float velAnt = velHist[velIdx];
  velHist[velIdx] = rpmFiltrada;
  velIdx = (velIdx + 1) % ACEL_N;
  float acel = (rpmFiltrada - velAnt) / (ACEL_N * dt);   // RPM/s
  acelFiltrada += 0.3f * (acel - acelFiltrada);

  // --- Potenciometro (filtrado, normalizado 0..1) ---
  potFiltrado += 0.2f * (analogRead(PIN_POT) - potFiltrado);
  float potNorm = potFiltrado / 4095.0f;

  // --- Setpoint y PID segun el modo ---
  if (modo == MODO_VEL) {
    if (setpointPot) {
      float c = (potNorm - 0.5f) * 2.0f;           // -1..+1
      if (fabs(c) < ZONA_MUERTA_POT) c = 0;
      setpoint = c * RPM_MAX;
    } else {
      setpoint = setpointSerial;
    }
    medicion = rpmFiltrada;
    salida = pidCalcular(pidVel, setpoint, medicion, dt);
    motorAplicar(salida);

  } else if (modo == MODO_SIERRA) {
    // Rampa lineal de 0 a sierraRpm durante sierraTRampa, luego pausa detenido
    float t = (millis() - sierraT0) / 1000.0f;
    float ciclo = sierraTRampa + sierraTPausa;
    float tc = fmodf(t, ciclo);
    medicion = rpmFiltrada;
    if (tc < sierraTRampa) {
      setpoint = sierraRpm * tc / sierraTRampa;
      salida = pidCalcular(pidVel, setpoint, medicion, dt);
      motorAplicar(salida);
    } else {
      setpoint = 0;
      salida = 0;
      pidReset(pidVel, medicion);
      if (SIERRA_FRENAR && habilitado) motorFreno();
      else motorAplicar(0);
    }

  } else {  // MODO_POS
    setpoint = setpointPot ? potNorm * POS_RANGO_DEG : setpointSerial;
    medicion = posDeg;
    if (fabs(setpoint - posDeg) < POS_TOLERANCIA_DEG) {
      salida = 0;
      pidCalcular(pidPos, setpoint, medicion, dt);  // mantiene estado actualizado
    } else {
      salida = pidCalcular(pidPos, setpoint, medicion, dt);
      // PWM minimo para vencer la friccion (solo en posicion)
      if (fabs(salida) < PWM_MIN_POS) salida = (salida >= 0 ? 1 : -1) * PWM_MIN_POS;
    }
    motorAplicar(salida);
  }
}

// ======================== TELEMETRIA (Serial Plotter) ========================
void telemetria() {
  float acelRad = acelFiltrada * RPM_S_A_RAD_S2;   // rad/s^2
  switch (telModo) {
    case 1:
      Serial.printf("SP:%.2f,PV:%.2f,U:%.0f\n", setpoint, medicion, salida);
      break;
    case 2:
      Serial.printf("Vel_RPM:%.2f,Acel_rad_s2:%.2f\n", rpmFiltrada, acelRad);
      break;
    case 3:
      Serial.printf("Pos_deg:%.2f,Vel_RPM:%.2f,Acel_rad_s2:%.2f\n", posActual, rpmFiltrada, acelRad);
      break;
    case 4:
      Serial.printf("SP_RPM:%.2f,Vel_RPM:%.2f\n", setpoint, rpmFiltrada);
      break;
  }
}

// ======================== SERIAL ========================
void imprimirEstado() {
  Serial.printf("Modo: %s | Setpoint: %s | Motor: %s\n",
                modo == MODO_VEL ? "VELOCIDAD (RPM)" :
                modo == MODO_POS ? "POSICION (grados)" : "SIERRA (rampas de velocidad)",
                setpointPot ? "potenciometro" : "serial",
                habilitado ? "habilitado" : "detenido");
  Serial.printf("Vel  -> Kp=%.3f Ki=%.3f Kd=%.3f\n", pidVel.kp, pidVel.ki, pidVel.kd);
  Serial.printf("Pos  -> Kp=%.3f Ki=%.3f Kd=%.3f\n", pidPos.kp, pidPos.ki, pidPos.kd);
  Serial.printf("Sierra -> %.1f RPM, rampa %.2f s, pausa %.2f s (aceleracion %.2f RPM/s)\n",
                sierraRpm, sierraTRampa, sierraTPausa, sierraRpm / sierraTRampa);
  Serial.printf("Curvas: modo %d (t 1 = SP/PV/U, t 2 = Vel/Acel, t 3 = Pos/Vel/Acel, t 4 = SP/Vel)\n", telModo);
  Serial.println("Comandos: v | p | sierra | pot | s <valor> | kp/ki/kd <x> | cero | t <1-4> | stop | go | ?");
}

void iniciarSierra() {
  habilitado = true;
  modo = MODO_SIERRA;
  pidReset(pidVel, rpmFiltrada);
  sierraT0 = millis();
  Serial.printf("Modo SIERRA: %.1f RPM, rampa %.2f s, pausa %.2f s (aceleracion %.2f RPM/s)\n",
                sierraRpm, sierraTRampa, sierraTPausa, sierraRpm / sierraTRampa);
}

void procesarComando(String c) {
  c.trim();
  c.toLowerCase();
  if (c.length() == 0) return;

  PID &pid = (modo == MODO_POS) ? pidPos : pidVel;

  if (c == "v") {
    modo = MODO_VEL; pidReset(pidVel, rpmFiltrada); Serial.println("Modo VELOCIDAD");
  } else if (c == "p") {
    modo = MODO_POS; pidReset(pidPos, medicion); Serial.println("Modo POSICION");
  } else if (c == "sierra") {
    iniciarSierra();
  } else if (c.startsWith("sierra ")) {
    float r, tr, tp;
    int n = sscanf(c.c_str(), "sierra %f %f %f", &r, &tr, &tp);
    if (n == 3 && r > 0 && tr >= 0.2f && tp >= 0) {
      sierraRpm = r; sierraTRampa = tr; sierraTPausa = tp;
      iniciarSierra();
    } else {
      Serial.println("Uso: sierra <rpm> <t_rampa_s> <t_pausa_s>   ej: sierra 25 3 1.5");
    }
  } else if (c == "pot") {
    setpointPot = true; Serial.println("Setpoint: potenciometro");
  } else if (c.startsWith("s ")) {
    setpointSerial = c.substring(2).toFloat();
    setpointPot = false;
    Serial.printf("Setpoint serial = %.2f\n", setpointSerial);
  } else if (c.startsWith("kp ")) {
    pid.kp = c.substring(3).toFloat(); Serial.printf("Kp = %.3f\n", pid.kp);
  } else if (c.startsWith("ki ")) {
    pid.ki = c.substring(3).toFloat(); pid.integral = 0; Serial.printf("Ki = %.3f\n", pid.ki);
  } else if (c.startsWith("kd ")) {
    pid.kd = c.substring(3).toFloat(); Serial.printf("Kd = %.3f\n", pid.kd);
  } else if (c.startsWith("t ")) {
    int m = c.substring(2).toInt();
    if (m >= 1 && m <= 4) {
      telModo = m;
      Serial.printf("Curvas: modo %d\n", telModo);
    } else {
      Serial.println("Usa t 1, t 2, t 3 o t 4");
    }
  } else if (c == "cero") {
    noInterrupts(); encCount = 0; interrupts();
    encPrev = 0; pidReset(pidPos, 0); Serial.println("Posicion en cero (home manual)");
  } else if (c == "stop") {
    habilitado = false;
    if (modo == MODO_SIERRA) {       // stop tambien sale del modo sierra
      modo = MODO_VEL;
      setpointPot = false;
      setpointSerial = 0;
    }
    Serial.println("Motor detenido");
  } else if (c == "go") {
    habilitado = true; pidReset(pid, medicion); Serial.println("Motor habilitado");
  } else if (c == "?") {
    imprimirEstado();
  } else {
    Serial.println("Comando no reconocido. Escribe '?'");
  }
}

void leerSerial() {
  static String buf;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      procesarComando(buf);
      buf = "";
    } else {
      buf += ch;
    }
  }
}

// ======================== LOOP ========================
void loop() {
  leerSerial();

  unsigned long ahora = micros();
  if (ahora - tPrev >= TS_US) {
    float dt = (ahora - tPrev) / 1e6f;
    tPrev = ahora;
    lazoControl(dt);
  }

  if (millis() - tTel >= TELEMETRIA_MS) {
    tTel = millis();
    telemetria();
  }
}
