#include <Wire.h>
#include <Preferences.h>

// ========== Parámetros físicos y límites ==========
const float VEL_MAXIMA_RPM = 130.0;
float pulsosPorVuelta      = 11.0 * 4.0 * 30.0;   // CPR
const float PULSOS_DEFECTO  = 11.0 * 4.0 * 30.0;
int   PWM_UMBRAL_MIN       = 250;                 

// Perfil de movimiento
float velMaxPosicion    = 30.0;     // VP (RPM)
float acelMax           = 40.0;     // AC (RPM/s)
float desacelMax        = 40.0;     // DE (RPM/s)

// ========== Control / Ganancias ==========
float gananciaPosicion   = 1.0;  // KP_POS
float gananciaPropVel    = 2.0;  // Kp
float gananciaIntegVel   = 15.0; // Ki
const float TOLERANCIA_GRADOS = 1.5;   
const float UMBRAL_RUIDO_RPM  = 0.5;

Preferences almacenamiento;

// ========== Asignación de Pines ==========
const int PIN_CANAL_A = 23;
const int PIN_CANAL_B = 18;
const int PIN_MOTOR_IN1 = 26;
const int PIN_MOTOR_IN2 = 27;
const int PIN_I2C_SDA  = 21;
const int PIN_I2C_SCL  = 22;

const int FRECUENCIA_PWM = 2000;
const int RESOLUCION_PWM = 10;
const int CICLO_MAX_PWM  = 1023;
const uint32_t INTERVALO_MUESTREO_MS = 10;
const float FACTOR_FILTRO_RPM = 0.3;

// ========== Variables de Estado ==========
bool  esModoPosicion   = false;
bool  enMovimiento     = false;
bool  faseCalibracion  = false;
float anguloObjetivo   = 0;     
float posTrayectoria   = 0;     // pRef
float velTrayectoria   = 0;     // vRef
float acelTrayectoria  = 0;     // aRef
float consignaVelocidad = 0;    // refTeclado
float sumaErrorIntegral = 0;    // integ
float ultimoAnguloGirado = 0;

// ========== Pantalla LCD I2C ==========
uint8_t direccionLCD = 0;
bool lcdDisponible = false;
const uint8_t BIT_RS = 0x01, BIT_EN = 0x04, BIT_BACKLIGHT = 0x08;

void transmitirNibbleLCD(uint8_t datos, uint8_t modo) {
  uint8_t paquete = (datos & 0xF0) | modo | BIT_BACKLIGHT;
  Wire.beginTransmission(direccionLCD);
  Wire.write(paquete | BIT_EN);
  Wire.write(paquete & ~BIT_EN);
  Wire.endTransmission();
  delayMicroseconds(50);
}

void enviarDatoLCD(uint8_t byte, uint8_t modo) { 
  transmitirNibbleLCD(byte & 0xF0, modo); 
  transmitirNibbleLCD((byte << 4) & 0xF0, modo); 
}

void enviarComandoLCD(uint8_t cmd) { 
  enviarDatoLCD(cmd, 0); 
  if (cmd <= 0x02) delay(2); 
}

void fijarCursorLCD(uint8_t col, uint8_t fila) { 
  enviarComandoLCD(0x80 | (col + (fila ? 0x40 : 0))); 
}

void escribirTextoLCD(const char* cadena) { 
  while (*cadena) enviarDatoLCD(*cadena++, BIT_RS); 
}

bool verificarDispositivoI2C(uint8_t dir) { 
  Wire.beginTransmission(dir); 
  return Wire.endTransmission() == 0; 
}

bool inicializarDisplay() {
  const uint8_t direccionesHabituales[] = { 0x27, 0x3F };
  for (uint8_t dir : direccionesHabituales) {
    if (verificarDispositivoI2C(dir)) { direccionLCD = dir; break; }
  }
  if (!direccionLCD) {
    for (uint8_t dir = 0x20; dir <= 0x3F; dir++) {
      if (verificarDispositivoI2C(dir)) { direccionLCD = dir; break; }
    }
  }
  if (!direccionLCD) return false;

  delay(50);
  transmitirNibbleLCD(0x30, 0); delay(5);
  transmitirNibbleLCD(0x30, 0); delay(1);
  transmitirNibbleLCD(0x30, 0);
  transmitirNibbleLCD(0x20, 0);
  enviarComandoLCD(0x28); enviarComandoLCD(0x0C); 
  enviarComandoLCD(0x06); enviarComandoLCD(0x01);
  return true;
}

// ========== Lectura de Encoder ==========
volatile long contadorPulsos = 0;
volatile uint8_t estadoEncoder = 0;
portMUX_TYPE cerrojoMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR manejadorInterrupcionEncoder() {
  static const int8_t matrizTransiciones[16] = { 0,-1,1,0, 1,0,0,-1, -1,0,0,1, 0,1,-1,0 };
  uint8_t estadoActual = (digitalRead(PIN_CANAL_A) << 1) | digitalRead(PIN_CANAL_B);
  portENTER_CRITICAL_ISR(&cerrojoMux);
  contadorPulsos += matrizTransiciones[(estadoEncoder << 2) | estadoActual];
  estadoEncoder = estadoActual;
  portEXIT_CRITICAL_ISR(&cerrojoMux);
}

long obtenerPulsos() { 
  portENTER_CRITICAL(&cerrojoMux); 
  long val = contadorPulsos; 
  portEXIT_CRITICAL(&cerrojoMux); 
  return val; 
}

float calcularAnguloGrados() { 
  return obtenerPulsos() / pulsosPorVuelta * 360.0f; 
}

// ========== Control del Motor ==========
void aplicarSalidaMotor(float accionControl) {
  int potenciaPWM = constrain((int)fabsf(accionControl), 0, CICLO_MAX_PWM);
  if (accionControl > 0) { 
    ledcWrite(PIN_MOTOR_IN1, potenciaPWM); 
    ledcWrite(PIN_MOTOR_IN2, 0); 
  } else if (accionControl < 0) { 
    ledcWrite(PIN_MOTOR_IN1, 0);    
    ledcWrite(PIN_MOTOR_IN2, potenciaPWM); 
  } else { 
    ledcWrite(PIN_MOTOR_IN1, 0);    
    ledcWrite(PIN_MOTOR_IN2, 0); 
  }
}

float estimarFeedforwardPWM(float rpmDeseada) {
  float magnitudRPM = fabsf(rpmDeseada);
  if (magnitudRPM < UMBRAL_RUIDO_RPM) return 0;
  float salidaPwm = PWM_UMBRAL_MIN + magnitudRPM / VEL_MAXIMA_RPM * (CICLO_MAX_PWM - PWM_UMBRAL_MIN);
  return rpmDeseada > 0 ? salidaPwm : -salidaPwm;
}

// ===================== CONTROLADOR DE VELOCIDAD =====================
float algoritmoPIVelocidad(float referencia, float velMedida, float deltaT) {
  if (fabsf(referencia) < UMBRAL_RUIDO_RPM) { 
    sumaErrorIntegral = 0; 
    return 0; 
  }
  float componenteFF = estimarFeedforwardPWM(referencia);
  float errorVel     = referencia - velMedida;
  float proximaInteg = sumaErrorIntegral + gananciaIntegVel * errorVel * deltaT;
  float controlU     = componenteFF + gananciaPropVel * errorVel + proximaInteg;

  // Anti-windup por saturación
  if (controlU > CICLO_MAX_PWM) { 
    controlU = CICLO_MAX_PWM; 
    if (errorVel < 0) sumaErrorIntegral = proximaInteg; 
  } else if (controlU < -CICLO_MAX_PWM) { 
    controlU = -CICLO_MAX_PWM; 
    if (errorVel > 0) sumaErrorIntegral = proximaInteg; 
  } else { 
    sumaErrorIntegral = proximaInteg; 
  }
  return controlU;
}

// ===================== LIMITADOR DE ACELERACIÓN =====================
float aplicarRampaAceleracion(float valorActual, float valorObjetivo, float deltaT) {
  bool estaAcelerando = (fabsf(valorObjetivo) > fabsf(valorActual)) && (valorObjetivo * valorActual >= 0);
  float tasaPermitida = (estaAcelerando ? acelMax : desacelMax) * deltaT;
  return valorActual + constrain(valorObjetivo - valorActual, -tasaPermitida, tasaPermitida);
}

// ===================== PERFIL TRAPEZOIDAL =====================
float calcularVelocidadPerfil() {
  float distanciaRestante = anguloObjetivo - posTrayectoria;
  float sentido = (distanciaRestante >= 0) ? 1.0f : -1.0f;
  float velFrenadoPermitida = sqrtf(desacelMax * fabsf(distanciaRestante) / 3.0f);
  return sentido * min(velMaxPosicion, velFrenadoPermitida);
}

// ===================== PERSISTENCIA =====================
void guardarConfiguracion() {
  almacenamiento.putFloat("cpr", pulsosPorVuelta);
  Serial.printf("# [CAL] Ajuste guardado: CPR=%.1f\n", pulsosPorVuelta);
}

void reportarEstado() {
  Serial.printf("# MODO %s | VP=%.0f RPM | AC=%.0f RPM/s | DE=%.0f RPM/s | PWM min=%d\n",
                esModoPosicion ? "POSICION" : "VELOCIDAD", velMaxPosicion, acelMax, desacelMax, PWM_UMBRAL_MIN);
  Serial.printf("# Kp_pos=%.2f | Kp_vel=%.2f | Ki_vel=%.2f | CPR=%.1f\n", 
                gananciaPosicion, gananciaPropVel, gananciaIntegVel, pulsosPorVuelta);
}

void mostrarMenuAyuda() {
  Serial.println("# p = modo posicion | v = modo velocidad | s = parar | g = estado | h = ayuda");
  Serial.println("# Velocidad: -130..130 RPM (negativo = sentido contrario)");
  Serial.println("# Posicion : grados RELATIVOS (90, -45, 720) | ab <grados> = posicion ABSOLUTA");
  Serial.println("# Limites  : vp <RPM> = vel. maxima | ac <RPM/s> = aceleracion | de <RPM/s> = desaceleracion");
  Serial.println("# Ajustes  : pm <PWM min> | kq <Kp posicion> | kp <Kp vel> | ki <Ki vel>");
  Serial.println("# Calibrar : cal + ok (1 vuelta a mano) | re <grados reales> | borrar");
}

void establecerDestino(float nuevoDestino) {
  anguloObjetivo = nuevoDestino;
  enMovimiento   = true;
  Serial.printf("# Desplazando a %.1f grados (VP=%.0f, AC=%.0f, DE=%.0f)\n", 
                nuevoDestino, velMaxPosicion, acelMax, desacelMax);
}

void reiniciarPerfil() {
  float posActual = calcularAnguloGrados();
  posTrayectoria  = posActual; 
  anguloObjetivo  = posActual; 
  velTrayectoria  = 0; 
  acelTrayectoria = 0; 
  sumaErrorIntegral = 0; 
  enMovimiento    = false;
}

// ===================== PROCESAMIENTO SERIAL =====================
void atenderComandosSerial() {
  if (!Serial.available()) return;
  String comando = Serial.readStringUntil('\n');
  comando.trim();
  comando.toLowerCase();
  if (comando.length() == 0) return;

  // Calibración
  if (comando == "cal") {
    consignaVelocidad = 0; 
    reiniciarPerfil();
    portENTER_CRITICAL(&cerrojoMux); contadorPulsos = 0; portEXIT_CRITICAL(&cerrojoMux);
    faseCalibracion = true;
    Serial.println("# [CAL] Motor desactivado. Girar eje A MANO 1 vuelta completa y enviar 'ok'");
    return;
  }
  if (comando == "ok") {
    if (!faseCalibracion) { Serial.println("# [CAL] Debe iniciar con 'cal'"); return; }
    faseCalibracion = false;
    long pulsosMedidos = labs(obtenerPulsos());
    if (pulsosMedidos < 20) { Serial.println("# [CAL] Pulsos insuficientes: revise conexiones."); return; }
    pulsosPorVuelta = pulsosMedidos; 
    guardarConfiguracion(); 
    reiniciarPerfil();
    return;
  }
  if (comando == "borrar") { 
    pulsosPorVuelta = PULSOS_DEFECTO; 
    guardarConfiguracion(); 
    reiniciarPerfil(); 
    return; 
  }

  // Modos de operación
  if (comando == "p") {
    consignaVelocidad = 0; 
    reiniciarPerfil(); 
    esModoPosicion = true;
    Serial.println("# MODO POSICION activo");
    return;
  }
  if (comando == "v") {
    esModoPosicion = false; 
    enMovimiento = false; 
    consignaVelocidad = 0; 
    sumaErrorIntegral = 0;
    Serial.println("# MODO VELOCIDAD activo");
    return;
  }

  // Entradas numéricas
  char primerCar = comando.charAt(0);
  if (isDigit(primerCar) || primerCar == '-' || primerCar == '+') {
    float valorNum = comando.toFloat();
    if (esModoPosicion) {
      if (fabsf(valorNum) > 0) { 
        ultimoAnguloGirado = valorNum; 
        establecerDestino(anguloObjetivo + valorNum); 
      }
    } else {
      consignaVelocidad = constrain(valorNum, -VEL_MAXIMA_RPM, VEL_MAXIMA_RPM);
      Serial.printf("# Objetivo: %.0f RPM\n", consignaVelocidad);
    }
    return;
  }

  if (comando == "s") {
    if (esModoPosicion) {
      anguloObjetivo = posTrayectoria + (velTrayectoria >= 0 ? 1 : -1) * 3.0f * velTrayectoria * velTrayectoria / max(desacelMax, 1.0f);
      Serial.println("# Parada progresiva en curso...");
    } else {
      consignaVelocidad = 0; 
      Serial.println("# Deteniendo con rampa...");
    }
    return;
  }
  if (comando == "g") { reportarEstado(); return; }
  if (comando == "h") { mostrarMenuAyuda(); return; }

  // Comandos con argumento
  if (comando.length() > 3 && comando.charAt(2) == ' ') {
    String clave = comando.substring(0, 2);
    float valor = comando.substring(3).toFloat();
    if      (clave == "pm") PWM_UMBRAL_MIN = constrain((int)valor, 0, CICLO_MAX_PWM - 1);
    else if (clave == "vp") velMaxPosicion = constrain(valor, 1.0f, VEL_MAXIMA_RPM);
    else if (clave == "ac") acelMax = max(1.0f, valor);
    else if (clave == "de") desacelMax = max(1.0f, valor);
    else if (clave == "kq") gananciaPosicion = max(0.0f, valor);
    else if (clave == "kp") gananciaPropVel = valor;
    else if (clave == "ki") { gananciaIntegVel = valor; sumaErrorIntegral = 0; }
    else if (clave == "ab") {
      if (!esModoPosicion) { Serial.println("# Cambie a modo posicion con 'p'"); return; }
      ultimoAnguloGirado = valor - anguloObjetivo;
      establecerDestino(valor);
      return;
    }
    else if (clave == "re") {
      if (ultimoAnguloGirado == 0 || valor == 0) { Serial.println("# [CAL] Realice un giro previo"); return; }
      pulsosPorVuelta = pulsosPorVuelta * fabsf(ultimoAnguloGirado) / fabsf(valor);
      guardarConfiguracion(); 
      reiniciarPerfil(); 
      ultimoAnguloGirado = 0;
      return;
    }
    else { Serial.println("# Comando desconocido"); return; }
    reportarEstado();
    return;
  }
  Serial.println("# Comando desconocido");
}

// ===================== ACTUALIZACIÓN DE PANTALLA =====================
void refrescarPantalla(float rpmActual) {
  if (!lcdDisponible) return;
  char bufferLinea1[17], bufferLinea2[17];

  if (faseCalibracion) {
    snprintf(bufferLinea1, sizeof(bufferLinea1), "CAL: 1 vuelta   ");
    snprintf(bufferLinea2, sizeof(bufferLinea2), "Pulsos:%8ld", labs(obtenerPulsos()));
  } else if (esModoPosicion) {
    snprintf(bufferLinea1, sizeof(bufferLinea1), "Obj:%8.1f gr ", anguloObjetivo);
    snprintf(bufferLinea2, sizeof(bufferLinea2), "Pos:%8.1f gr ", calcularAnguloGrados());
  } else {
    snprintf(bufferLinea1, sizeof(bufferLinea1), "SP (rpm):%7.0f", consignaVelocidad);
    snprintf(bufferLinea2, sizeof(bufferLinea2), "PV (rpm):%7.1f", rpmActual);
  }
  fijarCursorLCD(0, 0); escribirTextoLCD(bufferLinea1);
  fijarCursorLCD(0, 1); escribirTextoLCD(bufferLinea2);
}

// ===================== CONFIGURACIÓN INICIAL =====================
void setup() {
  Serial.begin(115200);
  delay(300);

  almacenamiento.begin("pendulo", false);
  pulsosPorVuelta = almacenamiento.getFloat("cpr", PULSOS_DEFECTO);

  ledcAttach(PIN_MOTOR_IN1, FRECUENCIA_PWM, RESOLUCION_PWM);
  ledcAttach(PIN_MOTOR_IN2, FRECUENCIA_PWM, RESOLUCION_PWM);
  aplicarSalidaMotor(0);

  pinMode(PIN_CANAL_A, INPUT_PULLUP);
  pinMode(PIN_CANAL_B, INPUT_PULLUP);
  estadoEncoder = (digitalRead(PIN_CANAL_A) << 1) | digitalRead(PIN_CANAL_B);
  attachInterrupt(digitalPinToInterrupt(PIN_CANAL_A), manejadorInterrupcionEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_CANAL_B), manejadorInterrupcionEncoder, CHANGE);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  lcdDisponible = inicializarDisplay();
  if (lcdDisponible) Serial.printf("# LCD detectada en dir 0x%02X\n", direccionLCD);
  else               Serial.println("# Error: LCD no encontrada");

  mostrarMenuAyuda();
  reportarEstado();
}

// ===================== BUCLE PRINCIPAL =====================
void loop() {
  static uint32_t tAnterior = millis(), tTelemetria = 0, tRefrescoLCD = 0;
  static long  pulsosAnteriores = 0;
  static float velocidadRPM = 0;          
  static float velTrayectoriaAnterior = 0;

  uint32_t tActual = millis();
  if (tActual - tAnterior < INTERVALO_MUESTREO_MS) return;
  float deltaTiempo = (tActual - tAnterior) / 1000.0f;
  tAnterior = tActual;

  atenderComandosSerial();

  // Cálculo de velocidad instantánea
  long pulsosActuales = obtenerPulsos();
  long cambioPulsos   = pulsosActuales - pulsosAnteriores;
  pulsosAnteriores    = pulsosActuales;
  velocidadRPM       += FACTOR_FILTRO_RPM * (((cambioPulsos / pulsosPorVuelta) / deltaTiempo * 60.0f) - velocidadRPM);
  float anguloActual  = pulsosActuales / pulsosPorVuelta * 360.0f;

  if (faseCalibracion) {
    aplicarSalidaMotor(0);
    if (tActual - tRefrescoLCD >= 250) { tRefrescoLCD = tActual; refrescarPantalla(0); }
    return;
  }

  float salidaControl = 0;

  if (esModoPosicion) {
    if (enMovimiento) {
      velTrayectoria  = aplicarRampaAceleracion(velTrayectoria, calcularVelocidadPerfil(), deltaTiempo);
      posTrayectoria += velTrayectoria * 6.0f * deltaTiempo;

      if (fabsf(anguloObjetivo - posTrayectoria) < 0.05f && fabsf(velTrayectoria) < 1.0f) {
        posTrayectoria = anguloObjetivo; 
        velTrayectoria = 0;
      }

      float errorPosicion = posTrayectoria - anguloActual;
      float velComandada  = constrain(velTrayectoria + gananciaPosicion * errorPosicion, -VEL_MAXIMA_RPM, VEL_MAXIMA_RPM);

      salidaControl = algoritmoPIVelocidad(velComandada, velocidadRPM, deltaTiempo);

      if (velTrayectoria == 0 && fabsf(anguloObjetivo - anguloActual) < TOLERANCIA_GRADOS) {
        enMovimiento = false; 
        sumaErrorIntegral = 0; 
        salidaControl = 0;
        Serial.printf("# Posicion alcanzada: %.1f grados\n", anguloActual);
      }
    } else {
      velTrayectoria = 0; 
      sumaErrorIntegral = 0; 
      salidaControl = 0;
      posTrayectoria = anguloActual;
    }
  } else {
    velTrayectoria = aplicarRampaAceleracion(velTrayectoria, consignaVelocidad, deltaTiempo);
    salidaControl  = algoritmoPIVelocidad(velTrayectoria, velocidadRPM, deltaTiempo);
  }

  acelTrayectoria = (velTrayectoria - velTrayectoriaAnterior) / deltaTiempo;
  velTrayectoriaAnterior = velTrayectoria;

  aplicarSalidaMotor(salidaControl);

  // Telemetría para Serial Plotter
  if (tActual - tTelemetria >= 100) {
    tTelemetria = tActual;
    if (esModoPosicion) {
      Serial.printf("obj:%.1f,pRef:%.1f,pos:%.1f,vRef:%.1f,rpm:%.1f,aRef:%.1f,pwm:%.0f\n",
                    anguloObjetivo, posTrayectoria, anguloActual, velTrayectoria, velocidadRPM, acelTrayectoria, salidaControl);
    } else {
      Serial.printf("obj:%.0f,vRef:%.1f,rpm:%.1f,aRef:%.1f,pwm:%.0f\n",
                    consignaVelocidad, velTrayectoria, velocidadRPM, acelTrayectoria, salidaControl);
    }
  }
  
  if (tActual - tRefrescoLCD >= 250) { 
    tRefrescoLCD = tActual; 
    refrescarPantalla(velocidadRPM); 
  }
}