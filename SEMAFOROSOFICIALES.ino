// ====================================================================== 
// SEMAFOROS ESP32 — 2 Semáforos independientes (Carro + Peatón)
// + COORDINADOS ENTRE SÍ + CONTADOR SOFTWARE (SÓLO SEMÁFORO 1)
// + INTEGRACIÓN BLYNK + PWM EN LED PEATONAL (SIN CANALES MANUALES)
// + SERVO BARRERA SEMÁFORO 1 EN PIN 2
// ======================================================================

#define BLYNK_TEMPLATE_ID "TMPL2As4E5iz-"
#define BLYNK_TEMPLATE_NAME "Semaforos Inteligentes"
#define BLYNK_AUTH_TOKEN "vjdAKiiGTVmTpfMnqwh7ihMNyHGI189U"

#define BLYNK_PRINT Serial

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>  

// ====== WiFi ======
char ssid[] = "UTP";
char pass[] = "tecnologica";


#define VP_STATE_S2       V2
#define VP_PED_TIME       V3
#define VP_RESET_CNT1     V4
#define VP_RESET_CNT2     V5
#define VP_PED_BTN1       V6
#define VP_PED_BTN2       V7

BlynkTimer timer;

// ================= SEMÁFORO 1 =================
const int redPin1     = 14;
const int yellowPin1  = 27;
const int greenPin1   = 26;
const int pedPin1     = 25;

const int buttonPin1  = 4;  
const int carPin1     = 5;  

// ================= SEMÁFORO 2 =================
const int redPin2     = 13;
const int yellowPin2  = 12;
const int greenPin2   = 23;
const int pedPin2     = 22;

const int buttonPin2  = 21;
const int carPin2     = 19;

// ================= RESET CONTADORES (YA SIN FLIP-FLOP, OPCIONALES) =================
const int resetCnt1Pin = 18; 
const int resetCnt2Pin = 15; 

// ====== BOTÓN FÍSICO RESET CONTADOR S1 ======
const int resetButtonS1Pin = 32; // botón físico adicional

// ================= SERVO BARRERA SEMÁFORO 1 =================
const int servoPin1 = 2;  // ÚNICO PIN LIBRE
Servo servo1;

const int SERVO_BARRERA_ABAJO  = 0;   // barrera cerrada
const int SERVO_BARRERA_ARRIBA = 90;  // barrera abierta

// Tiempos (ms)
const unsigned long RED_TIME    = 7000;
const unsigned long GREEN_TIME  = 7000;
const unsigned long YELLOW_TIME = 3500;
const unsigned long PED_TIME    = 8000;

// Estados
enum State {RED, YELLOW_AFTER_RED, GREEN, YELLOW_AFTER_GREEN, PEDESTRIAN};

// Estado y tiempos de cada semáforo
State state1 = RED;
State state2 = RED;
unsigned long lastChange1 = 0;
unsigned long lastChange2 = 0;

// Flags semáforo 1
bool solicitudPeaton1 = false;
bool pulsoCarro1 = false;
bool vengoDePeaton1 = false;

// Flags semáforo 2
bool solicitudPeaton2 = false;
bool pulsoCarro2 = false;
bool vengoDePeaton2 = false;

// Debounce y estados botón semáforo 1
int lastButtonState1 = HIGH;
unsigned long lastButtonBounce1 = 0;
bool buttonHeld1 = false;

// Debounce y estados botón semáforo 2
int lastButtonState2 = HIGH;
unsigned long lastButtonBounce2 = 0;
bool buttonHeld2 = false;

unsigned long lastButtonPressMillis1 = 0;
unsigned long lastButtonPressMillis2 = 0;

const unsigned long DEBOUNCE_MS = 50;

// ---------- quién está en fase peatonal (0 = nadie, 1 ó 2) ----------
int pedOwner = 0;

// ---------- contador software de carros S1 ----------
int carCount1 = 0;
int lastCount1 = -1;

// ---------- PWM "blink" para LEDs peatonales (sin canales explícitos) ----------
const int PED_PWM_MAX      = 255;      // valor máximo PWM
const unsigned long PED_BLINK_PERIOD = 100;  // ms, periodo de titileo

bool pedBlink1 = false;
bool pedBlink2 = false;
unsigned long lastPedBlink1 = 0;
unsigned long lastPedBlink2 = 0;
bool pedOn1 = false;
bool pedOn2 = false;

// prototipos
void fsm(int id);
void changeState(int id, State newState);
void allOff(int id);
String stateToText(State s);
void reportStateToBlynk(int id, State s);
void resetCounter1Pulse();
void resetCounter2Pulse();
void updatePedPWM();
void updateServoForS1(State s);

// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);

  Serial.println();
  Serial.println("========================================");
  Serial.println("      SISTEMA SEMÁFOROS ESP32 v1.0      ");
  Serial.println("  Carros + Peatones + Blynk + Servo     ");
  Serial.println("========================================");

  // ====== Blynk / WiFi ======
  Serial.println("[INFO] Conectando a WiFi/Blynk...");
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("[OK] Conectado a Blynk.");

  // Pines semáforo 1
  pinMode(redPin1, OUTPUT);
  pinMode(yellowPin1, OUTPUT);
  pinMode(greenPin1, OUTPUT);
  pinMode(pedPin1, OUTPUT);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(carPin1, INPUT_PULLUP);

  // Pines semáforo 2
  pinMode(redPin2, OUTPUT);
  pinMode(yellowPin2, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(pedPin2, OUTPUT);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(carPin2, INPUT_PULLUP);

  // Pines reset (si ya no van al flip-flop, quedan en HIGH)
  pinMode(resetCnt1Pin, OUTPUT);
  pinMode(resetCnt2Pin, OUTPUT);
  digitalWrite(resetCnt1Pin, HIGH);   // reposo HIGH
  digitalWrite(resetCnt2Pin, HIGH);   // reposo HIGH

  pinMode(resetButtonS1Pin, INPUT_PULLUP);

  // Servo barrera S1
  servo1.attach(servoPin1);
  servo1.write(SERVO_BARRERA_ABAJO);

  allOff(1);
  allOff(2);

  // Ambos arrancan en rojo
  changeState(1, RED);
  changeState(2, RED);

  Serial.println("[INIT] Semáforos inicializados en ROJO.");
  Serial.println("----------------------------------------");

  // Timer para enviar periódicamente info a Blynk (cada 200 ms)
  timer.setInterval(200L, []() {
    long remainingPedMs = 0;
    if (state1 == PEDESTRIAN) {
      remainingPedMs = (long)PED_TIME - (long)(millis() - lastChange1);
    } else if (state2 == PEDESTRIAN) {
      remainingPedMs = (long)PED_TIME - (long)(millis() - lastChange2);
    }
    if (remainingPedMs < 0) remainingPedMs = 0;
    float remainingSec = remainingPedMs / 1000.0;
    Blynk.virtualWrite(VP_PED_TIME, remainingSec);
  });
}

// ======================================================================
// LOOP
// ======================================================================
void loop() {
  Blynk.run();
  timer.run();

  fsm(1);
  fsm(2);

  // ====== ENVÍO DEL CONTADOR SOFTWARE S1 A SERIAL + BLYNK ======
  if (carCount1 != lastCount1) {
    lastCount1 = carCount1;
    Serial.printf("[S1][CONTADOR] Vehículos detectados: %02d\n", carCount1);
    Blynk.virtualWrite(VP_CAR_COUNT_S1, carCount1);
  }

  updatePedPWM();

  // ====== BOTÓN FÍSICO RESET CONTADOR S1 ======
  static bool lastResetBtn = HIGH;
  bool resetBtn = digitalRead(resetButtonS1Pin);

  if (lastResetBtn == HIGH && resetBtn == LOW) {
    Serial.println("[S1][RESET] Botón físico presionado.");
    resetCounter1Pulse();
    delay(200);  // antirebote simple y seguro
  }
  lastResetBtn = resetBtn;

  delay(5);
}

// ==================== FSM ====================
void fsm(int id) {
  unsigned long now = millis();

  volatile State &state = (id == 1 ? state1 : state2);
  volatile bool &solicitudPeaton = (id == 1 ? solicitudPeaton1 : solicitudPeaton2);
  volatile bool &pulsoCarro = (id == 1 ? pulsoCarro1 : pulsoCarro2);
  volatile bool &vengoDePeaton = (id == 1 ? vengoDePeaton1 : vengoDePeaton2);
  volatile unsigned long &lastChange = (id == 1 ? lastChange1 : lastChange2);
  volatile int &lastButtonState = (id == 1 ? lastButtonState1 : lastButtonState2);
  volatile unsigned long &lastButtonBounce = (id == 1 ? lastButtonBounce1 : lastButtonBounce2);
  volatile bool &buttonHeld = (id == 1 ? buttonHeld1 : buttonHeld2);
  volatile unsigned long &lastButtonPressMillis = (id == 1 ? lastButtonPressMillis1 : lastButtonPressMillis2);

  State &otherState            = (id == 1 ? state2 : state1);
  bool  &otherSolicitudPeaton  = (id == 1 ? solicitudPeaton2 : solicitudPeaton1);
  bool  &otherPulsoCarro       = (id == 1 ? pulsoCarro2 : pulsoCarro1);

  int buttonPin = (id == 1 ? buttonPin1 : buttonPin2);
  int carPin    = (id == 1 ? carPin1    : carPin2);

  int rawButton = digitalRead(buttonPin);

  if (rawButton != lastButtonState) lastButtonBounce = now;
  if (rawButton == LOW && (now - lastButtonBounce) > DEBOUNCE_MS) {
    if (!solicitudPeaton && !buttonHeld) {
      solicitudPeaton = true;
      buttonHeld = true;
      lastButtonPressMillis = now;
      Serial.printf("[S%d][PEATON] Botón presionado -> solicitud peatonal.\n", id);
    }
  }
  if (rawButton == HIGH) buttonHeld = false;
  lastButtonState = rawButton;

  static int lastCarState[3] = {1,1,1};
  int car = digitalRead(carPin);

  if (lastCarState[id] == 1 && car == 0) {
    // flanco de bajada en el sensor/botón de carro
    if (now - lastButtonPressMillis <= 120) {
      Serial.printf("[S%d][CARRO] Pulso ignorado (rebote o muy rápido).\n", id);
    } else {
      pulsoCarro = true;
      Serial.printf("[S%d][CARRO] Pulso registrado.\n", id);

      // ====== CONTADOR SOFTWARE SOLO PARA SEMÁFORO 1 (0–15, luego vuelve a 0) ======
      if (id == 1) {
        if (carCount1 >= 15) {
          carCount1 = 0;
        } else {
          carCount1++;
        }
      }
    }
  }
  lastCarState[id] = car;

  if (pedOwner != 0 && pedOwner != id) {
    if (state != GREEN) {
      changeState(id, GREEN);
    }
    return;
  }

  switch (state) {
    case RED:
      if (solicitudPeaton && pedOwner == 0) {
        solicitudPeaton = false;
        vengoDePeaton = false;

        pedOwner = id;
        changeState(id, PEDESTRIAN);

        {
          int otherId = (id == 1 ? 2 : 1);
          changeState(otherId, GREEN);
        }
        break;
      }

      if (pulsoCarro &&
          !otherPulsoCarro &&
          !otherSolicitudPeaton &&
          pedOwner == 0) {

        pulsoCarro = false;
        vengoDePeaton = false;
        changeState(id, YELLOW_AFTER_RED);
        break;
      }

      if ((now - lastChange) >= RED_TIME && pedOwner == 0) {
        if (otherState != GREEN && otherState != PEDESTRIAN) {
          changeState(id, YELLOW_AFTER_RED);
        }
      }
      break;

    case YELLOW_AFTER_RED:
      if (pulsoCarro &&
          !otherPulsoCarro &&
          !otherSolicitudPeaton &&
          pedOwner == 0) {

        pulsoCarro = false;
        vengoDePeaton = false;

        if (otherState != GREEN && otherState != PEDESTRIAN) {
          changeState(id, GREEN);
        }
        break;
      }

      if ((now - lastChange) >= YELLOW_TIME && pedOwner == 0) {
        if (otherState != GREEN && otherState != PEDESTRIAN) {
          changeState(id, GREEN);
        }
      }
      break;

    case GREEN:
      if (pulsoCarro) {
        pulsoCarro = false;
        Serial.printf("[S%d][CARRO] Pulso ignorado (ya está en VERDE).\n", id);
      }
      if ((now - lastChange) >= GREEN_TIME && pedOwner == 0) {
        changeState(id, YELLOW_AFTER_GREEN);
      }
      break;

    case YELLOW_AFTER_GREEN:
      if ((now - lastChange) >= YELLOW_TIME) {
        if (vengoDePeaton) {
          vengoDePeaton = false;
          if (otherState != GREEN && otherState != PEDESTRIAN) {
            changeState(id, GREEN);
          } else {
            changeState(id, RED);
          }
        } else {
          changeState(id, RED);
        }
      }
      break;

    case PEDESTRIAN:
      if (pulsoCarro) {
        pulsoCarro = false;
        Serial.printf("[S%d][CARRO] Pulso ignorado (fase peatonal activa).\n", id);
      }

      if ((now - lastChange) >= PED_TIME) {
        vengoDePeaton = true;
        solicitudPeaton = false;
        pedOwner = 0;
        changeState(id, YELLOW_AFTER_GREEN);
      }
      break;
  }
}

// ==================== CAMBIO DE ESTADO ====================
void changeState(int id, State newState) {
  allOff(id);
  (id == 1 ? state1 : state2) = newState;
  (id == 1 ? lastChange1 : lastChange2) = millis();

  int red    = (id == 1 ? redPin1    : redPin2);
  int yellow = (id == 1 ? yellowPin1 : yellowPin2);
  int green  = (id == 1 ? greenPin1  : greenPin2);
  int ped    = (id == 1 ? pedPin1    : pedPin2);

  switch (newState) {
    case RED:
      digitalWrite(red, HIGH);
      if (id == 1) pedBlink1 = false; else pedBlink2 = false;
      Serial.printf("[S%d][ESTADO] -> ROJO\n", id);
      break;

    case YELLOW_AFTER_RED:
      digitalWrite(yellow, HIGH);
      if (id == 1) pedBlink1 = false; else pedBlink2 = false;
      Serial.printf("[S%d][ESTADO] -> AMARILLO (desde ROJO)\n", id);
      break;

    case GREEN:
      digitalWrite(green, HIGH);
      if (id == 1) pedBlink1 = false; else pedBlink2 = false;
      Serial.printf("[S%d][ESTADO] -> VERDE\n", id);
      break;

    case YELLOW_AFTER_GREEN:
      digitalWrite(yellow, HIGH);
      if (id == 1) pedBlink1 = false; else pedBlink2 = false;
      Serial.printf("[S%d][ESTADO] -> AMARILLO (desde VERDE)\n", id);
      break;

    case PEDESTRIAN:
      digitalWrite(red, HIGH);
      if (id == 1) {
        pedBlink1 = true;
        lastPedBlink1 = millis();
        pedOn1 = false;
      } else {
        pedBlink2 = true;
        lastPedBlink2 = millis();
        pedOn2 = false;
      }
      Serial.printf("[S%d][ESTADO] -> PEATONAL (LED parpadeando)\n", id);
      break;
  }

  if (id == 1) {
    updateServoForS1(newState);
  }

  reportStateToBlynk(id, newState);
}

void allOff(int id) {
  digitalWrite((id == 1 ? redPin1    : redPin2), LOW);
  digitalWrite((id == 1 ? yellowPin1 : yellowPin2), LOW);
  digitalWrite((id == 1 ? greenPin1  : greenPin2), LOW);

  if (id == 1) {
    pedBlink1 = false;
    pedOn1 = false;
    analogWrite(pedPin1, 0);
  } else {
    pedBlink2 = false;
    pedOn2 = false;
    analogWrite(pedPin2, 0);
  }
}

// ---------- SERVO: lógica barrera S1 ----------
void updateServoForS1(State s) {
  if (s == GREEN) {
    servo1.write(SERVO_BARRERA_ARRIBA);
    Serial.println("[S1][SERVO] Barrera ARRIBA (VERDE).");
  } else {
    servo1.write(SERVO_BARRERA_ABAJO);
    Serial.println("[S1][SERVO] Barrera ABAJO (NO VERDE).");
  }
}

// ============= PWM / TITILEO PARA LEDS PEATONALES =============
void updatePedPWM() {
  unsigned long now = millis();

  if (pedBlink1) {
    if (now - lastPedBlink1 >= PED_BLINK_PERIOD) {
      lastPedBlink1 = now;
      pedOn1 = !pedOn1;
      analogWrite(pedPin1, pedOn1 ? PED_PWM_MAX : 0);
    }
  } else {
    analogWrite(pedPin1, 0);
  }

  if (pedBlink2) {
    if (now - lastPedBlink2 >= PED_BLINK_PERIOD) {
      lastPedBlink2 = now;
      pedOn2 = !pedOn2;
      analogWrite(pedPin2, pedOn2 ? PED_PWM_MAX : 0);
    }
  } else {
    analogWrite(pedPin2, 0);
  }
}

// ==================== BLYNK: ESTADOS ====================
String stateToText(State s) {
  switch (s) {
    case RED:               return "Red";
    case YELLOW_AFTER_RED:  return "Yellow_R";
    case GREEN:             return "Green";
    case YELLOW_AFTER_GREEN:return "Yellow_G";
    case PEDESTRIAN:        return "Pedestrian";
  }
  return "Unknown";
}

void reportStateToBlynk(int id, State s) {
  String txt = stateToText(s);
  if (id == 1) {
    Blynk.virtualWrite(VP_STATE_S1, txt);
  } else {
    Blynk.virtualWrite(VP_STATE_S2, txt);
  }
}

// ==================== BLYNK: BOTONES VIRTUALES ====================
BLYNK_WRITE(VP_RESET_CNT1) {
  int v = param.asInt();
  if (v == 1) {
    Serial.println("[S1][RESET] Botón virtual Blynk presionado.");
    resetCounter1Pulse();
  }
}

BLYNK_WRITE(VP_RESET_CNT2) {
  int v = param.asInt();
  if (v == 1) {
    Serial.println("[S2][RESET] Botón virtual Blynk (sin uso efectivo).");
    resetCounter2Pulse();
  }
}

BLYNK_WRITE(VP_PED_BTN1) {
  int v = param.asInt();
  if (v == 1) {
    if (!solicitudPeaton1) {
      solicitudPeaton1 = true;
      Serial.println("[S1][PEATON][BLYNK] Botón peatonal virtual presionado.");
    }
  }
}

BLYNK_WRITE(VP_PED_BTN2) {
  int v = param.asInt();
  if (v == 1) {
    if (!solicitudPeaton2) {
      solicitudPeaton2 = true;
      Serial.println("[S2][PEATON][BLYNK] Botón peatonal virtual presionado.");
    }
  }
}

// ==================== PULSOS DE RESET ====================
// Ahora solo resetea el contador software; los pines se dejan en HIGH
void resetCounter1Pulse() {
  carCount1 = 0;
  lastCount1 = -1;
  Blynk.virtualWrite(VP_CAR_COUNT_S1, carCount1);
  digitalWrite(resetCnt1Pin, HIGH);

  Serial.println();
  Serial.println("************ RESET CONTADOR S1 ************");
  Serial.println("*  Semáforo 1: contador de carros = 0    *");
  Serial.println("*******************************************");
  Serial.println();
}

void resetCounter2Pulse() {
  digitalWrite(resetCnt2Pin, HIGH);
  Serial.println("[S2][RESET] contador S2 (sin uso, solo pin HIGH).");
}