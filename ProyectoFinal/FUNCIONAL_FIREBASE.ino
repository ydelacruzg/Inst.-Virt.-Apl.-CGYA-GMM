#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----------------- Pines y variables de control PID -----------------
int Modo = 2;
float Potencia_1 = 0;
float Potencia_2 = 0;
int Setpoint = 35;   // SP

int Kc = 9;
int Tao_I = 25;
int Tiempo0 = 3000;

// Pines ESP32
const int A = 34;              // TMP36
const int pin_disparo = 25;
const int pin_cruce_cero = 26;
const int pin_ventilador = 27;

const int btn_up_sp = 32;
const int btn_down_sp = 33;
const int btn_up_Kc = 14;
const int btn_down_Kc = 12;
const int btn_up_Tao = 13;
const int btn_down_Tao = 15;

// Variables internas
volatile int detectado = 0;
int valor = 0;
unsigned long Tiempo_previo = 0;
unsigned long Tiempo_actual = 0;
const int Read_Delay = 1000;
float Temperatura = 0;

float Potencia = 0;
float PID_error = 0;
float PID_value = 0;
float Error_INT = 0;

// Botones
bool prev_up_sp = HIGH;
bool prev_down_sp = HIGH;
bool prev_up_Kc = HIGH;
bool prev_down_Kc = HIGH;
bool prev_up_Tao = HIGH;
bool prev_down_Tao = HIGH;

// ----------------- Firebase -----------------
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <FirebaseClient.h>
#include "ExampleFunctions.h"

// Network and Firebase credentials
#define WIFI_SSID "INFINITUM79D3"
#define WIFI_PASSWORD "bJs5fFuc2c"
#define Web_API_KEY "AIzaSyCEY1RBR5RWlt9i9w4F74V6nV-DjMblwFU"
#define DATABASE_URL "https://final-iva-default-rtdb.firebaseio.com"
#define USER_EMAIL "manuelgoytiamartinez414@gmail.com"
#define USER_PASS "123456789"

UserAuth user_auth(Web_API_KEY, USER_EMAIL, USER_PASS);
FirebaseApp app;
WiFiClientSecure ssl_client;
using AsyncClient = AsyncClientClass;
AsyncClient aClient(ssl_client);
RealtimeDatabase Database;

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 10000;  // 10 segundos

// ----------------- Interrupción -----------------
void IRAM_ATTR detectarCruce() {
  detectado = 1;
}

// ----------------- Setup -----------------
void setup() {
  Serial.begin(115200);

  pinMode(pin_disparo, OUTPUT);
  pinMode(pin_cruce_cero, INPUT);
  pinMode(pin_ventilador, OUTPUT);
  digitalWrite(pin_ventilador, LOW);

  pinMode(btn_up_sp, INPUT_PULLUP);
  pinMode(btn_down_sp, INPUT_PULLUP);
  pinMode(btn_up_Kc, INPUT_PULLUP);
  pinMode(btn_down_Kc, INPUT_PULLUP);
  pinMode(btn_up_Tao, INPUT_PULLUP);
  pinMode(btn_down_Tao, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(pin_cruce_cero), detectarCruce, RISING);

  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Wifi y Firebase
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nWi-Fi connected");

  ssl_client.setInsecure();
  ssl_client.setConnectionTimeout(1000);
  ssl_client.setHandshakeTimeout(5);

  initializeApp(aClient, app, getAuth(user_auth), processData, "authTask");
  app.getApp<RealtimeDatabase>(Database);
  Database.url(DATABASE_URL);
}

// ----------------- Loop -----------------
void loop() {
  Tiempo_actual = millis();
  valor = map(Potencia, 0, 100, 7600, 10);

  // Leer botones
  bool curr_up_sp = digitalRead(btn_up_sp);
  bool curr_down_sp = digitalRead(btn_down_sp);
  bool curr_up_Kc = digitalRead(btn_up_Kc);
  bool curr_down_Kc = digitalRead(btn_down_Kc);
  bool curr_up_Tao = digitalRead(btn_up_Tao);
  bool curr_down_Tao = digitalRead(btn_down_Tao);

  if (prev_up_sp && !curr_up_sp) Setpoint++;
  if (prev_down_sp && !curr_down_sp) Setpoint--;
  if (prev_up_Kc && !curr_up_Kc) Kc++;
  if (prev_down_Kc && !curr_down_Kc) { Kc--; if (Kc < 0) Kc = 0; }
  if (prev_up_Tao && !curr_up_Tao) Tao_I++;
  if (prev_down_Tao && !curr_down_Tao) { Tao_I--; if (Tao_I < 1) Tao_I = 1; }

  prev_up_sp = curr_up_sp;
  prev_down_sp = curr_down_sp;
  prev_up_Kc = curr_up_Kc;
  prev_down_Kc = curr_down_Kc;
  prev_up_Tao = curr_up_Tao;
  prev_down_Tao = curr_down_Tao;

  if (detectado) {
    delayMicroseconds(valor);
    digitalWrite(pin_disparo, HIGH);
    delayMicroseconds(100);
    digitalWrite(pin_disparo, LOW);
    detectado = 0;

    if (Tiempo_actual - Tiempo_previo >= Read_Delay) {
      Tiempo_previo = Tiempo_actual;

      // Promediar 10 lecturas del TMP36
      float sum = 0;
      for (int i = 0; i < 10; i++) {
        int raw = analogRead(A);
        float voltage = raw * (3.3 / 4095.0);
        sum += (voltage - 0.5) * 100.0;
        delay(5);
      }
      Temperatura = sum / 10.0;

      if (Temperatura < Setpoint) {
        digitalWrite(pin_ventilador, HIGH);
      } else {
        digitalWrite(pin_ventilador, LOW);
      }

      if (Modo == 1) {
        Potencia = (Tiempo_actual <= Tiempo0) ? Potencia_1 : Potencia_2;
      } else if (Modo == 2) {
        if (Tiempo_actual <= Tiempo0) {
          PID_value = 0;
        } else {
          PID_error = Setpoint - Temperatura;
          Error_INT += PID_error * (1000.0 / Read_Delay);
          PID_value = Kc * (PID_error + (1.0 / Tao_I) * Error_INT);
        }
        Potencia = constrain(PID_value, 0, 100);
      }

      Serial.print("Pot:");
      Serial.print(Potencia);
      Serial.print(" Temp:");
      Serial.print(Temperatura);
      Serial.print(" Kc:");
      Serial.print(Kc);
      Serial.print(" Ti:");
      Serial.println(Tao_I);

      lcd.setCursor(0, 0);
      lcd.print("SP:");
      lcd.print(Setpoint);
      lcd.print((char)223);
      lcd.print(" T:");
      lcd.print(Temperatura);
      lcd.print((char)223);
      lcd.print("   ");

      lcd.setCursor(0, 1);
      lcd.print("Kc:");
      lcd.print(Kc);
      lcd.print(" Ti:");
      lcd.print(Tao_I);
      lcd.print("  ");
    }
  }

  // Firebase envío cada 10s
  app.loop();
  if (app.ready()) {
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime >= sendInterval) {
      lastSendTime = currentTime;

      Database.set<float>(aClient, "/TyT/KC", (float)Kc, processData, "Send_KC");
      Database.set<float>(aClient, "/TyT/Ti", (float)Tao_I, processData, "Send_Ti");
      Database.set<float>(aClient, "/TyT/SetPoint", (float)Setpoint, processData, "Send_SP");
      Database.set<float>(aClient, "/TyT/Temp", (float)Temperatura, processData, "Send_Temp");
    }
  }
}

// ----------------- Procesar Firebase -----------------
void processData(AsyncResult &aResult) {
  if (!aResult.isResult()) return;

  if (aResult.isError()) {
    Firebase.printf("Error [%s]: %s (code %d)\n",
      aResult.uid().c_str(),
      aResult.error().message().c_str(),
      aResult.error().code());
  } else {
    Firebase.printf("Success [%s]: %s\n",
      aResult.uid().c_str(),
      aResult.c_str());
}
}