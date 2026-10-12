/**
 * @file main.cpp
 * @brief Firmware assíncrono e simplificado para Sistema de Irrigação com ESP32.
 * @author Tiago Cunha (Refatorado por Gemini)
 * @version 1.0
 * @date 03 de Agosto de 2025
 */

//=================================================================================
// BIBLIOTECAS
//=================================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Preferences.h>
#include <DHT.h>
#include <RTClib.h>
#include "WiFiConfig.hpp"

//=================================================================================
// DEFINIÇÕES
//=================================================================================
// Hardware
#define BUZZER_PIN 13
#define LED_INDICADOR_WIFI 12
#define LED_INDICADOR_RELE 14
#define BUTTON_MODO 27
#define BUTTON_DISPLAY 26
#define RELAY_PIN 25
#define PINO_DHT 33
#define PINO_SOLO_1 32
#define PINO_SOLO_2 35
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Timers e Lógica
#define LONG_PRESS_TIME 5000
#define DISPLAY_TIMEOUT 60000
#define AUTO_PRESS_TIME 3000
#define TEMPO_MAX_REGA 900000
#define UMIDADE_SOLO_MAX 98.0f
#define MANUAL_TOGGLE_COOLDOWN 500
#define VALOR_SENSOR_SECO 3000
#define VALOR_SENSOR_MOLHADO 1000

#define PROJECT_IRRIGACAO

//=================================================================================
// OBJETOS E VARIÁVEIS GLOBAIS
//=================================================================================
SetupWiFi setupWiFi;
AsyncMqttClient mqttClient;
RTC_DS3231 rtc;
DHT dht(PINO_DHT, DHT11);
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -10800, 60000);
Preferences preferences;
TimerHandle_t mqttReconnectTimer, mainLoopTimer;

enum RelayState
{
    R_OFF,
    R_WARNING,
    R_ON,
    R_OFF_WARNING
};
RelayState releStateMachine = R_OFF;
enum SystemState
{
    CONNECTING_WIFI,
    CONNECTING_MQTT,
    OPERATIONAL
};
SystemState currentState = CONNECTING_WIFI;

String mqttClientId;

int pinosSolo[] = {PINO_SOLO_1, PINO_SOLO_2};
float temperatura, umidade, valorSolo;
int modoDisplay = 1;
bool displayLigado = true, modoMudou = true, modoAutomatico = true, estadoRele = false;
int horaProgramada, minutoProgramado, hora, minuto;
char dataAtual[11], horaAtual[9];
unsigned long ultimoTempoDisplay = 0, tempoPressionadoDisplay = 0, tempoPressionadoModo = 0, tempoRegaInicio = 0, ultimoToggleManual = 0, tempoFimMsgTemporaria = 0;
unsigned long releWarningTimestamp = 0;
int releWarningCount = 0;
bool estadoBotaoDisplayAnterior = LOW, estadoBotaoModoAnterior = LOW;
bool setupCompleto = false;
bool rtcConfigurado = false;
bool sensor1Ativo = true;
bool sensor2Ativo = true;

byte bar1[] = {B00000, B00000, B00000, B00000, B00000, B00000, B00000, B11111}, bar2[] = {B00000, B00000, B00000, B00000, B00000, B00000, B11111, B11111}, bar3[] = {B00000, B00000, B00000, B00000, B11111, B11111, B11111, B11111}, bar4[] = {B00000, B00000, B11111, B11111, B11111, B11111, B11111, B11111}, bar5[] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111};

//=================================================================================
// PROTÓTIPOS
//=================================================================================
void connectToMqtt();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void requestRelayStateChange(bool ligar);
void publishAllStates();
void ativarDisplay();
void exibirMensagemTemporaria(String l1, String l2, int duracao_ms);
void atualizarHoraLocal();
void handleBotoes();
void handleRelayStateMachine();
void verificarRega();
void handleDisplay();
void salvarEstadoSensores();
void handleStatusLED(); // Nova função

//=================================================================================
// LÓGICA DE SENSORES
//=================================================================================
void lerSensores()
{
    float temp_lida = dht.readTemperature();
    float umid_lida = dht.readHumidity();
    if (!isnan(temp_lida) && !isnan(umid_lida))
    {
        temperatura = temp_lida;
        umidade = umid_lida;
    }
    analogSetAttenuation(ADC_11db);
    analogSetWidth(12);
    float somaPorcentagem = 0;
    int sensoresValidos = 0;
    if (sensor1Ativo && PINO_SOLO_1 > 0)
    {
        int rawValue1 = analogRead(PINO_SOLO_1);
        if (rawValue1 >= 0 && rawValue1 <= 4095)
        {
            float porcentagem1 = map(rawValue1, VALOR_SENSOR_SECO, VALOR_SENSOR_MOLHADO, 0, 100);
            somaPorcentagem += constrain(porcentagem1, 0, 100);
            sensoresValidos++;
        }
    }
    if (sensor2Ativo && PINO_SOLO_2 > 0)
    {
        int rawValue2 = analogRead(PINO_SOLO_2);
        if (rawValue2 >= 0 && rawValue2 <= 4095)
        {
            float porcentagem2 = map(rawValue2, VALOR_SENSOR_SECO, VALOR_SENSOR_MOLHADO, 0, 100);
            somaPorcentagem += constrain(porcentagem2, 0, 100);
            sensoresValidos++;
        }
    }
    valorSolo = (sensoresValidos > 0) ? (somaPorcentagem / sensoresValidos) : 0;
}

//=================================================================================
// SETUP
//=================================================================================
void setup()
{
    Serial.begin(115200);

    Wire.begin();
    pinMode(BUTTON_DISPLAY, INPUT_PULLUP);
    pinMode(BUTTON_MODO, INPUT_PULLUP);
    pinMode(RELAY_PIN, OUTPUT);
    pinMode(LED_INDICADOR_RELE, OUTPUT);
    pinMode(LED_INDICADOR_WIFI, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_INDICADOR_WIFI, LOW);

    lcd.init();
    lcd.backlight();
    lcd.createChar(0, bar1);
    lcd.createChar(1, bar2);
    lcd.createChar(2, bar3);
    lcd.createChar(3, bar4);
    lcd.createChar(4, bar5);
    exibirMensagemTemporaria("Iniciando...", "Versao 1.0", 2500);

    if (!rtc.begin())
    {
        exibirMensagemTemporaria("ERRO RTC!", "Verifique conexao", 10000);
        delay(10000);
    }
    else
    {
        if (rtc.lostPower() || rtc.now().year() < 2025)
        {
            Serial.println("RTC com data invalida. Aguardando WiFi para sincronizar...");
        }
        else
        {
            rtcConfigurado = true;
        }
    }

    setupWiFi.carregarConfiguracoes();
    preferences.begin("irrigacao", true);
    sensor1Ativo = preferences.getBool("sensor1", true);
    sensor2Ativo = preferences.getBool("sensor2", true);
    horaProgramada = preferences.getInt("hora", 21);
    minutoProgramado = preferences.getInt("minuto", 30);
    preferences.end();

    dht.begin();
    lerSensores();

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);

    // Callback para processar comandos recebidos do Home Assistant
    mqttClient.onMessage([](char *topic, char *payload, AsyncMqttClientMessageProperties p,
                            size_t len, size_t i, size_t t)
                         {
    String msg; msg.reserve(len);
    for(size_t i=0; i<len; i++) msg += (char)payload[i];

    String topicStr = String(topic);

    if (topicStr == "home/irrigacao/rele/set") {
        exibirMensagemTemporaria("Comando Recebido", msg == "ON" ? "Ligar Rele" : "Desligar Rele", 3000);
        if(!modoAutomatico) requestRelayStateChange(msg=="ON");
    }
    else if (topicStr == "home/irrigacao/mode/set") {
        exibirMensagemTemporaria("Comando Recebido", "Modo: " + msg, 3000);
        modoAutomatico = (msg=="AUTO");
        if(modoAutomatico) requestRelayStateChange(false);
        publishAllStates();
    }
    else if (topicStr == "home/irrigacao/hora/set") {
        horaProgramada = msg.toInt();
        exibirMensagemTemporaria("Nova Hora:", String(horaProgramada) + "h", 2000);
        preferences.begin("irrigacao", false);
        preferences.putInt("hora", horaProgramada);
        preferences.end();
        publishAllStates();
    }
    else if (topicStr == "home/irrigacao/minuto/set") {
        minutoProgramado = msg.toInt();
        exibirMensagemTemporaria("Novo Minuto:", String(minutoProgramado) + "m", 2000);
        preferences.begin("irrigacao", false);
        preferences.putInt("minuto", minutoProgramado);
        preferences.end();
        publishAllStates();
    }
    else if (topicStr == "home/irrigacao/sensor1/set") {
        sensor1Ativo = (msg == "ON");
        salvarEstadoSensores();
        exibirMensagemTemporaria("Sensor 1", sensor1Ativo ? "Ativado" : "Desativado", 2000);
        publishAllStates();
    }
    else if (topicStr == "home/irrigacao/sensor2/set") {
        sensor2Ativo = (msg == "ON");
        salvarEstadoSensores();
        exibirMensagemTemporaria("Sensor 2", sensor2Ativo ? "Ativado" : "Desativado", 2000);
        publishAllStates();
    } });

    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(5000), pdFALSE, (void *)0,
                                      [](TimerHandle_t xTimer)
                                      { connectToMqtt(); });
    mainLoopTimer = xTimerCreate("mainLoopTimer", pdMS_TO_TICKS(1000), pdTRUE, (void *)0,
                                 [](TimerHandle_t xTimer)
                                 { atualizarHoraLocal(); modoMudou = true; });

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
                     currentState = CONNECTING_MQTT;
                     timeClient.begin();

                     if (!rtcConfigurado)
                     {
                         timeClient.forceUpdate();
                         rtc.adjust(DateTime(timeClient.getEpochTime()));
                         rtcConfigurado = true;
                     }

                     xTimerStart(mainLoopTimer, 0);

                     xTimerStart(mqttReconnectTimer, 0); },
                 ARDUINO_EVENT_WIFI_STA_GOT_IP);

    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
                 {
    currentState = CONNECTING_WIFI;
    xTimerStop(mqttReconnectTimer, 0); }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

    WiFi.begin();
    if (rtcConfigurado)
        xTimerStart(mainLoopTimer, 0);
    setupCompleto = true;
}

//=================================================================================
// LOOP PRINCIPAL
//=================================================================================
void loop()
{
    if (setupCompleto)
    {
        handleBotoes();
        handleRelayStateMachine();
        verificarRega();
        handleDisplay();
        handleStatusLED();
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

//=================================================================================
// FUNÇÃO DE STATUS DO LED
//=================================================================================
void handleStatusLED()
{
    static uint32_t ledTimestamp = 0;
    uint32_t now = millis();

    switch (currentState)
    {
    case CONNECTING_WIFI: // Pisca rápido
        if (now - ledTimestamp > 150)
        {
            ledTimestamp = now;
            digitalWrite(LED_INDICADOR_WIFI, !digitalRead(LED_INDICADOR_WIFI));
        }
        break;
    case CONNECTING_MQTT: // Pisca lento (2s aceso, 1s apagado)
        if (digitalRead(LED_INDICADOR_WIFI) == HIGH)
        {
            if (now - ledTimestamp > 2000)
            {
                ledTimestamp = now;
                digitalWrite(LED_INDICADOR_WIFI, LOW);
            }
        }
        else
        {
            if (now - ledTimestamp > 1000)
            {
                ledTimestamp = now;
                digitalWrite(LED_INDICADOR_WIFI, HIGH);
            }
        }
        break;
    case OPERATIONAL: // Aceso
        if (!digitalRead(LED_INDICADOR_WIFI))
        {
            digitalWrite(LED_INDICADOR_WIFI, HIGH);
        }
        break;
    }
}

//=================================================================================
// MÁQUINA DE ESTADOS DO RELÉ
//=================================================================================
void handleRelayStateMachine()
{
    unsigned long now = millis();
    if (releStateMachine == R_WARNING)
    {
        if (now - releWarningTimestamp >= 700)
        {
            releWarningTimestamp = now;
            releWarningCount++;
            digitalWrite(LED_INDICADOR_RELE, HIGH);
            tone(BUZZER_PIN, 1200, 150);
            delay(150);
            digitalWrite(LED_INDICADOR_RELE, LOW);
        }
        if (releWarningCount >= 3)
        {
            digitalWrite(RELAY_PIN, HIGH);
            digitalWrite(LED_INDICADOR_RELE, HIGH);
            estadoRele = true;
            tempoRegaInicio = millis();
            releStateMachine = R_ON;
            if (mqttClient.connected())
                mqttClient.publish("home/irrigacao/rele/state", 1, true, "ON");
        }
    }
    else if (releStateMachine == R_OFF_WARNING)
    {
        if (now - releWarningTimestamp >= 250)
        {
            releWarningTimestamp = now;
            releWarningCount++;
            digitalWrite(LED_INDICADOR_RELE, HIGH);
            tone(BUZZER_PIN, 1500, 100);
            delay(100);
            digitalWrite(LED_INDICADOR_RELE, LOW);
        }
        if (releWarningCount >= 3)
        {
            digitalWrite(RELAY_PIN, LOW);
            digitalWrite(LED_INDICADOR_RELE, LOW);
            estadoRele = false;
            releStateMachine = R_OFF;
            if (mqttClient.connected())
                mqttClient.publish("home/irrigacao/rele/state", 1, true, "OFF");
        }
    }
}

void requestRelayStateChange(bool ligar)
{
    if ((ligar && releStateMachine == R_OFF) || (!ligar && releStateMachine == R_ON))
    {
        releStateMachine = ligar ? R_WARNING : R_OFF_WARNING;
        releWarningTimestamp = millis();
        releWarningCount = 0;
    }
}

//=================================================================================
// LÓGICA DE FUNCIONAMENTO
//=================================================================================

void verificarRega()
{
    if (estadoRele && (millis() - tempoRegaInicio >= TEMPO_MAX_REGA))
    {
        exibirMensagemTemporaria("Seguranca Ativada", "Tempo Esgotado", 3000);
        requestRelayStateChange(false);
    }
    if (!modoAutomatico || !rtcConfigurado)
        return;
    if (!estadoRele && hora == horaProgramada && minuto == minutoProgramado)
    {
        requestRelayStateChange(true);
    }
    if (estadoRele && (valorSolo >= UMIDADE_SOLO_MAX))
    {
        exibirMensagemTemporaria("Irrigacao Concluida", "Solo Umido", 3000);
        requestRelayStateChange(false);
    }
}

void handleDisplay()
{
    if (displayLigado && (millis() - ultimoTempoDisplay > DISPLAY_TIMEOUT))
    {
        displayLigado = false;
        lcd.noBacklight();
    }
    if (tempoFimMsgTemporaria > 0 && millis() >= tempoFimMsgTemporaria)
    {
        tempoFimMsgTemporaria = 0;
        modoMudou = true;
    }
    if (!displayLigado || tempoFimMsgTemporaria > 0 || !modoMudou)
        return;

    atualizarHoraLocal();
    static unsigned long lastSensorReadForDisplay = 0;
    if (millis() - lastSensorReadForDisplay > 2000)
    {
        lerSensores();
        lastSensorReadForDisplay = millis();
    }

    lcd.clear();
    char buffer[17];
    switch (modoDisplay)
    {
    case 1:
        lcd.setCursor(0, 0);
        lcd.print("Data: " + String(dataAtual));
        lcd.setCursor(0, 1);
        lcd.print("Hora: " + String(horaAtual));
        break;
    case 2:
        snprintf(buffer, sizeof(buffer), "T:%.1fC U:%.0f%%", temperatura, umidade);
        lcd.setCursor(0, 0);
        lcd.print(buffer);
        snprintf(buffer, sizeof(buffer), "Solo: %.1f%%", valorSolo);
        lcd.setCursor(0, 1);
        lcd.print(buffer);
        break;
    case 3:
        lcd.setCursor(0, 0);
        lcd.print("Rele: ");
        lcd.print(estadoRele ? "LIGADO" : "DESL");
        lcd.setCursor(0, 1);
        lcd.print("Modo: ");
        lcd.print(modoAutomatico ? "AUTO" : "MANUAL");
        break;
    case 4:
        char horaProg[17];
        sprintf(horaProg, "Irrigar as %02d:%02d", horaProgramada, minutoProgramado);
        lcd.setCursor(0, 0);
        lcd.print(horaProg);
        break;
    case 5:
        lcd.print("Sinal WiFi:");
        if (WiFi.isConnected())
        {
            long rssi = WiFi.RSSI();
            int quality;

            if (rssi <= -100)
                quality = 0;
            else if (rssi >= -50)
                quality = 100;
            else
                quality = 2 * (rssi + 100);

            int numBars = map(constrain(rssi, -100, -50), -100, -50, 1, 5);

            lcd.setCursor(0, 1);
            for (int i = 0; i < 5; i++)
            {
                lcd.write(byte(i < numBars ? i : 0));
            }
            lcd.print(" " + String(quality) + "%");
        }
        else
        {
            lcd.setCursor(0, 1);
            lcd.print("Desconectado");
        }
        break;
    }
    modoMudou = false;
}

void handleBotoes()
{
    if (!setupCompleto)
        return;
    bool btnDisplayState = (digitalRead(BUTTON_DISPLAY) == LOW);
    if (btnDisplayState != estadoBotaoDisplayAnterior)
    {
        estadoBotaoDisplayAnterior = btnDisplayState;
        if (btnDisplayState)
        {
            tempoPressionadoDisplay = millis();
        }
        else
        {
            if (millis() - tempoPressionadoDisplay >= LONG_PRESS_TIME)
            {
                setupWiFi.iniciarPortalConfiguracao();
            }
            else
            {
                if (!displayLigado)
                {
                    ativarDisplay();
                }
                else
                {
                    modoDisplay++;
                    if (modoDisplay > 5)
                        modoDisplay = 1;
                }
                modoMudou = true;
            }
        }
    }

    bool btnModoState = (digitalRead(BUTTON_MODO) == LOW);
    if (btnModoState != estadoBotaoModoAnterior)
    {
        estadoBotaoModoAnterior = btnModoState;
        if (btnModoState)
        {
            tempoPressionadoModo = millis();
        }
        else
        {
            if (millis() - tempoPressionadoModo >= AUTO_PRESS_TIME)
            {
                modoAutomatico = !modoAutomatico;
                tone(BUZZER_PIN, 1000, 250);
                exibirMensagemTemporaria("Modo Alterado:", modoAutomatico ? "AUTOMATICO" : "MANUAL", 2000);
                if (modoAutomatico)
                    requestRelayStateChange(false);
                publishAllStates();
            }
            else
            {
                if (modoAutomatico)
                {
                    exibirMensagemTemporaria("Acao bloqueada", "Modo AUTO", 2000);
                    tone(BUZZER_PIN, 500, 200);
                }
                else
                {
                    if (millis() - ultimoToggleManual > MANUAL_TOGGLE_COOLDOWN)
                    {
                        ultimoToggleManual = millis();
                        requestRelayStateChange(!estadoRele);
                    }
                }
            }
        }
    }
}

//=================================================================================
// FUNÇÕES DE CONECTIVIDADE E MQTT
//=================================================================================
void onMqttConnect(bool sessionPresent)
{
    Serial.println("Conectado ao Broker MQTT!");
    currentState = OPERATIONAL;

    mqttClient.subscribe("home/irrigacao/+/set", 1);

    const char *deviceJson = "{\"ids\":[\"esp32_irrigacao_01\"],\"name\":\"Sistema de Irrigacao\",\"mf\":\"TCASTROCNA\",\"mdl\":\"shx-irrigacion v1\",\"sw\":\"9.4\"}";
    char topic[128], payload[512];

    // publica entidades no HA
    snprintf(topic, sizeof(topic), "homeassistant/switch/irrigacao_rele/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Irrigacao Rele\",\"cmd_t\":\"home/irrigacao/rele/set\",\"stat_t\":\"home/irrigacao/rele/state\",\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    snprintf(topic, sizeof(topic), "homeassistant/select/irrigacao_mode/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Modo Irrigacao\",\"cmd_t\":\"home/irrigacao/mode/set\",\"stat_t\":\"home/irrigacao/mode/state\",\"options\":[\"AUTO\",\"MANUAL\"],\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    snprintf(topic, sizeof(topic), "homeassistant/number/irrigacao_hora/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Hora Irrigacao\",\"cmd_t\":\"home/irrigacao/hora/set\",\"stat_t\":\"home/irrigacao/hora/state\",\"min\":0,\"max\":23,\"step\":1,\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    snprintf(topic, sizeof(topic), "homeassistant/number/irrigacao_minuto/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Minuto Irrigacao\",\"cmd_t\":\"home/irrigacao/minuto/set\",\"stat_t\":\"home/irrigacao/minuto/state\",\"min\":0,\"max\":59,\"step\":1,\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    snprintf(topic, sizeof(topic), "homeassistant/switch/sensor_solo1/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Sensor Solo 1\",\"cmd_t\":\"home/irrigacao/sensor1/set\",\"stat_t\":\"home/irrigacao/sensor1/state\",\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    snprintf(topic, sizeof(topic), "homeassistant/switch/sensor_solo2/config");
    snprintf(payload, sizeof(payload), "{\"name\":\"Sensor Solo 2\",\"cmd_t\":\"home/irrigacao/sensor2/set\",\"stat_t\":\"home/irrigacao/sensor2/state\",\"dev\":%s}", deviceJson);
    mqttClient.publish(topic, 1, true, payload);

    publishAllStates();
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Desconectado do Broker MQTT.");
    currentState = CONNECTING_MQTT; // 🔥 volta LED para piscar lento

    if (WiFi.isConnected())
    {
        // agenda reconexão com delay já definido no timer
        xTimerStart(mqttReconnectTimer, 0);
    }
}

void connectToMqtt()
{
    if (WiFi.isConnected() && setupWiFi.mqtt_broker.length() > 0)
    {
        mqttClient.setServer(setupWiFi.mqtt_broker.c_str(), setupWiFi.mqtt_porta);
        mqttClient.setCredentials(setupWiFi.mqtt_usuario.c_str(), setupWiFi.mqtt_senha.c_str());

        mqttClientId = "ESP32-Irrigacao-" + String((uint32_t)ESP.getEfuseMac(), HEX);
        mqttClient.setClientId(mqttClientId.c_str());

        Serial.println("Tentando conectar ao broker MQTT...");
        mqttClient.connect();
    }
}

void publishAllStates()
{
    if (!mqttClient.connected())
        return;
    char buffer[10];
    dtostrf(temperatura, 4, 1, buffer);
    mqttClient.publish("home/irrigacao/temperatura", 1, true, buffer);
    dtostrf(umidade, 4, 1, buffer);
    mqttClient.publish("home/irrigacao/umidade", 1, true, buffer);
    dtostrf(valorSolo, 4, 1, buffer);
    mqttClient.publish("home/irrigacao/solo", 1, true, buffer);
    mqttClient.publish("home/irrigacao/rele/state", 1, true, estadoRele ? "ON" : "OFF");
    mqttClient.publish("home/irrigacao/mode/state", 1, true, modoAutomatico ? "AUTO" : "MANUAL");
    sprintf(buffer, "%d", horaProgramada);
    mqttClient.publish("home/irrigacao/hora/state", 1, true, buffer);
    sprintf(buffer, "%d", minutoProgramado);
    mqttClient.publish("home/irrigacao/minuto/state", 1, true, buffer);
    mqttClient.publish("home/irrigacao/sensor1/state", 1, true, sensor1Ativo ? "ON" : "OFF");
    mqttClient.publish("home/irrigacao/sensor2/state", 1, true, sensor2Ativo ? "ON" : "OFF");
}

//=================================================================================
// FUNÇÕES AUXILIARES
//=================================================================================
void salvarEstadoSensores()
{
    preferences.begin("irrigacao", false);
    preferences.putBool("sensor1", sensor1Ativo);
    preferences.putBool("sensor2", sensor2Ativo);
    preferences.end();
}

void ativarDisplay()
{
    if (!displayLigado)
    {
        lcd.backlight();
        displayLigado = true;
    }
    ultimoTempoDisplay = millis();
}

void exibirMensagemTemporaria(String l1, String l2, int duracao_ms)
{
    ativarDisplay();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(l1);
    lcd.setCursor(0, 1);
    lcd.print(l2);
    tempoFimMsgTemporaria = millis() + duracao_ms;
}

void atualizarHoraLocal()
{
    if (!rtcConfigurado)
    {
        strcpy(dataAtual, "--/--/----");
        strcpy(horaAtual, "--:--:--");
        return;
    }
    DateTime now = rtc.now();
    sprintf(dataAtual, "%02d/%02d/%04d", now.day(), now.month(), now.year());
    sprintf(horaAtual, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
    hora = now.hour();
    minuto = now.minute();
}