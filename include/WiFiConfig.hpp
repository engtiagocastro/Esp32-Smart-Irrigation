/**
 * @file WiFiConfig.hpp
 * @brief Classe PADRÃO para gerenciar a configuração de Wi-Fi e MQTT via WiFiManager.
 * @author Tiago Cunha (Padronizado e Corrigido por Gemini)
 * @version 1.0
 * @date 03 de Agosto de 2025
 */

#ifndef WIFICONFIG_HPP
#define WIFICONFIG_HPP

#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <LiquidCrystal_I2C.h>

// Informa a este arquivo que o objeto 'lcd' existe e será fornecido pelo 'main.cpp'
extern LiquidCrystal_I2C lcd;

//=================================================================================
// DEFINIÇÃO DA CLASSE SETUPWIFI
//=================================================================================
class SetupWiFi {
public:
    String mqtt_broker;
    String mqtt_usuario;
    int mqtt_porta;
    String mqtt_senha;

    SetupWiFi() { instance = this; }

    void carregarConfiguracoes() {
        Preferences preferences;
        preferences.begin("config", true);
        mqtt_broker = preferences.getString("broker", "");
        mqtt_usuario = preferences.getString("usuario", "");
        mqtt_porta = preferences.getInt("porta", 1883);
        mqtt_senha = preferences.getString("senha", "");
        preferences.end();
    }

    void salvarConfiguracoes() {
        if (p_mqtt_broker) mqtt_broker = p_mqtt_broker->getValue();
        if (p_mqtt_usuario) mqtt_usuario = p_mqtt_usuario->getValue();
        if (p_mqtt_porta) mqtt_porta = atoi(p_mqtt_porta->getValue());
        if (p_mqtt_senha) mqtt_senha = p_mqtt_senha->getValue();

        Preferences preferences;
        preferences.begin("config", false);
        preferences.putString("broker", mqtt_broker);
        preferences.putString("usuario", mqtt_usuario);
        preferences.putInt("porta", mqtt_porta);
        preferences.putString("senha", mqtt_senha);
        preferences.end();
        Serial.println("Configurações salvas na memória NVS.");
    }

    void iniciarPortalConfiguracao(bool bloquear = true) {
        WiFiManager wm;
        
        const char* portalName = "ESP32-Irrigation-Setup";

        // Mensagem no display para indicar o modo de configuração
        lcd.backlight();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Modo Configuracao");
        lcd.setCursor(0, 1);
        lcd.print(portalName);

        carregarConfiguracoes();
        char portaStr[6];
        sprintf(portaStr, "%d", mqtt_porta);
        
        p_mqtt_broker = new WiFiManagerParameter("broker", "MQTT Broker", mqtt_broker.c_str(), 40);
        p_mqtt_usuario = new WiFiManagerParameter("usuario", "MQTT Usuário", mqtt_usuario.c_str(), 20);
        p_mqtt_porta = new WiFiManagerParameter("porta", "MQTT Porta", portaStr, 6);
        p_mqtt_senha = new WiFiManagerParameter("senha", "MQTT Senha", mqtt_senha.c_str(), 40);

        wm.addParameter(p_mqtt_broker);
        wm.addParameter(p_mqtt_usuario);
        wm.addParameter(p_mqtt_porta);
        wm.addParameter(p_mqtt_senha);
        
        wm.setSaveParamsCallback(saveParamsCallback);
        wm.setConfigPortalTimeout(180);
        wm.setBreakAfterConfig(true);

        Serial.println("Iniciando portal de configuração...");
        bool salvou = wm.startConfigPortal(portalName);

        if (!salvou) {
            Serial.println("Portal expirou ou foi fechado sem salvar.");
        } else {
            Serial.println("Configuração salva pelo portal. Reiniciando...");
            lcd.clear();
            lcd.print("Config. Salva!");
            lcd.setCursor(0,1);
            lcd.print("Reiniciando...");
            delay(3000);
            ESP.restart();
        }
    }

private:
    static SetupWiFi* instance;
    WiFiManagerParameter* p_mqtt_broker;
    WiFiManagerParameter* p_mqtt_usuario;
    WiFiManagerParameter* p_mqtt_porta;
    WiFiManagerParameter* p_mqtt_senha;

    static void saveParamsCallback() {
        if (instance) instance->salvarConfiguracoes();
    }
};

SetupWiFi* SetupWiFi::instance = nullptr;

#endif // WIFICONFIG_HPP