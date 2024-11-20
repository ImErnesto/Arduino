#include <WiFi.h>
#include <PubSubClient.h>

// Configuración WiFi y MQTT
const char* ssid = ".a";
const char* password = "ernesto12345678";
const char* mqtt_server = "broker.emqx.io";

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

// Pines para dispositivos
const int pinR = 10;      // Rojo LED RGB
const int pinG = 9;       // Verde LED RGB
const int pinB = 8;       // Azul LED RGB
const int motordc = 11;   // Motor DC
const int potenciometro = 34;  // Potenciómetro (o simulador de temperatura)

bool estado1 = false, estado2 = false; // Estados de operación
int valorslider = 0;  // Valor recibido desde Node-RED (slider)

// Función para inicializar WiFi
void wifiInit() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado a WiFi");
}

// Función de callback MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    // Cambiar de estado (modo automático o manual)
    if (String(topic) == "control/mode") {
        if (message == "1") {
            estado1 = true;
            estado2 = false;
        } else if (message == "2") {
            estado1 = false;
            estado2 = true;
        }
    }
    // Control manual del motor (slider)
    else if (String(topic) == "control/slider" && estado2) {
        valorslider = message.toInt();
        if (valorslider < 0) valorslider = 0;
        if (valorslider > 100) valorslider = 100;
    }
}

// Reconexión MQTT
void reconnect() {
    while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP32Client")) {
            // Suscripciones
            mqttClient.subscribe("control/mode");     // Cambiar de estado
            mqttClient.subscribe("control/slider");  // Control manual del motor
        } else {
            delay(5000);
        }
    }
}

// Configurar pines y comunicación inicial
void setup() {
    Serial.begin(115200);

    // Configurar pines como salidas
    pinMode(pinR, OUTPUT);
    pinMode(pinG, OUTPUT);
    pinMode(pinB, OUTPUT);
    pinMode(motordc, OUTPUT);

    // Inicializar WiFi y MQTT
    wifiInit();
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
}

// Control RGB según temperatura
void controlarRGB(int temperatura) {
    int valorR = 0, valorG = 0, valorB = 0;

    if (temperatura >= 31) { // Alta temperatura (rojo)
        valorR = 255; valorG = 0; valorB = 0;
    } else if (temperatura >= 21) { // Temperatura media (verde)
        valorR = 0; valorG = 255; valorB = 0;
    } else { // Baja temperatura (azul)
        valorR = 0; valorG = 0; valorB = 255;
    }

    // Aplicar colores al LED RGB
    analogWrite(pinR, valorR);
    analogWrite(pinG, valorG);
    analogWrite(pinB, valorB);

    // Publicar estado del RGB
    String estadoRGB = String(valorR) + "," + String(valorG) + "," + String(valorB);
    mqttClient.publish("status/rgb", estadoRGB.c_str());
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Modo automático (estado1)
    if (estado1) {
        int lectura = analogRead(potenciometro);  // Leer potenciómetro
        int temperatura = map(lectura, 0, 4095, 10, 40);  // Simular temperatura (10-40 °C)

        // Publicar temperatura simulada
        mqttClient.publish("sensor/temperature", String(temperatura).c_str());

        // Controlar RGB y motor según temperatura
        controlarRGB(temperatura);
        int velocidad = map(lectura, 0, 4095, 0, 255);  // Mapeo para velocidad PWM
        analogWrite(motordc, velocidad);
    }
    // Modo manual (estado2)
    else if (estado2) {
        int velocidad = map(valorslider, 0, 100, 0, 255); // Mapeo del slider a PWM
        analogWrite(motordc, velocidad);

        // Apagar el LED RGB
        analogWrite(pinR, 0);
        analogWrite(pinG, 0);
        analogWrite(pinB, 0);
    }
    // Estado por defecto
    else {
        analogWrite(motordc, 0); // Apagar motor
        analogWrite(pinR, 0);
        analogWrite(pinG, 0);
        analogWrite(pinB, 0); // Apagar LED RGB
    }

    delay(100);
}
