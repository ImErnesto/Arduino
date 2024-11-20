#include <WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

// WiFi y MQTT
const char* ssid = ".a";
const char* password = "ernesto12345678";
const char* mqtt_server = "broker.emqx.io";

WiFiClient esp32Client;
PubSubClient mqttClient(esp32Client);

// Pines para dispositivos
const int LED_PIN = 2;               // LED
const int RGB_R_PIN = 27;            // Rojo LED RGB
const int RGB_G_PIN = 26;            // Verde LED RGB
const int RGB_B_PIN = 25;            // Azul LED RGB
const int LDR_PIN = 36;              // Fotoresistencia
const int POT_PIN = 34;              // Resistencia variable (potenciómetro)
Servo servoMotor;                    // Objeto Servo
const int SERVO_PIN = 32;            // Servo motor
const int STEP_PINS[] = {33, 25, 26, 27}; // Motor paso a paso
const int LUMINARIA_PIN = 15;        // Luminaria

// Inicialización WiFi
void wifiInit() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConectado a WiFi");
}

// Manejo de mensajes MQTT
void callback(char* topic, byte* payload, unsigned int length) {
    String command = "";
    for (int i = 0; i < length; i++) {
        command += (char)payload[i];
    }

    // Control LED RGB
    if (String(topic) == "control/rgb") {
        int r = command.substring(0, 3).toInt();
        int g = command.substring(4, 7).toInt();
        int b = command.substring(8, 11).toInt();
        analogWrite(RGB_R_PIN, r);
        analogWrite(RGB_G_PIN, g);
        analogWrite(RGB_B_PIN, b);

        // Publicar estado actual del RGB
        String status = String(r) + "," + String(g) + "," + String(b);
        mqttClient.publish("status/rgb", status.c_str());
    }
    // Control del servo motor
    else if (String(topic) == "control/servo") {
        int angle = command.toInt();
        servoMotor.write(angle);

        // Publicar ángulo actual del servo
        mqttClient.publish("status/servo", String(angle).c_str());
    }
    // Control del motor paso a paso
    else if (String(topic) == "control/stepper") {
        int steps = command.toInt();
        moveStepper(steps);

        // Publicar pasos ejecutados
        mqttClient.publish("status/stepper", String(steps).c_str());
    }
    // Control manual de la luminaria
    else if (String(topic) == "control/luminaria") {
        digitalWrite(LUMINARIA_PIN, command == "ON" ? HIGH : LOW);

        // Publicar estado actual de la luminaria
        mqttClient.publish("status/luminaria", command.c_str());
    }
}

// Reconexión al broker MQTT
void reconnect() {
    while (!mqttClient.connected()) {
        Serial.print("Conectando al broker MQTT...");
        if (mqttClient.connect("ESP32Client")) {
            Serial.println("Conectado");

            // Suscripciones
            mqttClient.subscribe("control/rgb");
            mqttClient.subscribe("control/servo");
            mqttClient.subscribe("control/stepper");
            mqttClient.subscribe("control/luminaria");
        } else {
            Serial.print("Fallo, rc=");
            Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

// Mover motor paso a paso
void moveStepper(int steps) {
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < 4; j++) {
            digitalWrite(STEP_PINS[j], (i & (1 << j)) ? HIGH : LOW);
            delay(2);
        }
    }
}

void setup() {
    Serial.begin(115200);

    // Configuración de pines
    pinMode(LED_PIN, OUTPUT);
    pinMode(RGB_R_PIN, OUTPUT);
    pinMode(RGB_G_PIN, OUTPUT);
    pinMode(RGB_B_PIN, OUTPUT);
    pinMode(LDR_PIN, INPUT);
    pinMode(POT_PIN, INPUT);
    pinMode(LUMINARIA_PIN, OUTPUT);

    for (int i = 0; i < 4; i++) {
        pinMode(STEP_PINS[i], OUTPUT);
    }

    servoMotor.attach(SERVO_PIN);

    // Inicializar WiFi y MQTT
    wifiInit();
    mqttClient.setServer(mqtt_server, 1883);
    mqttClient.setCallback(callback);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    // Publicar datos del LDR
    int lightLevel = analogRead(LDR_PIN);
    mqttClient.publish("sensor/light", String(lightLevel).c_str());

    // Publicar datos del potenciómetro
    int potValue = analogRead(POT_PIN);
    mqttClient.publish("sensor/pot", String(potValue).c_str());

    delay(2000); // Publicar cada 2 segundos
}
