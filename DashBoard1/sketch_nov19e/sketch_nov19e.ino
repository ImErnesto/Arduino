#include <WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>

const char* ssid = ".a";
const char* password = "ernesto12345678";
const char* mqtt_server = "broker.emqx.io";

WiFiClient esp32Client;
PubSubClient client(esp32Client);

// Pines y componentes
const int LDR_PIN = 36;
const int LUMINARIA_PIN = 15;
const int LED_PIN = 2;
const int RGB_R_PIN = 27;
const int RGB_G_PIN = 26;
const int RGB_B_PIN = 25;
Servo servoMotor;
const int SERVO_PIN = 32;
const int STEP_PINS[] = {33, 25, 26, 27}; // Pines del motor paso a paso

void setup_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String command = "";
    for (int i = 0; i < length; i++) {
        command += (char)payload[i];
    }

    if (String(topic) == "control/luminaria") {
        digitalWrite(LUMINARIA_PIN, command == "ON" ? HIGH : LOW);
    } else if (String(topic) == "control/led") {
        digitalWrite(LED_PIN, command == "ON" ? HIGH : LOW);
    } else if (String(topic) == "control/rgb") {
        int r = command.substring(0, 3).toInt();
        int g = command.substring(4, 7).toInt();
        int b = command.substring(8, 11).toInt();
        analogWrite(RGB_R_PIN, r);
        analogWrite(RGB_G_PIN, g);
        analogWrite(RGB_B_PIN, b);
    } else if (String(topic) == "control/servo") {
        int angle = command.toInt();
        servoMotor.write(angle);
    } else if (String(topic) == "control/stepper") {
        int steps = command.toInt();
        moveStepper(steps);
    }
}

void reconnect() {
    while (!client.connected()) {
        if (client.connect("ESP32Client")) {
            client.subscribe("control/luminaria");
            client.subscribe("control/led");
            client.subscribe("control/rgb");
            client.subscribe("control/servo");
            client.subscribe("control/stepper");
        } else {
            delay(5000);
        }
    }
}

void moveStepper(int steps) {
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < 4; j++) {
            digitalWrite(STEP_PINS[j], (i & (1 << j)) ? HIGH : LOW);
            delay(2);
        }
    }
}

void setup() {
    pinMode(LUMINARIA_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(RGB_R_PIN, OUTPUT);
    pinMode(RGB_G_PIN, OUTPUT);
    pinMode(RGB_B_PIN, OUTPUT);
    servoMotor.attach(SERVO_PIN);

    for (int i = 0; i < 4; i++) pinMode(STEP_PINS[i], OUTPUT);

    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Leer sensor LDR
    int lightLevel = analogRead(LDR_PIN);
    client.publish("sensor/light", String(lightLevel).c_str());
    delay(2000);
}
