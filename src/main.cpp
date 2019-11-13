/*
 *  Relay control with MQTT iterface based on Sonnoff Basic
 *
 *  Alex Lebedev
 *  alex.lebedev@me.com
 *  
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "creds.h"

#define BUTTON      0     // 0 - pushed
#define RELAY       12    // 1 - on
#define GREEN_LED   13    // 0 - on
#define RX_PIN      1
#define TX_PIN      3

#define HEARTBIT_INTERVAL       60 * 1000

WiFiClient espClient;
PubSubClient mqtt(espClient);

const char* mqtt_server       = "10.0.0.3";
const char* mqtt_device       = "esp_relay_3";
const char* mqtt_device_name  = "ESP Relay 3";

volatile unsigned long button_debounce = 0;
volatile bool button_pressed_flag = false;
bool is_on = false;
int status = 0; // 0 - all ok, -1 - no wifi, -2 - no mqtt
unsigned long last_mqtt_attempt = 0;
unsigned long last_heartbit = 0;
unsigned long last_read_sensor = 0;
float temperature = NAN, humidity = NAN;
bool sensor_error_sent = false;

const size_t topic_len = 100;
char topic[100];
const size_t msg_len = 25;
char msg[25];

void button_int_handler();
void mqtt_message_handler(char*, byte*, unsigned int);
void update_gpio();
bool is_button_pressed();
void blink_green_led(int);
void set_on();
void set_off();
void connect_mqtt_if_needed();
void check_status();
void update_mqtt(bool, bool, bool);
void send_heartbit();
void format_topic(const char*, const char*, bool);
void mqtt_connected();
void mqtt_debug(const char*);

void setup() {  
  pinMode(BUTTON, INPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON), button_int_handler, CHANGE);
  update_gpio();

  WiFi.begin(ssid, password);  

  mqtt.setServer(mqtt_server, 1884);
  mqtt.setCallback(mqtt_message_handler);
}

void loop() {
  mqtt.loop();

  if (is_button_pressed()) {
    if (is_on) {
      set_off();
    } else {
      set_on();
    }
    update_mqtt(false, true, false);    
  }

  connect_mqtt_if_needed();
  check_status();
  update_gpio();
  send_heartbit();

  delay(50);
}

void check_status() {
//  WiFi.printDiag(Serial);
  if (!WiFi.status() == WL_CONNECTED) {
    status= -1;
  } else if (!mqtt.connected()) {
    status = -2;
  } else {
    status = 0;
  }
}

// ============= MQTT ==============

void connect_mqtt_if_needed() {
  if (mqtt.connected() || millis() - last_mqtt_attempt < 5000) {
    return;
  }
  if (mqtt.connect(mqtt_device, mqtt_login, mqtt_pass)) {
    mqtt_connected();
  }
  last_mqtt_attempt = millis();
}

void mqtt_connected() {
  update_mqtt(true, true, true);
  last_heartbit = millis();

  format_topic("On", "", true);
  mqtt.subscribe(topic);
}

void send_heartbit() {
  if (millis() - last_heartbit < 10 * 1000) {
    return;
  }
  update_mqtt(false, true, true);
  last_heartbit = millis();
}

void format_topic(const char* control, const char* meta, bool setter) {
  if (*control) {
    if (*meta) {
      snprintf(topic, topic_len, "/devices/%s/controls/%s/meta/%s", mqtt_device, control, meta);
    } else {
      if (setter) {
        snprintf(topic, topic_len, "/devices/%s/controls/%s/on", mqtt_device, control);
      } else {
        snprintf(topic, topic_len, "/devices/%s/controls/%s", mqtt_device, control);
      }
    }
  } else {
    if (*meta) {
      snprintf(topic, topic_len, "/devices/%s/meta/%s", mqtt_device, meta);
    } else {
      snprintf(topic, topic_len, "/devices/%s", mqtt_device);
    }
  }
}

void update_mqtt(bool meta, bool status, bool info) {
  if (status || info) {
    format_topic("On", "", false);
    mqtt.publish(topic, is_on ? "1" : "0", true);
  }
  if (info) {
    format_topic("IP", "", false);
    mqtt.publish(topic, WiFi.localIP().toString().c_str(), true);

    format_topic("RSSI", "", false);
    snprintf(msg, msg_len, "%d dB", (int)WiFi.RSSI());
    mqtt.publish(topic, msg, true);
  }
  if (meta) {
    format_topic("", "name", false);
    mqtt.publish(topic, mqtt_device_name, true);

    format_topic("On", "type", false);
    mqtt.publish(topic, "switch", true);
    format_topic("On", "order", false);
    mqtt.publish(topic, "1", true);

    format_topic("IP", "type", false);
    mqtt.publish(topic, "text", true);
    format_topic("IP", "readonly", false);
    mqtt.publish(topic, "1", true);
    format_topic("IP", "order", false);
    mqtt.publish(topic, "2", true);

    format_topic("RSSI", "type", false);
    mqtt.publish(topic, "text", true);
    format_topic("RSSI", "readonly", false);
    mqtt.publish(topic, "1", true);
    format_topic("RSSI", "order", false);
    mqtt.publish(topic, "3", true);
  }
}

void mqtt_message_handler(char* rcv_topic, byte* payload, unsigned int length) {
  strncpy(msg, (char*)payload, length);
  msg[length] = 0;

  format_topic("On", "", true);
  if (strcmp(topic, rcv_topic) == 0) {
    if (strcmp(msg, "1") == 0) {
      if (is_on == false) {
        set_on();
      }
    } else if (strcmp(msg, "0") == 0) {
      if (is_on == true) {
        set_off();
      }
    }
    update_mqtt(false, true, false);
    return;
  }
}

void mqtt_debug(const char* text) {
  format_topic("Debug", "", false);
  mqtt.publish(topic, text, false);
}

// ================= Timer ======================

void set_on() {
  blink_green_led(5);
  is_on = true;
}

void set_off() {
  is_on = false;
}

// =========== GPIO =================

void update_gpio() {
  digitalWrite(RELAY, is_on);
  
  if (status == -1) {
    digitalWrite(GREEN_LED, millis() % 500 > 250);
  } else if (status == -2) {
    digitalWrite(GREEN_LED, millis() % 1000 > 500);
  } else {
    digitalWrite(GREEN_LED, is_on);
  }
}

void blink_green_led(int count) {
  for (int i=0; i<count; ++i) {
    digitalWrite(GREEN_LED, 1);
    delay(50);
    digitalWrite(GREEN_LED, 0);
    delay(50);
  }
}

// ============== Button =================

void button_int_handler() {
  if (digitalRead(BUTTON) == 0) { 
    if (millis() - button_debounce < 200) return;
    button_pressed_flag = true;
    button_debounce = millis();
  }
}

bool is_button_pressed() {
  bool flag = button_pressed_flag;
  button_pressed_flag = false;
  return flag;
}
