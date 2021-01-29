

#include "homeassistant.h"

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#include "Arduino.h"

void mqttcallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void HomeassistantMqtt::reconnect() {
  while (!mqtt.connected()) {
    Serial.println("Attempting MQTT connection...");

    String clientId = device_id;
    Serial.println("ClientID: '" + clientId + "'");

    if (mqtt.connect("pRF2", "mqtt", "mqttmqtt")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqtt.state());
      Serial.println(" try again in 5 seconds");

      delay(5000);
    }

    mqtt.loop();
  }
}

HomeassistantMqtt::HomeassistantMqtt() {
  espClient = WiFiClient();
  mqtt = PubSubClient(espClient);
}

void HomeassistantMqtt::init(const char* _device_id, const char* _mqtt_server) {
  mqtt_server = _mqtt_server;
  //char device_id = "pRF_" + _device_id;
  //device_id.toUpperCase();

  Serial.println("Setting MQTT Server");
  mqtt.setServer(mqtt_server, 1883);
  mqtt.setBufferSize(1024);
  mqtt.setCallback(mqttcallback);

  Serial.println("MQTT Connect");
  reconnect();
}

void HomeassistantMqtt::sendConfigStatus(uint32_t serialno) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String topic = "homeassistant/sensor/prf_" + serial_str + "_status/config";

  DynamicJsonDocument doc(2048);
  
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  JsonArray connections = device.createNestedArray("connections");
  JsonArray connection = connections.createNestedArray();

  identifiers.add("prf_" + serial_str);

  connection.add("serial");
  connection.add(serial_str);

  device["name"] = "prf_" + serial_str;
  device["model"] = "PapayaRF";
  device["sw_version"] = "0.3(papayarf)";
  device["manufacturer"] = "Papayawhip";

  doc["name"] = "PRF " + serial_str + " status";
  doc["stat_t"] = "~HASS_STATE";
  doc["avty_t"] = "~LWT";
  doc["frc_upd"] = "true";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["json_attributes_topic"] = "~HASS_STATE";
  doc["unit_of_meas"] = " ";
  doc["ic"] = "mdi:information-outline";
  doc["uniq_id"] = "prf_" + serial_str + "_status";
  doc["~"] = "prf_" + serial_str + "/tele/";

  Serial.println(topic);

  char message[800];
  serializeJson(doc, message);

  Serial.println(message);
  mqtt.publish(topic.c_str(), message, true);
  Serial.println("nach publish");
}

void HomeassistantMqtt::sendOnlineStatus(uint32_t serialno, String status) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String topic = "prf_" + serial_str + "/tele/LWT";
  mqtt.publish(topic.c_str(), status.c_str(), true);
}

void HomeassistantMqtt::sendIOConfigStatus(uint32_t serialno, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "homeassistant/binary_sensor/prf_" + serial_str + "_io" + ioid + "/config";

  DynamicJsonDocument doc(2048);
  
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  JsonArray connections = device.createNestedArray("connections");
  JsonArray connection = connections.createNestedArray();

  identifiers.add("prf_" + serial_str);

  connection.add("serial");
  connection.add(serial_str);

  doc["name"] = "PRF " + serial_str + " IO_" + ioid;
  doc["stat_t"] = "~stat";
  doc["avty_t"] = "~avail";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["uniq_id"] = "prf_" + serial_str + "_io" + ioid + "_status";
  doc["~"] = "prf_" + serial_str + "_io" + ioid + "/";
  doc["pl_on"] = "ON";
  doc["off_delay"] = 120;

  Serial.println(topic);

  char message[800];
  serializeJson(doc, message);
  mqtt.publish(topic.c_str(), message, true);
}

void HomeassistantMqtt::sendTemperatureConfigStatus(uint32_t serialno, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "homeassistant/sensor/prf_" + serial_str + "_temperature" + ioid + "/config";

  DynamicJsonDocument doc(2048);
  
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  JsonArray connections = device.createNestedArray("connections");
  JsonArray connection = connections.createNestedArray();

  identifiers.add("prf_" + serial_str);

  connection.add("serial");
  connection.add(serial_str);

  doc["name"] = "PRF " + serial_str + " Temperature_" + ioid;
  doc["device_class"] = "temperature";
  doc["unit_of_measurement"] = "°C";
  doc["stat_t"] = "~stat";
  doc["avty_t"] = "~avail";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["uniq_id"] = "prf_" + serial_str + "_temperature" + ioid + "_status";
  doc["~"] = "prf_" + serial_str + "_temperature_io" + ioid + "/";
  doc["expire_after"] = 120;

  Serial.println(topic);

  char message[800];
  serializeJson(doc, message);
  Serial.println(message);
  mqtt.publish(topic.c_str(), message, true);
}

void HomeassistantMqtt::sendTemperatureOnline(uint32_t serialno, String online, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "prf_" + serial_str + "_temperature_io" + ioid + "/avail";
  mqtt.publish(topic.c_str(), online.c_str(), true);
}

void HomeassistantMqtt::sendTemperatureStatus(uint32_t serialno, uint16_t temp, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  float f_value = temp;
  f_value = f_value/10;
  char c_value[10];
  dtostrf(f_value, 7, 1, c_value);

  String topic = "prf_" + serial_str + "_temperature_io" + ioid + "/stat";
  Serial.println(topic);
  Serial.println(c_value);
  mqtt.publish(topic.c_str(), c_value, true);
}

void HomeassistantMqtt::sendHumidityConfigStatus(uint32_t serialno, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "homeassistant/sensor/prf_" + serial_str + "_humidity" + ioid + "/config";

  DynamicJsonDocument doc(2048);
  
  JsonObject device = doc.createNestedObject("device");
  JsonArray identifiers = device.createNestedArray("identifiers");
  JsonArray connections = device.createNestedArray("connections");
  JsonArray connection = connections.createNestedArray();

  identifiers.add("prf_" + serial_str);

  connection.add("serial");
  connection.add(serial_str);

  doc["name"] = "PRF " + serial_str + " Feuchtigkeit_" + ioid;
  doc["device_class"] = "humidity";
  doc["unit_of_measurement"] = "%";
  doc["stat_t"] = "~stat";
  doc["avty_t"] = "~avail";
  doc["pl_avail"] = "Online";
  doc["pl_not_avail"] = "Offline";
  doc["uniq_id"] = "prf_" + serial_str + "_humidity" + ioid + "_status";
  doc["~"] = "prf_" + serial_str + "_humidity_io" + ioid + "/";
  doc["expire_after"] = 120;

  Serial.println(topic);

  char message[800];
  serializeJson(doc, message);
  Serial.println(message);
  mqtt.publish(topic.c_str(), message, true);
}

void HomeassistantMqtt::sendHumidityOnline(uint32_t serialno, String online, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "prf_" + serial_str + "_humidity_io" + ioid + "/avail";
  mqtt.publish(topic.c_str(), online.c_str(), true);
}

void HomeassistantMqtt::sendHumidityStatus(uint32_t serialno, uint16_t humidity, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  float f_value = humidity;
  f_value = f_value/10;
  char c_value[10];
  dtostrf(f_value, 7, 1, c_value);

  String topic = "prf_" + serial_str + "_humidity_io" + ioid + "/stat";
  Serial.println(topic);
  Serial.println(c_value);
  mqtt.publish(topic.c_str(), c_value, true);
}

void HomeassistantMqtt::sendIOOnline(uint32_t serialno, String online, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "prf_" + serial_str + "_io" + ioid + "/avail";
  mqtt.publish(topic.c_str(), online.c_str(), true);
}


void HomeassistantMqtt::sendIOStatus(uint32_t serialno, String status, uint8_t ionum) {
  String serial_str = String(serialno, HEX);
  serial_str.toUpperCase();

  String ioid = String(ionum, HEX);
  ioid.toUpperCase();

  String topic = "prf_" + serial_str + "_io" + ioid + "/stat";
  Serial.println(topic);
  mqtt.publish(topic.c_str(), status.c_str(), true);
}

void HomeassistantMqtt::loop(void) {
  if (!mqtt.connected()) {
    reconnect();
  }

  mqtt.loop();
}
