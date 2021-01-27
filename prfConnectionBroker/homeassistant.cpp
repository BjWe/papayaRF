

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

  const size_t bufferSize = JSON_OBJECT_SIZE(18) + JSON_ARRAY_SIZE(2);
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.createObject();
  JsonObject& device = jsonBuffer.createObject();
  JsonArray& identifiers = jsonBuffer.createArray();
  JsonArray& connections = jsonBuffer.createArray();
  JsonArray& connection = jsonBuffer.createArray();

  identifiers.add("prf_" + serial_str);

  //connection.add("mac");
  //connection.add(WiFi.macAddress());
  connection.add("serial");
  connection.add(serial_str);
  connections.add(connection);

  device["identifiers"] = identifiers;
  device["connections"] = connections;
  device["name"] = "prf_" + serial_str;
  device["model"] = "PapayaRF";
  device["sw_version"] = "0.2(papayarf)";
  device["manufacturer"] = "Papayawhip";

  root["name"] = "PRF " + serial_str + " status";
  root["stat_t"] = "~HASS_STATE";
  root["avty_t"] = "~LWT";
  root["frc_upd"] = "true";
  root["pl_avail"] = "Online";
  root["pl_not_avail"] = "Offline";
  root["json_attributes_topic"] = "~HASS_STATE";
  root["unit_of_meas"] = " ";
  //root["val_tpl"] = "{{value_json['RSSI']}}";
  root["ic"] = "mdi:information-outline";
  root["uniq_id"] = "prf_" + serial_str + "_status";
  root["device"] = device;
  root["~"] = "prf_" + serial_str + "/tele/";

  Serial.println(topic);

  char message[800];
  root.printTo(message, sizeof(message));
  Serial.println("vor publish");
  Serial.println(mqtt.state());
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

  const size_t bufferSize = JSON_OBJECT_SIZE(18) + JSON_ARRAY_SIZE(2);
  DynamicJsonBuffer jsonBuffer(bufferSize);
  JsonObject& root = jsonBuffer.createObject();

  JsonObject& device = jsonBuffer.createObject();
  JsonArray& identifiers = jsonBuffer.createArray();
  JsonArray& connections = jsonBuffer.createArray();
  JsonArray& connection = jsonBuffer.createArray();

  identifiers.add("prf_" + serial_str);

  //connection.add("mac");
  //connection.add(WiFi.macAddress());
  connection.add("serial");
  connection.add(serial_str);
  connections.add(connection);

  device["identifiers"] = identifiers;
  device["connections"] = connections;

  root["name"] = "PRF " + serial_str + " IO_" + ioid;
  root["stat_t"] = "~stat";
  root["avty_t"] = "~avail";
  root["pl_avail"] = "Online";
  root["pl_not_avail"] = "Offline";
  root["uniq_id"] = "prf_" + serial_str + "_io" + ioid + "_status";
  root["device"] = device;
  root["~"] = "prf_" + serial_str + "_io" + ioid + "/";
  //root["value_template"] = "{{value_json.STATE}}";*/
  root["pl_on"] = "ON";
  root["off_delay"] = 120;

  Serial.println(topic);

  char message[800];
  root.printTo(message, sizeof(message));
  mqtt.publish(topic.c_str(), message, true);
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
