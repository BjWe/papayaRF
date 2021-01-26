
#ifndef HOMEASSISTANT_H_
#define HOMEASSISTANT_H_

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class HomeassistantMqtt {
  public:
    HomeassistantMqtt();

    void init(const char* _device_id, const char* _mqtt_server);
    void sendConfigStatus(uint32_t serialno);
    void sendIOConfigStatus(uint32_t serialno, uint8_t ionum);

    void sendOnlineStatus(uint32_t serialno, String status);
    
    void sendIOOnline(uint32_t serialno, String online, uint8_t ionum);
    void sendIOStatus(uint32_t serialno, String status, uint8_t ionum);

    void loop(void);

  protected:
    WiFiClient espClient;
    PubSubClient mqtt;
    const char* mqtt_server;
    String device_id;

    void reconnect();
};

#endif
