#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <SPI.h>
#include <string.h>

#include "Arduino.h"
#include "LittleFS.h"
#include "SI4432.h"
#include "crc.h"
#include "deviceconfstore.h"
#include "homeassistant.h"
#include "localsettings.h"
#include "remoteproto.h"
#include "xtea.h"
#include "packetidstore.h"

const char *ssid = STASSID;
const char *password = STAPSK;

#define SHUTDOWNPIN 4
#define CSPIN 15

// ThirdParty OOK Sender
#define SP1 0
#define CMD_PUMP 0b110111011
#define CMD_RELEASE 0b101110111
#define CMD_MODESEL 0b100110011
#define CMD_MODEOFF 0b101010101

// Speicher der Konfiguration
DeviceConfStore cfgStore;

// Webserver
ESP8266WebServer server(80);

// MQTT Homeassistant Helper
HomeassistantMqtt mqtt;

// RFTransceiver (SI4432)
SI4432 rf1;

// Geräteregistrierung erlauben
uint8_t allowRegister = 0;

// Paketverfolgung
PacketIdStore pidstore;

enum REMOTEPROTO_Itemtype {
  ITEMTYPE_BUTTON = 0x01,
  ITEMTYPE_ROTARY = 0x02,
  ITEMTYPE_TEMPHUMI = 0x03
};

// Für später (Log im Webserver anzeigen)
void debugLog(String msg) { Serial.println(msg); }

// Callbackmethode um den RFTransceiver an und abzuschalten
void rf1_shutdown(uint8_t isShutdown) {
  pinMode(SHUTDOWNPIN, OUTPUT);
  if (isShutdown) {
    digitalWrite(SHUTDOWNPIN, HIGH);
  } else {
    digitalWrite(SHUTDOWNPIN, LOW);
  }
}

// Callbackmethode um den RFTransceiver zu selektieren
void rf1_chipselect(uint8_t isSelected) {
  pinMode(CSPIN, OUTPUT);
  if (isSelected) {
    digitalWrite(CSPIN, LOW);
  } else {
    digitalWrite(CSPIN, HIGH);
  }
}

// Geräteliste Senden
void handleListDevicesConf() {
  String message = "<table border=1>";

  message +=
      "<tr><td>Serial</td><td>Name</td><td>Key</td><td>Nextcode</td></tr>";

  cfgStore.prepareWalk();
  remote_file_info info;
  while (cfgStore.next(&info)) {
    String keystr = "";
    for (uint8_t i = 0; i < 16; i++) {
      keystr += String(info.key[i], HEX) + " ";
    }
    message += "<tr><td>" + String(info.serial, HEX) + "</td><td>" +
               String(info.name) + "</td><td>" + keystr + "</td><td>" +
               String(info.nextcode, HEX) + "</td></tr>";
  }

  message += "</table><br /><span>Registration is " +
             String(allowRegister ? "enabled" : "disabled");

  server.send(200, "text/html", message);
}

void sendMqttMessageForAllDevices() {
  cfgStore.prepareWalk();
  remote_file_info info;
  while (cfgStore.next(&info)) {
    mqtt.sendConfigStatus(info.serial);
    mqtt.sendOnlineStatus(info.serial, "Online");
    for (uint8_t i = 0; i < 32; i++) {
      if (info.io[i][0] != 0xFF) {
        mqtt.sendIOConfigStatus(info.serial, info.io[i][0]);
        mqtt.sendIOOnline(info.serial, "Online", info.io[i][0]);
        mqtt.sendIOStatus(info.serial, "OFF", info.io[i][0]);
      }
    }
  }
}

// HTTP 404 default handler
void handleNotFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
}

// "Öffnet" den Funkkanal um die Fernbedienung abzufragen
void rfOpenRx() {
  rf1.shutdown(1);
  delay(50);
  rf1.shutdown(0);
  delay(100);

  rf1.setCarrierFrequency(433.80E6);
  rf1.setModulationType(SI4432::GFSK);
  rf1.setModulationDataSource(SI4432::FIFO);
  rf1.setDataClockConfiguration(SI4432::NONE);
  rf1.setTransmissionPower(9);
  // rf1.setGPIOFunction(SI4432::GPIO1, SI4432::RX_STATE);
  // rf1.setGPIOFunction(SI4432::GPIO2, SI4432::TX_STATE);
  rf1.setCheckHeader(0x11337799);

  Serial.println(rf1.getCarrierFrequency());
}

// "Öffnet" den Funkkanal um als OOK Sender zu fungieren
void rfOpenTxPl() {
  rf1.shutdown(1);
  delay(50);
  rf1.shutdown(0);
  delay(100);

  rf1.setCarrierFrequency(433.92E6);
  rf1.setModulationType(SI4432::OOK);
  rf1.setModulationDataSource(SI4432::DIRECT_GPIO);
  rf1.setDataClockConfiguration(SI4432::NONE);
  rf1.setDataRate(1024);
  rf1.setTransmissionPower(20);
  rf1.setGPIOFunction(SI4432::GPIO2, SI4432::DATA_INPUT);
}

// Sendet eine 9-Bit Nachricht via OOK
void sendOOKNineBits(uint16_t command, uint16_t len) {
  rfOpenTxPl();

  rf1.enableTXMode();
  for (uint16_t y = 0; y < len; y++) {
    for (int8_t bnum = 8; bnum >= 0; bnum--) {
      if ((command >> bnum & 1)) {
        delay(1);
        digitalWrite(SP1, HIGH);
        delay(2);
        digitalWrite(SP1, LOW);
      } else {
        delay(2);
        digitalWrite(SP1, HIGH);
        delay(1);
        digitalWrite(SP1, LOW);
      }
    }
    delay(10);
  }

  rfOpenRx();
}

// Init-Routine
void setup() {
  // UART initialisieren
  Serial.begin(115200);

  // Konfigurationsspeicher initialisieren
  if (!LittleFS.begin()) {
    Serial.println("cannot open LittleFS, formating");
    LittleFS.format();
  } else {
    Serial.println("LittleFS open succ");
  }

  cfgStore.init("/devices");

  // SPI Schnittstelle für den SI4432 Treiber vorberieten
  digitalWrite(SS, HIGH);
  SPI.begin();
  // Zur Sicherheit eine eher langsame Clock auswählen
  // (Kabellänge etc.)
  SPI.setClockDivider(SPI_CLOCK_DIV128);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  // ThirdParty RF DataPin auf Ausgang setzen
  pinMode(SP1, OUTPUT);
  digitalWrite(SP1, LOW);

  // WLAN Station setzen
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  debugLog("Set WLAN Info");

  // Auf Verbindung warten
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqtt.init(String(ESP.getChipId(), HEX).c_str(), mqtt_server);
  sendMqttMessageForAllDevices();

  server.on("/", []() { server.send(200, "text/plain", "hi"); });

  server.on("/sync", []() {
    sendMqttMessageForAllDevices();
    server.send(200, "text/plain", "registration is allowed");
  });

  server.on("/enableregister", []() {
    allowRegister = 1;
    server.send(200, "text/plain", "registration is allowed");
  });

  server.on("/disableregister", []() {
    allowRegister = 1;
    server.send(200, "text/plain", "registration is denied");
  });

  server.on("/format", []() {
    server.send(200, "text/plain", "format");
    LittleFS.format();
    ESP.restart();
  });
  
  server.on("/restart", []() {
    server.send(200, "text/plain", "restart");
    ESP.restart();
  });

  server.on("/p/p", []() {
    sendOOKNineBits(CMD_PUMP, 50);
    server.send(200, "text/plain", "ok");
  });

  server.on("/p/r", []() {
    sendOOKNineBits(CMD_RELEASE, 5);
    server.send(200, "text/plain", "ok");
  });

  server.on("/p/m", []() {
    sendOOKNineBits(CMD_MODESEL, 5);
    server.send(200, "text/plain", "ok");
  });

  server.on("/p/o", []() {
    sendOOKNineBits(CMD_MODEOFF, 5);
    server.send(200, "text/plain", "ok");
  });

  server.on("/list", handleListDevicesConf);

  server.onNotFound(handleNotFound);

  server.begin();

  rf1 = SI4432(&rf1_shutdown, &rf1_chipselect);
  rfOpenRx();
}

// Nextcode prüfen - Nur wenn der Abstand nicht zu groß ist
// und remote größer ist, ist der Nextcode ok
int8_t checkNextcode(uint32_t local, uint32_t remote, uint16_t allowed_gap) {
  uint32_t gap;

  // Overflow könnte aufgetreten sein
  if (local > remote) {
    gap = (0xFFFFFFFF - local) + remote;
  } else if (local < remote) {
    gap = remote - local;
  } else if (local == remote) {
    // ?? Replay?
    return 0;
  }

  if (gap > allowed_gap) {
    return -1;
  } else {
    return 1;
  }
}

void processBasicMsg(uint8_t (&rx)[SI4432::MAX_PACKET_LENGTH]) {
  rf_packet_basic_t msg;
  memcpy(&msg, &rx, sizeof(rf_packet_basic_t));
  Serial.print("Basic Packet - SN: ");
  Serial.println(msg.header.serial, HEX);

  remote_file_info info;
  if (cfgStore.fetch(msg.header.serial, &info)) {
    rf_packet_payload_basic_t decrypted_payload;
    xtea_dec(&decrypted_payload, &msg.payload, &info.key);

    Serial.print("Decrypted V1 Packet ");

    uint8_t *ptr = (uint8_t *)&decrypted_payload;
    for (uint8_t n = 0; n < sizeof(rf_packet_payload_basic_t); n++) {
      Serial.print("0x");
      Serial.print(*ptr++, HEX);
      Serial.print(" ");
    }
    Serial.println("");

    uint8_t crc = crc8maxim(&decrypted_payload,
                            sizeof(rf_packet_payload_basic_t) - 1);
    if (crc == decrypted_payload.crc) {
      Serial.println("CRC OK");
    } else {
      Serial.println("CRC missmatch - 0x" + String(crc, HEX) + " <> 0x" +
                     String(decrypted_payload.crc, HEX));
    }

    if (checkNextcode(info.nextcode, decrypted_payload.nextcode, 50) == 1) {
      Serial.println("Nextcode OK");

      info.nextcode = decrypted_payload.nextcode;
      int8_t btn_cfg_pos = -1;
      for (uint8_t i = 0; i < 32; i++) {
        if (info.io[i][0] == (uint8_t)decrypted_payload.data) {
          btn_cfg_pos = i;
          break;
        }
      }

      if (btn_cfg_pos < 0) {
        Serial.println("Button not existing right now");
        for (uint8_t i = 0; i < 32; i++) {
          if (info.io[i][0] == 0xFF) {
            Serial.println("IO Pos 0x" + String(i, HEX) +
                           " is free");
            info.io[i][0] = (uint8_t)decrypted_payload.data;
            info.io[i][1] = ITEMTYPE_BUTTON;
            // mqtt.sendIOConfigStatus(rfinfo.serial,
            // rfinfo.io[btn_cfg_pos][0]);
            mqtt.sendIOConfigStatus(info.serial, info.io[i][0]);
            mqtt.sendIOOnline(info.serial, "Online", info.io[i][0]);
            btn_cfg_pos = i;
            break;
          }
        }
      }

      switch (decrypted_payload.msgtype) {
        case BUTTON_DOWN: {
          mqtt.sendIOStatus(info.serial, "ON",
                            info.io[btn_cfg_pos][0]);
          break;
        }
        case BUTTON_UP: {
          mqtt.sendIOStatus(info.serial, "OFF",
                            info.io[btn_cfg_pos][0]);
          break;
        }
      }

      cfgStore.store(info.serial, &info);
    } else {
      Serial.println("Nextcode NOT! OK");
    }
  } else {
    Serial.println("Serial unknown");
  }
}

void processBasicV2Msg(uint8_t (&rx)[SI4432::MAX_PACKET_LENGTH]) {
  // Serial.println("Basic V2 Msg Received");
  rf_packet_basic_v2_t msg;
  memcpy(&msg, &rx, sizeof(rf_packet_basic_v2_t));

  if(pidstore.exists(msg.header.serial, msg.transmission.packetnum)){
    Serial.println("Drop BasicV2 retransmission");
    return;  
  }
  pidstore.add(msg.header.serial, msg.transmission.packetnum);
  
  Serial.print("BasicV2 Packet - SN: ");
  Serial.print(msg.header.serial, HEX);

  Serial.print(" - PacketNum: ");
  Serial.print(msg.transmission.packetnum, DEC);
  Serial.print(" - Retransmission: ");
  Serial.println(msg.transmission.retransmit, DEC);

  remote_file_info info;
  if (cfgStore.fetch(msg.header.serial, &info)) {
    rf_packet_payload_basic_v2_t decrypted_payload;

    //Eine "Art" von CBC herstellen
    // XTEA arbeitet auf 64-Bit (8 Byte) Blöcken
    // Byte 0 - 7
    xtea_dec(&decrypted_payload, &msg.payload, &info.key);
   
    // Byte 8 - 15
    xtea_dec(((uint8_t *) &decrypted_payload) + 8, (uint8_t *)(&msg.payload) + 8, &info.key);
       

    Serial.print("Decrypted Basic V2 Packet ");

    uint8_t *ptr = (uint8_t *)&decrypted_payload;
    for (uint8_t n = 0; n < sizeof(rf_packet_payload_basic_v2_t); n++) {
      Serial.print("0x");
      Serial.print(*ptr++, HEX);
      Serial.print(" ");
    }
    Serial.println("");

    Serial.print  ("Nextcode: ");
    Serial.println(decrypted_payload.nextcode, HEX);

    Serial.print  ("CRC:      ");
    Serial.println(decrypted_payload.crc, HEX);

    Serial.print  ("MSGType:  ");
    Serial.println(decrypted_payload.msgtype, HEX);

    Serial.print  ("ID:       ");
    Serial.println(decrypted_payload.id, HEX);
    
     uint8_t crc = crc8maxim(&decrypted_payload,
                            sizeof(rf_packet_payload_basic_v2_t) - 1);
    if (crc == decrypted_payload.crc) {
      Serial.println("CRC OK");
    } else {
      Serial.println("CRC missmatch - 0x" + String(crc, HEX) + " <> 0x" +
                     String(decrypted_payload.crc, HEX));
    }

    if (checkNextcode(info.nextcode, decrypted_payload.nextcode, 50) == 1) {
      Serial.println("Nextcode OK");

      switch (decrypted_payload.msgtype) {
        
        case TEMPHUMI_VALUE: {
          Serial.print("Temp / Humi: ");
          uint16_t msg[4];
          memcpy(&msg, decrypted_payload.data, sizeof(msg));
          uint16_t temp = msg[1];
          uint16_t humi = msg[0];

          Serial.print(temp);
          Serial.print(" / ");
          Serial.println(humi);
        }
      }

      info.nextcode = decrypted_payload.nextcode;
      

      cfgStore.store(info.serial, &info);
    } else {
      Serial.println("Nextcode NOT! OK");
    }
  } else {
    Serial.println("Serial unknown");
  }
}

void processRekeyMsg(uint8_t (&rx)[SI4432::MAX_PACKET_LENGTH]) {
  if (allowRegister) {
    Serial.println("Registration is allowed");
    rf_packet_rekey_t msg;
    memcpy(&msg, &rx, sizeof(rf_packet_rekey_t));
    Serial.print("Rekey Packet - SN: ");
    Serial.println(msg.header.serial, HEX);

    String filename =
        "/devices/" + String(msg.header.serial, HEX) + ".conf";

    remote_file_info rfinfo;

    if (LittleFS.exists(filename)) {
      Serial.println("Serial is known");
      File cfgFile = LittleFS.open(filename, "r+");

      cfgFile.read((byte *)&rfinfo, sizeof(remote_file_info));
      memcpy(&rfinfo.key, &msg.payload.data, sizeof(rfinfo.key));
      rfinfo.nextcode = msg.payload.nextcode;

      cfgFile.seek(0, SeekSet);
      cfgFile.write((byte *)&rfinfo, sizeof(remote_file_info));
      cfgFile.close();
    } else {
      Serial.println("Serial not exists in conf");

      memcpy(&rfinfo.key, &msg.payload.data, sizeof(rfinfo.key));
      rfinfo.nextcode = msg.payload.nextcode;
      rfinfo.serial = msg.header.serial;
      String(msg.header.serial, HEX)
          .toCharArray(rfinfo.name, sizeof(rfinfo.name));
      memset(&rfinfo.io, 0xFF, sizeof(rfinfo.io));

      File cfgFile = LittleFS.open(filename, "w");
      cfgFile.write((byte *)&rfinfo, sizeof(remote_file_info));
      cfgFile.close();
    }
  } else {
    Serial.println("Registration is disabled");
  }
}

void processRf() {
  uint8_t rx[SI4432::MAX_PACKET_LENGTH];
  uint8_t rlen = rf1.receive((uint8_t *)rx, SI4432::MAX_PACKET_LENGTH, 100);

  if (rlen > 0) {
    Serial.print(rlen);
    for (uint8_t n = 0; n < rlen; n++) {
      Serial.print("0x");
      Serial.print(rx[n], HEX);
      Serial.print(" ");
    }
    Serial.println("");

    if ((rx[0] == 0xFD) && (rx[1] == 0xFD)) {
      if (rx[7] == BASIC) {
        processBasicMsg(rx);
      } else if (rx[7] == BASICV2) {
        processBasicV2Msg(rx);
      } else if (rx[7] == REKEY) {
        processRekeyMsg(rx);
      }
    }
  }
}

void loop() {
  mqtt.loop();
  server.handleClient();

  processRf();
}
