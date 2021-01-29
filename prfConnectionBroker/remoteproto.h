/*
 * remoteproto.h
 *
 * Created: 25.08.2020 13:54:55
 *  Author: Bjoern
 */

#ifndef REMOTEPROTO_H_
#define REMOTEPROTO_H_

#define REMOTEPROTO_STATICHEAD 0xFDFD
#define REMOTEPROTO_CRYPTOKEY_SIZE 16

enum REMOTEPROTO_Packettype {
  BASIC = 0x01,
  BASICV2 = 0x02,
  REKEY = 0x10
};

enum REMOTEPROTO_Messagetype {
  BUTTON_PRESSED = 0x01,
  BUTTON_UP = 0x02,
  BUTTON_DOWN = 0x03,
  ROTARY_LEFT = 0x11,
  ROTARY_RIGHT = 0x12,
  TEMPHUMI_VALUE = 0x21
};

typedef struct __attribute__((packed)) {
  uint8_t crc;
  uint32_t nextcode;
  uint8_t msgtype;
  uint16_t data;
} rf_packet_payload_basic_t;

typedef struct __attribute__((packed)) {
  uint8_t   crc;
  uint32_t  nextcode;
  uint8_t   msgtype;
  uint16_t  id;
  uint8_t   data[8];
} rf_packet_payload_basic_v2_t;

typedef struct __attribute__((packed)) {
  uint8_t crc;
  uint32_t nextcode;
  uint8_t data[16];
} rf_packet_payload_rekey_t;

typedef struct __attribute__((packed)) {
  uint16_t statichead;
  uint32_t serial;
  uint8_t length;
  uint8_t packettype;
} rf_packet_header_t;

typedef struct __attribute__((packed)) {
  uint16_t packetnum;
  uint8_t flags;
  uint8_t retransmit;
} rf_packet_transmission_info_t;

typedef struct __attribute__((packed)) {
  rf_packet_header_t header;
  rf_packet_payload_basic_t payload;
} rf_packet_basic_t;

typedef struct __attribute__((packed)) {
  rf_packet_header_t header;
  rf_packet_transmission_info_t transmission;
  rf_packet_payload_basic_v2_t payload;
} rf_packet_basic_v2_t;

typedef struct __attribute__((packed)) {
  rf_packet_header_t header;
  rf_packet_payload_rekey_t payload;
} rf_packet_rekey_t;

#endif /* REMOTEPROTO_H_ */
