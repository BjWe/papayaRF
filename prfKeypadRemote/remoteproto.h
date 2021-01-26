/*
 * remoteproto.h
 *
 * Created: 25.08.2020 13:54:55
 *  Author: Bjoern
 */ 


#ifndef REMOTEPROTO_H_
#define REMOTEPROTO_H_

#include "SI4432/si4432.h"

#define REMOTEPROTO_STATICHEAD 0xFDFD
#define REMOTEPROTO_CRYPTOKEY_SIZE  16

enum REMOTEPROTO_Packettype {
	BASIC = 0x01,
	REKEY = 0x10
};

enum REMOTEPROTO_Messagetype {
	BUTTON_PRESSED = 0x01,
	BUTTON_UP      = 0x02,
	BUTTON_DOWN    = 0x03,
	ROTARY_LEFT    = 0x11,
	ROTARY_RIGHT   = 0x12
};


typedef struct __attribute__((packed)) {
	uint8_t   crc;
	uint32_t  nextcode;
	uint8_t   msgtype;
	uint16_t  data;
} rf_packet_payload_basic_t;

typedef struct __attribute__((packed)) {
	uint8_t  crc;
	uint32_t nextcode;
	uint8_t  data[16];
} rf_packet_payload_rekey_t;

typedef struct __attribute__((packed)) {
	uint16_t statichead;
	uint32_t serial;
	uint8_t  length;
	uint8_t  packettype;
} rf_packet_header_t;

typedef struct __attribute__((packed)) {
	rf_packet_header_t   header;
	rf_packet_payload_basic_t  payload;
} rf_packet_basic_t;

typedef struct __attribute__((packed)) {
	rf_packet_header_t   header;
	rf_packet_payload_rekey_t  payload;
} rf_packet_rekey_t;


class RemoteProto {
	
	public:
	uint32_t serial_number;
	uint16_t eeprom_pos_cryptokey;
	uint16_t eeprom_pos_nextcode;
	
	uint32_t base_freq;
	
	set_uc_register_callback_t shutdown_cb;
	set_uc_register_callback_t chipselect_cb;
	
	void init(uint32_t _serial_nuber, uint16_t _eeprom_pos_cryptokey, uint16_t eeprom_pos_nextcode, uint32_t _base_freq, set_uc_register_callback_t _shutdown_cb, set_uc_register_callback_t _chipselect_cb);
	void openRf(uint8_t power);
	void closeRf(void);
	
	void reinitCryptoKeyAndNextcode(uint8_t *newkey, uint32_t *nextcode);
	
	uint32_t getNextcode(void);
	void incrementNextcode(void);
	
	rf_packet_basic_t generateBasicMessage(REMOTEPROTO_Messagetype msgtype, uint16_t data);
	rf_packet_rekey_t generateRekeyMessage(uint8_t key[16], uint32_t nextcode);
	void generateCRCForBasicPayload(rf_packet_payload_basic_t *payload);
	void generateCRCForRekeyPayload(rf_packet_payload_rekey_t *payload);
	void encryptPayload(rf_packet_payload_basic_t *payload);
	void sendBasicMessage(REMOTEPROTO_Messagetype msgtype, uint16_t data);
	void sendRekeyMessage(uint8_t key[16], uint32_t nextcode);
	
	private:
	SI4432 rf;
	uint8_t cryptokey[REMOTEPROTO_CRYPTOKEY_SIZE];
	
	void transfer();
	
};


#endif /* REMOTEPROTO_H_ */