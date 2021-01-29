/*
 * remoteproto.cpp
 *
 * Created: 26.08.2020 13:39:48
 *  Author: Bjoern
 */ 

#include <avr/io.h>
#include <avr/eeprom.h>
#include <string.h>
#include "Global.h"
#include "remoteproto.h"

#include "ADC/ADC.h"
#include "XTEA/xtea.h"
#include "CRC/crc.h"
#include "UART/UART.h"


void RemoteProto::init(uint32_t _serial_nuber, uint16_t _eeprom_pos_cryptokey, uint16_t _eeprom_pos_nextcode, uint32_t _base_freq, set_uc_register_callback_t _shutdown_cb, set_uc_register_callback_t _chipselect_cb){
	serial_number = _serial_nuber;
	eeprom_pos_cryptokey = _eeprom_pos_cryptokey;
	eeprom_pos_nextcode = _eeprom_pos_nextcode;
	
	base_freq = _base_freq;
   
	
	shutdown_cb = _shutdown_cb;
	chipselect_cb =_chipselect_cb;
	
	for(uint8_t i = 0; i < REMOTEPROTO_CRYPTOKEY_SIZE; i++){
		cryptokey[i] = eeprom_read_byte((uint8_t *)(eeprom_pos_cryptokey+i));
	}
	
	rf = SI4432(shutdown_cb, chipselect_cb);
	rf.shutdown(1);
	
}

void RemoteProto::openRf(uint8_t power){
	rf.shutdown(0);
	_delay_ms(50);
	
	//uart_put_dw(base_freq);
	
	rf.setCarrierFrequency(base_freq);
	rf.setModulationType(SI4432::GFSK);
	rf.setModulationDataSource(SI4432::FIFO);
	rf.setDataClockConfiguration(SI4432::NONE);
	rf.setTransmissionPower(power);
	rf.setGPIOFunction(SI4432::GPIO0, SI4432::TX_STATE);
	rf.setGPIOFunction(SI4432::GPIO1, SI4432::TX_STATE);
	rf.setGPIOFunction(SI4432::GPIO2, SI4432::TX_STATE);
	rf.setTransmitHeader(0x11337799);
	
	//uart_put_dw(rf.getCarrierFrequency());
}

void RemoteProto::closeRf(void){
	rf.shutdown(1);
}

void RemoteProto::reinitCryptoKeyAndNextcode(uint8_t *newkey, uint32_t *nextcode) {
		
	for(uint8_t i = 0; i < REMOTEPROTO_CRYPTOKEY_SIZE; i++){
		uint8_t buff = random_adc_seed8(ADC_SEED_CHANNEL);
		*(newkey + i) = buff;
		cryptokey[i] = buff;
		eeprom_update_byte((uint8_t *)(eeprom_pos_cryptokey+i), buff);
	}
	
	*nextcode = random_adc_seed32(ADC_SEED_CHANNEL);
	eeprom_update_dword((uint32_t *)(eeprom_pos_nextcode), *nextcode);
}

uint32_t RemoteProto::getNextcode(void){
	return eeprom_read_dword((uint32_t *)(eeprom_pos_nextcode));
}

void RemoteProto::incrementNextcode(void){
	
	uint32_t nxc = getNextcode();
	#ifdef DEBUG
	uart_puts("update nxc: ");
	uart_print_uint32(nxc + 1, 10);
	uart_puts(" to 0x");
	uart_print_uint32(eeprom_pos_nextcode, 16);
	uart_puts("\r\n");
	#endif
	eeprom_update_dword((uint32_t *)eeprom_pos_nextcode, nxc + 1);
	
	uart_puts("Update OK\r\n");
}

rf_packet_basic_t RemoteProto::generateBasicMessage(REMOTEPROTO_Messagetype msgtype, uint16_t data){
	rf_packet_basic_t msg;
	msg.header.statichead = REMOTEPROTO_STATICHEAD;
	msg.header.serial = serial_number;
	msg.header.packettype = BASIC;
	msg.header.length = sizeof(rf_packet_payload_basic_t) + 1;
	
	msg.payload.msgtype = msgtype;
	msg.payload.nextcode = getNextcode();

	msg.payload.data = data; 
	
	return msg;
}

rf_packet_basic_v2_t RemoteProto::generateBasicV2Message(REMOTEPROTO_Messagetype msgtype, uint16_t id, uint8_t *data){
	rf_packet_basic_v2_t msg;
	msg.header.statichead = REMOTEPROTO_STATICHEAD;
	msg.header.serial = serial_number;
	msg.header.packettype = BASICV2;
	msg.header.length = sizeof(rf_packet_payload_basic_v2_t) + 1;
	
	msg.payload.msgtype = msgtype;
	msg.payload.id = id;
	msg.payload.nextcode = getNextcode();

 	memcpy(&msg.payload.data, data, sizeof(msg.payload.data));
	
	return msg;
}

rf_packet_rekey_t RemoteProto::generateRekeyMessage(uint8_t key[REMOTEPROTO_CRYPTOKEY_SIZE], uint32_t nextcode){
	rf_packet_rekey_t msg;
	msg.header.statichead = REMOTEPROTO_STATICHEAD;
	msg.header.serial = serial_number;
	msg.header.packettype = REKEY;
	msg.header.length = sizeof(rf_packet_payload_rekey_t) + 1;
	
	msg.payload.nextcode = nextcode;
	for(uint8_t i = 0; i < REMOTEPROTO_CRYPTOKEY_SIZE; i++){
		msg.payload.data[i] = key[i];
	}
	
	return msg;
}

void RemoteProto::generateCRCForBasicPayload(rf_packet_payload_basic_t *payload){
	payload->crc = crc8maxim(payload + 1, sizeof(rf_packet_payload_basic_t) - 1);
}

void RemoteProto::generateCRCForBasicV2Payload(rf_packet_payload_basic_v2_t *payload){
	payload->crc = crc8maxim(payload + 1, sizeof(rf_packet_payload_basic_v2_t) - 1);
}

void RemoteProto::generateCRCForRekeyPayload(rf_packet_payload_rekey_t *payload){
	payload->crc = crc8maxim(payload + 1, sizeof(rf_packet_payload_rekey_t) - 1);
}

void RemoteProto::encryptPayload(rf_packet_payload_basic_t *payload){
	rf_packet_payload_basic_t plainpayload;
	memcpy(&plainpayload, payload, sizeof(rf_packet_payload_basic_t));
	
	xtea_enc(payload, &plainpayload, &cryptokey);
}

void RemoteProto::encryptPayloadV2(rf_packet_payload_basic_v2_t *payload){
	rf_packet_payload_basic_v2_t plainpayload;
	memcpy(&plainpayload, payload, sizeof(rf_packet_payload_basic_v2_t));
	
	// XTEA arbeitet auf 64-Bit (8 Byte) Blöcken
	// Byte 0 - 7
	xtea_enc(payload, &plainpayload, &cryptokey);
	/*
	//Eine "Art" von CBC herstellen
	uint8_t cbckey[REMOTEPROTO_CRYPTOKEY_SIZE];
	memcpy(&cbckey, &cryptokey, REMOTEPROTO_CRYPTOKEY_SIZE);
	
	// Jedes "Gerade" Byte vom Schlüssel durch die Nachricht aus Block eins ersetzen
	uint8_t *data = (uint8_t *) payload;
	for(uint8_t num = 0; num <= 7; num++){
		cbckey[num * 2] = *data++;
	}
	*/
	// Byte 8 - 15
	xtea_enc(((uint8_t *) payload) + 8, ((uint8_t *) &plainpayload) + 8, &cryptokey);
}

void dump_rf_packet_basic_t(rf_packet_basic_t *d){
	uint8_t *ptr = (uint8_t *)d;
	for (uint8_t i = 0; i < sizeof(rf_packet_basic_t); i++){
		uart_print_uint8(*(ptr+i), 16);	
		uart_putc(' ');
	}
	uart_puts("\r\n");
}

void dump_rf_packet_basic_v2_t(rf_packet_basic_v2_t *d){
	uint8_t *ptr = (uint8_t *)d;
	for (uint8_t i = 0; i < sizeof(rf_packet_basic_v2_t); i++){
		uart_print_uint8(*(ptr+i), 16);	
		uart_putc(' ');
	}
	uart_puts("\r\n");
}

void dump_rf_packet_rekey_t(rf_packet_rekey_t *d){
	uint8_t *ptr = (uint8_t *)d;
	for (uint8_t i = 0; i < sizeof(rf_packet_rekey_t); i++){
		uart_print_uint8(*(ptr+i), 16);	
		uart_putc(' ');
	}
	uart_puts("\r\n");
}

void RemoteProto::sendBasicMessage(REMOTEPROTO_Messagetype msgtype, uint16_t data){

	rf_packet_basic_t msg = generateBasicMessage(msgtype, data);
	//dump_rf_packet_basic_t(&msg);
	generateCRCForBasicPayload(&msg.payload);
	//dump_rf_packet_basic_t(&msg);
	encryptPayload(&msg.payload);
	//dump_rf_packet_basic_t(&msg);
	
	rf.send(&msg, sizeof(msg));
	
	incrementNextcode();	
}

void RemoteProto::sendBasicV2Message(REMOTEPROTO_Messagetype msgtype, uint8_t retries, uint16_t msg_num, uint16_t id, uint8_t *data){

    rf_packet_basic_v2_t msg = generateBasicV2Message(msgtype, id, data);
	//dump_rf_packet_basic_v2_t(&msg);
    generateCRCForBasicV2Payload(&msg.payload);
	dump_rf_packet_basic_v2_t(&msg);
	encryptPayloadV2(&msg.payload);
    
    msg.transmission.packetnum = msg_num;

    for(uint8_t retry = 0; retry < retries; retry++){
      msg.transmission.retransmit = retry;
      
	  rf.send(&msg, sizeof(msg));
	  
	  // Warten
	  _delay_ms(100);
    }
	
	incrementNextcode();	
}

void RemoteProto::sendRekeyMessage(uint8_t key[16], uint32_t nextcode){
	rf_packet_rekey_t msg = generateRekeyMessage(key, nextcode);
	generateCRCForRekeyPayload(&msg.payload);
	rf.send(&msg, sizeof(msg));
	_delay_ms(2);
	//dump_rf_packet_rekey_t(&msg);
	//_delay_ms(200);
}