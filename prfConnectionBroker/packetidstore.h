                                                                                                     
#ifndef PACKETIDSTORE_H_
#define PACKETIDSTORE_H_

#include "Arduino.h"

#define MAX_SLOTS   20

typedef struct {
  uint32_t serial;
  uint16_t packetnum;
} rf_packet_list_entry_t;


class PacketIdStore {
 public:
  PacketIdStore();
  void add(uint32_t serial, uint16_t packetnum);
  uint8_t exists(uint32_t serial, uint16_t packetnum);

 protected:
   uint8_t nextSlot;
   rf_packet_list_entry_t items[MAX_SLOTS];
};

#endif
