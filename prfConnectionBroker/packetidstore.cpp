
#include "Arduino.h"
#include "packetidstore.h"
                       
PacketIdStore::PacketIdStore() {
  nextSlot = 0;
   
}                    
                       
void PacketIdStore::add(uint32_t serial, uint16_t packetnum){
  items[nextSlot].serial = serial;
  items[nextSlot].packetnum = packetnum;
  
  nextSlot++;
  if(nextSlot >= MAX_SLOTS){
    nextSlot = 0;
  }
}

uint8_t PacketIdStore::exists(uint32_t serial, uint16_t packetnum){
  for(uint8_t i = 0; i < MAX_SLOTS; i++){
    if(items[i].serial == serial && items[i].packetnum == packetnum){
      return 1;
    }
  }
  return 0;
}
