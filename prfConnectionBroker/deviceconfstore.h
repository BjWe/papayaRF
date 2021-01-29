
#ifndef DEVICECONFSTORE_H_
#define DEVICECONFSTORE_H_

#include "Arduino.h"
#include "LittleFS.h"

#define DEVICE_KEY_SIZE  16

typedef struct __attribute__((packed)) {
  uint32_t serial;
  uint8_t key[DEVICE_KEY_SIZE];
  uint32_t nextcode;
  char name[32];
  uint8_t io[32][2];
} remote_file_info;

typedef void (*conf_each_callback_t)(remote_file_info *);

class DeviceConfStore {
 public:
  String directory;
  Dir directory_obj;

  void init(String _directory);

  String confFilename(uint32_t device_id);
  bool exists(uint32_t device_id);
  bool fetch(uint32_t device_id, remote_file_info *info);
  void store(uint32_t device_id, remote_file_info *info);

  void prepareWalk(void);
  bool next(remote_file_info *info);
};

#endif
