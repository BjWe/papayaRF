
#include "deviceconfstore.h"

#include <debug.h>
#include <string.h>

#include "Arduino.h"
#include "LittleFS.h"

void DeviceConfStore::init(String _directory) {
  directory = _directory;

  if (!LittleFS.exists(directory)) {
#ifdef DEBUG
    Serial.println("Conf dir '" + directory + "' not exists. Creating ");
#endif
    LittleFS.mkdir(directory);
  }

  directory_obj = LittleFS.openDir(directory);
}

String DeviceConfStore::confFilename(uint32_t device_id) {
  return directory + "/" + String(device_id, HEX) + ".conf";
}

bool DeviceConfStore::exists(uint32_t device_id) {
  return LittleFS.exists(confFilename(device_id));
}

bool DeviceConfStore::fetch(uint32_t device_id, remote_file_info *info) {
  String filename = confFilename(device_id);
  if (LittleFS.exists(filename)) {
    File cfgFile = LittleFS.open(filename, "r+");
    cfgFile.read((byte *)info, sizeof(remote_file_info));
    cfgFile.close();
    return true;
  }
  return false;
}

void DeviceConfStore::store(uint32_t device_id, remote_file_info *info) {
  String filename = confFilename(device_id);
  File cfgFile = LittleFS.open(filename, "r+");
  cfgFile.seek(0, SeekSet);
  cfgFile.write((byte *)info, sizeof(remote_file_info));
  cfgFile.close();
}

void DeviceConfStore::prepareWalk(void) {
  directory_obj.rewind();
}

bool DeviceConfStore::next(remote_file_info *info) {
  if (directory_obj.next()) {
    if (directory_obj.fileSize()) {
      Serial.println("Walk next: " + directory_obj.fileName());

      File cfgFile = directory_obj.openFile("r");
      cfgFile.read((byte *)info, sizeof(remote_file_info));
      cfgFile.close();

      return true;
    }
  }
  return false;
}

/*
void DeviceConfStore::eachConf(conf_each_callback_t cb){
  Dir dir = LittleFS.openDir(directory);
  while (dir.next()) {
    if(dir.fileSize()) {

      File cfgFile = dir.openFile("r");
      remote_file_info info;
      cfgFile.read((byte*) &info, sizeof(remote_file_info));
      cfgFile.close();
      
      cb(&info);
    }
  }
}
*/
