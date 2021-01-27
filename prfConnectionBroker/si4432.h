
#ifndef rfm22b_h
#define rfm22b_h

//#include "../SPI/spi.h"
#if defined(ESP8266) || defined(ESP32)
#include "Arduino.h"
#endif

typedef void (*set_uc_register_callback_t)(uint8_t);

class SI4432 {
 public:
// Include enums from separate file
//	This is inside class definition so enums stay within class scope
#include "si4432_enums.h"

  set_uc_register_callback_t shutdown_cb;
  set_uc_register_callback_t chipselect_cb;

  SI4432(){};

  // Constructor requires SPI device path, passes this is SPI class
  SI4432(set_uc_register_callback_t _shutdown_cb, set_uc_register_callback_t _chipselect_cb) {
    shutdown_cb = _shutdown_cb;
    chipselect_cb = _chipselect_cb;

    chipselect_cb(0);
    shutdown(0);
  }

  void transfer(uint8_t *tx, uint8_t *rx, uint16_t length);
  void shutdown(uint8_t isShutdown);

  // Set or get the carrier frequency (in Hz);
  void setCarrierFrequency(uint32_t frequency);
  unsigned int getCarrierFrequency();

  // Set or get the frequency hopping step size (in Hz, but it is floored to nearest 10kHz)
  void setFrequencyHoppingStepSize(uint16_t step);
  uint16_t getFrequencyHoppingStepSize();

  // Set or get the frequency hopping channel
  void setChannel(uint8_t channel);
  uint8_t getChannel();

  // Set or get the frequency deviation (in Hz, but floored to the nearest 625Hz)
  void setFrequencyDeviation(uint32_t deviation);
  uint32_t getFrequencyDeviation();

  // Set or get the TX data rate (bps)
  // NOTE: This does NOT configure the receive data rate! To properly set
  // up the device for receiving, use the magic register values
  // calculated using the Si443x-Register-Settings_RevB1.xls Excel sheet.
  void setDataRate(uint32_t rate);
  uint32_t getDataRate();

  // Set or get the modulation type
  void setModulationType(SI4432_Modulation_Type modulation);
  SI4432_Modulation_Type getModulationType();

  // Set or get the modulation data source
  void setModulationDataSource(SI4432_Modulation_Data_Source source);
  SI4432_Modulation_Data_Source getModulationDataSource();

  // Set or get the data clock source
  void setDataClockConfiguration(SI4432_Data_Clock_Configuration clock);
  SI4432_Data_Clock_Configuration getDataClockConfiguration();

  // Set or get the transmission power
  void setTransmissionPower(uint8_t power);
  uint8_t getTransmissionPower();

  // Set or get the GPIO configuration
  void setGPIOFunction(SI4432_GPIO gpio, SI4432_GPIO_Function funct);
  // This should probably return enum, but this needs a lot of cases
  uint8_t getGPIOFunction(SI4432_GPIO gpio);

  // Enable or disable interrupts
  // No ability to get interrupt enable status as this would need a lot of case statements
  void setInterruptEnable(SI4432_Interrupt interrupt, bool enable);

  // Get the status of an interrupt
  bool getInterruptStatus(SI4432_Interrupt interrupt);

  // Set the operating mode
  //	This function does not toggle individual pins as with other functions
  //	It expects a bitwise-ORed combination of the modes you want set
  void setOperatingMode(uint16_t mode);

  // Get operating mode (bitwise-ORed)
  uint16_t getOperatingMode();

  // Manually enable RX or TX modes
  void enableRXMode();
  void enableTXMode();

  // Reset the device
  void reset();

  // Set or get the trasmit header
  void setTransmitHeader(uint32_t header);
  uint32_t getTransmitHeader();

  // Set or get the check header
  void setCheckHeader(uint32_t header);
  uint32_t getCheckHeader();

  // Set or get the CRC mode
  void setCRCMode(SI4432_CRC_Mode mode);
  SI4432_CRC_Mode getCRCMode();

  // Set or get the CRC polynominal
  void setCRCPolynomial(SI4432_CRC_Polynomial poly);
  SI4432_CRC_Polynomial getCRCPolynomial();

  // Get and set all the FIFO threshold
  void setTXFIFOAlmostFullThreshold(uint8_t thresh);
  void setTXFIFOAlmostEmptyThreshold(uint8_t thresh);
  void setRXFIFOAlmostFullThreshold(uint8_t thresh);
  uint8_t getTXFIFOAlmostFullThreshold();
  uint8_t getTXFIFOAlmostEmptyThreshold();
  uint8_t getRXFIFOAlmostFullThreshold();

  // Get RSSI value or input dBm
  uint8_t getRSSI();
  int8_t getInputPower();

  // Get length of last received packet
  uint8_t getReceivedPacketLength();

  // Set length of packet to be transmitted
  void setTransmitPacketLength(uint8_t length);

  // Clear the FIFOs
  void clearRXFIFO();
  void clearTXFIFO();

  // Send data
  //void send(uint8_t *data, int length);
  void send(void *ptr, int length);

  // Receive data (blocking with timeout). Returns number of bytes received
  uint8_t receive(uint8_t *data, uint8_t length, uint16_t timeout = 30000);

  // Helper functions for getting and getting individual registers
  uint8_t getRegister(uint8_t reg);
  uint16_t get16BitRegister(uint8_t reg);
  uint32_t get32BitRegister(uint8_t reg);
  void setRegister(uint8_t reg, uint8_t value);
  void set16BitRegister(uint8_t reg, uint16_t value);
  void set32BitRegister(uint8_t reg, uint32_t value);

  static const uint8_t MAX_PACKET_LENGTH = 64;

 private:
  void setFIFOThreshold(SI4432_Register reg, uint8_t thresh);
};
#endif