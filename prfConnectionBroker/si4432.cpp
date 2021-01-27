

#if defined(ESP8266) || defined(ESP32)
#define ARDUINO
#endif

#ifndef ARDUINO
#include "si4432.h"

#include <avr/io.h>

#include "../SPI/SPI.h"
#include "../UART/UART.h"
#else
#include <SPI.h>

#include "Arduino.h"
#include "si4432.h"
#endif

void SI4432::transfer(uint8_t *tx, uint8_t *rx, uint16_t length) {
  chipselect_cb(1);

  for (uint16_t i = 0; i < length; i++) {
#ifndef ARDUINO
    rx[i] = spi_transceive8(tx[i]);
#else
    rx[i] = SPI.transfer(tx[i]);
#endif
  }

  chipselect_cb(0);
}

void SI4432::shutdown(uint8_t isShutdown) {
  shutdown_cb(isShutdown);
}

// Set the frequency of the carrier wave
//	This function calculates the values of the registers 0x75-0x77 to achieve the
//	desired carrier wave frequency (without any hopping set)
//	Frequency should be passed in integer Hertz
void SI4432::setCarrierFrequency(uint32_t frequency) {
  // Don't set a frequency outside the range specified in the datasheet
  if (frequency < 240E6 || frequency > 960E6) {
    //printf("Cannot set carrier frequency to %fMHz, out of range!",frequency/1E6f);
    return;
  }

  // The following determines the register values, see Section 3.5.1 of the datasheet

  // Are we in the 'High Band'? (i.e. is hbsel == 1)
  uint8_t hbsel = (frequency >= 480E6);

  // What is the integer part of the frequency
  uint8_t fb = frequency / 10E6 / (hbsel + 1) - 24;

  // Calculate register 0x75 from hbsel and fb. sbsel (bit 6) is always set
  uint8_t fbs = (1 << 6) | (hbsel << 5) | fb;

  // Calculate the fractional part of the frequency
  uint16_t fc = (frequency / (10E6f * (hbsel + 1)) - fb - 24) * 64000;

  // Split the fractional part in to most and least significant bits
  // (Registers 0x76 and 0x77 respectively)
  uint8_t ncf1 = (fc >> 8);
  uint8_t ncf0 = fc & 0xff;

  // Write the registers to the device
  this->setRegister(FREQUENCY_BAND_SELECT, fbs);
  this->setRegister(NOMINAL_CARRIER_FREQUENCY_1, ncf1);
  this->setRegister(NOMINAL_CARRIER_FREQUENCY_0, ncf0);
}

// Get the frequency of the carrier wave in integer Hertz
//	Without any frequency hopping
unsigned int SI4432::getCarrierFrequency() {
  // Read the register values
  uint8_t fbs = this->getRegister(FREQUENCY_BAND_SELECT);
  uint8_t ncf1 = this->getRegister(NOMINAL_CARRIER_FREQUENCY_1);
  uint8_t ncf0 = this->getRegister(NOMINAL_CARRIER_FREQUENCY_0);

  // The following calculations ceom from Section 3.5.1 of the datasheet

  // Determine the integer part
  uint8_t fb = fbs & 0x1F;

  // Are we in the 'High Band'?
  uint8_t hbsel = (fbs >> 5) & 1;

  // Determine the fractional part
  uint16_t fc = (ncf1 << 8) | ncf0;

  // Return the frequency
  return 10E6 * (hbsel + 1) * (fb + 24 + fc / 64000.0);
}

// Get and set the frequency hopping step size
//	Values are in Hertz (to stay SI) but floored to the nearest 10kHz
void SI4432::setFrequencyHoppingStepSize(uint16_t step) {
  if (step > 255) {
    step = 255;
  }
  this->setRegister(FREQUENCY_HOPPING_STEP_SIZE, step);
}

uint16_t SI4432::getFrequencyHoppingStepSize() {
  return this->getRegister(FREQUENCY_HOPPING_STEP_SIZE);
}

// Get and set the frequency hopping channel
void SI4432::setChannel(uint8_t channel) {
  this->setRegister(FREQUENCY_HOPPING_CHANNEL_SELECT, channel);
}
uint8_t SI4432::getChannel() {
  return this->getRegister(FREQUENCY_HOPPING_CHANNEL_SELECT);
}

// Set or get the frequency deviation (in Hz, but floored to the nearest 625Hz)
void SI4432::setFrequencyDeviation(uint32_t deviation) {
  if (deviation > 320000) {
    deviation = 320000;
  }
  this->setRegister(FREQUENCY_DEVIATION, deviation / 625);
}
uint32_t SI4432::getFrequencyDeviation() {
  return this->getRegister(FREQUENCY_DEVIATION) * 625;
}

// Set or get the data rate (bps)
void SI4432::setDataRate(uint32_t rate) {
  // Get the Modulation Mode Control 1 register (for scaling bit)
  uint8_t mmc1 = this->getRegister(MODULATION_MODE_CONTROL_1);

  uint32_t txdr;
  // Set the scale bit (5th bit of 0x70) high if data rate is below 30kbps
  // and calculate the value for txdr registers (0x6E and 0x6F)
  if (rate < 30000) {
    mmc1 |= (1 << 5);
    txdr = rate * (((uint32_t)1 << (16 + 5)) / 1E6);
  } else {
    mmc1 &= ~(1 << 5);
    txdr = rate * (((uint32_t)1 << (16)) / 1E6);
  }

  // Set the data rate bytes
  this->set16BitRegister(TX_DATA_RATE_1, txdr);

  // Set the scaling byte
  this->setRegister(MODULATION_MODE_CONTROL_1, mmc1);
}
uint32_t SI4432::getDataRate() {
  // Get the data rate scaling value (5th bit of 0x70)
  uint8_t txdtrtscale = (this->getRegister(MODULATION_MODE_CONTROL_1) >> 5) & 1;

  // Get the data rate registers
  uint16_t txdr = this->get16BitRegister(TX_DATA_RATE_1);
  //uart_put_w(txdr);

  // Return the data rate (in bps, hence extra 1E3)
  /*uint32_t d1 = txdr * 1E6;
	uart_put_dw(d1);
	uint32_t d2 =  (uint32_t) 1 << (16 + (5 * txdtrtscale));
	uart_put_dw(d2);
	return d1 / d2;
	*/
  return ((uint32_t)txdr * 1E6) / ((uint32_t)1 << (16 + 5 * txdtrtscale));
}

// Set or get the modulation type
void SI4432::setModulationType(SI4432_Modulation_Type modulation) {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Clear the modtyp bits
  mmc2 &= ~0x03;

  // Set the desired modulation
  mmc2 |= modulation;

  // Set the register
  this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
SI4432::SI4432_Modulation_Type SI4432::getModulationType() {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Determine modtyp bits
  uint8_t modtyp = mmc2 & 0x03;

  // Ugly way to return correct enum
  switch (modtyp) {
    case 1:
      return OOK;
    case 2:
      return FSK;
    case 3:
      return GFSK;
    case 0:
    default:
      return UNMODULATED_CARRIER;
  }
}

void SI4432::setModulationDataSource(SI4432_Modulation_Data_Source source) {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Clear the dtmod bits
  mmc2 &= ~(0x03 << 4);

  // Set the desired data source
  mmc2 |= source << 4;

  // Set the register
  this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
SI4432::SI4432_Modulation_Data_Source SI4432::getModulationDataSource() {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Determine modtyp bits
  uint8_t dtmod = (mmc2 >> 4) & 0x03;

  // Ugly way to return correct enum
  switch (dtmod) {
    case 1:
      return DIRECT_SDI;
    case 2:
      return FIFO;
    case 3:
      return PN9;
    case 0:
    default:
      return DIRECT_GPIO;
  }
}

void SI4432::setDataClockConfiguration(SI4432_Data_Clock_Configuration clock) {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Clear the trclk bits
  mmc2 &= ~(0x03 << 6);

  // Set the desired data source
  mmc2 |= clock << 6;

  // Set the register
  this->setRegister(MODULATION_MODE_CONTROL_2, mmc2);
}
SI4432::SI4432_Data_Clock_Configuration SI4432::getDataClockConfiguration() {
  // Get the Modulation Mode Control 2 register
  uint8_t mmc2 = this->getRegister(MODULATION_MODE_CONTROL_2);

  // Determine modtyp bits
  uint8_t dtmod = (mmc2 >> 6) & 0x03;

  // Ugly way to return correct enum
  switch (dtmod) {
    case 1:
      return GPIO;
    case 2:
      return SDO;
    case 3:
      return NIRQ;
    case 0:
    default:
      return NONE;
  }
}

// Set or get the transmission power
void SI4432::setTransmissionPower(uint8_t power) {
  // Saturate to maximum power
  if (power > 20) {
    power = 20;
  }

  // Get the TX power register
  uint8_t txp = this->getRegister(TX_POWER);

  // Clear txpow bits
  txp &= ~(0x07);

  // Calculate txpow bits (See Section 5.7.1 of datasheet)
  uint8_t txpow = (power + 1) / 3;

  // Set txpow bits
  txp |= txpow;

  // Set the register
  this->setRegister(TX_POWER, txp);
}
uint8_t SI4432::getTransmissionPower() {
  // Get the TX power register
  uint8_t txp = this->getRegister(TX_POWER);

  // Get the txpow bits
  uint8_t txpow = txp & 0x07;

  // Calculate power (see Section 5.7.1 of datasheet)
  if (txpow == 0) {
    return 1;
  } else {
    return txpow * 3 - 1;
  }
}

// Set or get the GPIO configuration
void SI4432::setGPIOFunction(SI4432_GPIO gpio, SI4432_GPIO_Function func) {
  // Get the GPIO register
  uint8_t gpioX = this->getRegister(gpio);

  // Clear gpioX bits
  gpioX &= ~((1 << 5) - 1);

  // Set the gpioX bits
  gpioX |= func;

  // Set the register
  this->setRegister(gpio, gpioX);
}

uint8_t SI4432::getGPIOFunction(SI4432_GPIO gpio) {
  // Get the GPIO register
  uint8_t gpioX = this->getRegister(gpio);

  // Return the gpioX bits
  // This should probably return an enum, but this needs a lot of cases
  return gpioX & ((1 << 5) - 1);
}

// Enable or disable interrupts
void SI4432::setInterruptEnable(SI4432_Interrupt interrupt, bool enable) {
  // Get the (16 bit) register value
  uint16_t intEnable = this->get16BitRegister(INTERRUPT_ENABLE_1);

  // Either enable or disable the interrupt
  if (enable) {
    intEnable |= interrupt;
  } else {
    intEnable &= ~interrupt;
  }

  // Set the (16 bit) register value
  this->set16BitRegister(INTERRUPT_ENABLE_1, intEnable);
}

// Get the status of an interrupt
bool SI4432::getInterruptStatus(SI4432_Interrupt interrupt) {
  // Get the (16 bit) register value
  uint16_t intStatus = this->get16BitRegister(INTERRUPT_STATUS_1);

  // Determine if interrupt bit is set and return
  if ((intStatus & interrupt) > 0) {
    return true;
  } else {
    return false;
  }
}

// Set the operating mode
//	This function does not toggle individual pins as with other functions
//	It expects a bitwise-ORed combination of the modes you want set
void SI4432::setOperatingMode(uint16_t mode) {
  this->set16BitRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1, mode);
}

// Get operating mode (bitwise-ORed)
uint16_t SI4432::getOperatingMode() {
  return this->get16BitRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1);
}

// Manuall enter RX or TX mode
void SI4432::enableRXMode() {
  this->setOperatingMode(READY_MODE | RX_MODE);
}
void SI4432::enableTXMode() {
  this->setOperatingMode(READY_MODE | TX_MODE);
}

// Reset the device
void SI4432::reset() {
  this->setOperatingMode(READY_MODE | RESET);
}

// Set or get the trasmit header
void SI4432::setTransmitHeader(uint32_t header) {
  this->set32BitRegister(TRANSMIT_HEADER_3, header);
}
uint32_t SI4432::getTransmitHeader() {
  return this->get32BitRegister(TRANSMIT_HEADER_3);
}

// Set or get the check header
void SI4432::setCheckHeader(uint32_t header) {
  this->set32BitRegister(CHECK_HEADER_3, header);
}
uint32_t SI4432::getCheckHeader() {
  return this->get32BitRegister(CHECK_HEADER_3);
}

// Set or get the CRC mode
void SI4432::setCRCMode(SI4432::SI4432_CRC_Mode mode) {
  uint8_t dac = this->getRegister(DATA_ACCESS_CONTROL);

  dac &= ~0x24;

  switch (mode) {
    case CRC_DISABLED:
      break;
    case CRC_DATA_ONLY:
      dac |= 0x24;
      break;
    case CRC_NORMAL:
    default:
      dac |= 0x04;
      break;
  }

  this->setRegister(DATA_ACCESS_CONTROL, dac);
}
SI4432::SI4432_CRC_Mode SI4432::getCRCMode() {
  uint8_t dac = this->getRegister(DATA_ACCESS_CONTROL);

  if (!(dac & 0x04)) {
    return CRC_DISABLED;
  }
  if (dac & 0x20) {
    return CRC_DATA_ONLY;
  }
  return CRC_NORMAL;
}

// Set or get the CRC polynomial
void SI4432::setCRCPolynomial(SI4432::SI4432_CRC_Polynomial poly) {
  uint8_t dac = this->getRegister(DATA_ACCESS_CONTROL);

  dac &= ~0x03;

  dac |= poly;

  this->setRegister(DATA_ACCESS_CONTROL, dac);
}
SI4432::SI4432_CRC_Polynomial SI4432::getCRCPolynomial() {
  uint8_t dac = this->getRegister(DATA_ACCESS_CONTROL);

  switch (dac & 0x03) {
    case 0:
      return CCITT;
    case 1:
      return CRC16;
    case 2:
      return IEC16;
    case 3:
      return BIACHEVA;
  }
  return CRC16;
}

// Get and set all the FIFO threshold
void SI4432::setTXFIFOAlmostFullThreshold(uint8_t thresh) {
  this->setFIFOThreshold(TX_FIFO_CONTROL_1, thresh);
}
void SI4432::setTXFIFOAlmostEmptyThreshold(uint8_t thresh) {
  this->setFIFOThreshold(TX_FIFO_CONTROL_2, thresh);
}
void SI4432::setRXFIFOAlmostFullThreshold(uint8_t thresh) {
  this->setFIFOThreshold(RX_FIFO_CONTROL, thresh);
}
uint8_t SI4432::getTXFIFOAlmostFullThreshold() {
  return this->getRegister(TX_FIFO_CONTROL_1);
}
uint8_t SI4432::getTXFIFOAlmostEmptyThreshold() {
  return this->getRegister(TX_FIFO_CONTROL_2);
}
uint8_t SI4432::getRXFIFOAlmostFullThreshold() {
  return this->getRegister(RX_FIFO_CONTROL);
}
void SI4432::setFIFOThreshold(SI4432_Register reg, uint8_t thresh) {
  thresh &= ((1 << 6) - 1);
  this->setRegister(reg, thresh);
}

// Get RSSI value
uint8_t SI4432::getRSSI() {
  return this->getRegister(RECEIVED_SIGNAL_STRENGTH_INDICATOR);
}
// Get input power (in dBm)
//	Coefficients approximated from the graph in Section 8.10 of the datasheet
int8_t SI4432::getInputPower() {
  return 0.56 * this->getRSSI() - 128.8;
}

// Get length of last received packet
uint8_t SI4432::getReceivedPacketLength() {
  return this->getRegister(RECEIVED_PACKET_LENGTH);
}

// Set length of packet to be transmitted
void SI4432::setTransmitPacketLength(uint8_t length) {
  return this->setRegister(TRANSMIT_PACKET_LENGTH, length);
}

void SI4432::clearRXFIFO() {
  //Toggle ffclrrx bit high and low to clear RX FIFO
  this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 2);
  this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 0);
}

void SI4432::clearTXFIFO() {
  //Toggle ffclrtx bit high and low to clear TX FIFO
  this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 1);
  this->setRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_2, 0);
}

// Send data
/*
void SI4432::send(uint8_t *data, int length) {
	// Clear TX FIFO
	this->clearTXFIFO();

	// Initialise rx and tx arrays
	uint8_t tx[MAX_PACKET_LENGTH+1] = { 0 };
	uint8_t rx[MAX_PACKET_LENGTH+1] = { 0 };

	// Set FIFO register address (with write flag)
	tx[0] = FIFO_ACCESS | (1<<7);

	// Truncate data if its too long
	if (length > MAX_PACKET_LENGTH) {
		length = MAX_PACKET_LENGTH;
	}

	// Copy data from input array to tx array
	for (int i = 1; i <= length; i++) {
		tx[i] = data[i-1];
	}

	// Set the packet length
	this->setTransmitPacketLength(length);

	// Make the transfer
	this->transfer(tx,rx,length+1);

	// Enter TX mode
	this->enableTXMode();

	// Loop until packet has been sent (device has left TX mode)
	while (((this->getRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1)>>3) & 1)) {}

	return;
};
*/

void SI4432::send(void *ptr, int length) {
  uint8_t *data = (uint8_t *)ptr;

  // Clear TX FIFO
  this->clearTXFIFO();

  // Initialise rx and tx arrays
  uint8_t tx[MAX_PACKET_LENGTH + 1] = {0};
  uint8_t rx[MAX_PACKET_LENGTH + 1] = {0};

  // Set FIFO register address (with write flag)
  tx[0] = FIFO_ACCESS | (1 << 7);

  // Truncate data if its too long
  if (length > MAX_PACKET_LENGTH) {
    length = MAX_PACKET_LENGTH;
  }

  // Copy data from input array to tx array
  for (int i = 1; i <= length; i++) {
    tx[i] = *data++;
  }

  // Set the packet length
  this->setTransmitPacketLength(length);

  // Make the transfer
  this->transfer(tx, rx, length + 1);

  // Enter TX mode
  this->enableTXMode();

  // Loop until packet has been sent (device has left TX mode)
  while (((this->getRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1) >> 3) & 1)) {
  }

  return;
};

// Receive data (blocking with timeout). Returns number of bytes received
uint8_t SI4432::receive(uint8_t *data, uint8_t length, uint16_t timeout) {
  // Make sure RX FIFO is empty, ready for new data
  this->clearRXFIFO();

  // Enter RX mode
  this->enableRXMode();

  // Initialise rx and tx arrays
  uint8_t tx[MAX_PACKET_LENGTH + 1] = {0};
  uint8_t rx[MAX_PACKET_LENGTH + 1] = {0};

  // Set FIFO register address
  tx[0] = FIFO_ACCESS;

  // Timing for the interrupt loop timeout
  uint16_t remainingloops = timeout;

  // Loop endlessly on interrupt or timeout
  //	Don't use interrupt registers here as these don't seem to behave consistently
  //	Watch the operating mode register for the device leaving RX mode. This is indicitive
  //	of a valid packet being received
  while (((this->getRegister(OPERATING_MODE_AND_FUNCTION_CONTROL_1) >> 2) & 1) && (remainingloops > 1)) {
    remainingloops--;
#ifndef ARDUINO
    _delay_ms(1);
#else
    ESP.wdtFeed();
    delay(1);
#endif
  }

  // If timeout occured, return -1
  if (remainingloops == 1) {
    return 0;
  }

  // Get length of packet received
  uint8_t rxLength = this->getReceivedPacketLength();

  if (rxLength > length) {
    rxLength = length;
  }

  // Make the transfer
  this->transfer(tx, rx, rxLength + 1);

  // Copy the data to the output array
  for (int i = 1; i <= rxLength; i++) {
    data[i - 1] = rx[i];
  }

  return rxLength;
};

// Helper function to read a single byte from the device
uint8_t SI4432::getRegister(uint8_t reg) {
  // rx and tx arrays must be the same length
  // Must be 2 elements as the device only responds whilst it is being sent
  // data. tx[0] should be set to the requested register value and tx[1] left
  // clear. Once complete, rx[0] will be left clear (no data was returned whilst
  // the requested register was being sent), and rx[1] will contain the value
  uint8_t tx[] = {0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00};

  tx[0] = reg;

  this->transfer(tx, rx, 2);

  return rx[1];
}

// Similar to function above, but for readying 2 consequtive registers as one
uint16_t SI4432::get16BitRegister(uint8_t reg) {
  uint8_t tx[] = {0x00, 0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00, 0x00};

  tx[0] = reg;

  this->transfer(tx, rx, 3);

  return (rx[1] << 8) | rx[2];
}

// Similar to function above, but for readying 4 consequtive registers as one
uint32_t SI4432::get32BitRegister(uint8_t reg) {
  uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00};

  tx[0] = reg;

  this->transfer(tx, rx, 5);

  return (rx[1] << 24) | (rx[2] << 16) | (rx[3] << 8) | rx[4];
}

// Helper function to write a single byte to a register
void SI4432::setRegister(uint8_t reg, uint8_t value) {
  // tx and rx arrays required even though we aren't receiving anything
  uint8_t tx[] = {0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00};

  // tx[0] is the requested register with the final bit set high to indicate
  // a write operation (see Section 3.1 of the datasheet)
  tx[0] = reg | (1 << 7);

  // tx[1] is the value to be set
  tx[1] = value;

  this->transfer(tx, rx, 2);
}

// As above, but for 2 consequitive registers
void SI4432::set16BitRegister(uint8_t reg, uint16_t value) {
  // tx and rx arrays required even though we aren't receiving anything
  uint8_t tx[] = {0x00, 0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00, 0x00};

  // tx[0] is the requested register with the final bit set high to indicate
  // a write operation (see Section 3.1 of the datasheet)
  tx[0] = reg | (1 << 7);

  // tx[1-2] is the value to be set
  tx[1] = (value >> 8);
  tx[2] = (value)&0xFF;

  this->transfer(tx, rx, 3);
}

// As above, but for 4 consequitive registers
void SI4432::set32BitRegister(uint8_t reg, uint32_t value) {
  // tx and rx arrays required even though we aren't receiving anything
  uint8_t tx[] = {0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t rx[] = {0x00, 0x00, 0x00, 0x00, 0x00};

  // tx[0] is the requested register with the final bit set high to indicate
  // a write operation (see Section 3.1 of the datasheet)
  tx[0] = reg | (1 << 7);

  // tx[1-4] is the value to be set
  tx[1] = (value >> 24);
  tx[2] = (value >> 16) & 0xFF;
  tx[3] = (value >> 8) & 0xFF;
  tx[4] = (value)&0xFF;

  this->transfer(tx, rx, 5);
}
