#pragma once
#include "modbus_primitives.hpp"
#include "blocks_defs.hpp"
#include <stddef.h>
#include <memory>
#include <sstream>
#include <iomanip>

MODBUS_NS_HEAD

class Block{
public:
  std::shared_ptr<Block> SharedPtr;
  /*!
   * \brief serialize makes sure the buffer is ready to go
   * and then returns a primitives::WordBuffer.
   * \return a primitives::WordBuffer ready for modbus
   */
  virtual primitives::WordBuffer buffer() = 0;
  /*!
   * \brief size computes the size of the block in WORDS
   * \return the size of the block in words
   */
  virtual size_t size() = 0;
  /*!
   * \brief writeAddress returns the address of the block if you want
   * to write the values to the device
   * \return
   */
  virtual int modbusAddress() = 0;
  /*!
   * \brief hexString returns the buffer as a hex string, usefull for
   * debugging
   * \return a std::string representing the buffer as hex
   */
  virtual std::string hexString(){
    std::stringstream ss;
    for (size_t i = 0; i < size(); ++i) {
      ss << "0x" << std::hex << std::uppercase << std::setfill('0') << std::setw(4) << buffer()[i];
      if (i < size() - 1) {
        ss << ", ";
      }
    }
    return ss.str();
  }
};

MODBUS_NS_FOOT
