#pragma once
#include "../modbus_defs.hpp"
#include "../../primitives.hpp"
#include <cstdint>
#include <stddef.h>

ROSHIP_IO_NS_HEAD
namespace primitives {
typedef word*          WordBuffer;
/*!
 * \brief The BigEndianPrimative struct represents a big endian version
 * of various primitives.  Also includes overloaded casting operators
 * so it can be assigned to standard primatives with minimal extra syntax.
 */
template <typename PrimT>
struct BigEndianPrimativeByWord{
  union prim_union
  {
      PrimT u;
      uint16_t words[sizeof(PrimT)/2];
  } raw;
  /*!
   * \brief returns the machine-endian version of the variable in
   * the specified (PrimT) type.
   * \return a machine endian version fo the variable
   */
  PrimT get() const{
    prim_union output = raw;
    auto words = sizeof(PrimT)/2;
    for(int i = 0; i < words; i++){
      output.words[i] = raw.words[words - 1 - i];
    }
    return output.u;
  }
  void set(PrimT val){
    prim_union input;
    input.u = val;
    auto words = sizeof(PrimT)/2;
    for(int i = 0; i < words; i++){
      raw.words[i] = input.words[words - 1 - i];
    }
  }

  operator int8_t() const { return get(); }
  operator int16_t() const { return get(); }
  operator int32_t() const { return get(); }

  operator uint8_t() const { return get(); }
  operator uint16_t() const { return get(); }
  operator uint32_t() const { return get(); }

  operator float() const {return get(); }
  operator double() const {return get(); }

  template <typename T>
  BigEndianPrimativeByWord&operator=(T other){this->set(other);}
};

typedef BigEndianPrimativeByWord<u32>     BE_WORD_u32;
typedef BigEndianPrimativeByWord<s8>      BE_WORD_s8;
typedef BigEndianPrimativeByWord<s16>     BE_WORD_s16;
typedef BigEndianPrimativeByWord<s32>     BE_WORD_s32;
typedef BigEndianPrimativeByWord<f32>     BE_WORD_f32;
typedef BigEndianPrimativeByWord<f64>     BE_WORD_f64;

}
NS_FOOT
