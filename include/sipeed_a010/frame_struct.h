#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define FRAME_BEGIN_FLAG (0xFF00)
#define FRAME_END_FLAG (0xDD)

#define FRAME_HEAD_SIZE (20)
#define FRAME_HEAD_DATA_SIZE (16)
#define FRAME_CHECKSUM_SIZE (1)
#define FRAME_END_SIZE (1)

typedef struct
{
  uint16_t frame_begin_flag_;
  uint16_t frame_data_len_;
  uint8_t reserved1_;    // fixed to 0xff
  uint8_t output_mode_;  // 0:depth only, 1:depth+ir
  uint8_t senser_temp_;
  uint8_t driver_temp_;
  uint8_t exposure_time_[4];
  uint8_t error_code_;
  uint8_t reserved2_;  // fixed to 0x00
  uint8_t resolution_rows_;
  uint8_t resolution_cols_;
  uint16_t frame_id_;  // 12-bit, 0~4095
  uint8_t isp_version_;
  uint8_t reserved3_;  // fixed to 0xff
} __attribute__((packed)) frame_head_t;
static_assert(FRAME_HEAD_SIZE == sizeof(frame_head_t), "err");

typedef struct
{
  frame_head_t frame_head_;
  uint8_t payload_[];
} __attribute__((packed)) frame_t;

typedef struct
{
  uint8_t cali_mode_;  // 0:Normal, 1:Fisheye
  uint32_t fx_;        // fixpoint: u14p18
  uint32_t fy_;        // fixpoint: u14p18
  uint32_t u0_;        // fixpoint: u14p18
  uint32_t v0_;        // fixpoint: u14p18
  uint32_t k1_;        // fixpoint: s5p27
  uint32_t k2_;        // fixpoint: s5p27
  uint32_t k3_;        // fixpoint: s5p27
  uint32_t k4_p1_;     // fixpoint: s5p27, normal mode is k4, fisheye mode is p1
  uint32_t k5_p2_;     // fixpoint: s5p27, normal mode is k5 or unused, fisheye
                       // mode is p2
  uint32_t skew_;      // fixpoint: s8p24
} __attribute__((packed)) LensCoeff_t;

#ifdef __cplusplus
}
#endif