/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_RAWDATA_TYPE_H_
#define SDK_RAWDATA_TYPE_H_

#include <stdint.h>

namespace innovusion {

enum RawDataType {
  kRawDataTypeOther  = 0,
  kRawDataTypeAdc    = 1,  // 1, 3, 5, 7
  // kRawDataTypeAdcRaw = 2,
  kRawDataTypePadding = 4,
  kRawDataTypeEncoder = 6,
  kRawDataTypeTrigger = 8,
};

enum RawDataSubType {
  kRawDataSubTypePtp = 0x0,
  kRawDataSubTypeNtp = 0x1,
  kRawDataSubTypeMagic = 0x2,
  kRawDataSubTypeGps = 0xf,
};

enum RawDataTimeType {
  RAW_DATA_TIME_TYPE_NO = 0,
  RAW_DATA_TIME_TYPE_HW = 1,
};

/*
  https://innovusion.atlassian.net/wiki/spaces/EE/pages/1641644095/Foxib+Padding+Packet+Format
*/

class __attribute__((packed)) PaddingData {
 public:
  /* type == 4 */
  unsigned char type              : 4;
  /* all 0 */
  unsigned char padding_0         : 4;
  /* 0 means normal, 1 means has data dropped */
  unsigned char detail            : 4;
  unsigned char drop              : 4;
};

class __attribute__((packed)) GpsData {
 public:
  /* type == 0 */
  unsigned char type              : 4;
  unsigned char sub_type          : 4;
  uint64_t fpga_clock             : 60;
  unsigned char zero_a            : 4;
  unsigned char zero_b            : 4;
  unsigned char s2                : 4;
  unsigned char s1                : 4;
  unsigned char m2                : 4;
  unsigned char m1                : 4;
  unsigned char h2                : 4;
  unsigned char h1                : 4;
  unsigned char y2                : 4;   // 9
  unsigned char y1                : 4;   // 1
  unsigned char M2                : 4;   // start from 1
  unsigned char M1                : 4;
  unsigned char d2                : 4;
  unsigned char d1                : 4;
  unsigned char zero_c            : 2;
  unsigned char pps               : 1;
  unsigned char gprmc             : 1;
};

class __attribute__((packed)) PtpData {
 public:
  /* type == 0 */
  unsigned char type                 : 4;
  unsigned char sub_type             : 4;
  union Msg {
    struct __attribute__((packed)) TimeMsg {
      uint64_t fpga_clock            : 60;
      unsigned char s2               : 4;
      unsigned char s1               : 4;
      unsigned char m2               : 4;
      unsigned char m1               : 4;
      unsigned char h2               : 4;
      unsigned char h1               : 4;
      unsigned char y2               : 4;
      unsigned char y1               : 4;
      unsigned char M2               : 4;
      unsigned char M1               : 4;
      unsigned char d2               : 4;
      unsigned char d1               : 4;
      int16_t diff_10us              : 11;
      unsigned char ptp              : 1;
    } time;
    struct __attribute__((packed)) InfoMsg {
      uint32_t seq_num               : 32;
      uint32_t timestamp_ms          : 32;
      uint32_t reserved_0            : 32;
      unsigned char gm               : 1;
      unsigned char locked_once      : 1;
      unsigned char fpga_clk_stoped  : 1;
      unsigned char fpga_clk_resumed : 1;
      uint32_t reserved_3            : 19;
      unsigned char ptp              : 1;
    } info;
  } msg;
};

class __attribute__((packed)) NtpData {
 public:
  /* type == 0 */
  unsigned char type              : 4;
  unsigned char sub_type          : 4;
  uint64_t fpga_clock             : 60;
  unsigned char s2                : 4;
  unsigned char s1                : 4;
  unsigned char m2                : 4;
  unsigned char m1                : 4;
  unsigned char h2                : 4;
  unsigned char h1                : 4;
  unsigned char y2                : 4;
  unsigned char y1                : 4;
  unsigned char M2                : 4;
  unsigned char M1                : 4;
  unsigned char d2                : 4;
  unsigned char d1                : 4;
  int16_t      offset             : 10;     // max: 1024 * 100us = 102.4ms
  unsigned char offset_10_100us   : 1;      // 0: 10us; 1: 100us
  unsigned char has_locked        : 1;
};


typedef union {
  /* type == 2 */
  unsigned char raw_bytes[8];
  struct {
    unsigned char laser_trigger_d0: 1;
    unsigned char cha_s0          : 7;
    unsigned char p_enc_I         : 1;
    unsigned char cha_s1          : 7;
    unsigned char p_enc_A         : 1;
    unsigned char cha_s2          : 7;
    unsigned char b_enc_I         : 1;
    unsigned char cha_s3          : 7;
    unsigned char b_enc_A         : 1;
    unsigned char cha_s4          : 7;
    unsigned char reserved_0      : 1;
    unsigned char cha_s5          : 7;
    unsigned char reserved_1      : 1;
    unsigned char cha_s6          : 7;
    unsigned char reserved_2      : 1;
    unsigned char cha_s7          : 7;
  };
} AdcRawData;

class __attribute__((packed)) EncoderData {
 public:
  inline uint16_t galvo_position(void) const {
    return galvo_enc_0 |
        (galvo_enc_1 << 3) |
        (galvo_enc_2 << 4) |
        (galvo_enc_3 << 7) |
        (galvo_enc_4 << 8);
  }

 public:
  /* type == 6 */
  unsigned char type            : 4;
  unsigned char time_stamp_0    : 4;
  unsigned char time_stamp_1    : 8;
  unsigned char time_stamp_2    : 8;
  unsigned char time_stamp_3    : 8;

  unsigned char galvo_enc_0     : 3;
  unsigned char enc_A           : 1;
  unsigned char enc_B           : 1;
  unsigned char enc_I           : 1;
  unsigned char galvo_sbit      : 1;  // up direction
  unsigned char galvo_enc_1     : 1;

  unsigned char galvo_enc_2     : 3;
  unsigned char v_enc_A         : 1;
  unsigned char v_enc_B         : 1;
  unsigned char v_enc_I         : 1;
  unsigned char v_home_sensor   : 1;
  unsigned char galvo_enc_3     : 1;

  unsigned char f_enc_A         : 1;
  unsigned char f_enc_B         : 1;
  unsigned char f_enc_I         : 1;
  unsigned char galvo_rbit      : 1;  // in roi
  unsigned char vf_enc_A        : 1;
  unsigned char vf_enc_B        : 1;
  unsigned char vf_enc_I        : 1;
  unsigned char v_galvo_enc     : 1;

  unsigned char galvo_enc_4     : 8;

  union {
    struct {
      unsigned char reserved_0  : 8;
      unsigned char reserved_1  : 8;
      unsigned char reserved_2  : 8;
      unsigned char reserved_3  : 8;

      unsigned char reserved_4  : 8;
      unsigned char reserved_5  : 8;
      unsigned char laser_temp  : 8;
      unsigned char board_temp  : 8;
    };
    struct {
      unsigned char g_tstamp_type : 4;
      unsigned char g_tstamp_0  : 4;
      unsigned char g_tstamp_1  : 8;
      unsigned char g_tstamp_2  : 8;
      unsigned char g_tstamp_3  : 8;

      unsigned char g_tstamp_4  : 8;
      unsigned char g_tstamp_5  : 8;
      unsigned char g_tstamp_6  : 8;
      unsigned char g_tstamp_7  : 8;
    };
  };
};

/* 4 bytes */
class __attribute__((packed)) TriggerData {
 public:
  /* type == 8 */
  unsigned char type              : 4;
  unsigned char time_stamp_0      : 4;
  unsigned char time_stamp_1      : 8;
  unsigned char time_stamp_2      : 8;
  unsigned char has_overflow      : 1;
  unsigned char sync_timestamp_only : 1;
  unsigned char reserved            : 6;
};

/* 4 bytes */
class __attribute__((packed)) AdcComputedData {
 public:
  /* type == 1, note that type only has one bit */
  unsigned char type              : 1;
  /* 0/1/2/3: Channel A/B/C/D */
  unsigned char channel           : 2;
  /* 13 bits timestamp, covers 8us,
     min trigger rate is 125Khz */
  unsigned char time_stamp_0      : 5;
  unsigned char time_stamp_1      : 5;
  unsigned char time_shift        : 3;
  unsigned char ref_flag          : 1;
  unsigned char offs_frac         : 5;
  unsigned char intensity_shift_1 : 2;
  unsigned char intensity_shift_2 : 1;
  unsigned char intensity_base    : 7;
};

class __attribute__((packed)) AdcComputedDataExt {
 public:
  /* type == 1, note that type only has one bit */
  unsigned char type              : 1;
  /* 0/1/2/3: Channel A/B/C/D */
  unsigned char channel           : 2;
  unsigned char time_stamp_0      : 5;

  unsigned char time_stamp_1      : 5;
  unsigned char time_shift        : 3;

  unsigned char ref_flag          : 1;
  unsigned char offs_frac         : 5;
  unsigned char intensity_shift_1 : 2;

  unsigned char intensity_shift_2 : 2;
  unsigned char intensity_base_1  : 6;

  unsigned char intensity_base_2  : 1;
  /* real pulse width */
  unsigned char pulse_width       : 4;

  /* real peak position */
  unsigned char peak_pos_1        : 3;

  unsigned char peak_pos_2        : 1;
  /* peak_value / 8 */
  unsigned char peak_value        : 7;
};
}  // namespace innovusion
#endif  // SDK_RAWDATA_TYPE_H_
