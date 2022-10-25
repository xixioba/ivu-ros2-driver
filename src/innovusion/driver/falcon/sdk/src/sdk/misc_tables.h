/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_MISC_TABLES_H_
#define SDK_MISC_TABLES_H_

#include <math.h>

#include "sdk_common/inno_lidar_packet.h"
#include "utils/inno_lidar_log.h"
#include "utils/types_consts.h"

namespace innovusion {
#include "sdk/iv_params_code_gen.gen_h"
class MiscTables {
 public:
  static const int kMaxIntensity = 130048;
  static const int kIntensityTableSize = 704;  // 128 + 64 * 9

  static const int kPolygonMinWinCorrAngle = -35;  // in degree
  static const int kPolygonMaxWinCorrAngle = 35;
  static const int kPolygonWinCorrTableStep = 2;
  static const int kPolygonWinCorrTableSize =
    (kPolygonMaxWinCorrAngle - kPolygonMinWinCorrAngle) /
    kPolygonWinCorrTableStep + 1;

  static const int kGalvoMinWinCorrAngle = 57;
  static const int kGalvoMaxWinCorrAngle = 68;
  static constexpr float kGalvoWinCorrTableStep = 0.5;
  static const int kGalvonWinCorrTableSize =
    (kGalvoMaxWinCorrAngle - kGalvoMinWinCorrAngle) /
    kGalvoWinCorrTableStep + 1;

  // for the internally generated aperture correction table
  static const int kApertureMinAngle = -(61 * kInnoAngleUnitPerPiRad /
                                       kInnoDegreePerPiRad);
  static const int kApertureMaxAngle = 61 * kInnoAngleUnitPerPiRad /
                                       kInnoDegreePerPiRad;
  static const int kApertureTableAngleShift = 8;  // resolution: 1.4 degree
  static const int kApertureTableAngleSize =
    ((kApertureMaxAngle - kApertureMinAngle) >> kApertureTableAngleShift) + 2;

  static const int kApertureMinDist = 0 * kInnoDistanceUnitPerMeter;
  // 20.48 * kInnoDistanceUnitPerMeter;
  static const int kApertureMidDist = 4096;
  // 102.4 * kInnoDistanceUnitPerMeter;
  static const int kApertureMaxDist = 20480;
  static const int kApertureTableDistShift0 = 6;   // resolution: 0.32m
  static const int kApertureTableDistShift1 = 10;  // resolution: 5.12m
  static const int kApertureTableDistSize0 =       // 64
    (kApertureMidDist - kApertureMinDist) >> kApertureTableDistShift0;
  // 64 + 16
  static const int kApertureTableDistSize =  kApertureTableDistSize0 +
    ((kApertureMaxDist - kApertureMidDist) >> kApertureTableDistShift1);

  static const int kPowerDistanceMinDist = 0 * kInnoDistanceUnitPerMeter;
  static const int kPowerDistanceMidDist = 80 * kInnoDistanceUnitPerMeter;
  static const int kPowerDistanceMaxDist = 592 * kInnoDistanceUnitPerMeter;
  static const int kPowerDistanceTableShift0 = 5;  // resolution: 0.16m
  static const int kPowerDistanceTableShift1 = 8;  // resolution: 1.28m
  static const int kPowerDistanceTableSize0 =      // 500
    (kPowerDistanceMidDist - kPowerDistanceMinDist) >>
    kPowerDistanceTableShift0;
  // 500 + 401
  static const int kPowerDistanceTableSize = 1 + kPowerDistanceTableSize0 +
    ((kPowerDistanceMaxDist - kPowerDistanceMidDist) >>
    kPowerDistanceTableShift1);

  // for old delay correction table
  static const int kDCMinIntensity = 0;
  static const int kDCMaxIntensity = 39936;
  static const int kDCIntensityShift = 5;
  static const int kDCEntries =
      ((kDCMaxIntensity - kDCMinIntensity) >> kDCIntensityShift) + 1;
  static const int kPolynomialDegree = 9;

 private:
  static const float aperture_table_default_[100][121];
  static const float window_transmittance_default_[121];
  static const uint32_t intensity_to_power_table_default_[kIntensityTableSize];
  static const uint32_t intensity_to_power_table_2_default_\
                                       [kIntensityTableSize];
  static const uint32_t power_to_intensity_table_default_[900];
  static const float distance_correction_default_[kPowerDistanceTableSize];
  static const float delay_correction_default_[kInnoChannelNumber]\
                                              [kIntensityTableSize];
  static const float delay_correction_old_falcon_I_[kDCEntries][2];

  static const uint32_t max_per_table_[];
  static const uint32_t min_per_table_[];
  static const uint32_t interval_per_table_[];
  static const float win_corr_table_hori_default_[kInnoChannelNumber]\
    [kGalvonWinCorrTableSize][kPolygonWinCorrTableSize];
  static const float win_corr_table_vert_default_[kInnoChannelNumber]\
    [kGalvonWinCorrTableSize][kPolygonWinCorrTableSize];

 private:
  float win_corr_table_hori_[kInnoChannelNumber][kGalvonWinCorrTableSize]\
                                                [kPolygonWinCorrTableSize];
  float win_corr_table_vert_[kInnoChannelNumber][kGalvonWinCorrTableSize]\
                                                [kPolygonWinCorrTableSize];
  float aperture_table_[100][121];
  float window_transmittance_[121];
  float aperture_correction_[kApertureTableAngleSize]\
                            [kApertureTableDistSize];
  uint32_t intensity_table_[kIntensityTableSize];
  uint32_t intensity_to_power_table_[kIntensityTableSize];
  uint32_t intensity_to_power_table_2_[kIntensityTableSize];
  uint32_t power_to_intensity_table_[9][100];
  float distance_correction_[kPowerDistanceTableSize];
  float delay_correction_[kInnoChannelNumber][kIntensityTableSize];
  InnoSubNsOffset16 delay_correction_subns_[kInnoChannelNumber]\
                                           [kIntensityTableSize];

 private:
  void setup_tables_from_default_();
  void setup_delay_correction_with_polynomials_(const IvParams &params);
  void setup_delay_correction_with_old_table_();
  void setup_delay_correction_table_();

 public:
  MiscTables() {
    setup_tables_from_default_();
  }
  ~MiscTables() {}
  void setup_aperture_correction_table(double aperture_offset);
  void setup_power_distance_correction_table(double aperture_offset_2,
                                             double aperture_offset_3);
  void setup_tables_from_yaml(const IvParams &params);
  void get_window_correction(int p_angle_in_unit,
                             int g_angle_in_unit,
                             int16_t *h_angle_corr,
                             int16_t *v_angle_corr) const;

  inline InnoSubNsOffset16 get_delay_correction(int32_t channel,
                                                int32_t intensity_seq) const {
    return delay_correction_subns_[channel][intensity_seq];
  }

  inline uint32_t convert_to_power(uint32_t intensity_seq) const {
    return intensity_to_power_table_[intensity_seq];
  }

  inline uint32_t convert_to_intensity(uint32_t power) const {
    uint32_t intensity = kMaxIntensity;
    for (uint32_t i = 0; i < sizeof(power_to_intensity_table_)/
                             sizeof(power_to_intensity_table_[0]); i++) {
      if (power < max_per_table_[i]) {
        int index = (power - min_per_table_[i]) / interval_per_table_[i];
        intensity = power_to_intensity_table_[i][index];
#if 0
        inno_log_info("convert_to_intensity - power: %u, "
                      "max_per_table: %u, min_per_table: %u, "
                      "intervals_per_table: %u "
                      "i: %u, result_intensity: %u",
                      power, max_per_table_[i], min_per_table_[i],
                      interval_per_table_[i], i, intensity);
#endif
        return intensity;
      }
    }
    return intensity;
  }

  inline uint32_t convert_to_intensity_seq(uint32_t intensity) const {
    int base, exponent, intensity_seq;
    if (intensity < 256) {
      base = intensity >> 1;
      exponent = 0;
    } else {
      int shift = 25 - __builtin_clz(intensity);
      exponent = shift - 1;
      base = intensity >> shift;
    }
    intensity_seq = exponent * 64 + base;
    if (intensity_seq >= kIntensityTableSize) {
      intensity_seq = kIntensityTableSize - 1;
    }
    return intensity_seq;
  }

  inline uint32_t intensity_to_power(uint32_t intensity) const {
    return convert_to_power(convert_to_intensity_seq(intensity));
  }

  inline uint32_t convert_to_power_2(uint32_t intensity_seq) const {
    return intensity_to_power_table_2_[intensity_seq];
  }

  inline uint32_t intensity_to_power_2(uint32_t intensity) const {
    return convert_to_power_2(convert_to_intensity_seq(intensity));
  }

  inline float get_scaled_reflectance(uint32_t intensity_seq,
                                      uint32_t ref_intensity,
                                      uint32_t radius,
                                      int32_t h_angle) const {
    // we use reference intensity to represent reference power
    // since it is in the linear region, therefore we don't
    // convert ref_intensity to power
    uint32_t raw_power = convert_to_power_2(intensity_seq);

    int d_index, d_index2;
    if (radius < kApertureMidDist) {
      d_index = radius >> kApertureTableDistShift0;
    } else {
      d_index = kApertureTableDistSize0 +
        ((radius - kApertureMidDist) >> kApertureTableDistShift1);
      if (d_index >= kApertureTableDistSize) {
        d_index = kApertureTableDistSize - 1;
      }
    }

    if (radius < kPowerDistanceMidDist) {
      d_index2 = radius >> kPowerDistanceTableShift0;
    } else {
      d_index2 = kPowerDistanceTableSize0 +
        ((radius - kPowerDistanceMidDist) >> kPowerDistanceTableShift1);
      if (d_index2 >= kPowerDistanceTableSize) {
        d_index2 = kPowerDistanceTableSize - 1;
      }
    }

    int32_t angle_offset = h_angle - kApertureMinAngle;
    if (angle_offset < 0) {
      angle_offset = 0;
    }
    int a_index = angle_offset >> kApertureTableAngleShift;
    if (a_index >= kApertureTableAngleSize) {
      a_index = kApertureTableAngleSize - 1;
    }

    float r = aperture_correction_[a_index][d_index] *
      distance_correction_[d_index2] * raw_power / ref_intensity;

#if 0
    inno_log_info("h_angle: %d radius: %d ap: %f d: %f "
                  "raw_intensity : %d/%d reference: %d "
                  "reflectance: %f",
                  h_angle, radius,
                  aperture_correction_[a_index][d_index],
                  distance_correction_[d_index2],
                  raw_intensity, raw_power, ref_intensity, r);
#endif
    return r;
  }
};
}  // namespace innovusion

#endif  // SDK_MISC_TABLES_H_
