/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/misc_tables.h"
#include "utils/inno_lidar_log.h"
#include "sdk/delay_correction.h"

namespace innovusion {
void MiscTables::setup_aperture_correction_table(double aperture_offset) {
  int angle_in_unit, distance_in_unit;
  double angle, distance, a_offset, d_offset;
  int a_index, d_index;
  double a0, a1, d0, d1;
  double v0, v1, v2;
  double transmittance_correction;
  double factor = 1.0 / pow(1.5, -(80 - aperture_offset) / 20);

  for (int i = 0; i < kApertureTableAngleSize; i++) {
    angle_in_unit = kApertureMinAngle + (i << kApertureTableAngleShift);
    angle = angle_in_unit * kDegreePerInnoAngleUnit;
    a_offset = angle - (-60);  // the aperture table min angle is -60 degree
    if (a_offset < 0) {
      a_index = 0;
      a0 = 0;
      a1 = 1;
    } else if (a_offset >= 120) {  // the aperture table max angle is 60 degree
      a_index = 119;
      a0 = 1;
      a1 = 0;
     } else {
      a_index = static_cast<int>(a_offset);
      a0 = a_offset - a_index;
      a1 = 1 - a0;
    }

    v0 = window_transmittance_[a_index] * a1 +
         window_transmittance_[a_index + 1] * a0;
    transmittance_correction = 1 / v0;

    for (int j = 0; j < kApertureTableDistSize; j++) {
      if (j < kApertureTableDistSize0) {
        distance_in_unit = kApertureMinDist + (j << kApertureTableDistShift0);
      } else {
        distance_in_unit = kApertureMidDist +
          ((j - kApertureTableDistSize0) << kApertureTableDistShift1);
      }
      distance = distance_in_unit * kMeterPerInnoDistanceUnit;
      d_offset = distance * factor - 1;  // (distanc - 1.0 / factor) * factor
      if (d_offset < 0) {
        d_index = 0;
        d0 = 0;
        d1 = 1;
      } else if (d_offset >= 99) {  // aperture table max distance index is 99
        d_index = 98;
        d0 = 1;
        d1 = 0;
      } else {
        d_index = static_cast<int>(d_offset);
        d0 = d_offset - d_index;
        d1 = 1 - d0;
      }

      v0 = aperture_table_[d_index][a_index] * a1 +
           aperture_table_[d_index][a_index + 1] * a0;
      v1 = aperture_table_[d_index + 1][a_index] * a1 +
           aperture_table_[d_index + 1][a_index + 1] * a0;
      v2 = v0 * d1 + v1 * d0;

      aperture_correction_[i][j] = transmittance_correction / v2;
#if 0
      inno_log_info("i %d j %d angle %f distance %f v2 %f d_index %d "
                    "a_index %d %f/%f/%f/%f ap %f",
                    i, j, angle, distance, v2, d_index, a_index,
                    aperture_[d_index][a_index],
                    aperture_[d_index][a_index+1],
                    aperture_[d_index+1][a_index],
                    aperture_[d_index+1][a_index+1],
                    aperture_correction_[i][j]);
#endif
    }
  }
}

void MiscTables::setup_power_distance_correction_table(\
  double aperture_offset_2, double aperture_offset_3) {
  if (aperture_offset_2 == 0) {
    return;
  }

  double m = 2.11 - aperture_offset_3 * 0.0045;
  double offset = aperture_offset_2;
  // double A = 0.555 + 0.00015 * pow(offset - 30, 2);
  double A = 0.10 + 0.0002 * pow(offset - 30, 2);
  double w = 7.2 + (80 - offset) * 0.015;
  double d0 = 4.8 + (80 - offset) * 0.03;
  double pi = 3.1415926;
  double d = 15.0;
  // y0 is 0.010891 when offset = 80
  double y0 = 0.02 - (2*A/pi) * w/(4 * pow(d-d0, 2) + pow(w, 2));

  // printf("A %f w %f d0: %f\n", A, w, d0);

  for (int i = 0; i < kPowerDistanceTableSize; i++) {
    int units;
    double y;
    if (i < kPowerDistanceTableSize0) {
      units = i << kPowerDistanceTableShift0;
    } else {
      units = kPowerDistanceMidDist +
        ((i - kPowerDistanceTableSize0) << kPowerDistanceTableShift1);
    }
    d = units * kMeterPerInnoDistanceUnit;
    if (d < 15) {
      y = y0 + (2*A/pi) * w/(4 * pow(d-d0, 2) + pow(w, 2));
    } else {
      y = 0.02 * pow(15/d, m);
    }
    distance_correction_[i] = 1/y;
  }
}

void MiscTables::setup_delay_correction_table_() {
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    for (uint32_t i = 0; i < kIntensityTableSize; i++) {
      delay_correction_subns_[channel][i] =
        static_cast<InnoSubNsOffset16>(delay_correction_[channel][i] *
                                       (1 << InnoConsts::kSubNsBits));
      inno_log_trace("channel %u i %u delay_correction %.2f",
                     channel, i, delay_correction_subns_[channel][i]/32.0);
    }
  }
}

void MiscTables::setup_tables_from_default_() {
  memcpy(&window_transmittance_[0],
         &window_transmittance_default_[0],
         sizeof(window_transmittance_default_));
  memcpy(&aperture_table_[0][0],
         &aperture_table_default_[0][0],
         sizeof(aperture_table_default_));
  memcpy(&intensity_to_power_table_[0],
         &intensity_to_power_table_default_[0],
         sizeof(intensity_to_power_table_default_));
  memcpy(&intensity_to_power_table_2_[0],
         &intensity_to_power_table_2_default_[0],
         sizeof(intensity_to_power_table_2_default_));
  memcpy(&power_to_intensity_table_[0][0],
         &power_to_intensity_table_default_[0],
         sizeof(power_to_intensity_table_default_));
  memcpy(&distance_correction_[0],
         &distance_correction_default_[0],
         sizeof(distance_correction_default_));
  memcpy(&delay_correction_[0][0],
         &delay_correction_default_[0][0],
         sizeof(delay_correction_default_));
  memcpy(&win_corr_table_hori_[0][0][0],
         &win_corr_table_hori_default_[0][0][0],
         sizeof(win_corr_table_hori_default_));
  memcpy(&win_corr_table_vert_[0][0][0],
         &win_corr_table_vert_default_[0][0][0],
         sizeof(win_corr_table_vert_default_));

  // calculate intensity table
  int k = 0;
  for (int i = 0; i < 128; i++) {
    intensity_table_[k++] = i << 1;
  }
  for (int exp = 1; exp <= 9; exp++) {
    for (int i = 64; i < 128; i++) {
      intensity_table_[k++] = i << (exp + 1);
    }
  }

  // default aperture_offset is 80
  setup_aperture_correction_table(80.0);

  setup_delay_correction_table_();
}

void MiscTables::setup_delay_correction_with_old_table_() {
  inno_log_assert(sizeof(delay_correction_old_falcon_I_)/sizeof(float)/2 ==
                  kDCEntries, "incorrect delay_correction_old_falcon_I size");
  int k = 0;
  double dc = 0;
  for (int i = 0; i < kIntensityTableSize; i++) {
      int intensity = intensity_table_[i];
      while (k < kDCEntries &&
             intensity > delay_correction_old_falcon_I_[k][0]) {
        k++;
      }
      if (k == 0) {
        dc = delay_correction_old_falcon_I_[0][1];
      } else if (k < kDCEntries) {
        double intensity_0 = delay_correction_old_falcon_I_[k - 1][0];
        double intensity_1 = delay_correction_old_falcon_I_[k][0];
        double dc_0 = delay_correction_old_falcon_I_[k - 1][1];
        double dc_1 = delay_correction_old_falcon_I_[k][1];
        // linear interpolation
        dc = dc_0 + (intensity - intensity_0) / (intensity_1 - intensity_0) *
                    (dc_1 - dc_0);
      } else {
        dc = delay_correction_old_falcon_I_[kDCEntries - 1][1];
      }

      for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
        delay_correction_[channel][i] = dc;
      }
  }
}

void MiscTables::setup_delay_correction_with_polynomials_(\
  const IvParams &params) {
  int max_table_intensity = kDCMaxIntensity;
  int dist_corr_max_intensity = params.dist_corr_max_intensity;
  if (dist_corr_max_intensity > 0 &&
      dist_corr_max_intensity < max_table_intensity) {
    max_table_intensity = dist_corr_max_intensity;
  }

  const double *polynomial, *wc_polynomial, *le_polynomial;
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    if (channel == 0) {
      wc_polynomial = params.channel_0_wc_polynomial;
      le_polynomial = params.channel_0_le_polynomial;
    } else if (channel == 1) {
      wc_polynomial = params.channel_1_wc_polynomial;
      le_polynomial = params.channel_1_le_polynomial;
    } else if (channel == 2) {
      wc_polynomial = params.channel_2_wc_polynomial;
      le_polynomial = params.channel_2_le_polynomial;
    } else {
      wc_polynomial = params.channel_3_wc_polynomial;
      le_polynomial = params.channel_3_le_polynomial;
    }

    double saved_dc = 0;
    for (int i = 0; i < kIntensityTableSize; i++) {
      int intensity = intensity_table_[i];
      if (intensity > max_table_intensity) {
        delay_correction_[channel][i] = saved_dc;
        continue;
      }

      if (intensity < params.dist_corr_transition_intensity) {
        polynomial = wc_polynomial;
      } else {
        polynomial = le_polynomial;
      }

      double dc = 0;
      for (uint32_t j = 0; j <= kPolynomialDegree; j++) {
        dc = dc * intensity + polynomial[kPolynomialDegree - j];
      }
      saved_dc = dc;
      delay_correction_[channel][i] = dc;
    }
  }
}

void MiscTables::setup_tables_from_yaml(const IvParams &params) {
  if (params.intensity_to_power_table[512] != 0) {
    memcpy(&intensity_to_power_table_[0],
           &params.intensity_to_power_table[0],
           sizeof(params.intensity_to_power_table));
  }

  if (params.intensity_to_power_table_2[512] != 0) {
    memcpy(&intensity_to_power_table_2_[0],
           &params.intensity_to_power_table_2[0],
           sizeof(params.intensity_to_power_table_2));
  } else if (params.intensity_to_power_table[512] != 0) {
    memcpy(&intensity_to_power_table_2_[0],
           &params.intensity_to_power_table[0],
           sizeof(params.intensity_to_power_table));
  }

  if (params.power_to_intensity_table[512] != 0) {
    memcpy(&power_to_intensity_table_[0][0],
           &params.power_to_intensity_table[0],
           sizeof(params.power_to_intensity_table));
  }

  if (params.power_vs_distance_table[512] != 0) {
    // 0 - 80m,   resolution: 0.16m, 500 entries
    // 80 - 592m, resolution: 1.28m, 401 entries
    memcpy(&distance_correction_[0],
           &params.power_vs_distance_table[0],
           sizeof(params.power_vs_distance_table));
  }

  if (params.channel_0_delay_correction[512] != 0) {
    memcpy(&delay_correction_[0][0],
           &params.channel_0_delay_correction[0],
           sizeof(params.channel_0_delay_correction));
  } else if (params.channel_0_le_polynomial[1] != 0) {
    setup_delay_correction_with_polynomials_(params);
  } else if (params.dist_corr_transition_high == 0) {
    setup_delay_correction_with_old_table_();
  }

  if (params.channel_1_delay_correction[512] != 0) {
    memcpy(&delay_correction_[1][0],
           &params.channel_1_delay_correction[0],
           sizeof(params.channel_1_delay_correction));
  } else if (params.channel_0_delay_correction[512] != 0) {
    memcpy(&delay_correction_[1][0],
           &params.channel_0_delay_correction[0],
           sizeof(params.channel_0_delay_correction));
  }

  if (params.channel_2_delay_correction[512] != 0) {
    memcpy(&delay_correction_[2][0],
           &params.channel_2_delay_correction[0],
           sizeof(params.channel_2_delay_correction));
  } else if (params.channel_0_delay_correction[512] != 0) {
    memcpy(&delay_correction_[2][0],
           &params.channel_0_delay_correction[0],
           sizeof(params.channel_0_delay_correction));
  }

  if (params.channel_3_delay_correction[512] != 0) {
    memcpy(&delay_correction_[3][0],
           &params.channel_3_delay_correction[0],
           sizeof(params.channel_3_delay_correction));
  } else if (params.channel_0_delay_correction[512] != 0) {
    memcpy(&delay_correction_[3][0],
           &params.channel_0_delay_correction[0],
           sizeof(params.channel_0_delay_correction));
  }

  if (params.window_correction_table[512] != 0) {
    memcpy(&win_corr_table_hori_[0][0][0],
           &params.window_correction_table[0],
           sizeof(params.window_correction_table) / 2);
    memcpy(&win_corr_table_vert_[0][0][0],
           &params.window_correction_table\
           [sizeof(params.window_correction_table) / sizeof(float) / 2],
           sizeof(params.window_correction_table) / 2);
  }

  if (params.aperture_table[512] != 0) {
    memcpy(&aperture_table_[0][0],
           &params.aperture_table[0],
           sizeof(params.aperture_table));
  }

  if (params.window_transmittance[64] != 0) {
    memcpy(&window_transmittance_[0],
           &params.window_transmittance[0],
           sizeof(params.window_transmittance));
  }

  setup_aperture_correction_table(params.aperture_offset);

  setup_power_distance_correction_table(params.aperture_offset_2,
                                        params.aperture_offset_3);

  setup_delay_correction_table_();
}

void MiscTables::get_window_correction(int p_angle_in_unit,
                                       int g_angle_in_unit,
                                       int16_t *h_angle_corr,
                                       int16_t *v_angle_corr) const {
  double p_angle, g_angle;
  double p_angle_in_step, g_angle_in_step;
  int p_index, g_index;
  p_angle = p_angle_in_unit * kDegreePerInnoAngleUnit;
  g_angle = g_angle_in_unit * kDegreePerInnoAngleUnit;

  p_angle_in_step = (p_angle - kPolygonMinWinCorrAngle) /
                               kPolygonWinCorrTableStep;
  g_angle_in_step = (g_angle - kGalvoMinWinCorrAngle) /
                               kGalvoWinCorrTableStep;
  p_index = static_cast<int>(p_angle_in_step);
  g_index = static_cast<int>(g_angle_in_step);
  if (p_index < 0) {
    p_index = 0;
  } else if (p_index > kPolygonWinCorrTableSize - 2) {
    p_index = kPolygonWinCorrTableSize - 2;
  }
  if (g_index < 0) {
    g_index = 0;
  } else if (g_index > kGalvonWinCorrTableSize - 2) {
    g_index = kGalvonWinCorrTableSize - 2;
  }
  double p0, p1, g0, g1;
  p0 = p_angle_in_step - p_index;
  p1 = 1 - p0;
  g0 = g_angle_in_step - g_index;
  g1 = 1 - g0;
  for (uint32_t i = 0; i < kInnoChannelNumber; i++) {
    double v0, v1, v2;
    v0 = win_corr_table_hori_[i][g_index][p_index] * p1 +
         win_corr_table_hori_[i][g_index][p_index + 1] * p0;
    v1 = win_corr_table_hori_[i][g_index + 1][p_index] * p1 +
         win_corr_table_hori_[i][g_index + 1][p_index + 1] * p0;
    v2 = v0 * g1 + v1 * g0;
    h_angle_corr[i] = static_cast<int16_t>(v2 / kDegreePerInnoAngleUnit);

    v0 = win_corr_table_vert_[i][g_index][p_index] * p1 +
         win_corr_table_vert_[i][g_index][p_index + 1] * p0;
    v1 = win_corr_table_vert_[i][g_index + 1][p_index] * p1 +
         win_corr_table_vert_[i][g_index + 1][p_index + 1] * p0;
    v2 = v0 * g1 + v1 * g0;
    v_angle_corr[i] = static_cast<int16_t>(v2 / kDegreePerInnoAngleUnit);
  }
}

}  // namespace innovusion
