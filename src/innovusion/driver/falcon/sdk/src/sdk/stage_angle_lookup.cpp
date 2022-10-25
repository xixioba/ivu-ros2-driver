/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/stage_angle_lookup.h"

#include "sdk/lidar.h"

#include "utils/log.h"
#include "utils/math_tables.h"
#include "utils/types_consts.h"

namespace innovusion {

StageAngleLookup::StageAngleLookup(InnoLidar *l)
    : lidar_(l)
    , params_(l->get_params())
    , misc_tables_(&l->get_misc_tables())
    , is_table_inited_(false)
    , table_version_(0) {
}

StageAngleLookup::~StageAngleLookup(void) {
}

void StageAngleLookup::calc_refl_beam_(int p_index,
                                       int channel,
                                       int p_angle,
                                       int g_angle,
                                       double v[]) {
  double N1[3] = {1, 0, 0}, N1acur[3], N1bcur[3];
  double N2[3], N1cur[3], N2cur[3], d1[3];
  double cos_theta, sin_theta, cos_phi, sin_phi;
  double g_tilt, sin_gtilt, cos_gtilt;
  double g_tilt2, sin_gtilt2, cos_gtilt2;
  double theta, phi, temp, tilt, shift;
  double alpha, gamma, cg, sg, ca, sa;
  const IvParams *params = &params_.iv_params;

  theta = p_angle * kDegreePerInnoAngleUnit;
  phi = g_angle * kDegreePerInnoAngleUnit;
  alpha = params->f_alpha[channel];
  gamma = params->f_gamma[channel];
  cg = MathTables::lookup_cos_table_exact(gamma);
  sg = MathTables::lookup_sin_table_exact(gamma);
  ca = MathTables::lookup_cos_table_exact(alpha);
  sa = MathTables::lookup_sin_table_exact(alpha);
  double d0[3] =  {sg, cg*sa, cg*ca};

  tilt = params->p_tilt[p_index];
  N2[0] = -MathTables::lookup_sin_table_exact(kPolygonSurfaceAngle + tilt);
  N2[1] = 0;
  N2[2] = MathTables::lookup_cos_table_exact(kPolygonSurfaceAngle + tilt);

  shift = params->p_shift[p_index];
  /* theta is polygon rotation angle, phi is galvo rotation angle */
  cos_theta = MathTables::lookup_cos_table_exact(theta + shift);
  sin_theta = MathTables::lookup_sin_table_exact(theta + shift);

  cos_phi = MathTables::lookup_cos_table_exact(phi);
  sin_phi = MathTables::lookup_sin_table_exact(phi);

  g_tilt = params->g_tilt;
  sin_gtilt = MathTables::lookup_sin_table_exact(g_tilt);
  cos_gtilt = MathTables::lookup_cos_table_exact(g_tilt);

  g_tilt2 = params->g_tilt2;
  sin_gtilt2 = MathTables::lookup_sin_table_exact(g_tilt2);
  cos_gtilt2 = MathTables::lookup_cos_table_exact(g_tilt2);

  N1acur[0] = N1[0] * cos_gtilt - N1[1] * sin_gtilt;
  N1acur[1] = N1[0] * sin_gtilt + N1[1] * cos_gtilt;
  N1acur[2] = N1[2];

  /* galvo normal rotation */
  N1bcur[0] = N1acur[0] * cos_phi + N1acur[2] * sin_phi;
  N1bcur[1] = N1acur[1];
  N1bcur[2] = -N1acur[0] * sin_phi + N1acur[2] * cos_phi;

  N1cur[0] = N1bcur[0] * cos_gtilt2 - N1bcur[1] * sin_gtilt2;
  N1cur[1] = N1bcur[0] * sin_gtilt2 + N1bcur[1] * cos_gtilt2;
  N1cur[2] = N1bcur[2];

  /* polygon normal rotation */
  N2cur[0] = N2[0];
  N2cur[1] = N2[1] * cos_theta - N2[2] * sin_theta;
  N2cur[2] = N2[1] * sin_theta + N2[2] * cos_theta;

  /* reflect off galvo */
  temp = N1cur[0] * d0[0] + N1cur[1] * d0[1] + N1cur[2] * d0[2];
  d1[0] = d0[0] - 2 * temp * N1cur[0];
  d1[1] = d0[1] - 2 * temp * N1cur[1];
  d1[2] = d0[2] - 2 * temp * N1cur[2];

  /* reflect off polygon */
  temp = N2cur[0] * d1[0] + N2cur[1] * d1[1] + N2cur[2] * d1[2];
  v[0] = d1[0] - 2 * temp * N2cur[0];
  v[1] = d1[1] - 2 * temp * N2cur[1];
  v[2] = d1[2] - 2 * temp * N2cur[2];
}

int StageAngleLookup::build_table_(int ignore_window_correction) {
  int p_angle, g_angle;
  double v[3];
  int32_t v_angle_offset[kInnoChannelNumber];
  int16_t h_angle_corr[kInnoChannelNumber] = {0};
  int16_t v_angle_corr[kInnoChannelNumber] = {0};
  for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
    v_angle_offset[channel] = channel * kInnoVAngleDiffBase;
  }
  for (int p = 0; p < kPolygonMaxFacets; p++) {
    for (int i = 0; i < kGalvoTableSize; i++) {
      g_angle = kGalvoMinAngle + (i << kEncoderTableShift);
      for (int j = 0; j < kPolygonTableSize; j++) {
        p_angle = kPolygonMinAngle + (j << kEncoderTableShift);
        if (!ignore_window_correction) {
          misc_tables_->get_window_correction(-p_angle,
                                              g_angle,
                                              h_angle_corr,
                                              v_angle_corr);
        }
        for (size_t channel = 0; channel < kInnoChannelNumber; channel++) {
          calc_refl_beam_(p, channel, p_angle, g_angle, v);
          int32_t value_atan = 0;
          int32_t value_asin = 0;
          MathTables::lookup_atan_table_exact(v[1]/v[2], &value_atan);
          MathTables::lookup_asin_table_exact(v[0], &value_asin);
          // xxx todo: internally we may want to use higher resolution
          // so that intermediate steps are more accurate.
          // we may also want to round the fraction to integer.
          table_[p][i][j][channel].h = value_atan + h_angle_corr[channel] -
            (channel > 0 ? table_[p][i][j][0].h : 0);
          table_[p][i][j][channel].v = value_asin + v_angle_corr[channel] -
            (channel > 0 ? table_[p][i][j][0].v : 0) - v_angle_offset[channel];

          inno_log_trace("p_angle %d g_angle %d c %lu h %d "
                         "corr %d v %d corr %d",
                         p_angle, g_angle, channel,
                         value_atan, h_angle_corr[channel],
                         value_asin, v_angle_corr[channel]);
        }
      }
    }
  }

  return 0;
}

inline int32_t StageAngleLookup::get_facet_and_polygon_(int32_t polygon_v,
                                                        int32_t *facet) const {
  static const int kInnoAngleUnitInFacet = kInnoAngleUnitPerPiRad * 2 /
                                           InnoConsts::kPolygonFacet;
  static const int kInnoAngleUnitIn3HalfFacet = kInnoAngleUnitInFacet * 3 / 2;

  // angle polygon_v relative to the last I index, in InnoAngleUnit
  inno_log_verify(polygon_v >= 0, "polygon too small %d", polygon_v);

  while ((uint32_t)polygon_v > kInnoAngleUnitPerPiRad * 2) {
    inno_log_warning("polygon out of range %d", polygon_v);
    polygon_v -= kInnoAngleUnitPerPiRad * 2;
  }

  // another ways is to use multiple range check instead of divison,
  // which one is faster?
  int32_t corrected_polygon_v = polygon_v - converted_p_offset_;
  // effectively doing round(corrected_polygon_v / kInnoAngleUnitInFacet)
  *facet = (corrected_polygon_v + kInnoAngleUnitIn3HalfFacet) /
            kInnoAngleUnitInFacet - 1;

  // xxx todo: up to 180/32768 * 4 = 0.02 degree error,
  // should we use double here?
  // or we might to increase angle resolution for internal computation
  int32_t polygon_mod = corrected_polygon_v - (*facet) * kInnoAngleUnitInFacet;

  // effectively doing *facet % kPolygonFacet
  if (*facet >= static_cast<int32_t>(InnoConsts::kPolygonFacet)) {
    *facet -= InnoConsts::kPolygonFacet;
  } else if (*facet < 0) {
    *facet += InnoConsts::kPolygonFacet;
  }
  return -polygon_mod;
}


void StageAngleLookup::map_to_angles(InnoFrameDirection direction,
                                     int32_t polygon_v,
                                     int32_t galvo_v,
                                     int16_t *h_angle,
                                     int16_t *v_angle,
                                     uint32_t angle_array_size,
                                     int32_t *facet,
                                     int32_t *polygon_mod) {
  inno_log_assert(kInnoChannelNumber == angle_array_size,
                  "%u vs %u", kInnoChannelNumber, angle_array_size);

  *polygon_mod = get_facet_and_polygon_(polygon_v, facet);

  // try to avoid div ops
  int32_t v_offset_total = galvo_v - kGalvoMinAngle;
  int32_t h_offset_total = *polygon_mod - kPolygonMinAngle;
  if (v_offset_total < 0) {
    v_offset_total = 0;
  }
  if (h_offset_total < 0) {
    h_offset_total = 0;
  }

  int h_idx = h_offset_total >> kEncoderTableShift;
  int v_idx = v_offset_total >> kEncoderTableShift;
  int h_offset = h_offset_total & kEncoderTableMask;
  int v_offset = v_offset_total & kEncoderTableMask;
  int h_offset2 = kEncoderTableStep - h_offset;
  int v_offset2 = kEncoderTableStep - v_offset;

  if (h_idx > signed(kPolygonTableSize - 2)) {
    h_idx = kPolygonTableSize - 2;
  }
  if (v_idx > signed(kGalvoTableSize - 2)) {
    v_idx = kGalvoTableSize - 2;
  }

  AngleHV *b11, *b12, *b21, *b22;
  b11 = &table_[*facet][v_idx][h_idx][0];
  for (uint32_t ich = 0; ich < kInnoChannelNumber; ich++, b11++) {
    b12 = b11 + kInnoChannelNumber;
    b21 = b11 + kPolygonTableSize * kInnoChannelNumber;
    b22 = b21 + kInnoChannelNumber;

    /* 12 mul ops */
    int32_t v1, v2;
    int64_t v3;
    v1 = b11->h * h_offset2 + b12->h * h_offset;
    v2 = b21->h * h_offset2 + b22->h * h_offset;
    v3 = v1 * v_offset2 + v2 * v_offset;
    h_angle[ich] = (v3 >> (kEncoderTableShift + kEncoderTableShift));

    v1 = b11->v * h_offset2 + b12->v * h_offset;
    v2 = b21->v * h_offset2 + b22->v * h_offset;
    v3 = v1 * v_offset2 + v2 * v_offset;
    v_angle[ich] = (v3 >> (kEncoderTableShift + kEncoderTableShift));
  }
  return;
}

const char *StageAngleLookup::get_name_() {
  return lidar_->get_name();
}

void StageAngleLookup::init_table() {
  converted_p_offset_ = static_cast<int32_t>(params_.iv_params.p_offset /
                                             kDegreePerInnoAngleUnit);
  inno_log_verify(build_table_(params_.iv_params.ignore_window_correction) == 0,
                  "%s cannot build table",
                  get_name_());
  is_table_inited_ = true;

  set_version(params_.get_version());
}
}  // namespace innovusion
