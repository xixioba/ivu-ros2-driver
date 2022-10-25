/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#ifndef SDK_STAGE_ANGLE_LOOKUP_H_
#define SDK_STAGE_ANGLE_LOOKUP_H_

#include <stddef.h>
#include <stdint.h>

#include "sdk_common/inno_lidar_packet.h"
#include "sdk/misc_tables.h"
#include "utils/log.h"

namespace innovusion {
class InnoLidar;
class LidarParams;

class AngleHV {
 public:
  int16_t v;
  int16_t h;
};

class StageAngleLookup {
 public:
  static constexpr double kPolygonSurfaceAngle = 27.0;
  static const int kGalvoMinAngle = 53 * kInnoAngleUnitPerPiRad /
                                    kInnoDegreePerPiRad;
  static const int kGalvoMaxAngle = 75 * kInnoAngleUnitPerPiRad /
                                    kInnoDegreePerPiRad;
  static const int kPolygonMaxFacets = 7;
  static const int kPolygonMinAngle = - (40 * kInnoAngleUnitPerPiRad /
                                         kInnoDegreePerPiRad);
  static const int kPolygonMaxAngle = 40 * kInnoAngleUnitPerPiRad /
                                      kInnoDegreePerPiRad;
  static const int kEncoderTableShift = 8;  // table resolution
  static const int kEncoderTableStep = 1 << kEncoderTableShift;
  static const int kEncoderTableMask = kEncoderTableStep - 1;
  static const int kGalvoTableSize =
    (kGalvoMaxAngle - kGalvoMinAngle) >> kEncoderTableShift;
  static const int kPolygonTableSize =
    (kPolygonMaxAngle - kPolygonMinAngle) >> kEncoderTableShift;

 public:
  explicit StageAngleLookup(InnoLidar *l);
  ~StageAngleLookup(void);

  void map_to_angles(InnoFrameDirection direction,
                     int32_t polygon,
                     int32_t galvo,
                     int16_t *h_angle,
                     int16_t *v_angle,
                     uint32_t angle_array_size,
                     int32_t *uintfacet,
                     int32_t *polygon_mod);
  void init_table();

  bool is_table_inited() {
    return is_table_inited_;
  }

  uint64_t get_version() {
    return table_version_;
  }

  void set_version(uint64_t version) {
    table_version_ = version;
  }

 private:
  const char *get_name_();
  void calc_refl_beam_(int p_index, int channel,
                       int p_angle, int g_angle,
                       double v[]);
  int build_table_(int ignore_window_correction);
  int32_t get_facet_and_polygon_(int32_t polygon_v, int32_t *facet) const;

 private:
  InnoLidar *lidar_;
  const LidarParams &params_;
  const MiscTables *misc_tables_;
  bool is_table_inited_;
  uint64_t table_version_;
  int32_t converted_p_offset_;

  AngleHV table_[kPolygonMaxFacets][kGalvoTableSize]\
                [kPolygonTableSize][kInnoChannelNumber];
};

}  // namespace innovusion

#endif  // SDK_STAGE_ANGLE_LOOKUP_H_

