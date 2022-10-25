/**
 *  Copyright (C) 2018 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */
#ifndef INNO_LIDAR_INNOVUSION_POINTCLOUD_INCLUDE_INNOVUSION_POINTCLOUD_POINT_TYPES_H_
#define INNO_LIDAR_INNOVUSION_POINTCLOUD_INCLUDE_INNOVUSION_POINTCLOUD_POINT_TYPES_H_

#include <pcl/point_types.h>

namespace innovusion_pointcloud {
/** Euclidean Innovusion coordinate, including intensity and ring number. */
struct PointXYZIR {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 // laser intensity reading
  std::uint16_t ring;              // laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
};

struct PointXYZIT {
  PCL_ADD_POINT4D;  // quad-word XYZ
  double timestamp;
  std::uint8_t intensity;
  std::uint8_t flags;
#if 0
  std::uint16_t scan_id;
  std::uint16_t scan_seq;
  std::uint16_t cluster;
#endif
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
};

struct PointRaw {
  PCL_ADD_POINT4D;
  float intensity;
  float ref_intensity;
  float ref_time_0;
  float ref_time_1;
  // float    theta;
  // float    phi;
  std::uint16_t raw_data[16];
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace innovusion_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(innovusion_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(std::uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    innovusion_pointcloud::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(double, timestamp, timestamp)(
        std::uint8_t, intensity, intensity)(std::uint8_t, flags, flags))
#if 0
                                  (std::uint16_t, scan_id, scan_id)
                                  (std::uint16_t, scan_seq, scan_seq)
                                  (std::uint16_t, cluster, cluster)
#endif
POINT_CLOUD_REGISTER_POINT_STRUCT(
    innovusion_pointcloud::PointRaw,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, ref_intensity, ref_intensity)(float, ref_time_0,
                                             ref_time_0)(float, ref_time_1,
                                                         ref_time_1)
    // (float, theta, theta)
    // (float, phi, phi)
    (std::uint16_t, raw_data[0],
     raw_data_0)(std::uint16_t, raw_data[1],
                 raw_data_1)(std::uint16_t, raw_data[2], raw_data_2)(
        std::uint16_t, raw_data[3],
        raw_data_3)(std::uint16_t, raw_data[4],
                    raw_data_4)(std::uint16_t, raw_data[5], raw_data_5)(
        std::uint16_t, raw_data[6],
        raw_data_6)(std::uint16_t, raw_data[7],
                    raw_data_7)(std::uint16_t, raw_data[8], raw_data_8)(
        std::uint16_t, raw_data[9],
        raw_data_9)(std::uint16_t, raw_data[10],
                    raw_data_10)(std::uint16_t, raw_data[11],
                                 raw_data_11)(std::uint16_t, raw_data[12],
                                              raw_data_12))
#endif  // INNO_LIDAR_INNOVUSION_POINTCLOUD_INCLUDE_INNOVUSION_POINTCLOUD_POINT_TYPES_H_
