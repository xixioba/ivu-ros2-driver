/**
 *  Copyright (C) 2021 - Innovusion Inc.
 *
 *  All Rights Reserved.
 *
 *  $Id$
 */

#include "sdk/params.h"

#include <stdio.h>
#include <stdlib.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>

#include "sdk_common/inno_lidar_packet.h"
#include "utils/inno_lidar_log.h"

namespace innovusion {
class SimpleYAMLParser {
 public:
  SimpleYAMLParser() {
  }

  ~SimpleYAMLParser() {
  }

  bool load_yaml(const char *file) {
    return simple_read_yaml_file(file, &nvp_);
  }

  std::string to_string() {
    std::stringstream ss;
    for (std::map<std::string, std::string>::const_iterator it = nvp_.begin();
         it != nvp_.end();
         ++it) {
      ss << it->first << ": " << it->second << "\n";
    }
    return ss.str();
  }

  void get_param_value_from_map(const char *name, int *value) const {
    if (nvp_.find(name) == nvp_.end()) {
      *value = 0;
    } else {
      *value = atoi(nvp_.find(name)->second.c_str());
    }
  }

  void get_param_value_from_map(const char *name, double *value) const {
    if (nvp_.find(name) == nvp_.end()) {
      *value = 0;
    } else {
      *value = atof(nvp_.find(name)->second.c_str());
    }
  }

  void get_param_value_from_map(const char *name, float *value) const {
    if (nvp_.find(name) == nvp_.end()) {
      *value = 0;
    } else {
      *value = atof(nvp_.find(name)->second.c_str());
    }
  }

 private:
  // trim from left
  static inline std::string& ltrim_line(std::string *s,
                                        const char *t = " \t\n\r\f\v") {
    s->erase(0, s->find_first_not_of(t));
    return *s;
  }

  // trim from right
  inline std::string& rtrim_line(std::string *s,
                                 const char *t = " \t\n\r\f\v") {
    s->erase(s->find_last_not_of(t) + 1);
    return *s;
  }

  // trim from left & right
  inline std::string& trim_line(std::string *s,
                                const char *t = " \t\n\r\f\v") {
    return ltrim_line(&rtrim_line(s, t), t);
  }

  bool simple_read_yaml_file(const char *yaml,
                             std::map<std::string, std::string> *nvp) {
    inno_log_trace("yaml %s", yaml);
    std::istringstream infile(yaml);
    std::string line;
    std::string key;
    int array_mode = 0;
    while (std::getline(infile, line)) {
      inno_log_trace("%s", line.c_str());
      trim_line(&line);
      std::istringstream is_line(line);
      if (line.find(':') != std::string::npos) {
        std::getline(is_line, key, ':');
        // std::cout << "key= " << key << "\n";
        array_mode = 0;
        if (key[0] == '#') {
          continue;
        }
        std::string value;
        if (std::getline(is_line, value)) {
          trim_line(&value);
          if (value == "") {
            // std::cout << "array\n";
            array_mode = 1;
          } else {
            (*nvp)[key] = value;
            // std::cout << key << " = " << value << "\n";
          }
        } else {
          // std::cout << "array\n";
          array_mode = 1;
        }
      } else {
        if (array_mode) {
          std::string value;
          if (line.find('-') != std::string::npos) {
            std::getline(is_line, value, '-');
            if (std::getline(is_line, value)) {
              // std::cout << "value:" << value << "\n";
              trim_line(&value);
              if (value != "") {
                std::stringstream ss;
                ss << array_mode - 1;
                std::string s = ss.str();
                std::string ka = key + "_" + s;
                (*nvp)[ka] = value;
                // std::cout << ka << " = " << value << "\n";
                array_mode++;
              }
            }
          }
        }
      }
    }
    return true;
  }

 private:
  std::map<std::string, std::string> nvp_;
};

#include "sdk/iv_params_code_gen.gen_cc"

int LidarParams::read(const char *filename) {
  std::string file_fullpath = filename;
  {
    std::ifstream fin(file_fullpath.c_str());
    bool file_readable = true;
    if (!fin.is_open()) {
      const char *ppwd = getenv("PWD");
      if (ppwd != NULL) {
        std::string pwd = ppwd;
        file_fullpath = pwd + "/" + filename;
        std::ifstream fin2(file_fullpath.c_str());
        file_readable = fin2.is_open();
        fin2.close();
      }
    }
    if (!file_readable) {
      initialized = false;
      inno_log_error("cannot open YAML file %s", filename);
      return 1;
    }
    fin.close();
  }

  char *yaml;
  FILE *infile = fopen(file_fullpath.c_str(), "r");
  if (infile == NULL) {
    inno_log_error("cannot open YAML file %s", file_fullpath.c_str());
    return 1;
  }
  fseek(infile, 0L, SEEK_END);
  size_t numbytes = ftell(infile);

  fseek(infile, 0L, SEEK_SET);
  yaml = reinterpret_cast<char *>(calloc(numbytes + 1, sizeof(char)));

  if (yaml == NULL) {
    inno_log_error("cannot malloc for %s, %lu",
                   file_fullpath.c_str(), numbytes);
    fclose(infile);
    return 1;
  } else {
    size_t read_so_far = fread(yaml, sizeof(char), numbytes, infile);
    if (read_so_far == 0 || read_so_far != numbytes) {
      inno_log_error("read %s %lu != %lu",
                     file_fullpath.c_str(), numbytes, read_so_far);
      fclose(infile);
      free(yaml);
      return 1;
    } else {
      fclose(infile);
      int ret = parse(yaml);
      free(yaml);
      return ret;
    }
  }
}

int LidarParams::parse(const char *yaml) {
  initialized = false;
  try {
    SimpleYAMLParser p;
    p.load_yaml(yaml);
    get_params_from_simple_parser(p, &iv_params);

    for (int i = 0; i < 3; i++) {
      iv_params.ctr[i] = pow(10, iv_params.ctr[i]/10.0);
    }
    iv_params.tilt_n_x = sin(iv_params.tilt_angle * M_PI / 180.0);
    iv_params.tilt_n_z = cos(iv_params.tilt_angle * M_PI / 180.0);
    iv_params.shift_n_y = sin(iv_params.shift_angle * M_PI / 180.0);
    iv_params.shift_n_z = cos(iv_params.shift_angle * M_PI / 180.0);

    // convert from meter to distance unit
    iv_params.k_dis *= kInnoDistanceUnitPerMeter;
    iv_params.k_dis_2 *= kInnoDistanceUnitPerMeter;
    for (int i = 0; i < 4; i++) {
      iv_params.distance_correction[i] *= kInnoDistanceUnitPerMeter;
      iv_params.distance_correction_2[i] *= kInnoDistanceUnitPerMeter;
    }
  } catch (...) {
    inno_log_error("Invalid YAML file");
    return !initialized;
  }

  // check if important params are set
  if (is_invalid()) {
    inno_log_error("Invalid YAML file %s", yaml);
  } else {
    initialized = true;
    set_default_for_missing();
    inno_log_info("Use YAML file init=%d", initialized);
    inno_log_info("YAML file content:\n%s", to_string().c_str());
  }
  return !initialized;
}

bool LidarParams::is_invalid() {
  return (iv_params.p_offset == 0 &&
          iv_params.g_center_angle == 0 &&
          iv_params.g_scan_range == 0);
}

std::string LidarParams::to_string() {
  std::ostringstream ss;
  ss << iv_params;
  std::string s = ss.str();
  return s;
}

void LidarParams::set_default() {
  memset(&iv_params, 0, sizeof(iv_params));
  for (int i = 0; i < 6; i++) {
    iv_params.t_ifactor[i] = 1.0;
    iv_params.b_ifactor[i] = 1.0;
  }
  iv_params.tilt_n_z = 1.0;
  iv_params.shift_n_z = 1.0;
  iv_params.fov_top_half_p_angle = 34;
  iv_params.fov_bottom_half_p_angle = 34.5;
}

void LidarParams::set_default_for_missing() {
  // to be compatible with old yaml file which does not have
  // these parameters defined.
  for (int i = 0; i < 6; i++) {
    if (iv_params.t_ifactor[i] == 0) {
      iv_params.t_ifactor[i] = 1.0;
    }
    if (iv_params.b_ifactor[i] == 0) {
      iv_params.b_ifactor[i] = 1.0;
    }
    if (iv_params.b_shift_1[i] == 0) {
      iv_params.b_shift_1[i] = iv_params.b_shift[i];
    }
    if (iv_params.t_temp_vbr0 == 0) {
      iv_params.t_temp_vbr0 = iv_params.temp_vbr0;
    }
    if (iv_params.b_temp_vbr0 == 0) {
      iv_params.b_temp_vbr0 = iv_params.temp_vbr0;
    }
  }
  if (iv_params.t_refl_factor == 0) {
    iv_params.t_refl_factor = 1.0;
  }
  if (iv_params.b_refl_factor == 0) {
    iv_params.b_refl_factor = 1.0;
  }
  for (int i = 0; i < 4; i++) {
    if (iv_params.refl_factor[i] == 0) {
      iv_params.refl_factor[i] = 1.0;
    }
  }
  if (iv_params.n_b == 0) {
    iv_params.n_b = 4;
  }
  if (iv_params.n_p == 0) {
    iv_params.n_p = 5;
  }
  if (iv_params.g_scan_range == 0) {
    iv_params.g_scan_range = 20.0;
  }
  if (iv_params.g_center_angle == 0) {
    iv_params.g_center_angle = 64;
  }
  if (iv_params.ctr[0] == 1.0) {
    iv_params.ctr[0] = pow(10, 45/10.0);
  }
  if (iv_params.ctr[1] == 1.0) {
    iv_params.ctr[1] = pow(10, 50/10.0);
  }
  if (iv_params.ctr[2] == 1.0) {
    iv_params.ctr[2] = pow(10, 50/10.0);
  }
  if (iv_params.mcti == 0) {
    iv_params.mcti = 8000;
  }
  if (iv_params.retro_intensity == 0) {
    iv_params.retro_intensity = 12000;
  }
  if (iv_params.retro_intensity_2 == 0) {
    iv_params.retro_intensity_2 = 12000;
  }
  if (iv_params.dist_corr_transition_high == 0) {
    for (int i = 0; i < 4; i++) {
      iv_params.distance_correction_2[i] = iv_params.distance_correction[i];
    }
  }
  if (iv_params.k_dis_2 == 0) {
    iv_params.k_dis_2 = 75.0 * kInnoDistanceUnitPerMeter;
  }
  if (iv_params.k_ref_2 == 0) {
    iv_params.k_ref_2 = 2.0;
  }
  if (iv_params.aperture_offset == 0) {
    iv_params.aperture_offset = 80;
  }
  if (iv_params.aperture_offset_3 == 0) {
    iv_params.aperture_offset_3 = 80;
  }
  if (iv_params.fov_top_half_p_angle == 0) {
    iv_params.fov_top_half_p_angle = 34;
  }
  if (iv_params.fov_bottom_half_p_angle == 0) {
    iv_params.fov_bottom_half_p_angle = 34.5;
  }
}

#define SET_PARAM(field) \
  iv_params.field = src_params->iv_params.field;

#define SET_PARAM_3(field, count) \
  for (int i = 0; i < count; i++) { \
    iv_params.field[i] = src_params->iv_params.field[i]; \
  }

#define COMPARE_PARAM(field) \
  (iv_params.field != src_params->iv_params.field)

bool LidarParams::copy_from_src(LidarParams *src_params) {
  bool change_table =
    COMPARE_PARAM(aperture_offset) || COMPARE_PARAM(aperture_offset_2) ||
    COMPARE_PARAM(aperture_offset_3);
  // no memcpy here because we don't want to copy large tables
  // memcpy(&iv_params, &src_params->iv_params, sizeof(IvParams));
  SET_PARAM(p_offset);
  SET_PARAM(p_off_center);
  SET_PARAM(g_scan_range);
  SET_PARAM(g_center_angle);
  SET_PARAM(g_tilt);
  SET_PARAM(g_tilt2);
  SET_PARAM(tilt_angle);
  SET_PARAM(shift_angle);
  SET_PARAM(n_p);
  SET_PARAM(retro_intensity);
  SET_PARAM(retro_intensity_2);
  SET_PARAM_3(ctr, 3);
  SET_PARAM(mcti);
  SET_PARAM_3(v_adjustment, 4);
  SET_PARAM_3(ho_adjustment, 4);
  SET_PARAM_3(f_alpha, 4);
  SET_PARAM_3(f_gamma, 4);
  SET_PARAM_3(f_int_vbr0, 4);
  SET_PARAM_3(p_tilt, 7);
  SET_PARAM_3(p_shift, 7);
  SET_PARAM_3(distance_correction, 4);
  SET_PARAM_3(distance_correction_2, 4);
  SET_PARAM_3(reference_time, 4);
  SET_PARAM_3(refl_factor, 4);
  SET_PARAM(dist_corr_transition_intensity);
  SET_PARAM(dist_corr_transition_low);
  SET_PARAM(dist_corr_transition_high);
  SET_PARAM(dist_corr_max_intensity);
  SET_PARAM(aperture_offset);
  SET_PARAM(aperture_offset_2);
  SET_PARAM(aperture_offset_3);
  SET_PARAM(k_dis);
  SET_PARAM(k_int);
  SET_PARAM(k_ref);
  SET_PARAM(k_dis_2);
  SET_PARAM(k_ref_2);
  SET_PARAM(fov_top_half_p_angle);
  SET_PARAM(fov_bottom_half_p_angle);

  for (int i = 0; i < 3; i++) {
    iv_params.ctr[i] = pow(10, iv_params.ctr[i]/10.0);
  }
  iv_params.tilt_n_x = sin(iv_params.tilt_angle * M_PI / 180.0);
  iv_params.tilt_n_z = cos(iv_params.tilt_angle * M_PI / 180.0);
  iv_params.shift_n_y = sin(iv_params.shift_angle * M_PI / 180.0);
  iv_params.shift_n_z = cos(iv_params.shift_angle * M_PI / 180.0);

  if (iv_params.dist_corr_transition_high == 0) {
    for (int i = 0; i < 4; i++) {
      iv_params.distance_correction_2[i] = iv_params.distance_correction[i];
    }
  }

  // convert from meter to distance unit
  iv_params.k_dis *= kInnoDistanceUnitPerMeter;
  iv_params.k_dis_2 *= kInnoDistanceUnitPerMeter;
  for (int i = 0; i < 4; i++) {
    iv_params.distance_correction[i] *= kInnoDistanceUnitPerMeter;
    iv_params.distance_correction_2[i] *= kInnoDistanceUnitPerMeter;
  }

  inc_version();
  inno_log_info("IvParams version: %" PRI_SIZEU, get_version());
  return change_table;
}
}  // namespace innovusion

#ifdef TEST_ME
#ifndef inno_log_info
#define inno_log_info()
#endif
#ifndef inno_log_error
#define inno_log_error()
#endif
int main() {
  innovusion::LidarParams l("abc");
  return 0;
}
#endif
