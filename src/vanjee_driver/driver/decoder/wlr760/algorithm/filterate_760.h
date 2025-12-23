#pragma once
#include <stdint.h>

#ifdef _WIN32
#ifdef BUILDING_DLL
#define FILTER_EXPORT_API_DLL_ __declspec(dllexport)
#else
#define FILTER_EXPORT_API_DLL_ __declspec(dllimport)
#endif
#else
#define FILTER_EXPORT_API_DLL_ __attribute__((visibility("default")))
#endif

namespace Laser760Filter {

#pragma pack(push, 1)
struct ExportPointSizeCn {
  size_t echo1_data_size = 0;
  size_t echo2_data_size = 0;
};
#pragma pack(pop)

#pragma pack(push, 1)

class ExportOnePointInfo {
 public:
  int32_t index = 0;
  int32_t line_id = 0;
  int32_t col_index = 0;
  int8_t mirror = 0;
  float hor_angle = 0;
  float ver_angle = 0;
  float distance = 0;
  float x = 0;
  float y = 0;
  float z = 0;
  uint8_t reflectivity = 0;
  uint16_t low_pulse_width = 0;
  uint16_t tall_pulse_width = 0;
  uint64_t time = 0;
  uint8_t strong_weak_flag = 0;
  uint8_t echo_type = 0;
  uint8_t confidence = 0;
  int32_t hor_azimuth = 0;

  uint16_t motor_speed = 0;
  uint8_t device_model = 0;
  uint32_t distance_mm = 0;
  float cos_ride_sin = 0;
  float cos_rid_cos = 0;
  float sin_rid_thousandth = 0;
  uint8_t ground_flag = 0;
  uint8_t non_filterable = 0;
};
#pragma pack(pop)

extern "C" {

FILTER_EXPORT_API_DLL_ void init_filter();

FILTER_EXPORT_API_DLL_ void filter(int* ptr, int* ptr_pre);

FILTER_EXPORT_API_DLL_ void get_all_filter(const char*** names, int* count);

FILTER_EXPORT_API_DLL_ const char* get_filter_version(int index);

FILTER_EXPORT_API_DLL_ void set_filter_global_enable(bool enable);

FILTER_EXPORT_API_DLL_ bool get_filter_global_enable();

FILTER_EXPORT_API_DLL_ void multi_thread_enable(bool enable);

FILTER_EXPORT_API_DLL_ void set_thread_num(uint32_t thread_num);

FILTER_EXPORT_API_DLL_ uint32_t get_sys_thrand_num();

FILTER_EXPORT_API_DLL_ bool set_filter_enable(int index, bool enable);

FILTER_EXPORT_API_DLL_ bool get_filter_enable(int index);

FILTER_EXPORT_API_DLL_ void set_filter_time_enable(bool enable);

FILTER_EXPORT_API_DLL_ uint32_t get_filter_time(int index);

FILTER_EXPORT_API_DLL_ uint32_t get_filter_avg_time(int index);

FILTER_EXPORT_API_DLL_ int* create_one_point_info_manage();

FILTER_EXPORT_API_DLL_ void delete_one_point_info_manage(int* ptr);

FILTER_EXPORT_API_DLL_ size_t create_one_point_info_index(size_t line, size_t line_id);

FILTER_EXPORT_API_DLL_ void clear_one_point_info_manage(int* ptr);

FILTER_EXPORT_API_DLL_ ExportOnePointInfo* create_one_point_info(int* ptr, uint32_t index, int EchoType, int line, int SrcHAngle, bool& old);

FILTER_EXPORT_API_DLL_ void update_add_point_size(int* ptr, ExportPointSizeCn psize);

FILTER_EXPORT_API_DLL_ size_t get_echo1_data_size(int* ptr);

FILTER_EXPORT_API_DLL_ size_t get_echo2_data_size(int* ptr);

FILTER_EXPORT_API_DLL_ ExportOnePointInfo* get_one_point_info(int* ptr, int EchoType, int line, int SrcHAngle);

FILTER_EXPORT_API_DLL_ size_t get_making_points(ExportOnePointInfo** out_points);
}

}  // namespace Laser760Filter
