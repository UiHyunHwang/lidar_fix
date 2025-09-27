#include "lidar_fix/lidar_fix_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

using namespace lidar_fix_params; 
const float NaN = std::numeric_limits<float>::quiet_NaN();

// (x,y) cell key
struct CellKey {
  int i;
  int j;
  bool operator==(const CellKey& o) const { return (i == o.i) && (j == o.j); }
};

// hash function for (x,y) key
struct CellKeyHash {
  std::size_t operator()(const CellKey& k) const noexcept {
    std::uint64_t x = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.i)) << 32)
                    ^ (static_cast<std::uint32_t>(k.j));
    // 64-bit mix
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    x = x ^ (x >> 31);
    return static_cast<std::size_t>(x);
  }
};

// cell indexing function
static inline CellKey cell_of(float x, float y) {
  return CellKey{ static_cast<int>(std::floor(x / kXYRes)),
                  static_cast<int>(std::floor(y / kXYRes)) };
}
// bin indexing function
static inline int k_of(float z, float grnd_z) {
  const float skip_low  = grnd_z - kSkipZone; 
  const float skip_high = grnd_z + kSkipZone; 
  if (z < skip_low) {
        // 지면보다 아래
        return static_cast<int>(std::floor((z - kZMin) / kZBin));
    } else if (z > skip_high) {
        // 지면보다 위: bin 인덱스를 건너뛰어야 함
        int raw = static_cast<int>(std::floor((z - kZMin) / kZBin));
        int skip_bins = static_cast<int>(std::ceil((skip_high - skip_low) / kZBin));
        return raw + skip_bins; // gap 만큼 인덱스를 밀어줌
    } else {
        // skip zone: 이 경우 bin을 만들지 않음
        return -1;
    }
}

// +++++++++++++++++++++++++++++++++++++++
// Added for testing, csv dump utility
// +++++++++++++++++++++++++++++++++++++++
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstdlib>

namespace {
  namespace fs = std::filesystem;
  static int g_frame_idx = 0; // frame counter

  static inline std::string output_dir() {
    const char* home = std::getenv("HOME");
    std::string base = (home ? std::string(home) : std::string("")) + "/lidar_fix_2_ws/src/lidar_fix";
    return base;
  }

  static inline std::string make_filename(int frame_idx) {
    using clock = std::chrono::system_clock;
    auto now = clock::now();
    auto t   = clock::to_time_t(now);
    auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    std::tm tm{};
    localtime_r(&t, &tm);

    std::ostringstream oss;
    oss << "zhist_"
        << std::setfill('0') << std::setw(4) << (tm.tm_year + 1900)
        << std::setw(2) << (tm.tm_mon + 1)
        << std::setw(2) << tm.tm_mday << "_"
        << std::setw(2) << tm.tm_hour
        << std::setw(2) << tm.tm_min
        << std::setw(2) << tm.tm_sec
        << "_" << frame_idx
        << "_" << std::setw(3) << ms
        << ".csv";
    return oss.str();
  }

  // z_ground에 해당하는 bin(ks_ground)에는 CSV에 -1000을 기록
  static void dump_zhist_csv_with_ground(
    const std::unordered_map<CellKey, std::vector<uint16_t>, CellKeyHash>& zhist,
    int ks_ground,
    int frame_idx)
  {
    const std::string dir = output_dir();
    fs::create_directories(dir);

    const std::string path = dir + "/" + make_filename(frame_idx);
    std::ofstream ofs(path, std::ios::out);
    if (!ofs.is_open()) return;

    // 헤더: i,j,bin0,bin1,...,bin{N-1}
    ofs << "i,j";
    for (int k = 0; k < kNBins; ++k) ofs << ",bin" << k;
    ofs << "\n";

    // 데이터 (행 수가 300을 초과하지 않도록 제한)
    int rows_written = 0;
    for (const auto& kv : zhist) {
      if (rows_written >= 8000) break; // <-- 300행 제한

      const CellKey& key = kv.first;
      const std::vector<uint16_t>& hist = kv.second;

      ofs << key.i << "," << key.j;
      for (int k = 0; k < kNBins; ++k) {
        if (k == ks_ground) {
          ofs << "," << -1000;               // <-- z_ground에서만 -1000을 기록
        } else {
          ofs << "," << static_cast<unsigned int>(hist[k]);
        }
      }
      ofs << "\n";
      ++rows_written;
    }
    ofs.close();
  }
} 
//++++++++++++++++++++++++++++++++++++++++++++++++++++


// LidarFix Node
LidarFix::LidarFix() : rclcpp::Node("lidar_fix_node")
{
  this->declare_parameter("topic_in", "/ouster/points");
  this->declare_parameter("topic_out", "/ouster/points/fix");

  const std::string topic_in  = this->get_parameter("topic_in").as_string();
  const std::string topic_out = this->get_parameter("topic_out").as_string();
  
  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_in, rclcpp::SensorDataQoS(),
      std::bind(&LidarFix::lidar_callback, this, std::placeholders::_1));
  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_out, 10);
}

//===================================================
// Callback function
//===================================================
void LidarFix::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // estimate lidar z from the ground (update for each frame)
  const float grnd_z = estimate_ground_z(msg); 

  // create copy
  auto modified_msg = *msg;

  // z-histogram formation for each (x,y) cell
  std::unordered_map<CellKey, std::vector<uint16_t>, CellKeyHash> zhist;
  zhist.reserve(4096);

  {
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(z)) continue;
      if (z < kZMin || z > kZMax) continue; 

      const CellKey key = cell_of(x, y);
      auto it = zhist.find(key); // zhist에서 cell index에 맞는 데이터 찾기 (hash function 사용) (it->first: CellKey, it->second: zhist 벡터 )
      if (it == zhist.end()) { // cell index에 맞는 zhist가 없다면 : 새로 만들어서 
        it = zhist.emplace(key, std::vector<uint16_t>(kNBins, 0)).first;
      }
      if(k_of(z, grnd_z) > 0 && k_of(z, grnd_z) < kNBins) {
        it->second[k_of(z, grnd_z)]++; // 해당 zhist 내에서 z index에 pcl 수 추가하기`
      }
    }
  }
  /*
  if (g_frame_idx == 100) {
    const int ks_ground = k_of(grnd_z);
    dump_zhist_csv_with_ground(zhist, ks_ground, g_frame_idx);
  }
  g_frame_idx++;*/

  // Check the existence of symmetric point if z < grnd_z
  sensor_msgs::PointCloud2Iterator<float> iter_x(modified_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(modified_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(modified_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
    if (!std::isfinite(*iter_z)) {
		  if (kDropInvalidOrUnmirrored) { *iter_x = NaN; *iter_y = NaN; *iter_z = NaN; }
    	continue;
    }
    
    if (*iter_z < grnd_z)  // Check if z < grnd_z
    {
      const CellKey key = cell_of(*iter_x, *iter_y);
      const float z_symm = 2.f * grnd_z - (*iter_z);  // symmetric point height: z*

      bool mirrored = false; // indicator
      auto it = zhist.find(key);
      if (it != zhist.end()) {
        const auto& hist = it->second; 
        const int ks = k_of(z_symm, grnd_z); // z*의 bin index
        const int s  = ks - kTolBins; //search 범위 하한
        const int e  = ks + kTolBins; //search 범위 상한
        for (int k = s; k <= e; ++k) { // z* 근처 bin들에 실제 점이 있는지 확인
          if (hist[k] >= 1) { // bin이 존재함
			      mirrored = true; 
			      break; 
		      }
		    }
      }

      if (mirrored) { // If symmetric point exists -> fix (x, y, z)
        const float ratio = grnd_z / (*iter_z);
        *iter_x *= ratio;
        *iter_y *= ratio;
        *iter_z = grnd_z;
      }
    }
  }

  // publish modified msg
  pub_->publish(modified_msg);
}

// ============================
// Estimate ground z
// ============================
float estimate_ground_z(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                        float bin_sz, float z_min, float z_max)
{
  const float d2_min = kD2Min;
  const float d2_max = kD2Max;
  int nbins = static_cast<int>((z_max - z_min) / bin_sz) + 1;
  if (nbins < 3) nbins = 3;

  std::vector<float> hist(nbins, 0.f);
  std::vector<int>   cnt (nbins, 0);

  // x, y, z iteraters
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    const float z = *iter_z;
    if (!std::isfinite(z)) continue;
    if (z < z_min || z > z_max) continue;

    const float x  = *iter_x;
    const float y  = *iter_y;
    const float d2 = x*x + y*y;
    if (d2 < d2_min || d2 > d2_max) continue;

    int idx = static_cast<int>((z - z_min) / bin_sz);
    if (idx < 0) idx = 0;
    if (idx >= nbins) idx = nbins - 1;

    hist[idx] += z;
    cnt [idx] += 1;
  }

  // find max hist bin
  int max_idx = -1;
  int max_cnt = -1;
  for (int i = 1; i < nbins - 1; ++i) {
    if (cnt[i] > max_cnt) {
      max_cnt = cnt[i];
      max_idx = i;
    }
  }
  if (max_idx < 0 || max_cnt <= 0) return 0.f;

  // Compute peak z with average of 3 bins 
  float ground_z = (hist[max_idx] + hist[max_idx - 1] + hist[max_idx + 1]) / (cnt[max_idx] + cnt[max_idx - 1] + cnt[max_idx + 1]);

  // Verification
  int si1 = max_idx, ei1 = max_idx;
  while (si1 > 1 && (cnt[si1 - 1] <= cnt[si1])) si1--;
  while (ei1 < nbins - 1 && (cnt[ei1 + 1] <= cnt[ei1])) ei1++;

  // Secondary peak (left side)
  int max_idx2 = -1, max_cnt2 = -1;
  for (int i = 1; i < si1; ++i) {
    if (cnt[i] > max_cnt2) {
      max_cnt2 = cnt[i];
      max_idx2 = i;
    }
  }
  if (max_idx2 < 0) return ground_z;

  int si2 = max_idx2, ei2 = max_idx2;
  while (si2 > 1 && (cnt[si2 - 1] <= cnt[si2])) si2--;
  while (ei2 < si1 && (cnt[ei2 + 1] <= cnt[ei2])) ei2++;

  // Compare volume
  int vol1 = 0; for (int i = si1; i <= ei1; ++i) vol1 += cnt[i];
  int vol2 = 0; for (int i = si2; i <= ei2; ++i) vol2 += cnt[i];

  if (vol1 < vol2) {// If secondary peak has larger volume, compute ground_z with the peak
	  max_idx = max_idx2;
	  ground_z = (hist[max_idx] + hist[max_idx - 1] + hist[max_idx + 1]) / (cnt[max_idx] + cnt[max_idx - 1] + cnt[max_idx + 1]);
  }
  return ground_z;
}
