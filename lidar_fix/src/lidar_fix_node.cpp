#include "lidar_fix/lidar_fix_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <limits>

using namespace lidar_fix_params; 

namespace {
  const float NaN = std::numeric_limits<float>::quiet_NaN();
  static int g_frame_idx = 0;
}

// cell indexing function
inline CellKey LidarFix::cell_of(float x, float y) {
  return CellKey{ static_cast<int>(std::floor(x / kXYRes)),
                  static_cast<int>(std::floor(y / kXYRes)) };
}
// bin indexing function, skipt the bin around ground_z
int LidarFix::k_of(float z, float grnd_z) {
  if (z < kZMin || z > kZMax) return -1;

  const float skip_low  = grnd_z - kSkipZone; 
  const float skip_high = grnd_z + kSkipZone; 

  if (z > skip_high) {    
    // 지면보다 위: bin 인덱스를 건너뛰어야 함
      const int raw = static_cast<int>(std::floor((z - kZMin) / kZBin));
      return raw + 1;     // gap에 해당하는 index를 건너뜀
  } else if (z < skip_low) {           
    // 지면보다 아래
    return static_cast<int>(std::floor((z - kZMin) / kZBin));
  } else {                
    // skip zone: 무시함
    return -1;
  }
}


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

  zhist_.max_load_factor(0.7f);
  zhist_.rehash(kInitBuckets); // kInitBuckets = 8192
}

//===================================================
// Callback function
//===================================================
void LidarFix::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // Start the frame by clearing out the elements of bucket
  zhist_.clear(); 

  // estimate lidar z from the ground (update for each frame)
  const float grnd_z = estimate_ground_z(msg); 

  // create copy
  auto modified_msg = *msg;

  {
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const float x = *ix, y = *iy, z = *iz;
      if (!std::isfinite(z)) continue;

      const int k = k_of(z, grnd_z);
      if (k < 0 || k  >= kNBins) continue;

      const CellKey key = cell_of(x, y);
      auto it = zhist_.find(key); // zhist에서 cell index에 맞는 데이터 찾기 (hash function 사용) (it->first: CellKey, it->second: zhist 벡터 )
      if (it == zhist_.end()) { // cell index에 맞는 zhist가 없다면 : 새로 만들어서 
        it = zhist_.emplace(key, std::vector<uint16_t>(kNBins, 0)).first;
      }
      it->second[k]++; // 해당 zhist 내에서 z index에 pcl 수 추가하기`
    }
  }

  // Check the existence of symmetric point if z < grnd_z
  sensor_msgs::PointCloud2Iterator<float> iter_x(modified_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(modified_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(modified_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
    if (!std::isfinite(*iter_z)) {
		  *iter_x = NaN; *iter_y = NaN; *iter_z = NaN;
    	continue;
    }
    
    if (*iter_z < grnd_z)  // Check if z < grnd_z
    {
      const CellKey key = cell_of(*iter_x, *iter_y);
      const float z_symm = 2.f * grnd_z - (*iter_z);  // symmetric point height: z*

      bool mirrored = false; // indicator
      auto it = zhist_.find(key);
      if (it != zhist_.end()) {
        const auto& hist = it->second; 
        const int ks = k_of(z_symm, grnd_z); // z*의 bin index
        if (ks >= 0 && ks < kNBins) {
          const int s = std::max(0, ks - kTolBins);
          const int e = std::min(kNBins - 1, ks + kTolBins);
          for (int k = s; k <= e; ++k) {
            if (hist[k] >= 1) { mirrored = true; break; }
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

  if (g_frame_idx == 100) {
    RCLCPP_INFO(this->get_logger(), "Frame %d: total cells = %zu",
              g_frame_idx, zhist_.size());
  }
  ++g_frame_idx;
}

// ============================
// Estimate ground z
// ============================
float LidarFix::estimate_ground_z(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
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
