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

// (x,y) → cell index
CellKey LidarFix::cell_of(float x, float y) {
  return CellKey{ static_cast<int>(std::floor(x / kXYRes)),
                  static_cast<int>(std::floor(y / kXYRes)) };
}

// z → bin index (grnd_z 기준 위/아래 분리; skip zone 영역은 skip)
int LidarFix::k_of(float z, float grnd_z) {
  if (z < kZMin || z > kZMax) return -1;

  const float skip_low  = grnd_z - kSkipZone;
  const float skip_high = grnd_z + kSkipZone;

  if (z > skip_high) {
    const int raw = static_cast<int>(std::floor((z - kZMin) / kZBin));
    return raw + 1; // simple 1-bin gap
  } else if (z < skip_low) {
    return static_cast<int>(std::floor((z - kZMin) / kZBin));
  } else {
    return -1; // skip zone
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

  // hash container tuning
  zhist_.max_load_factor(0.7f);
  zhist_.rehash(kInitBuckets);

  below_map_.max_load_factor(0.7f);
  below_map_.rehash(kInitBuckets);
}

//===================================================
// Callback function (optimized: raw access + fewer per-point costs)
//===================================================
void LidarFix::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 프레임 시작: 컨테이너 초기화
  zhist_.clear();
  below_map_.clear();

  // 프레임별 지면 z 추정
  const float grnd_z = estimate_ground_z(msg);

  // 수정본 메시지
  auto modified_msg = *msg;

  // ---------- 공통 오프셋/스텝 & 상수 사전 계산 ----------
  // (1) 원본(msg)에서 읽기 위한 오프셋
  const uint32_t src_step = msg->point_step;
  size_t src_off_x = 0, src_off_y = 0, src_off_z = 0;
  for (const auto& f : msg->fields) {
    if (f.name == "x") src_off_x = f.offset;
    else if (f.name == "y") src_off_y = f.offset;
    else if (f.name == "z") src_off_z = f.offset;
  }
  const uint8_t* src_base = msg->data.data();
  const size_t src_size = msg->data.size();

  // (2) 수정 대상(modified_msg) 쓰기 위한 오프셋
  const uint32_t dst_step = modified_msg.point_step;
  size_t dst_off_x = 0, dst_off_y = 0, dst_off_z = 0;
  for (const auto& f : modified_msg.fields) {
    if (f.name == "x") dst_off_x = f.offset;
    else if (f.name == "y") dst_off_y = f.offset;
    else if (f.name == "z") dst_off_z = f.offset;
  }
  uint8_t* dst_base = modified_msg.data.data();
  const size_t dst_size = modified_msg.data.size();

  // (3) k_of에서 쓰는 값들 프레임당 1회만 계산 (분기/연산 감소)
  const float skip_low  = grnd_z - kSkipZone;
  const float skip_high = grnd_z + kSkipZone;
  const float inv_kZBin = 1.0f / kZBin;

  auto fast_k_of = [&](float z) -> int {
    if (z < kZMin || z > kZMax) return -1;
    if (z > skip_high) {
      const int raw = static_cast<int>(std::floor((z - kZMin) * inv_kZBin));
      return raw + 1;
    } else if (z < skip_low) {
      return static_cast<int>(std::floor((z - kZMin) * inv_kZBin));
    } else {
      return -1;
    }
  };

  // ---------- 1패스: 히스토그램 구축 + 하부점 인덱스 수집 (raw) ----------
  {
    uint32_t pidx = 0;
    for (size_t off = 0; off + src_step <= src_size; off += src_step, ++pidx) {
      const float z = *reinterpret_cast<const float*>(src_base + off + src_off_z);
      if (!std::isfinite(z)) continue; // invalid는 패스 (2패스에서 NaN 처리함)

      const int k = fast_k_of(z);
      if (k < 0 || k >= kNBins) continue;

      const float x = *reinterpret_cast<const float*>(src_base + off + src_off_x);
      const float y = *reinterpret_cast<const float*>(src_base + off + src_off_y);

      const CellKey key = cell_of(x, y);

      auto it = zhist_.find(key);
      if (it == zhist_.end()) {
        it = zhist_.try_emplace(key, std::vector<uint16_t>(kNBins, 0)).first;
      }
      it->second[k]++;

      if (z < grnd_z) {
        below_map_[key].push_back(pidx);
      }
    }
  }

  // ---------- (옵션 유지) 비유한 점 NaN 처리: raw로 빠르게 ----------
  {
    for (size_t off = 0; off + src_step <= src_size && off + dst_step <= dst_size; off += src_step) {
      const float z = *reinterpret_cast<const float*>(src_base + off + src_off_z);
      if (!std::isfinite(z)) {
        float* dx = reinterpret_cast<float*>(dst_base + off + dst_off_x);
        float* dy = reinterpret_cast<float*>(dst_base + off + dst_off_y);
        float* dz = reinterpret_cast<float*>(dst_base + off + dst_off_z);
        *dx = NaN; *dy = NaN; *dz = NaN;
      }
    }
  }

  // ---------- 2패스: 하부점만 셀 단위로 처리(맵 재탐색 최소화, raw) ----------
  {
    for (auto& kv : below_map_) {
      const CellKey& key = kv.first;
      const std::vector<uint32_t>& idxs = kv.second;

      // 이 셀의 히스토그램 (셀당 1회만 조회)
      auto hit = zhist_.find(key);
      if (hit == zhist_.end()) continue;
      const auto& hist = hit->second;

      // 하부점들 보정
      for (uint32_t idx : idxs) {
        const size_t off_src = static_cast<size_t>(idx) * src_step;
        const size_t off_dst = static_cast<size_t>(idx) * dst_step;
        if (off_src + src_step > src_size || off_dst + dst_step > dst_size) continue;

        float* px = reinterpret_cast<float*>(dst_base + off_dst + dst_off_x);
        float* py = reinterpret_cast<float*>(dst_base + off_dst + dst_off_y);
        float* pz = reinterpret_cast<float*>(dst_base + off_dst + dst_off_z);

        // (주의) 위에서 invalid는 이미 NaN 세팅함. 여기선 valid만 들어옴.
        if (!std::isfinite(*pz)) continue;

        const float z_val = *pz;
        if (!(z_val < grnd_z)) continue; // 안전장치

        const float z_symm = 2.f * grnd_z - z_val;
        const int   ks     = fast_k_of(z_symm);

        bool mirrored = false;
        if (ks >= 0 && ks < kNBins) {
          const int s = std::max(0, ks - kTolBins);
          const int e = std::min(kNBins - 1, ks + kTolBins);
          for (int k = s; k <= e; ++k) {
            if (hist[k] >= 1) { mirrored = true; break; }
          }
        }

        if (mirrored) {
          const float ratio = grnd_z / z_val;
          *px *= ratio;
          *py *= ratio;
          *pz = grnd_z;
        }
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
// Estimate ground z (original)
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

  int max_idx = -1, max_cnt = -1;
  for (int i = 1; i < nbins - 1; ++i) {
    if (cnt[i] > max_cnt) { max_cnt = cnt[i]; max_idx = i; }
  }
  if (max_idx < 0 || max_cnt <= 0) return 0.f;

  float ground_z = (hist[max_idx] + hist[max_idx - 1] + hist[max_idx + 1]) /
                   (cnt [max_idx] + cnt [max_idx - 1] + cnt [max_idx + 1]);

  int si1 = max_idx, ei1 = max_idx;
  while (si1 > 1 && (cnt[si1 - 1] <= cnt[si1])) si1--;
  while (ei1 < nbins - 1 && (cnt[ei1 + 1] <= cnt[ei1])) ei1++;

  int max_idx2 = -1, max_cnt2 = -1;
  for (int i = 1; i < si1; ++i) {
    if (cnt[i] > max_cnt2) { max_cnt2 = cnt[i]; max_idx2 = i; }
  }
  if (max_idx2 < 0) return ground_z;

  int si2 = max_idx2, ei2 = max_idx2;
  while (si2 > 1 && (cnt[si2 - 1] <= cnt[si2])) si2--;
  while (ei2 < si1 && (cnt[ei2 + 1] <= cnt[ei2])) ei2++;

  int vol1 = 0; for (int i = si1; i <= ei1; ++i) vol1 += cnt[i];
  int vol2 = 0; for (int i = si2; i <= ei2; ++i) vol2 += cnt[i];

  if (vol1 < vol2) {
    int m = max_idx2;
    ground_z = (hist[m] + hist[m - 1] + hist[m + 1]) /
               (cnt [m] + cnt [m - 1] + cnt [m + 1]);
  }
  return ground_z;
}
