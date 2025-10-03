#include "lidar_fix/lidar_fix_node.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <limits>

using namespace lidar_fix_params;

namespace {
  const float NaN = std::numeric_limits<float>::quiet_NaN();
}

// --------------------------------------------------
// LidarFix Nodel
// --------------------------------------------------
LidarFix::LidarFix() : rclcpp::Node("lidar_fix_node")
{
  this->declare_parameter<std::string>("topic_in",  "/ouster/points");
  this->declare_parameter<std::string>("topic_out", "/ouster/points/fix");

  const auto topic_in  = this->get_parameter("topic_in").as_string();
  const auto topic_out = this->get_parameter("topic_out").as_string();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    topic_in, rclcpp::SensorDataQoS(),
    std::bind(&LidarFix::lidar_callback, this, std::placeholders::_1));

  pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_out, 10);
}

// --------------------------------------------------
// helpers: azimuth/elevation → (ix, iy)
// --------------------------------------------------
int LidarFix::az_to_ix(float az) const {
  // az ∈ [-pi, pi] → [0, kCols-1]
  float u = (az + static_cast<float>(M_PI)) / (2.0f * static_cast<float>(M_PI));
  int ix = static_cast<int>(std::floor(u * static_cast<float>(kCols)));
  if (ix < 0) ix = 0;
  if (ix >= kCols) ix = kCols - 1;
  return ix;
}
int LidarFix::el_to_iy(float el) const {
  // el ∈ [elMin, elMax] → [0, kRings-1]
  float u = (el - elMin) / (elMax - elMin);
  int iy = static_cast<int>(std::round(u * static_cast<float>(kRings - 1)));
  if (iy < 0) iy = 0;
  if (iy >= kRings) iy = kRings - 1;
  return iy;
}

// --------------------------------------------------
// range image formulation (fixed size: 64 x 1024)
// --------------------------------------------------
void LidarFix::init_index_and_radius(const sensor_msgs::msg::PointCloud2& cloud,
                                     cv::Mat& index,
                                     std::vector<float>& r_h,
                                     std::vector<float>& az,
                                     std::vector<float>& el)
{
  const int H = kRings, W = kCols; // (64, 1024)
  index = cv::Mat(H, W, CV_32SC1, cv::Scalar(-1));

  const size_t N = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
  r_h.assign(N, -1.f);
  az .assign(N,  0.f);
  el .assign(N,  0.f);

  // field offsets
  size_t off_x = 0, off_y = 0, off_z = 0;
  for (const auto& f : cloud.fields) {
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
  }
  const uint8_t* base = cloud.data.data();
  const uint32_t step = cloud.point_step;
  const size_t   all  = cloud.data.size();

  size_t i = 0;
  for (size_t off = 0; off + step <= all; off += step, ++i) {
    const float x = *reinterpret_cast<const float*>(base + off + off_x);
    const float y = *reinterpret_cast<const float*>(base + off + off_y);
    const float z = *reinterpret_cast<const float*>(base + off + off_z);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) continue;

    const float rh  = std::hypot(x, y);
    const float azv = std::atan2(y, x);   // [-pi, pi]
    const float elv = std::atan2(z, rh);  // elevation

    if (i < r_h.size()) {
      r_h[i] = rh;
      az [i] = pan;
      el [i] = elv;
    }

    // 수직 FOV 밖은 스킵
    if (elv < elMin || elv > elMax) continue;

    int iy = static_cast<int>(i / static_cast<size_t>(W));
    if (iy < 0 || iy >= H) continue;

    float u = (1.0f - (pan / static_cast<float>(M_PI))) * 0.5f; 
    int ix = static_cast<int>(std::floor(u * static_cast<float>(W)));
    if (ix < 0)   ix = 0;
    if (ix >= W)  ix = W - 1;

    index.at<int>(iy, ix) = static_cast<int>(i);
  }
}

// --------------------------------------------------
// main callback: symmetric-point check + restore
// --------------------------------------------------
void LidarFix::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // 0) ground z 추정
  const float ground_z = estimate_ground_z(msg, kGrndBinSz, kGrndZMin, kGrndZMax);

  // 1) range image (index, r_h, az, el)
  cv::Mat index;
  std::vector<float> r_h, az, el;
  init_index_and_radius(*msg, index, r_h, az, el);

  // 2) 출력 메시지 준비(원본 복사)
  auto out = *msg;

  // field offsets
  const uint32_t step = out.point_step;
  size_t off_x = 0, off_y = 0, off_z = 0;
  for (const auto& f : out.fields) {
    if      (f.name == "x") off_x = f.offset;
    else if (f.name == "y") off_y = f.offset;
    else if (f.name == "z") off_z = f.offset;
  }

  uint8_t*       db  = out.data.data();
  const uint8_t* sb  = msg->data.data();
  const size_t   all = out.data.size();

  // skip zone around ground
  const float skip_low  = ground_z - kSkipZone;
  const float skip_high = ground_z + kSkipZone;

  auto dz_thr = [&](float r){ return kDzBase + kDzPerMeter * r; };
  auto dr_thr = [&](float r){ return kDrBase + kDrPerMeter * r; };

  // 3) 전체 포인트 순회
  size_t i = 0;
  for (size_t off = 0; off + step <= all; off += step, ++i) {
    float& X = *reinterpret_cast<float*>(db + off + off_x);
    float& Y = *reinterpret_cast<float*>(db + off + off_y);
    float& Z = *reinterpret_cast<float*>(db + off + off_z);

    const float x = *reinterpret_cast<const float*>(sb + off + off_x);
    const float y = *reinterpret_cast<const float*>(sb + off + off_y);
    const float z = *reinterpret_cast<const float*>(sb + off + off_z);

    // invalid → NaN
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      X = Y = Z = NaN; continue;
    }

    // skip zone 안/위는 그대로 통과
    if (!(z < skip_low)) { X = x; Y = y; Z = z; continue; }

    // 현재 포인트의 기본량
    const float rh  = (i < r_h.size()) ? r_h[i] : -1.f;
    const float azv = (i < az.size())  ? az[i]  :  0.f;
    if (rh < 0.f) { X = x; Y = y; Z = z; continue; } // range image 매핑 실패

    // 목표 대칭 z
    const float z_sym = 2.f * ground_z - z;

    // 목표 고도각/인덱스
    const float el_sym = std::atan2(z_sym, rh);
    int ix = az_to_ix(azv);
    int iy = el_to_iy(el_sym);

    // 3×3 근방 탐색
    const int xs = std::max(0, ix - kAzTol);
    const int xe = std::min(kCols - 1, ix + kAzTol);
    const int ys = std::max(0, iy - kRingTol);
    const int ye = std::min(kRings - 1, iy + kRingTol);

    bool mirrored = false;
    float bestCost = 1e9f;

    for (int cx = xs; cx <= xe; ++cx) {
      for (int cy = ys; cy <= ye; ++cy) {
        int k = index.at<int>(cy, cx);
        if (k < 0) continue;

        const size_t offk = static_cast<size_t>(k) * step;
        if (offk + step > all) continue;

        const float qx = *reinterpret_cast<const float*>(sb + offk + off_x);
        const float qy = *reinterpret_cast<const float*>(sb + offk + off_y);
        const float qz = *reinterpret_cast<const float*>(sb + offk + off_z);
        if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz)) continue;

        const float rh_q = std::hypot(qx, qy);
        const float dz   = std::fabs(qz - z_sym);
        const float dr   = std::fabs(rh_q - rh);

        if (dz <= dz_thr(rh) && dr <= dr_thr(rh)) {
          const float cost = dz + dr;
          if (cost < bestCost) { bestCost = cost; mirrored = true; }
        }
      }
    }

    if (mirrored) {
      // 복원: 원점 방사 스케일로 z=ground_z
      const float ratio = (z == 0.f) ? 1.f : (ground_z / z);
      X = x * ratio;
      Y = y * ratio;
      Z = ground_z;
    } else {
      // 미복원
      X = x; Y = y; Z = z;
    }
  }

  pub_->publish(out);
}

// --------------------------------------------------
// ground z estimator (원본 유지)
// --------------------------------------------------
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

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
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
