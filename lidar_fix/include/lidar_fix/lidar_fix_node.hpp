#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <unordered_map>
#include <vector>
#include <cstdint>

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

// LidarFix node
class LidarFix : public rclcpp::Node
{
public:
  LidarFix();

private:
  // callback
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 지면 z 추정
  float estimate_ground_z(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                          float bin_sz = 0.02f,   // 2 cm
                          float z_min  = -1.2f,
                          float z_max  = -0.2f);

  // (x,y) → cell index
  static inline CellKey cell_of(float x, float y);

  // z → bin index (grnd_z 기준 위/아래 분리; skip zone 영역은 skip)
  static inline int k_of(float z, float grnd_z);

  // subsribe/publsih
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  /* 프레임 간 재사용하는 z-hist container (요소만 clear)
  - key: CellKey (i,j)
  - value: 해당 cell에 속하는 point들을 z 축으로 binning한 histogram의 주소
  */
  std::unordered_map<CellKey, std::vector<uint16_t>, CellKeyHash> zhist_;

  // zhist 초기 버킷 수(성능용)
  static constexpr std::size_t kInitBuckets = 8192;
};

// namespace lidar_fix_params
namespace lidar_fix_params {

  // sensor height (m)
  inline constexpr float sensor_height = 0.32f;

  inline constexpr float kSkipZone = 0.01f; // ±1 cm skip zone

  // (x,y) cell size [m]
  inline constexpr float kXYRes   = 0.1f;   // 10 cm

  // z histogram range [m]
  inline constexpr float kZMin    = -(sensor_height + 3.0f);
  inline constexpr float kZMax    =  (3.0f - sensor_height);

  // z bin size [m]
  inline constexpr float kZBin    =  0.5f;  // 50 cm

  // number of bins
  inline constexpr int   kNBins   = static_cast<int>((kZMax - kZMin) / kZBin) + 1;

  // mirrored 판정 시 허용 bin 오차 (±kTolBins)
  inline constexpr int   kTolBins = 0;       // 필요시 1~2로 키워도 됨

  // ground_z 추정 시 사용할 반경 제곱 범위 [m^2] (1m ~ 3m)
  inline constexpr float kD2Min   = 1.0f;    // 1^2
  inline constexpr float kD2Max   = 9.0f;    // 3^2
}
