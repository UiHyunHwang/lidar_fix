#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// LidarFix 노드 선언부
class LidarFix : public rclcpp::Node
{
public:
  LidarFix();

private:
  // 콜백
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  // 구독/퍼블리시
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

// 지면 z 추정 함수(선언). 기본인자는 여기(헤더)에만 둡니다.
float estimate_ground_z(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg,
  float bin_sz = 0.02f,   // 2 cm
  float z_min  = -1.2f,
  float z_max  = -0.2f    // 필요시 조정 가능
);

namespace lidar_fix_params {
// ============================
// (x,y) cell/ z-histogram parameters
// ============================

// sensor height (m)
inline constexpr float sensor_height = 0.61f;

// (x,y) cell size [m]
inline constexpr float kXYRes   = 0.03f;   // 3 cm
// z histogram range [m]
inline constexpr float kZMin    = -(sensor_height + 3)f; // set it smaller than -(sensor height + height of the space)
inline constexpr float kZMax    =  (3 - sensor_height)f;
// z bin size [m]
inline constexpr float kZBin    =  0.03f;  // 1 cm
// number of bins
inline constexpr int   kNBins   = static_cast<int>((kZMax - kZMin) / kZBin) + 1;
// mirrored 판정 시 허용 bin 오차 (±kTolBins)
inline constexpr int   kTolBins = 1;       // ±1 cm
// ground_z 추정 시 사용할 반경 제곱 범위 [m^2] (1m ~ 3m)
inline constexpr float kD2Min   = 1.0f;    // 1^2
inline constexpr float kD2Max   = 9.0f;    // 3^2
// NaN 처리 정책: 유효하지 않은 포인트나 미러 실패시 드롭할지 여부
inline constexpr bool  kDropInvalidOrUnmirrored = true;
} 