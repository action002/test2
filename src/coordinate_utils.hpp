#ifndef COORDINATE_UTILS_HPP
#define COORDINATE_UTILS_HPP

#include <Eigen/Dense>
#include <cmath>

// 地球的 WGS84 坐标系参数
const double WGS84_A = 6378137.0;  // 长半轴 (meters)
const double WGS84_E2 = 0.00669437999014;  // 偏心率平方

// 定义坐标转换函数

// 将 WGS84 坐标系下的经纬度（lat, lon, alt）转换为 ECEF 坐标系（X, Y, Z）
Eigen::Vector3d wgs84_to_ecef(double lat, double lon, double alt)
{
    // 将角度转换为弧度
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // 计算WGS84坐标系下的 N（曲率半径）
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(lat_rad) * std::sin(lat_rad));

    // 计算 ECEF 坐标
    double X = (N + alt) * std::cos(lat_rad) * std::cos(lon_rad);
    double Y = (N + alt) * std::cos(lat_rad) * std::sin(lon_rad);
    double Z = ((1 - WGS84_E2) * N + alt) * std::sin(lat_rad);

    return Eigen::Vector3d(X, Y, Z);
}

// 将 ECEF 坐标系下的坐标转换为 ENU 坐标系
Eigen::Vector3d ecef_to_enu(const Eigen::Vector3d& ecef, const Eigen::Vector3d& ref_ecef, double ref_lat, double ref_lon)
{
    // 计算参考点的纬度和经度的弧度
    double ref_lat_rad = ref_lat * M_PI / 180.0;
    double ref_lon_rad = ref_lon * M_PI / 180.0;

    // 计算参考点在 ECEF 坐标系下的坐标
    double N = WGS84_A / std::sqrt(1.0 - WGS84_E2 * std::sin(ref_lat_rad) * std::sin(ref_lat_rad));
    Eigen::Vector3d ref_ecef_transformed = wgs84_to_ecef(ref_lat, ref_lon, 0.0);  // 默认高度为0

    // 计算从参考点到当前点的方向向量
    Eigen::Vector3d d_ecef = ecef - ref_ecef_transformed;

    // 构造转换矩阵（ECEF -> ENU）
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << -std::sin(ref_lon_rad), std::cos(ref_lon_rad), 0,
                       -std::sin(ref_lat_rad) * std::cos(ref_lon_rad), -std::sin(ref_lat_rad) * std::sin(ref_lon_rad), std::cos(ref_lat_rad),
                       std::cos(ref_lat_rad) * std::cos(ref_lon_rad), std::cos(ref_lat_rad) * std::sin(ref_lon_rad), std::sin(ref_lat_rad);

    // 将 ECEF 坐标转换为 ENU 坐标
    Eigen::Vector3d enu = rotation_matrix * d_ecef;
    
    return enu;
}

#endif  // COORDINATE_UTILS_HPP

