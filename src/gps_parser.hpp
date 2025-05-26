#ifndef GPS_PARSER_HPP
#define GPS_PARSER_HPP

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

struct GPSPoint {
    double lat;
    double lon;
    double alt;

    GPSPoint(double latitude, double longitude, double altitude)
        : lat(latitude), lon(longitude), alt(altitude) {}
};

class GpsParser {
public:
    GpsParser() : ref_initialized_(false) {}

    std::vector<GPSPoint> load_gps_data(const std::string &filepath) {
        std::vector<GPSPoint> gps_data;
        std::ifstream infile(filepath);

        if (!infile.is_open()) {
            std::cerr << "无法打开文件: " << filepath << std::endl;
            return gps_data;
        }

        std::string line;
        while (std::getline(infile, line)) {
            double lat, lon, alt;
            if (sscanf(line.c_str(), "lat = %lf   lon = %lf   alt = %lf", &lat, &lon, &alt) != 3) {
                std::cerr << "格式不匹配: " << line << std::endl;
                continue;
            }

            double x, y, z;
            wgs84_to_ecef(lat, lon, alt, x, y, z);

            if (!ref_initialized_) {
                ref_lat_rad_ = lat * M_PI / 180.0;
                ref_lon_rad_ = lon * M_PI / 180.0;
                ref_x_ = x;
                ref_y_ = y;
                ref_z_ = z;
                ref_initialized_ = true;
            }

            double enu_x, enu_y, enu_z;
            ecef_to_enu(x, y, z, enu_x, enu_y, enu_z);

            GPSPoint point(enu_x, enu_y, enu_z);
            gps_data.push_back(point);
        }

        return gps_data;
    }

private:
    // 地球常数
    const double a_ = 6378137.0;
    const double e_sq_ = 6.69437999014e-3;

    double ref_lat_rad_, ref_lon_rad_;
    double ref_x_, ref_y_, ref_z_;
    bool ref_initialized_;

    void wgs84_to_ecef(double lat, double lon, double alt, double &x, double &y, double &z) {
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;

        double N = a_ / sqrt(1 - e_sq_ * sin(lat_rad) * sin(lat_rad));
        x = (N + alt) * cos(lat_rad) * cos(lon_rad);
        y = (N + alt) * cos(lat_rad) * sin(lon_rad);
        z = (N * (1 - e_sq_) + alt) * sin(lat_rad);
    }

    void ecef_to_enu(double x, double y, double z, double &enu_x, double &enu_y, double &enu_z) {
        double dx = x - ref_x_;
        double dy = y - ref_y_;
        double dz = z - ref_z_;

        enu_x = -sin(ref_lon_rad_) * dx + cos(ref_lon_rad_) * dy;
        enu_y = -sin(ref_lat_rad_) * cos(ref_lon_rad_) * dx
              - sin(ref_lat_rad_) * sin(ref_lon_rad_) * dy
              + cos(ref_lat_rad_) * dz;
        enu_z =  cos(ref_lat_rad_) * cos(ref_lon_rad_) * dx
              + cos(ref_lat_rad_) * sin(ref_lon_rad_) * dy
              + sin(ref_lat_rad_) * dz;
    }
};

#endif // GPS_PARSER_HPP

