#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

class GpsToEnuPathNode : public rclcpp::Node {
public:
    GpsToEnuPathNode() : Node("ins_to_ecef_enu_node") {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        path_.header.frame_id = "enu";

        read_and_convert_gps();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&GpsToEnuPathNode::publish_path, this));
    }

private:
    nav_msgs::msg::Path path_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 地球常数
    const double a_ = 6378137.0;
    const double e_sq_ = 6.69437999014e-3;

    double ref_lat_rad_, ref_lon_rad_;
    double ref_x_, ref_y_, ref_z_;
    bool ref_initialized_ = false;

    void read_and_convert_gps() {
        std::ifstream infile(this->declare_parameter("gps_file", std::string("src/test2/data/gps.txt")));
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "无法打开 gps.txt");
            return;
        }

        std::string line;
        size_t count = 0;

        while (std::getline(infile, line)) {
            double lat, lon, alt;
            if (sscanf(line.c_str(), "lat = %lf   lon = %lf   alt = %lf", &lat, &lon, &alt) != 3) {
                RCLCPP_WARN(this->get_logger(), "格式不匹配: %s", line.c_str());
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

            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "enu";
            pose.header.stamp = this->get_clock()->now();
            pose.pose.position.x = enu_x;
            pose.pose.position.y = enu_y;
            pose.pose.position.z = enu_z;
            pose.pose.orientation.w = 1.0;

            path_.poses.push_back(pose);
            count++;
        }

        RCLCPP_INFO(this->get_logger(), "读取并转换 %ld 个 GPS 点", count);
    }

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

    void publish_path() {
        path_.header.stamp = this->get_clock()->now();
        for (auto &pose : path_.poses)
            pose.header.stamp = path_.header.stamp;
        path_pub_->publish(path_);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GpsToEnuPathNode>());
    rclcpp::shutdown();
    return 0;
}

