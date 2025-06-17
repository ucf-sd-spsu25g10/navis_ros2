#include <chrono>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cmath>

using namespace std::chrono_literals;

class IMUPublisher : public rclcpp::Node {
public:
    IMUPublisher() : Node("imu_publisher") {

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(IMU_READ_RATE_MS), std::bind(&IMUPublisher::read_and_publish, this));

        // Open I2C device
        if ((i2c_file_ = open(IMU_I2C_FILE, O_RDWR)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open I2C bus.");
            rclcpp::shutdown();
            return;
        }

        // Connect to IMU device
        if (ioctl(i2c_file_, I2C_SLAVE, IMU_I2C_ADDRESS) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to connect to IMU at 0x%X", i2c_addr_);
            rclcpp::shutdown();
            return;
        }

        // Wake up the IMU (for MPU6050)
        write_register(0x6B, 0x00); // PWR_MGMT_1 register
    }

private:
    void write_register(uint8_t reg, uint8_t value) {
        uint8_t buffer[2] = {reg, value};
        write(i2c_file_, buffer, 2);
    }

    bool read_word_checked(int i2c_fd, uint8_t reg, int16_t &out_value) {
        uint8_t buf[2];
        buf[0] = reg;

        if (write(i2c_fd, buf, 1) != 1) {
            return false;
        }
        if (read(i2c_fd, buf, 2) != 2) {
            return false;
        }

        out_value = (int16_t)((buf[0] << 8) | buf[1]);
        return true;
    }

    void read_and_publish() {
        int16_t acc_x, acc_y, acc_z;
        int16_t gyro_x, gyro_y, gyro_z;

        bool ok = true;
        ok &= read_word_checked(i2c_file_, ACCEL_XOUT_H, acc_x);
        ok &= read_word_checked(i2c_file_, ACCEL_YOUT_H, acc_y);
        ok &= read_word_checked(i2c_file_, ACCEL_ZOUT_H, acc_z);
        ok &= read_word_checked(i2c_file_, GYRO_XOUT_H, gyro_x);
        ok &= read_word_checked(i2c_file_, GYRO_YOUT_H, gyro_y);
        ok &= read_word_checked(i2c_file_, GYRO_ZOUT_H, gyro_z);

        if (!ok) {
            RCLCPP_WARN(this->get_logger(), "IMU read failed: %s", strerror(errno));
            return; // Skip publishing invalid data
        }

        auto imu_msg = sensor_msgs::msg::Imu();

        // Convert to m/s² and rad/s (MPU6050 full-scale default: ±2g, ±250 deg/s)
        constexpr double accel_scale = 16384.0; // 2g range
        constexpr double gyro_scale = 131.0;    // 250 dps range

        imu_msg.linear_acceleration.x = acc_x / accel_scale * 9.81;
        imu_msg.linear_acceleration.y = acc_y / accel_scale * 9.81;
        imu_msg.linear_acceleration.z = acc_z / accel_scale * 9.81;

        imu_msg.angular_velocity.x = gyro_x / gyro_scale * (M_PI / 180.0);
        imu_msg.angular_velocity.y = gyro_y / gyro_scale * (M_PI / 180.0);
        imu_msg.angular_velocity.z = gyro_z / gyro_scale * (M_PI / 180.0);

        imu_msg.header.stamp = this->get_clock()->now();
        imu_msg.header.frame_id = "imu_link";

        publisher_->publish(imu_msg);
    }

    static constexpr uint8_t IMU_I2C_ADDRESS = 0x68;       // MPU6050 address
    static constexpr char IMU_I2C_FILE[] = "/dev/i2c-1";   // Typical I2C bus on Raspberry Pi
    static constexpr int IMU_READ_RATE_MS = 100;           // Poll every 100ms

    // Gotten from:
    //      https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf
    // Accelerometer data registers
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t ACCEL_XOUT_L = 0x3C;
    static constexpr uint8_t ACCEL_YOUT_H = 0x3D;
    static constexpr uint8_t ACCEL_YOUT_L = 0x3E;
    static constexpr uint8_t ACCEL_ZOUT_H = 0x3F;
    static constexpr uint8_t ACCEL_ZOUT_L = 0x40;

    // Gyroscope data registers
    static constexpr uint8_t GYRO_XOUT_H = 0x43;
    static constexpr uint8_t GYRO_XOUT_L = 0x44;
    static constexpr uint8_t GYRO_YOUT_H = 0x45;
    static constexpr uint8_t GYRO_YOUT_L = 0x46;
    static constexpr uint8_t GYRO_ZOUT_H = 0x47;
    static constexpr uint8_t GYRO_ZOUT_L = 0x48;

    int i2c_file_;
    uint8_t i2c_addr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMUPublisher>());
    rclcpp::shutdown();
    return 0;
}