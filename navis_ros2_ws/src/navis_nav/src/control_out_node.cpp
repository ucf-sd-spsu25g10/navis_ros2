#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#include "navis_msgs/msg/control_out.hpp"

class ControlOutNode: public rclcpp::Node {
public:
    ControlOutNode() : Node("control_out_node") {
        
        control_out_sub_ = this->create_subscription<navis_msgs::msg::ControlOut>(
            "control_output",
            rclcpp::QoS(10),
            std::bind(&ControlOutNode::write_callback, this, std::placeholders::_1)
        );
        
        open_uart();
    }

    ~ControlOutNode() {
        close_uart();
    }

private:

    void open_uart() {
        uart_fd_ = open("/dev/ttyAMA1", O_RDWR);
        if (uart_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error opening UART");
            return;
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(uart_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error getting UART attributes");
            return;
        }

        // Set UART parameters (115200 8N1)
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error setting UART attributes");
            return;
        }
    }

    void close_uart() {
        if (uart_fd_ >= 0) {
            close(uart_fd_);
        }
    }

    void write_uart(int val) {
        if (uart_fd_ < 0) return;

        uint8_t packet = static_cast<uint8_t>(val);

        write(uart_fd_, packet, sizeof(packet));
    }

    void write_callback(const navis_msgs::msg::ControlOut::SharedPtr msg) {
        int direction = msg->buzzer_strength;

        write_uart(direction);
    }

    // Topic Subscription
    rclcpp::Subscription<navis_msgs::msg::ControlOut>::SharedPtr control_out_sub_;

    int uart_fd_ = -1;

};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlOutNode>());
    rclcpp::shutdown();
    return 0;
}