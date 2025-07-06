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

    void write_uart(const char* packet) {
        if (uart_fd_ < 0) {
            RCLCPP_WARN_ONCE(this->get_logger(), "UART not available. Is it open?");
            return;
        }
        // Add a newline to delimit commands for the receiver
        char buffer[32];
        int len = snprintf(buffer, sizeof(buffer), "%s\n", packet);
        if (write(uart_fd_, buffer, len) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to write to UART");
        }
    }

    void write_callback(const navis_msgs::msg::ControlOut::SharedPtr msg) {
        // Haptic feedback command is always sent
        char haptic_packet[8]; // e.g., "h-255"
        snprintf(haptic_packet, sizeof(haptic_packet), "h%d", msg->buzzer_strength);
        write_uart(haptic_packet);

        // Audio feedback command, only if there is a cue
        if (msg->speaker_wav_index > 0) {
            char audio_packet[6]; // e.g., "a255"
            snprintf(audio_packet, sizeof(audio_packet), "a%d", msg->speaker_wav_index);
            write_uart(audio_packet);
        }
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