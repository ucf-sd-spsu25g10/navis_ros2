#include <rclcpp/rclcpp.hpp>

#include "navis_msgs/msg/control_out.hpp"

class ControlOutNode: public rclcpp::Node {
public:
    ControlOutNode() : Node("control_out_node") {
        
        control_out_sub_ = this->create_subscription<navis_msgs::msg::ControlOut>(
            "control_output",
            rclcpp::QoS(10),
            std::bind(&ControlOutNode::write_callback, this, std::placeholders::_1)
        );

        buzzer_address = 0;
        speaker_address = 0;
        
        open_uart();
    }

    ~ControlOutNode() {
        close_uart();
    }

private:

    void open_uart() {
        return;
    }

    void close_uart() {
        return;
    }

    // Feel free to change address, i just dont know how we're differentiating between speaker and buzzer writes
    void write_uart(int address, int val) {
        return;
    }

    void write_callback(const navis_msgs::msg::ControlOut::SharedPtr msg) {
        int buzzer_strength = msg->buzzer_strength;
        int speaker_wav_index = msg->speaker_wav_index;

        write_uart(buzzer_address, buzzer_strength);
        write_uart(speaker_address, speaker_wav_index);
    }

    // Topic Subscription
    rclcpp::Subscription<navis_msgs::msg::ControlOut>::SharedPtr control_out_sub_;

    int buzzer_address;
    int speaker_address;


};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlOutNode>());
    rclcpp::shutdown();
    return 0;
}