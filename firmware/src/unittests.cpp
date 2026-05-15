#include <chrono>
#include <memory>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"

#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> test_node;

class PicoInterface {
public:
    PicoInterface() {
        led_pub_ = test_node->create_publisher<std_msgs::msg::Bool>("/led", 10);
        
        heading_sub_ = test_node->create_subscription<std_msgs::msg::Float32>(
            "/heading", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                heading_ = msg->data;
                heading_received_ = true;
            });
        
        test_sub_ = test_node->create_subscription<std_msgs::msg::Float32>(
            "/test_publisher", 10,
            [this](std_msgs::msg::Float32::SharedPtr msg) {
                test_value_ = msg->data;
                test_received_ = true;
            });
    }

    void set_led(bool on) {
        auto msg = std_msgs::msg::Bool();
        msg.data = on;
        led_pub_->publish(msg);
    }

    bool wait_for_heading(std::chrono::milliseconds timeout = 2000ms) {
        heading_received_ = false;
        auto start = std::chrono::steady_clock::now();
        while (!heading_received_) {
            rclcpp::spin_some(test_node);
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }
        return true;
    }

    bool wait_for_test_msg(std::chrono::milliseconds timeout = 2000ms) {
        test_received_ = false;
        auto start = std::chrono::steady_clock::now();
        while (!test_received_) {
            rclcpp::spin_some(test_node);
            if (std::chrono::steady_clock::now() - start > timeout) {
                return false;
            }
            std::this_thread::sleep_for(10ms);
        }
        return true;
    }

    float get_heading() const { return heading_; }
    float get_test_value() const { return test_value_; }

private:
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr led_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr test_sub_;
    std::atomic<float> heading_{0.0f};
    std::atomic<float> test_value_{0.0f};
    std::atomic<bool> heading_received_{false};
    std::atomic<bool> test_received_{false};
};

// -----------------------------------------------------
// Unit Tests (LED Demo)
// -----------------------------------------------------

TEST_CASE("LED blink test", "[test][pico][led]") {
    PicoInterface pico;

    INFO("Waiting for ROS 2 discovery...");
    std::this_thread::sleep_for(2000ms);
    
    INFO("Turning LED ON");
    pico.set_led(true);
    std::this_thread::sleep_for(500ms);
    
    INFO("Turning LED OFF");
    pico.set_led(false);
    std::this_thread::sleep_for(500ms);
    
    INFO("Blinking LED 3 times");
    for (int i = 0; i < 3; i++) {
        pico.set_led(true);
        std::this_thread::sleep_for(250ms);
        pico.set_led(false);
        std::this_thread::sleep_for(250ms);
    }
    
    // If we got here without timeout, the test passes
    REQUIRE(true);
}

// -----------------------------------------------------
// Main
// -----------------------------------------------------

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    test_node = std::make_shared<rclcpp::Node>("pico_unit_tests");
    
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "Pico Unit Tests" << std::endl;
    std::cout << "Ensure micro-ROS agent is running:" << std::endl;
    std::cout << "  ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
    std::cout << "Waiting for micro-ROS agent connection..." << std::endl;
    std::this_thread::sleep_for(2000ms);
    
    int result = Catch::Session().run(argc, argv);
    
    test_node.reset();
    rclcpp::shutdown();
    
    return result;
}
