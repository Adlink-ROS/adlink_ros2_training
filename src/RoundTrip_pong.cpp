// Copyright 2017 ADLINK Technology, Inc.
// Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class RoundTripPONG : public rclcpp::Node
{
    public:
    RoundTripPONG() : Node("roundtrip_pong"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("roadtrip_pong", rmw_qos_profile_parameters); //topic, QoS
        subscription_ = this->create_subscription<std_msgs::msg::String>("roadtrip_ping", 
                        std::bind(&RoundTripPONG::topic_callback, this, _1), rmw_qos_profile_parameters);
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        publisher_->publish(msg);
        //std::cout << "I heard: " << msg->data.c_str() << ", Id: " << count_ << std::endl; //debug
        count_ ++;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "RoundTrip Testing: pong side !" << std::endl;
    rclcpp::spin(std::make_shared<RoundTripPONG>());
    rclcpp::shutdown();
    return 0;
}
