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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class PoseArrayPub : public rclcpp::Node
{
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
        size_t count_;

        void timer_callback()
        {
            auto message = geometry_msgs::msg::PoseArray();
            message.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
            message.header.frame_id = "neonj_link";
            message.poses.resize(1);
            
            message.poses[0].position.x = 1.0;
            message.poses[0].position.y = 2.0;
            message.poses[0].position.z = 3.0;
            message.poses[0].orientation.w = 1.0;

            publisher_->publish(message);
            count_ = count_ + 1;

            //Debug Info
            std::cout << "Publishing PoseArray!, topic: " << message.header.frame_id << ", size: " << message.poses.size() << ", seq: " << count_ << std::endl;
        }
        
    public:
        PoseArrayPub(): Node("pose_array_pub")
        {
            publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/neonj_poses", rmw_qos_profile_sensor_data); //qos profit: sensor
            timer_ = this->create_wall_timer(100ms, std::bind(&PoseArrayPub::timer_callback, this)); // 10Hz (100ms)
            count_ = 0;
        }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto pap = std::make_shared<PoseArrayPub>();
    rclcpp::spin(pap);
    rclcpp::shutdown();
    return 0;
}
