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
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
using std::placeholders::_1;

class PoseSubscriber : public rclcpp::Node
{
    public:
    PoseSubscriber()
    : Node("pose_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/amcl_pose", std::bind(&PoseSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        printf(" position: [%+6.3f %+6.3f %+6.3f]\n",
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();
    return 0;
}
