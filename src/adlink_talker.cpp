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
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" 
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class AdlinkTalker : public rclcpp::Node
{
    public:
        AdlinkTalker(std::string pub_topic): Node("adlink_talker"), count_(0)
        {
            pub_topic_ = pub_topic;
            publisher_ = this->create_publisher<std_msgs::msg::String>(pub_topic_); //topic name
            timer_ = this->create_wall_timer(500ms, std::bind(&AdlinkTalker::timer_callback, this));
        }

    private:
        void timer_callback()
        {
            auto message = std_msgs::msg::String();
            message.data = "Hi ADLink! " + std::to_string(count_++);
            printf("Publishing: [%s]\n", message.data.c_str());
            publisher_->publish(message);
        }
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        size_t count_;
        std::string pub_topic_;
};

void print_usage()
{
    printf("Usage for adlink talker app:\n");
    printf("talker [-t topic_name] [-h]\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_name : Specify the topic on which to publish. Defaults to chatter.\n");
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
    {
        print_usage();
        return 0;
    }

    auto pub_topic = std::string("chatter");
    if (rcutils_cli_option_exist(argv, argv + argc, "-t"))
    {
        pub_topic = std::string(rcutils_cli_get_option(argv, argv + argc, "-t"));
    }

    auto at = std::make_shared<AdlinkTalker>(pub_topic);
    rclcpp::spin(at);
    rclcpp::shutdown();
    return 0;
}
