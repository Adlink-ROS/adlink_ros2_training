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
#include <iomanip>      // std::setw
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" 
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

class RoundTripPING : public rclcpp::Node
{
    public:
        RoundTripPING(int pub_timer, int payload): Node("roundtrip_ping"), pub_count_(0), sub_count_(0)
        {
            pub_timer_ = pub_timer;
            payload_ = payload;
            publisher_ = this->create_publisher<std_msgs::msg::String>("roadtrip_ping", rmw_qos_profile_sensor_data); //topic, QoS
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                            "roadtrip_pong", std::bind(&RoundTripPING::topic_callback, this, _1), rmw_qos_profile_sensor_data);
            timer_ = this->create_wall_timer(milliseconds(pub_timer_), std::bind(&RoundTripPING::timer_callback, this));

            message_.data = "a";
            if(payload_ > 1)
            {            
                for(int i=0; i< (payload_ -1); i++)
                    message_.data = message_.data + "a";
            }
            std::cout << "WriteAccess duration (ms),  RoundTrip duration,  Overall RoundTrip duration" << std::endl;
        }

    private:       
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std_msgs::msg::String message_ = std_msgs::msg::String();
        size_t pub_count_, sub_count_;
        int pub_timer_;
        int payload_;
        time_point<high_resolution_clock> preWriteTime;
        time_point<high_resolution_clock> postWriteTime;
        time_point<high_resolution_clock> postReadTime;

        void timer_callback()
        {
            preWriteTime = high_resolution_clock::now();      
            publisher_->publish(message_);
            postWriteTime = high_resolution_clock::now();    
            //std::cout << "writeAccess duration (ms): "  << writeAccess.count()*1000 << ", id: " << count_ << std::endl; // debug
            pub_count_ ++;
        }

        void topic_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            postReadTime = high_resolution_clock::now();            
            if( (pub_count_-1)!= sub_count_ )
            {
                std::cout << "Losing data!  pubId: " << pub_count_-1 << ", subId: " << sub_count_ << std::endl;
                pub_count_ = 0;
                sub_count_ = 0;
                return;
            }
            std::cout << std::setw(11) << duration<double>(postWriteTime-preWriteTime).count()*1000.0
                      << std::setw(11) << duration<double>(postReadTime-postWriteTime).count()*1000.0 
                      << std::setw(11) << duration<double>(postReadTime-preWriteTime).count()*1000.0 << std::endl;
                
            sub_count_++;
        }
};

void print_usage()
{
    printf("Usage for adlink talker app:\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_timer : Specify the publishing frequency (ms).\n");
    printf("-p msg_payload : Specify the size of payload for publishing .\n");
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
    {
        print_usage();
        return 0;
    }

    int topic_timer = 500;
    if (rcutils_cli_option_exist(argv, argv + argc, "-t"))
    {
        topic_timer = atoi(rcutils_cli_get_option(argv, argv + argc, "-t"));
        if (topic_timer > 1000)
            topic_timer = 1000;
        if (topic_timer <= 1)
            topic_timer = 1;
    }

    int payload = 5;
    if (rcutils_cli_option_exist(argv, argv + argc, "-p"))
    {
        payload = atoi(rcutils_cli_get_option(argv, argv + argc, "-p"));
        if (payload > 64*1024)
            payload = 64*1024;
        if (payload <= 1)
            payload = 1;
    }


    auto ping = std::make_shared<RoundTripPING>(topic_timer, payload);
    rclcpp::spin(ping);
    rclcpp::shutdown();
    return 0;
}
