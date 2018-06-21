// Copyright 2018 ADLINK Technology, Inc.
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
#include <iomanip>      // std::setw & setprecision
#include <string>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" 
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

class RoundTripPING : public rclcpp::Node
{
    public:
        RoundTripPING(int argc, char * argv[]): Node("roundtrip_ping"), pub_count_(0), sub_count_(0), max_loop_achieved_(false)
        {
            // Verify command arguments            
            pub_timer_ = 500;
            if (rcutils_cli_option_exist(argv, argv + argc, "-t"))
            {
                pub_timer_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-t"));
                if (pub_timer_ > 1000) // 1000ms -> 1Hz
                    pub_timer_ = 1000;
                if (pub_timer_ <= 1) // 1ms -> 1000Hz
                    pub_timer_ = 1;
            }

            payload_ = 1;
            if (rcutils_cli_option_exist(argv, argv + argc, "-p"))
            {
                payload_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-p"));
                if (payload_ > 1024*1024) // Max = 1MB
                    payload_ = 1024*1024;
                if (payload_ <= 1) // Min = 1 Byte
                    payload_ = 1;
            }

            max_loop_ = 0;
            if (rcutils_cli_option_exist(argv, argv + argc, "-i"))
            {
                max_loop_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-i"));
                if (max_loop_ <= 0)
                    max_loop_ = 0;
            }

            file_name_ = std::string("round_trip.log");
            char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-s");
            if (nullptr != cli_option) 
            {
                file_name_ = std::string(cli_option);
            }

            debug_info_ = false;
            if (rcutils_cli_option_exist(argv, argv + argc, "-d"))
            {
                debug_info_ = true;
            }


            publisher_ = this->create_publisher<std_msgs::msg::String>("roadtrip_ping", rmw_qos_profile_parameters); //topic, QoS
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                            "roadtrip_pong", std::bind(&RoundTripPING::topic_callback, this, _1), rmw_qos_profile_parameters);
            timer_ = this->create_wall_timer(milliseconds(pub_timer_), std::bind(&RoundTripPING::timer_callback, this));

            /*
            // resize vector
            if(max_loop_ > 0)
            {
                //preWriteTime_Vec_.resize(max_loop);
                //postWriteTime_Vec_.resize(max_loop);
                //postReadTime_Vec_.resize(max_loop);
                WriteAccessDuration_Vec_.resize(max_loop_);
                RoundTripDuration_Vec_.resize(max_loop_);
                OverallRoundTripDuration_Vec_.resize(max_loop_);
            }*/

            // create payload
            message_.data = "a";
            if(payload_ > 1)
            {            
                for(int i=0; i< (payload_ -1); i++)
                    message_.data = message_.data + "a";
            }
            std::cout << "WriteAccess duration (ms),  RoundTrip duration,  Overall RoundTrip duration" << std::endl;
        }

        void show_result()
        {
            double RoundTripDuration_mean = vector_avg(RoundTripDuration_Vec_);
            double OverallRoundTripDuration_mean = vector_avg(OverallRoundTripDuration_Vec_);

            std::cout << "========== Testing Result ==========" << std::endl;
            std::cout << "@RoundTripDuration average: " <<  RoundTripDuration_mean << ", variance: " << vector_var(RoundTripDuration_Vec_, RoundTripDuration_mean) << std::endl;
            std::cout << "@OverallRoundTripDuration average: " <<  OverallRoundTripDuration_mean << ", variance: " << vector_var(OverallRoundTripDuration_Vec_, OverallRoundTripDuration_mean) << std::endl;
        }

    private:       
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        std_msgs::msg::String message_ = std_msgs::msg::String();
        size_t pub_count_, sub_count_;
        int pub_timer_, payload_, max_loop_;
        bool debug_info_, max_loop_achieved_;
        std::string file_name_; 
        //std::vector<double> preWriteTime_Vec_, postWriteTime_Vec_, postReadTime_Vec_;
        std::vector<double> WriteAccessDuration_Vec_, RoundTripDuration_Vec_, OverallRoundTripDuration_Vec_;
        time_point<high_resolution_clock> preWriteTime_;
        time_point<high_resolution_clock> postWriteTime_;
        time_point<high_resolution_clock> postReadTime_;

        //Callback function for timer
        void timer_callback()
        {
            // reach the max loop            
            if(pub_count_ > max_loop_ && max_loop_ > 0 )  
            {
                if(!max_loop_achieved_) std::cout << "Max loop achieved!, Stop publishing!" << std::endl;                
                max_loop_achieved_ = true;
                return;
            }
          
            preWriteTime_ = high_resolution_clock::now();      
            publisher_->publish(message_);
            postWriteTime_ = high_resolution_clock::now();    
            //std::cout << "writeAccess duration (ms): "  << writeAccess_.count()*1000 << ", id: " << count_ << std::endl; // debug
            pub_count_ ++;
        } // end of func

        //Callback function for subscription
        void topic_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            postReadTime_ = high_resolution_clock::now();  
            sub_count_++;          
            if( pub_count_!= sub_count_ )
            {
                std::cout << "Losing data!  pubId: " << pub_count_-1 << ", subId: " << sub_count_ << std::endl;
                sub_count_ = pub_count_;
                return;
            }
            
            double WriteAccessDuration = duration<double>(postWriteTime_ - preWriteTime_).count()*1000.0;
            double RoundTripDuration = duration<double>(postReadTime_ - postWriteTime_).count()*1000.0/2.0;
            double OverallRoundTripDuration = duration<double>(postReadTime_ - preWriteTime_).count()*1000.0/2.0;

            //preWriteTime_Vec_.push_back(preWriteTime_);
            //postWriteTime_Vec_.push_back(postWriteTime_);
            //postReadTime_Vec_.push_back(postReadTime_);
            WriteAccessDuration_Vec_.push_back(WriteAccessDuration);
            RoundTripDuration_Vec_.push_back(RoundTripDuration);
            OverallRoundTripDuration_Vec_.push_back(OverallRoundTripDuration);

            if(debug_info_)
            {
                std::cout << std::fixed    << std::setprecision(6) 
                          << std::setw(11) << WriteAccessDuration
                          << std::setw(11) << RoundTripDuration
                          << std::setw(11) << OverallRoundTripDuration  << std::endl;
            }    
            
        } // end of func

        //Function for average
        double vector_avg ( std::vector<double>& v )
        {
            double return_value = 0.0;
            int n = v.size();
           
            for ( int i=0; i < n; i++)
            {
                return_value += v[i];
            }
           
            return ( return_value / n);
        } // end of func

        //Function for variance
        double vector_var ( std::vector<double>& v , double mean )
        {
            double sum = 0.0;
            double temp =0.0;
            double var =0.0;
           
            for ( int j =0; j < v.size(); j++)
            {
                temp = pow((v[j] - mean),2);
                sum += temp;
            }
           
            return var = sum/(v.size() -2);
        } // end of func
        
};

void print_usage()
{
    printf("ROS2 RoundTrip testing tool:\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-t topic_timer : Specify the publishing period time (ms). [default=100]\n");
    printf("-p msg_payload : Specify the size of payload for publishing (Byte) [default=5].\n");
    printf("-i max_iterate : Specify the number of loops for testing (0 for nonstop) [default=0].\n");
    printf("-f logfile_name: Specify the logfile name. [default=].\n");
    printf("-d debug_info  : Showing the debug info.\n");
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Print the help function
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
    {
        print_usage();
        return 0;
    }

    auto ping = std::make_shared<RoundTripPING>(argc, argv);
    rclcpp::spin(ping);
    rclcpp::shutdown();
    ping->show_result();
    return 0;
}
