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
#include <vector>
#include <unistd.h>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class NonStopDemo : public rclcpp::Node
{
    public:
        NonStopDemo() : Node("nonstop_demo")
        {
            publisher_  = this->create_publisher<geometry_msgs::msg::TransformStamped>("/swarm_goals");         
            subscriper_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
              "/swarm_poses", std::bind(&NonStopDemo::topic_callback, this, _1));
            timer_ = this->create_wall_timer(500ms, std::bind(&NonStopDemo::timer_callback, this));
            //robot1
            this->robot1_goalx_.push_back(0.353); //robot1 goal1
            this->robot1_goaly_.push_back(0.162);
            this->robot1_goalx_.push_back(-0.34); //robot1 goal2           
            this->robot1_goaly_.push_back(0.103);
            robot1_goalID_ = -1;    
            robot1_reached_ = false;
            robot1_ID_ = "adlinkbot01";
            //robot2
            this->robot2_goalx_.push_back(-0.066); //robot2 goal1
            this->robot2_goaly_.push_back(0.196);
            this->robot2_goalx_.push_back(0.466); //robot2 goal2
            this->robot2_goaly_.push_back(0.284);
            robot2_goalID_ = -1;    
            robot2_reached_ = false;
            robot2_ID_ = "adlinkbot02";

            this->goal_radius_ = 0.25; // unit: meter
            map_ID_ = "map";
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscriper_;
        rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::vector<double> robot1_goalx_, robot1_goaly_, robot2_goalx_, robot2_goaly_;
        std::string robot1_ID_, robot2_ID_, map_ID_;
        int robot1_goalID_, robot2_goalID_;
        bool robot1_reached_, robot2_reached_;
        double goal_radius_;

        void sendGoal(const std::string &robotID, const double &goal_x, const double &goal_y)
        {
            auto msg_out = geometry_msgs::msg::TransformStamped();
            msg_out.header.frame_id = map_ID_;
            msg_out.header.stamp = rclcpp::Time::now();
            msg_out.child_frame_id = robotID;
            msg_out.transform.translation.x = goal_x;
            msg_out.transform.translation.y = goal_y;
            msg_out.transform.rotation.x = 0.0;
            msg_out.transform.rotation.y = 0.0;
            msg_out.transform.rotation.z = 0.0;
            msg_out.transform.rotation.w = 1.0;
            publisher_->publish(msg_out);
        }
     
        double distance(const double &a_x, const double &a_y, const double &b_x, const double &b_y)
        { return std::sqrt( (a_x-b_x)*(a_x-b_x) + (a_y-b_y)*(a_y-b_y) ); }

        void timer_callback()
        {
            if( robot1_goalID_<0 || robot2_goalID_<0 )
            {
                sendGoal(robot1_ID_, robot1_goalx_[0], robot1_goaly_[0]);
                //usleep(100);
                sendGoal(robot2_ID_, robot2_goalx_[0], robot2_goaly_[0]);
                robot1_goalID_ = 0;
                robot2_goalID_ = 0;
                std::cout << "Go to initial position !" << std::endl;
                return;
            }
            
            if( robot1_reached_ || robot2_reached_)
            {
                if(robot1_goalID_ == 0)
                {                
                    robot1_goalID_ = 1;
                    robot2_goalID_ = 1;
                    sendGoal(robot1_ID_, robot1_goalx_[robot1_goalID_], robot1_goaly_[robot1_goalID_]);
                    sendGoal(robot2_ID_, robot2_goalx_[robot2_goalID_], robot2_goaly_[robot2_goalID_]);
                    robot1_reached_ = false;
                    robot2_reached_ = false;
                    std::cout << "Change to goal set 2 !" << std::endl;
                }
                else
                {                
                    robot1_goalID_ = 0;
                    robot2_goalID_ = 0;
                    sendGoal(robot1_ID_, robot1_goalx_[robot1_goalID_], robot1_goaly_[robot1_goalID_]);
                    sendGoal(robot2_ID_, robot2_goalx_[robot2_goalID_], robot2_goaly_[robot2_goalID_]);
                    robot1_reached_ = false;
                    robot2_reached_ = false;
                    std::cout << "Change to goal set 1 !" << std::endl;
                }
            }
            //std::cout << "timer triggered !" << std::endl; // debug
        }
        
        void topic_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg_in)
        {
            if( robot1_goalID_<0 || robot2_goalID_<0 )
                return;        

            double curr_x = msg_in->transform.translation.x,
                   curr_y = msg_in->transform.translation.y; 
            
            std::string robotId = msg_in->child_frame_id;
            //std::cout << "I heard: " << robotId << std::endl; // debug
                                
            if(robotId == robot1_ID_)
                if(  distance(curr_x, curr_y, robot1_goalx_[robot1_goalID_], robot1_goaly_[robot1_goalID_]) <= goal_radius_  ) 
                {
                    robot1_reached_ = true;
                    std::cout << "robot1 reach goal !" << std::endl;
                }         
            
            if(robotId == robot2_ID_)
                if(  distance(curr_x, curr_y, robot2_goalx_[robot2_goalID_], robot2_goaly_[robot2_goalID_]) <= goal_radius_  ) 
                {
                    robot2_reached_ = true;
                    std::cout << "robot2 reach goal !" << std::endl;
                }  

        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NonStopDemo>());
    rclcpp::shutdown();
    return 0;
}
