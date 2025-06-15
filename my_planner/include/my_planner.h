#ifndef MY_PLANNER_H
#define MY_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

namespace my_planner{
    class MyPlanner : public nav_core::BaseLocalPlanner{
        public:
            MyPlanner();
            ~MyPlanner();

            /**
             * @brief  初始化函数，在movebase加载时调用
             * @param name The name to give this instance of the local planner
             * @param tf A pointer to a transform listener
             * @param costmap_ros The cost map to use for assigning costs to local plans
             */
            void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);


            /**
             * @brief  接收来自全局规划器的导航路线
             * @param plan The plan to pass to the local planner
             * @return True if the plan was updated successfully, false otherwise
             */
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);


            /**
             * @brief  Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
             * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
             * @return True if a valid velocity command was found, false otherwise
             * 备注：无需专门去发送话题，把控制指令返回到cmd_vel即可，movebase节点会自动发送
             */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            /**
             * @brief  Check if the goal pose has been achieved by the local planner
             * @return True if achieved, false otherwise
             */
            bool isGoalReached();
    };
}


#endif