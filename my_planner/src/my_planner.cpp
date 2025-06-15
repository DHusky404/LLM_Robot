#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

// 引入tf头文件，实现地图坐标系的全局路径点转换为机器人机体坐标系的路径点
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

// 将my_planner注册成插件
PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner, nav_core::BaseLocalPlanner)


namespace my_planner{
    MyPlanner::MyPlanner(){
        // 将字符串函数本地化，用于显示中文
        setlocale(LC_ALL, "");
    }
    MyPlanner::~MyPlanner(){}

    // 初始始化函数，在movebase加载时调用
    // 定义一个tf监听对象
    tf::TransformListener *tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    bool fineTunning;
    void MyPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
        // 实例化对象，在函数内部实例化，确保tf功能已经完全启动
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;
    }


    // 接收来自全局规划器的导航路线
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        target_index = 0;
        global_plan_ = plan;
        fineTunning = false;
        return true;
    }


    geometry_msgs::PoseStamped cur_pose;
    double linearError;
    double angularError;
    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
        // 获取局部代价地图
        costmap_2d::Costmap2D *costMap = costmap_ros_->getCostmap();
        unsigned char* mapData = costMap->getCharMap();
        unsigned char *map_data = costMap->getCharMap();
        unsigned int size_row = costMap->getSizeInCellsX();
        unsigned int size_col = costMap->getSizeInCellsY();

        // 寻找下一个需要跟踪的目标点
        // 定义一个临时变量，将惯性坐标系的状态转换到本体坐标系
        geometry_msgs::PoseStamped target_body_state;
        for(int i=target_index; i<global_plan_.size(); ++i){
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose("base_link", global_plan_[i], target_body_state);

            // 计算偏差
            linearError = std::sqrt(target_body_state.pose.position.x*target_body_state.pose.position.x + 
                                    target_body_state.pose.position.y*target_body_state.pose.position.y);
            // 当角度变化很小时，角度偏差可近似为正弦值
            angularError = target_body_state.pose.position.y / (0.0001 + target_body_state.pose.position.x);

            // 仅依靠距离误差判断是否为下一个目标点
            if(linearError > 0.2 || i == global_plan_.size()-1){
                cur_pose = target_body_state;
                target_index = i;
                break;
            }
        }


        // 动态避障
        // 获取局部代价地图中目标路径点的参数
        geometry_msgs::PoseStamped cost_target_state;
        // 预先值索引
        int checkObstalIndex = target_index;
        for(int i=10; i>=0; --i){
            if(target_index+i < global_plan_.size()){
                checkObstalIndex = target_index + i;
                break;
            }
        }
        global_plan_[checkObstalIndex].header.stamp = ros::Time(0);
        tf_listener_->transformPose("odom", global_plan_[checkObstalIndex], cost_target_state);
        int cost_target_state_x = (cost_target_state.pose.position.x - costMap->getOriginX()) / costMap->getResolution();
        int cost_target_state_y = (cost_target_state.pose.position.y - costMap->getOriginY()) / costMap->getResolution();
        // 判断路径点是否有障碍物出现
        int mapIndex = cost_target_state_y*size_row + cost_target_state_x;
        if(mapData[mapIndex] >= 253) return false;


        // 使用比例控制计算速度值
        if(linearError > 0.1 && !fineTunning){
            cmd_vel.linear.x = cur_pose.pose.position.x * 1.5;
            cmd_vel.angular.z = angularError * 2.5;
        }
        else if(target_index == global_plan_.size()-1){
            cmd_vel.linear.x = 0;

            // 将四元素转换为欧拉角
            tf::Quaternion quat;
            tf::quaternionMsgToTF(target_body_state.pose.orientation, quat);
            double roll, pitch, yaw;//定义存储r\p\y的容器
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//进行转换

            angularError = yaw;
            cmd_vel.angular.z = angularError * 1;
            fineTunning = true;
        }

        
        return true;
    }


    bool MyPlanner::isGoalReached(){
        if(fineTunning && std::abs(angularError) <= 0.01){
            ROS_WARN("已到达指定位置");
            return true;
        }
        return false;
    }
}