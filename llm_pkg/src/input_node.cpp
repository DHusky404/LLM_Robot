#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char** argv){
    // 节点初始化
    ros::init(argc, argv, "input_node");
    setlocale(LC_ALL, "");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    if(argc > 1){
        ros::Publisher pub = nh.advertise<std_msgs::String>("/user_input_topic", 1);
        std_msgs::String msgs;
        msgs.data = argv[1];

        rate.sleep();
        pub.publish(msgs);
        ROS_WARN("finish send: %s", argv[1]);
    }
    else{
        ROS_WARN("请输入任务指令");
    }

    rate.sleep();
    return 0;
}