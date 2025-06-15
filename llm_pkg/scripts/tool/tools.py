import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import tf
from actionlib_msgs.msg import GoalStatusArray
from openai import OpenAI
import os
from RAG_database import database


# ============================================================ #
# 发布目标状态
def send_target_state(x: float, y: float, yaw: float):
    # 回调函数
    def status_callback(msg):
        nonlocal goal_reached

        # 状态列表为空，不做处理
        if not msg.status_list:
            return  
        
        # 获取最新消息
        latest_status = msg.status_list[-1]
        # 返回状态编号
        goal_reached = latest_status.status
        # rospy.loginfo(goal_reached)
    

    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(10)

    msgs = PoseStamped() 
    msgs.header.stamp = rospy.Time.now()
    msgs.header.frame_id = "map"

    msgs.pose.position.x = x
    msgs.pose.position.y = y
    msgs.pose.position.z = 0.0

    # 将欧拉角转换为四元素
    quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
    msgs.pose.orientation.x = quat[0]
    msgs.pose.orientation.y = quat[1]
    msgs.pose.orientation.z = quat[2]
    msgs.pose.orientation.w = quat[3]

    goal_reached = 0
    send_flag = False
    sub = rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)

    # 等待机器人运动到目标位置
    while not rospy.is_shutdown():
        if goal_reached!=1 and send_flag==False:
            pub.publish(msgs)
            rate.sleep()
        
        elif goal_reached == 1 and send_flag==False:
            send_flag = True
            rospy.logwarn("目标点已发送")

        elif goal_reached == 3 and send_flag==True:
            sub.unregister()  # 主动取消订阅，回调就不会再被调用
            break
        elif goal_reached == 4 and send_flag==True:
            return "Error: 机器人无法到达该目标点，可能是目标点处存在物品，或目标点不在房间区域，请重新设置目标！！"

    return "机器人已经到达期望位置"



# ============================================================ #
# 获取目标地点的位置
def search_memory(query: str):
    temp = database.search_date(query)
    resluts = "以下都是历史记忆信息，不是当前机器人的状态，切勿混淆:\n"
    rospy.loginfo(temp)
    if len(temp) >= 2:
        resluts += "1. "+ temp[0] + ";\n" + "2. " + temp[1]
    elif len(temp) == 1:
        resluts += temp[0]
    else:
        resluts = "没有相关的记忆内容，请向用户需求帮助"
    return resluts



# ============================================================ #
# 获取机器人当前状态信息
def get_robot_state():
    # 定义一个tf监听器
    listener = tf.TransformListener()

    # 等待tf数载入数据
    rospy.sleep(0.5)

    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)

    # 构造返回值
    result = f"当前机器人的状态为[x={trans[0]}, y={trans[1]}, yaw={yaw}]"

    return result



import base64
import shutil
# ============================================================ #
# 分析机器人看到的场景内容
def analysing_image_content(query_str=""):
    #  base 64 编码格式
    def encode_image(image_path):
        with open(image_path, "rb") as image_file:
            return base64.b64encode(image_file.read()).decode("utf-8")
        
    # 图片复制函数，避免处理图片时，图片发生刷新
    image_path = rospy.get_param('~image_path')
    # 指定源文件和目标路径
    src_file = f"{image_path}/captured_image.png"
    dst_file = f"{image_path}/temp.png"

    # 复制文件
    shutil.copy2(src_file, dst_file)

    # 创建VLM的端口
    client = OpenAI(
        api_key=rospy.get_param('~VLM/api_key'),
        base_url=rospy.get_param('~VLM/base_url')
    )

    # 本地图像的绝对路径
    base64_image = encode_image(dst_file)
    completion = client.chat.completions.create(
        model=rospy.get_param('~VLM/model'),
        messages=[
            {
                "role": "system",
                "content": [{"type": "text", "text": """
                                你是一个移动机器人的视觉中心，需要处理机器人在仿真环境中看到的图像。
                                - 输出时，**描述看到的物品信息**(描述物品时尽量包含物品类别信息，位置信息和数量信息，便于机器人做下一步的决策，必要时可分点描述)；
                                - 如果只看到了物体的一部分也需要说明，如在画面左边看到了“桌角“，也许控制机器人向左转可以看到桌子全貌。
                                """}],
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:image/png;base64,{base64_image}"},
                    },
                    {"type": "text", "text": f"""
                                            请描述一下你看到的图片内容，如果用户需要识别特定的物体或有更加具体的任务需求，将在<query>标签说明。
                                            <query>
                                                {query_str}
                                            </query>
                                        """},
                ],
            },
        ],
    )

    # 返回图片描述
    return completion.choices[0].message.content


# ============================================================ #
from geometry_msgs.msg import PointStamped
import numpy as np
# 将机体坐标系的状态转换到惯性坐标系
def get_robot_pose_in_map(x: float, y: float):
    listener = tf.TransformListener()
    # 等待 tf 中 map → base_link 的变换
    listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))

    # 将平移和旋转四元数组成 4x4 齐次矩阵
    T = listener.fromTranslationRotation(trans, rot)

    point_base = np.array([x, y, 0, 1])
    point_map = np.dot(T, point_base)

    return f"转换后的惯性坐标为[x={point_map[0]}, y={point_map[1]}]"















