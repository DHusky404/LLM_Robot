#!/usr/bin/env python3
#coding=utf-8

import rospy
from std_msgs.msg import String
from openai import OpenAI
import os
import prompt
from tool.init_tool import tool_list
from function_call import function_calling
import json


# 与大模型对话
def get_response(msg_log: str, tool_list = tool_list):
    global model_engin
    completion = client.chat.completions.create(
        model = model_engin,
        messages = msg_log,
        extra_body={
        "enable_thinking": True,
        "thinking_budget": 50
        },
        tools=tool_list,
    )

    return completion



# 接收问题字符串回调函数
def cbQuestion(msg):
    rospy.loginfo("--------------------")
    # 打印用户提问的信息
    rospy.loginfo("用户：" + msg.data)

    global conversation
    global msg_log
    if(conversation == True):
        msg_log.append({"role": "user", "content": msg.data})
    else:
        systemPrompt = """
                        现在你作为差速双轮移动机器人的决策中心，请分析用户的任务需求，可以使用所提供的工具完成任务。标签内的描述摘要如下：
                        - 在当前环境中，机器人使用了两种坐标系：一种是**惯性坐标系**，另一种是**机体坐标系**。坐标系的详细描述在<coordinate>中
                        - 标签<environment>内保存的是你所处于的全局环境说明;
                        - 标签<state>是对机器人状态的说明，包括状态的定义和运动学特征等，在控制机器人运动时请遵守动力学规则；
                        - <requirement>表示每次思考、调用函数，或者回答用户的指令需要遵守的规则；
                        - <search>说明了在搜寻任务中的一种寻找方案；
                        - <orientations>说明了机器人使用机体坐标系的场景；
                        - <moving>当机器人看到目标物体，需要移动到物体旁却没有具体坐标时可以通过“前进”、“向左”、“向右”慢慢移动靠近。

                        <coordinate>
                            1. **惯性坐标系**是一种**静态**的笛卡尔直角坐标系，它以机器人的初始位置为原点固定在地图中，用于描述机器人和房间内各个物品的位置和姿态；
                            2. **机体坐标系**也可以称为机器人的本体坐标系，**固连在机器人本体，会随着机器人的运动而运动**，其x轴指向机器人前进方向，y轴指向机器人左侧方向。
                                机体坐标系不用于导航，**只在涉及到自身方向时使用**，如前进一段距离，向左移动一段距离等。
                            **切记：机体坐标系通常不用于导航，它的存在只是让你知道机器人的前进方向或者左侧是哪里，如果要确定期望状态需要转换到惯性坐标系**
                        </coordinate>

                        <environment>
                            - 移动机器人现在处于用户的家中，共包括一个卧室，一个餐厅，一个客厅和一个书房，机器人开始位于餐厅；
                            - 你可以通过**analysing_image_content**分析摄像头看到的内容，以更准确地知道自己处于什么区域；
                            - 当你不记得某个地点在惯性坐标系下的坐标时，可以调用**search_memory**函数读取之前记忆的坐标信息，**需要注意的是，你应该仔细分析输出的结果，
                                如果没有包含目标点信息则不要随意控制机器人，而是进一步询问用户**；
                        </environment>

                        <state>
                            - **在惯性坐标系下**，移动机器人的状态变量包括[横坐标x, 纵坐标y, 偏航角yaw]，可以使用**get_robot_state**函数获取在惯性坐标系下机器人当前的状态信息；
                            - 控制机器人移动的工具已经实现了，如果你想让机器人运动到指定位置，可以使用**send_target_state**函数给机器人底盘发送期望状态信息（惯性坐标系下的坐标）；
                            - 当机器人运动到目标附近而需要进一步动作时，**记住“不要一次性移动太多”，避免机器人发生碰撞**，可以进行零点几米的微调；
                        </state>

                        <search>
                            - 当你在房间里找不到物品时，请尝试沿着逆时针方向旋转一定角度（比如0.34rad），重新看看相机画面是否有目标物体。通过不断的旋转更新画面寻找物品，
                                 当旋转一圈即$2*\pi$弧度后，若还未找到物品，则请求用户给出进一步的提示。
                        </search>

                        <orientations>
                            - 机器人的车头正方向为前进方向，**前进方向也是机体坐标系（运动坐标系）的x轴方向**；此外，描述机器人状态使用的是惯性坐标系（静坐标系），请注意区分。
                            - 请注意区分诸如“前进1m”，“向左移动5m”等明确机器人运动方向的指令，当以机器人的方向为基准时需要使用机体坐标系。如“前进1m”对应的是机体坐标系[1,0]的目标位置，
                              如果向让机器人运动至该目标位置，需要使用**get_robot_pose_in_map**函数转换到惯性坐标系后，使用**send_target_state**发布惯性坐标系下的目标位置指导机器人运动。
                        </orientations>

                        <requirement>
                            1. 请在每次回答问题前仔细思考，避免逻辑漏洞；
                            2. 请在调用工具执行命令前，仔细思考这样做的逻辑是否正确；
                            3. 当一个任务需要多步执行时，请反思每一步的动作；
                            4. 确定当前方位的时候，请采用**analysing_image_content**分析自己所处的区域，避免使用“应该”，“大概”等模糊词汇回答问题；
                        </requirement>

                        <moving>
                            - 移动思路参考：当目标物体位于画面右前方时，可以直接在机体坐标输入[1,-1]，然后转换至惯性坐标系使机器人朝着右前方移动，然后再次
                                分析图片信息，以规划下一步的运动，最终慢慢靠近目标物体；
                            - 靠近目标物体至少要保证画面内没有其他物体的遮挡，且返回目标物体的所在区域信息和位置情况（如在客厅的茶几旁边）。
                        </moving>
                    """
        msg_log = prompt.prompt_template(system=systemPrompt, user=msg.data)
        conversation = True


    # 调用大模型进行回答
    completion = get_response(msg_log)
    assistant_output = completion.choices[0].message
    msg_log.append(assistant_output)
    iter = 1

    # 如果需要调用工具，则进行模型的多轮调用，直到模型判断无需调用工具
    while assistant_output.tool_calls != None:
        rospy.logwarn(f"开始{iter}轮调用")
        rospy.logwarn("Reasoning: " + assistant_output.content)

        # 解析调用函数所需的输入参数
        argumens = assistant_output.tool_calls[0].function.arguments
        fun_name = assistant_output.tool_calls[0].function.name
        function_output = function_calling(fun_name, argumens)

        rospy.logwarn(f"函数调用结果：{function_output}")

        # 将结果返回给大模型，进行下一步操作
        msg_log.append({"role": "tool", "content": function_output, "tool_call_id": assistant_output.tool_calls[0].id})
        assistant_output = get_response(msg_log).choices[0].message
        msg_log.append(assistant_output)
        rospy.logwarn(f"完成{iter}轮调用")
        iter += 1

    # 调用函数结束后回复用户的问题
    rospy.loginfo("结束调用")
    rospy.logwarn("Qwen：" + assistant_output.content)

    

# 主函数
if __name__ == "__main__":
    # 初始化节点参数
    rospy.init_node("llm_node")
    # 建立LLM的客户端
    model_engin = rospy.get_param('~decision/model')
    client = OpenAI(base_url = rospy.get_param('~decision/base_url'),
                    api_key = rospy.get_param('~decision/api_key'))
    
    rospy.logwarn("ChatGPT: 当前使用模型 %s",model_engin)

    # 订阅外部输入的问话
    question_sub = rospy.Subscriber("/user_input_topic", String, cbQuestion, queue_size=1)
    msg_log = []
    # 是否开启多轮对话
    conversation = False

    # 等待回调函数
    rospy.spin()