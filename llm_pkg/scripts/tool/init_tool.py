from . import tools

# 函数映射表
function_mapper = {
    "send_target_state":            tools.send_target_state,
    "search_memory":                tools.search_memory,
    "get_robot_state":              tools.get_robot_state,
    "analysing_image_content":      tools.analysing_image_content,
    "get_robot_pose_in_map":        tools.get_robot_pose_in_map,
}


# 工具列表
tool_list = [

# 1. 发送期望位置
{
    "type": "function",
    "function": {
        "name": "send_target_state",
        "description": "发送期望到达的目标位置，机器人将自动前往",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "float",
                    "description": "惯性坐标系下，期望到达的目标位置横坐标，单位：m",
                },
                "y": {
                    "type": "float",
                    "description": "惯性坐标系下，期望到达的目标位置纵坐标，单位：m",
                },
                "yaw": {
                    "type": "float",
                    "description": "惯性坐标系下，期望到达的目标位置偏航角，单位：rad",
                }
            },
            "required": ["x", "y", "yaw"]
        }
    }
},

# 2. 获取目标地点的信息
{
    "type": "function",
    "function": {
        "name": "search_memory",
        "description": "保存了机器人运动的历史记忆，当需要回忆某个地点的惯性坐标位置或信息时，可以调用该函数",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "需要回忆的信息",
                },
            },
            "required": ["name"]
        }
    }
},

# 3. 获取机器人的当前状态
{
    "type": "function",
    "function": {
        "name": "get_robot_state",
        "description": "获取机器人在惯性坐标系下的当前状态",
        "parameters": {
            "type": "object",
            "properties": {},
            "required": []
        }
    }
},

# 4. 分析机器人摄像机拍摄的画面信息
{
    "type": "function",
    "function": {
        "name": "analysing_image_content",
        "description": "分析和理解当前机器人所携带摄像机拍摄的画面信息",
        "parameters": {
            "type": "object",
            "properties": {
                "query_str": {
                    "type": "string",
                    "description": "可以提问更加具体的问题，如杯子的颜色，椅子的个数等",
                },
            },
            "required": []
        }
    }
},

# 5. 将机体坐标系的状态转换到惯性坐标系
{
    "type": "function",
    "function": {
        "name": "get_robot_pose_in_map",
        "description": "将机器人在机体坐标系的状态转换到惯性坐标系",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "float",
                    "description": "在机体坐标系下，机器人的横坐标",
                },
                "y": {
                    "type": "float",
                    "description": "在机体坐标系下，机器人的纵坐标",
                },
            },
            "required": ["x", "y"]
        }
    }
},
]










