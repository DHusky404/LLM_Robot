import json
from tool.init_tool import function_mapper
import rospy

def function_calling(function_name: str, arguments_string, function_mapper = function_mapper):
    """
        输入变量：
            function_name    --- 需要调用的函数名称
            arguments_string --- 调用函数的输入参数（JSON格式）
            function_mapper  --- 函数映射表（默认值即可）
    """
    
    rospy.logwarn(f"正在调用函数:{function_name}")
    # 使用json模块解析参数字符串
    arguments = json.loads(arguments_string)
    # 获取函数实体
    function = function_mapper[function_name]
    rospy.logwarn(f"输入参数:{arguments}")
    # 如果入参为空，则直接调用函数
    function_output = function(**arguments)

    return function_output