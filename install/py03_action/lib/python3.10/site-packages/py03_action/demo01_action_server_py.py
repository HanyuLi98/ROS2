"""
需求：编写动作服务端，需要解析客户端提交的数据数字，遍历该数字并累加求和，最终结果响应回客户端
且请求响应过程中，需要生成连续反馈。
分析：
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈
    4.响应最终结果
    5.处理取消请求
流程：
1.包含头文件；
2.初始化 ROS2 客户端；
3.定义节点类；
    3.1 创建动作客户端
    3.2 发送请求
    3.3 处理关于目标值的服务端响应（回调函数）
    3.4 处理连续反馈（回调函数）
          3.5 处理最终响应结果（回调函数）
4.调用spin函数，并传入节点对象指针；
5.释放资源。
"""
 
# 1.导包
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress
#自定义节点类
class ProgressActionClient(Node):
    def __init__(self):
        super().__init__("progress_action_client_node_py")
        self.get_logger().info("动作通讯服务端创建")
        #3.1 创建动作服务端对象
        #node: Any, action_type: Any, action_name: Any, execute_callback: Any,
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback

        )
    # 3.2 处理提交的目标值 （回调函数）----有默认实现（无论提交什么都会被接受）
    # 3.3 处理取消请求（回调函数）----有默认实现（无论提交什么都会被拒绝）ctrl+ ActionServer 180行，可以进行冲在
    # 3.4 生成连续反馈与最终响应（回调函数）
    def execute_callback(self,goal_handle):
        #1.生成连续反馈
        #首先要获取目标值，然后遍历，遍历中进行累加，且每循环一次，就计算进度，并作为连续反馈发布。
        # goal_handle的类是ActionServer,可以用Ctrl+ActionServer去看下面有什么
        num = goal_handle.request.num
        sum = 0
        for i in range(1, num+1):
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
                                            # 注意这里有空格
            self.get_logger().info("连续反馈:%2f" % feedback.progress)
            time.sleep(1.0)        

        #2.响应最终结果
        goal_handle.succeed()
        result = Progress.Result()
        result.sum = sum 
        self.get_logger().info("计算结果:%d" % result.sum)
        return result


def main():
    #2.初始化ROS2客户端
    rclpy.init()
    #4.调用spin函数，并传入节点对象
    rclpy.spin(ProgressActionClient())
    #5.资源释放
    rclpy.shutdown()

if __name__ == "__main__":
    main()