"""
需求：
流程：
    1.导包
    2.初始化ROS2客户端
    3.自定义节点
    4.调用spin函数，并传入节点对象
    5.资源释放
"""
 
# 1.导包
import rclpy
from rclpy.node import Node

#自定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__("mynode_node_py")

def main():
    #2.初始化ROS2客户端
    rclpy.init()
    #4.调用spin函数，并传入节点对象
    rclpy.spin(MyNode)
    #5.资源释放
    rclpy.shutdown()

if __name__ == "__main__":
    main()