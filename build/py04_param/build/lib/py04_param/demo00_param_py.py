"""
需求：演示参数API使用。
流程：
    1.导包
    2.初始化ROS2客户端
    3.自定义节点
        3.1 参数对象创建
        3.2 参数对象解析（获取键，值，将值转换成字符串...）
    4.调用spin函数，并传入节点对象
    5.资源释放
"""
 
# 1.导包
import rclpy
from rclpy.node import Node

#自定义节点类
class MyPara(Node):
    def __init__(self):
        super().__init__("my_para_node_py")
        self.get_logger().info("参数API使用（python）")
        
        #3.1 参数对象创建
        p1 = rclpy.Parameter("car_name",value="Tiger")
        p2 = rclpy.Parameter("width",value=1.5)
        p3 = rclpy.Parameter("wheels",value=2)

        #3.2 解析参数
        self.get_logger().info("car_name = %s" % p1.value)
        self.get_logger().info("width = %.2f" % p2.value)
        self.get_logger().info("wheels = %d" % p3.value)
        
        self.get_logger().info("key = %s" % p1.name)
        
        


def main():
    #2.初始化ROS2客户端
    rclpy.init()
    #4.调用spin函数，并传入节点对象
    rclpy.spin(MyPara())
    #5.资源释放
    rclpy.shutdown()

if __name__ == "__main__":
    main()