"""
需求：以固定频率发布学生信息
流程：
    1.导包
    2.初始化ROS2客户端
    3.自定义节点
        创建发布方
        创建定时器
        组织并发布学生信息
    4.调用spin函数，并传入节点对象
    5.资源释放
"""
 
# 1.导包
import rclpy
from rclpy.node import Node

from base_interfaces_demo.msg import Student

#自定义节点类
class TalkerStu(Node):
    def __init__(self):
        super().__init__("talkerstu_node_py")
        self.count = 0
        #创建发布方
        self.publisher = self.create_publisher(Student,"chatter_stu",10)
        #创建定时器
        self.timer = self.create_timer(0.5, self.on_timer)
        

    def on_timer(self):
        #组织并发布学生信息
        stu = Student()
        stu.name = "奥特曼"
        stu.age = self.count
        stu.height = 1.40
        self.publisher.publish(stu)
        self.count += 1
        self.get_logger().info("学生信息(%s,%d,%.2f)"%(stu.name, stu.age, stu.height))
def main():
    #2.初始化ROS2客户端
    rclpy.init()
    #4.调用spin函数，并传入节点对象
    rclpy.spin(TalkerStu())
    #5.资源释放
    rclpy.shutdown()

if __name__ == "__main__":
    main()