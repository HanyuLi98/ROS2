""" 
需求：以某个固定频率发送文本“hel1 o world1”,文本后提编号，每发送一条消息，编号违增1，
步骤：
1.导入包：
2.初始化R0S2客户端：
3.定义节点类：
3-1.创建发布方：
3-2.创建定时器：
3-3.组织消息并发布。
4.调用spin通数，传入自定义对象;
5.释放资源。



"""
#1.导入包：
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#3.定义节点类：
class Talker(Node):

    def __init__(self):
        #创建父节点，因为父节点没有无参构造
        super().__init__("talker_node_py")
        self.get_logger().info("发布方创建了（py）")

        #设置计数器
        self.count = 0

        #3-1.创建发布方：
        """ 
            参数：
                消息类型
                话题名称
                队列长度
            返回值：发布对象 
        """
        self.publisher = self.create_publisher(String,"chatter",10)
        #3-2.创建定时器：
        """ 
            参数：
                时间间隔
                回调函数
            返回值：定时器对象

        """
        self.timer = self.create_timer(1.0, self.on_timer)
    
    def on_timer(self):
        #3-3.组织消息并发布。
        message = String()
        message.data = "hello world(python)" + str(self.count)
        self.publisher.publish(message)
        self.count += 1
        self.get_logger().info("发布的数据：%s" %message.data)

def main():
    print('Hi from py01_topic.')
#2.初始化R0S2客户端：
    rclpy.init()

#4.调用spin通数，传入自定义对象;
    rclpy.spin(Talker())
#5.释放资源。
    rclpy.shutdown()

    


if __name__ == '__main__':
    main()
