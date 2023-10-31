
""" 

需求：订阅发布方发布的消息，并输出到终端。    

  1.导包；        
  2.初始化 ROS2 客户端；        
  3.定义节点类；            
    3-1.创建订阅方；            
    3-2.解析并输出语句        
  4.调用spin函数，并传入节点对象；        
  5.释放资源。 


"""
#  1.导包；   
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
#   3.定义节点类；
class Listener(Node):

    def __init__(self):
        super().__init__("listener_node_py")
        self.get_logger().info("订阅方创建了（python）")
    #   3-1.创建订阅方；
        """ 
            参数：
                消息类型
                话题名称
                回调函数
                QOS队列长度
            返回值：订阅对象
        """
        self.subscription = self.create_subscription(String,"chatter",self.do_cb,10)
    
    def do_cb(self,msg):               
    #   3-2.解析并输出语句
        self.get_logger().info("订阅的数据: %s" % msg.data)



def main():

#  2.初始化 ROS2 客户端；  
    rclpy.init()

#  4.调用spin函数，并传入节点对象；
    rclpy.spin(Listener())
#  5.释放资源。 
    rclpy.shutdown()

if __name__ == 'main':
    main()

    