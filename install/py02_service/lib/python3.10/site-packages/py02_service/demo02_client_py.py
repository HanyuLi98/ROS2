"""
需求：编写客户端实现，提交两个整形数据，并处理响应结果
前提：需要判断main函数中提交的参数是否正确。
流程：
    1.包含头文件；
    2.初始化 ROS2 客户端；
    3.定义节点类；
        3.1 创建客户端
        3.2 连接服务器（对于服务通讯，如果客户端链接不到服务器，那么不能发送请求）
        3.3 发送请求
    4.创建对象指针；
        需要调用连接服务的函数，根据连接结果作下一步处理
        连接服务后，调用请求发送函数
        再处理相应结果
    5.释放资源。
"""
 
# 1.导包
import rclpy
from rclpy.node import Node
#没有实例化 仍然使用getlogger
from rclpy.logging import get_logger
#sys里面有argv
from base_interfaces_demo.srv import AddInts
import sys



#自定义节点类
class AddIntsClient(Node):
    def __init__(self):
        super().__init__("add_ints_client_node_py")
        self.get_logger().info("客户端创建了！（python）")
        
        #3.1 创建客户端
        self.client = self.create_client(AddInts,"add_ints")

        #3.2 连接服务器（对于服务通讯，如果客户端链接不到服务器，那么不能发送请求）
        # 1s内连接上了 返回true ，连接不上 返回false
        while (not self.client.wait_for_service(1.0)):
            self.get_logger().info("服务连接中！")

    #3.3 发送请求
    def send_request(self):
        #创建对象
        request = AddInts.Request()
        request.num1 = int(sys.argv[1])
        request.num2 = int(sys.argv[2])
        self.future = self.client.call_async(request)
        

def main():
    #校验操作
    if len(sys.argv)!=3:
                        # .info
        get_logger("rclpy").error("请提交两个整型数据！")
        return
    
    #2.初始化ROS2客户端
    rclpy.init()
    client = AddIntsClient()
    #发送请求
    client.send_request()
    #处理响应                          接受服务段响应结果
    rclpy.spin_until_future_complete(client,client.future)
    #来判断通过future获取响应对象能否成功，成功则继续执行
    try:
        #返回响应对象
        response = client.future.result()
        client.get_logger().info("响应结果 sum = %d" % response.sum)
    except Exception:
        client.get_logger().error("服务响应失败！")

    #客户端不需要spin函数挂起
    #rclpy.spin(AddIntsClient())
    
    #5.资源释放
    rclpy.shutdown()

if __name__ == "__main__":
    main()