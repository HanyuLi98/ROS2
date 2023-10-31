/* 
需求：创建客户端，组织数据并提交，然后处理相应结果（需要关注业务流程）。
前提：需要判断main函数中提交的参数是否正确。

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
*/
//1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/srv/add_ints.hpp"
using base_interfaces_demo::srv::AddInts;
using namespace std::chrono_literals;
//3.定义节点类；
class AddIntsClient: public rclcpp::Node{
public:
    AddIntsClient():Node("addIntsClient_node_cpp"){
        RCLCPP_INFO(this->get_logger(),"客户端节点创建");
        //3.1 创建客户端
        /* 
            模板：服务接口
            参数：服务话题名称
            返回参数：服务对象指针
         */

        client_ = this->create_client<AddInts>("add_ints");
    }
        //3.2 连接服务器（对于服务通讯，如果客户端链接不到服务器，那么不能发送请求）
        /* 
            连接服务器实现，如果连接成功返回True，否则返回False
         */
    bool connect_server(){

        //在指定超时时间内连接服务器，如果连接上，那么返回True，否则返回False    
        //client_->wait_for_service(1s);
        while (!client_->wait_for_service(1s))//循环以1s为超时时间连接服务器，直到连接服务器才退出循环。
        {   
            //存在按下ctrl+c停不下来的小bug，可以通过ctrl+g终止
            //对 ctrl+c 这个操作作出特殊处理
            //按下ctrl+c 是结束 ROS2 程序，意味着要释放资源，比如关闭 contex，所以这里不使用 this->get_getlogger(),rclcpp::get_logger不受context影响。
            //1.怎么判断 ctrl+c 按下？ 2. 如何处理？
            // rclcpp 正常执行为true， ctrl+c 之后rclcpp ok 为false
            if (!rclcpp::ok())
            {   
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强行终止客户端！");
                return false;
            }
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"服务连接中！");
        }
        return true;
    }
        
        //3.3 发送请求
        //编写发送请求函数
        /* 参数：两个整型数据
           返回值：提交请求后，服务端的返回结果 */
    rclcpp::Client<AddInts>::FutureAndRequestId send_request(int num1, int num2){
        //组织请求数据

        //发送
        /* 
        返回值
        rclcpp::Client<base_interfaces_demo::srv::AddInts>::FutureAndRequestId 
        参数                                                            
        async_send_request(std::shared_ptr<base_interfaces_demo::srv::AddInts_Request> request) //AddI::Request
         */
        auto request = std::make_shared<AddInts::Request>();
        request->num1 = num1;
        request->num2 = num2;
        
        return client_->async_send_request(request);
    }

    

private:
    rclcpp::Client<AddInts>::SharedPtr client_;
};
 
int main(int argc, char const *argv[])
{   
    //判断main中提交的参数是否正确
    if (argc != 3){                 //给日志起名
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"请提交两个整形数字！");
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"请提交两个整形数字！");
        return 1;
    }


    //2.初始化 ROS2 客户端； 
    rclcpp::init(argc,argv);
    
    //客户端client不需要挂起
    //rclcpp::spin(std::make_shared<AddIntsClient>());  
    
    //4.创建客户端对象
    auto client = std::make_shared<AddIntsClient>();
    //调用客户端对象的连接服务器功能
    bool flag = client->connect_server();
    //根据连接结果作进一步处理
    if (!flag)
    {   
        /* 
            rclcpp::get_logger("name") 创建logger 对象不依赖于context
        
         */
        RCLCPP_INFO(rclcpp::get_logger("rcl.cpp"),"服务器连接失败，程序退出！");
        return 0;
    }
    //执行后续操作...
    //调用请求提交函数，接受并处理相应结果
                            //atoi 字符串（string）转换为整数（integer）。它的名称是 "ASCII to Integer" 的缩写，它将表示整数的字符串转换为整数值。
    auto future = client->send_request(atoi(argv[1]),atoi(argv[2]));

    //处理响应                          //当前节点指针，future指针                      //有SUCCESS，INTERRUPTED，TIMEOUT三种
    if (rclcpp::spin_until_future_complete(client,future) == rclcpp::FutureReturnCode::SUCCESS)//成功
    {
        RCLCPP_INFO(client->get_logger(),"响应成功！sum = %d",future.get()->sum);
    }
    else
    {
        RCLCPP_INFO(client->get_logger(),"响应失败");
    }
    

    //5.释放资源。 
    rclcpp::shutdown();
    return 0;
}