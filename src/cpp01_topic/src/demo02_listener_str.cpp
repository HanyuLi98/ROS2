
/* 

需求：订阅发布方发布的消息，并输出到终端。    

  1.包含头文件；        
  2.初始化 ROS2 客户端；        
  3.定义节点类；            
    3-1.创建订阅方；            
    3-2.处理订阅到的消息。        
  4.调用spin函数，并传入节点对象指针；        
  5.释放资源。 

 */

//  1.包含头文件；  
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//   3.定义节点类；        

class Listener:public rclcpp::Node{

public:
    Listener():Node("listener_node_cpp"){
        RCLCPP_INFO(this->get_logger(), "订阅方创建");
//     3-1.创建订阅方； 

/*  
        模板： 消息类型；
        参数： 
            1.消息名称
            2.QOS 队列长度
            3.回调函数
        返回值：订阅对象的指针


 */
//                                                        //订阅的话题，必须跟发布的话题名字一致。//被this当前对象调用 //传入的参数是订阅到的消息，使用占为符
        subscription_ = this->create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&Listener::do_cb, this, std::placeholders:: _1));
        /*
        f(x, y)
        auto g = bind(f, 42);
        g(x) = f(42, x)
        */
    }

private:                                 //为方便使用引用类型
    void do_cb(const std_msgs::msg::String &msg){
//         3-2.处理订阅到的消息。
    RCLCPP_INFO(this->get_logger(),"订阅到的消息是：%s", msg.data.c_str());

    }

    //成员变量
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

};



int main(int argc, char const *argv[])
{
    //2.初始化 ROS2 客户端；   
    rclcpp::init(argc,argv);
    //4.调用spin函数，并传入节点对象指针；  
    rclcpp::spin(std::make_shared<Listener>());  
    //5.释放资源。 
    rclcpp::shutdown();
    return 0;
}

