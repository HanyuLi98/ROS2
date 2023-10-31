/* 
需求：以某个固定频率发送文本“hel1 o world1”,文本后提编号，每发送一条消息，编号违增1，
步骤：
1.包含头文件：
2.初始化R0S2客户端：
3.定义节点类：
3-1.创建发布方：
3-2.创建定时器：
3-3.组织消息并发布。
4.调用spin通数，并传入节点对象指针，
5.释放资源。

 */



//1.包含头文件：
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//std::chrono库用于处理时间和持续时间的操作，以提供更精确的时间测量和处理功能
using namespace std::chrono_literals;
//3.定义节点类：
class Talker:public rclcpp::Node{

public:                           //构造函数count初始化
  Talker():Node("talker_node_cpp"),count(0){
    RCLCPP_INFO(this->get_logger(),"发布节点创建");
    //3-1.创建发布方：                                    
    /* 
        模板：被发布的消息类型；
        参数：
          1.话题名称；
          2.QOS(消息队列长度)
        返回值：发布对象的指针
     */
                                                    //队列长度为10,在网路堵塞的时候最多可以放置10条数据
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter",10);
    
    //3-2.创建定时器： 
    /* 
        参数：
          1.时间间隔；
          2.回调函数；
        返回值：定时器对象指针

     */

    /* std::bind(&Talker::on_timer, this): 这部分使用了std::bind函数，它将&Talker::on_timer与当前类的this指针绑定在一起，
      创建了一个可调用对象。&Talker::on_timer是成员函数指针，表示要在定时器触发时调用的成员函数。this指针用于指定成员函数的对象。
      综合起来，这行代码的作用是创建一个定时器，每隔1秒触发一次，并且在触发时调用当前类（Talker类）的on_timer成员函数。
      通常，这种技巧用于在定时器触发时执行特定的任务，例如周期性地发布消息或执行其他操作。 */

    
                                                //&Talker 当前类
    timer_ =this->create_wall_timer(1s,std::bind(&Talker::on_timer,this));

  }

private:
  void on_timer(){
    //3-3.组织消息并发布。
    auto message = std_msgs::msg::String();
    message.data = "hello world!" + std::to_string(count++);
    RCLCPP_INFO(this->get_logger(), "发布方发布的消息：%s", message.data.c_str());
    publisher_->publish(message);

  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count;
};
  

int main(int argc, char ** argv)
{

//2.初始化R0S2客户端：
  rclcpp::init(argc, argv);

//4.调用spin通数，并传入节点对象指针。 spin函数会等待回调函数。
  rclcpp::spin(std::make_shared<Talker>());

//5,释放资源。
  rclcpp::shutdown();
  printf("hello world cpp01_topic package\n");
  return 0;
}

