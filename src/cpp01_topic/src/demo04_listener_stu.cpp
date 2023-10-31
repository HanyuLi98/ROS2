/* 
需求：订阅学生信息，并打印在终端。
1.包含头文件；
2.初始化 ROS2 客户端；
3.定义节点类；
    3.1 创建订阅方
    3.2 回调函数订阅并解析数据
4.调用spin函数，并传入节点对象指针；
5.释放资源。
*/
//1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "base_interfaces_demo/msg/student.hpp"

using base_interfaces_demo::msg::Student;


//3.定义节点类；
class ListenerStu: public rclcpp::Node{
public:
    ListenerStu():Node("listenerstu_node_cpp"){
        
        substription_ = this->create_subscription<Student>("chatter_stu", 10, std::bind(&ListenerStu::do_cb, this,std::placeholders::_1));
        //3.1 创建订阅方
    }

private:
                        //引用调用
    void do_cb(const Student &stu){
        //3.2 回调函数订阅并解析数据
        RCLCPP_INFO(this->get_logger(), "订阅的学生信息 name=%s, age=%d, height=%.2f",stu.name.c_str(), stu.age, stu.height);}
    rclcpp::Subscription<Student>::SharedPtr substription_;
};
 
int main(int argc, char const *argv[])
{
    //2.初始化 ROS2 客户端； 
    rclcpp::init(argc,argv);
    //4.调用spin函数，并传入节点对象指针；  
    rclcpp::spin(std::make_shared<ListenerStu>());  
    //5.释放资源。 
    rclcpp::shutdown();
    return 0;
}
