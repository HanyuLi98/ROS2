/* 
需求：编写动作服务端，需要解析客户端提交的数据数字，遍历该数字并累加求和，最终结果响应回客户端
且请求响应过程中，需要生成连续反馈。
分析：
    1.创建动作服务端对象
    2.处理提交的目标值
    3.生成连续反馈
    4.响应最终结果
    5.处理取消请求
流程：
1.包含头文件；
2.初始化 ROS2 客户端；
3.定义节点类；
    3.1 创建动作服务端对象
    3.2 处理提交的目标值 （回调函数）
    3.3 处理取消请求（回调函数）
    3.4 生成连续反馈与最终响应（回调函数）
4.调用spin函数，并传入节点对象指针；
5.释放资源。
*/
//1.包含头文件；
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_interfaces_demo/action/progress.hpp"
using base_interfaces_demo::action::Progress;
using std::placeholders::_1;
using std::placeholders::_2;
//3.定义节点类；
class ProgressActionServer: public rclcpp::Node{
public:
    ProgressActionServer():Node("progress_action_server_node_cpp"){
      RCLCPP_INFO(this->get_logger(), "action 服务端创建！");
      // 3.1 创建动作服务端对象
      /* 
      返回值
      rclcpp_action::Server<ActionT>::SharedPtr 
      模板          
      create_server<ActionT, NodeT>, action给出，node 默认
      参数
      (NodeT node, const std::string &name, 
      rclcpp_action::Server<ActionT>::GoalCallback handle_goal, 
      rclcpp_action::Server<ActionT>::CancelCallback handle_cancel,
      rclcpp_action::Server<ActionT>::AcceptedCallback handle_accepted, 
      const rcl_action_server_options_t &options = rcl_action_server_get_default_options(),
      rclcpp::CallbackGroup::SharedPtr group = nullptr)
       */

      //ctrl + create_server, 看源码，再 ctrl + typername 后的 Server，可以看到goal/cancel/accepted的定义
      server_ = rclcpp_action::create_server<Progress>(
        this,
        "get_sum",
        std::bind(&ProgressActionServer::handle_goal,this,_1,_2),
        std::bind(&ProgressActionServer::handle_cancel,this,_1),
        std::bind(&ProgressActionServer::handle_accepted,this,_1)
        );
    }
    
    // 3.2 处理提交的目标值 （回调函数）
    //std::function<GoalResponse(const GoalUUID &, std::shared_ptr<const typename ActionT::Goal>)>;      
    rclcpp_action::GoalResponse handle_goal (const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Progress::Goal> goal){
        (void) uuid; //没有使用的参数需要void声明一下。
        //业务逻辑：判断提交的数字是否大于1,是就接受，否则就拒绝
        if(goal->num <= 1){
            RCLCPP_INFO(this->get_logger(), "提交的目标必须大于1");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(), "提交的目标值合法！");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }
    // 3.3 处理取消请求（回调函数） 
    //std::function<void (std::shared_ptr<ServerGoalHandle<ActionT>>)>;

    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(),"接受到任务取消请求。");
        //当前是无条件统一
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // 3.4 生成连续反馈与最终响应（回调函数）
    void execute(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
        (void)goal_handle;
        //1.需要生成连续反馈,返回给客户端
        //void publish_feedback(std::shared_ptr<base_interfaces_demo::action::Progress_Feedback> feedback_msg)
        //goal_handle->publish_feedback();
        //首先要获取目标值，然后遍历，遍历中进行累加，且每循环一次，就计算进度，并作为连续反馈发布。
        int num = goal_handle->get_goal()->num;
        int sum = 0;
        auto feedback = std::make_shared<Progress::Feedback>();
        //设置休眠
        rclcpp::Rate rate(1.0); //1hz 一秒钟一次

        auto result = std::make_shared<Progress::Result>();

        for (int i = 1; i <= num; i++)
        {
            sum += i;
            double progress = i / (double)num; //计算进度
            feedback->progress = progress;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(),"连续反馈中，进度%2f",progress);

            //判断是否接收到了取消请求,使用ctrl+c取消
            if(goal_handle->is_canceling()){
            //goal_handle->is_canceling();
            //goal_handle->canceled();
                goal_handle->canceled(result);
                result->sum = sum;
                RCLCPP_INFO(this->get_logger(),"任务被取消了");
            //如果接收到了，终止程序，return
                return;
            }
            rate.sleep();//会休眠1s
        }
        
        
        //2.生成最终响应结果
        //void succeed(std::shared_ptr<base_interfaces_demo::action::Progress_Result> result_msg)
        //goal_handle->succeed();

        if(rclcpp::ok()){
            
            result->sum = sum;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(),"最终结果%d",sum);

        }

    }
    //std::function<CancelResponse(std::shared_ptr<ServerGoalHandle<ActionT>>)>;  
    void handle_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<Progress>> goal_handle){
        //生成连续反馈，最终结果是一个费时的过程
        //新建子线程，处理耗时的主逻辑实现,匿名线程
        std::thread(std::bind(&ProgressActionServer::execute, this, goal_handle)).detach();

    }           
private:
  rclcpp_action::Server<Progress>::SharedPtr server_;        
};
 
int main(int argc, char const *argv[])
{
    //2.初始化 ROS2 客户端； 
    rclcpp::init(argc,argv);
    //4.调用spin函数，并传入节点对象指针；  
    rclcpp::spin(std::make_shared<ProgressActionServer>());  
    //5.释放资源。 
    rclcpp::shutdown();
    return 0;
}