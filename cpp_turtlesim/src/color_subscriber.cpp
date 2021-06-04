#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/color.hpp"
using std::placeholders::_1;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("TurtleSim_Color_Subscriber")
    {
      subscription_ = this->create_subscription<turtlesim::msg::Color>(
      "/turtle1/color_sensor", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const turtlesim::msg::Color::SharedPtr msg) const
    {
      std::cout<< "-----//-----"<< std::endl;
      std::cout<< unsigned(msg->r)<< std::endl;
      std::cout<< unsigned(msg->g)<< std::endl;
      std::cout<< unsigned(msg->b)<< std::endl;
      
    }
    rclcpp::Subscription<turtlesim::msg::Color>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}