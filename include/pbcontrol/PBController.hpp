#include "rclcpp/rclcpp.hpp"


struct SingleFeedbackPolicy
{

  typedef std_msgs::msg::String FeedbackMsg;
  typedef std_msgs::msg::String CommandMsg;

protected:
  ~SingleFeedbackPolicy(){}
};

struct MultipleFeedbackPolicy
{
  typedef std::vector<std_msgs::msg::String> FeedbackMsg;
  typedef std_msgs::msg::String CommandMsg;

protected:
  ~MultipleFeedbackPolicy(){}
};

/*
template<typename T>
using IsNotArray = std::enable_if_t<!std::is_array_v<T> >;

template<typename T_ = T, typename = IsNotArray<T_> >
void some_func(const T &value){}
*/

struct DefaultString
{
  std::string base;
  uint32_t value;
  DefaultString(const std::string &base) : base(base), value(0U) {}

  operator std::string() const
  {
    std::stringstream ss;
    ss << base << "_" << value;
    return ss.str();
  }

  std::string operator++(std::string)
  {
    value++;
    return *this;
  }
};

/*
template<class ControlPolicy>
class Controller : public rclcpp::Node, public: ControlPolicy
{
public:
  Controller(const std::string &name)
    : Node(name)
  {
  }

  

};
*/


int main(int argv, char **argc)
{
  DefaultString str("foobar");
  std::cout << str << std::endl;
  std::cout << str++ << std::endl;
  std::cout << str << std::endl;
  std::cout << str++ << std::endl;
  std::cout << str << std::endl;

  return 0;
}
