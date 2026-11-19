#pragma once
// Native ROS 2 transport adapter for EPIC's algorithm modules. No ROS 1 runtime.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <sstream>
#include <thread>
#include <type_traits>
#include <typeindex>
#include <algorithm>
using boost::placeholders::_1;
using boost::placeholders::_2;
namespace epic_ros {
inline rclcpp::Node::SharedPtr& storage(){static rclcpp::Node::SharedPtr n;return n;}
inline rclcpp::Node::SharedPtr node(){if(!storage())throw std::logic_error("EPIC ROS 2 node not initialized");return storage();}
inline rclcpp::Logger logger(){return rclcpp::get_logger("epic");}
inline void init(int argc,char** argv,const std::string& name){rclcpp::init(argc,argv);storage()=std::make_shared<rclcpp::Node>(name,rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));}
inline bool ok(){return rclcpp::ok();}
inline void spin(){rclcpp::spin(node());}
inline void spinOnce(){rclcpp::spin_some(node());}
inline void shutdown(){rclcpp::shutdown();storage().reset();}
struct Duration {
 double value=0.;Duration(double s=0.):value(s){}double toSec()const{return value;}
 void sleep()const{if(value>0)std::this_thread::sleep_for(std::chrono::duration<double>(value));}
};
struct Time {
 int64_t ns=0;Time(double s=0.):ns(static_cast<int64_t>(s*1e9)){}
 Time(const builtin_interfaces::msg::Time& t):ns(int64_t(t.sec)*1000000000LL+t.nanosec){}
 static Time now(){Time t;t.ns=node()->now().nanoseconds();return t;}
 double toSec()const{return ns*1e-9;}
 operator builtin_interfaces::msg::Time()const{return static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(ns,RCL_ROS_TIME));}
 operator rclcpp::Time()const{return rclcpp::Time(ns,RCL_ROS_TIME);}
 Time operator+(Duration d)const{Time t;t.ns=ns+static_cast<int64_t>(d.value*1e9);return t;}
 Time operator-(Duration d)const{return *this+Duration(-d.value);}
 Duration operator-(Time t)const{return Duration((ns-t.ns)*1e-9);}
 bool operator<(Time t)const{return ns<t.ns;}bool operator>(Time t)const{return ns>t.ns;}
 bool operator<=(Time t)const{return ns<=t.ns;}bool operator>=(Time t)const{return ns>=t.ns;}
 bool operator==(Time t)const{return ns==t.ns;}bool operator!=(Time t)const{return ns!=t.ns;}
};
struct TimerEvent{};
class Timer {
 rclcpp::TimerBase::SharedPtr timer_;
 public:Timer()=default;explicit Timer(rclcpp::TimerBase::SharedPtr t):timer_(std::move(t)){}
 void stop(){if(timer_)timer_->cancel();}void start(){if(timer_)timer_->reset();}
};
struct TransportHints{TransportHints& tcpNoDelay(){return *this;}};
class Publisher {
 // ROS1-style globals may outlive storage(); keep their ROS2 node alive.
 rclcpp::Node::SharedPtr owner_;
 std::shared_ptr<void> handle_;std::function<void(const void*)> send_;std::function<size_t()> count_;std::type_index type_{typeid(void)};
 public:Publisher()=default;
 template<class M>explicit Publisher(std::shared_ptr<rclcpp::Publisher<M>> p):owner_(node()),handle_(p),type_(typeid(M)){
 send_=[p](const void* m){p->publish(*static_cast<const M*>(m));};count_=[p](){return p->get_subscription_count();};}
 template<class M>void publish(const M& m)const{if(!send_)return;if(type_!=std::type_index(typeid(M)))throw std::logic_error("EPIC publisher message mismatch");send_(&m);}
 template<class M>void publish(const std::shared_ptr<M>& m)const{publish(*m);}
 size_t getNumSubscribers()const{return count_?count_():0;}explicit operator bool()const{return bool(handle_);}
};
using Subscriber=rclcpp::SubscriptionBase::SharedPtr;
template<class T>struct MessageArg{using type=T;static constexpr bool shared=false;};
template<class T>struct MessageArg<std::shared_ptr<T>>{using type=std::remove_const_t<T>;static constexpr bool shared=true;};
class NodeHandle {
 rclcpp::Node::SharedPtr owner_;
 std::string prefix_;
 static std::string key(std::string s){std::replace(s.begin(),s.end(),'/','.');while(!s.empty()&&s.front()=='.')s.erase(s.begin());return s;}
 template<class T> static void assign(const rclcpp::Parameter& p,T& v){
  if constexpr(std::is_same_v<T,bool>)v=p.as_bool();
  else if constexpr(std::is_integral_v<T>)v=static_cast<T>(p.as_int());
  else if constexpr(std::is_floating_point_v<T>)v=p.get_type()==rclcpp::ParameterType::PARAMETER_INTEGER?static_cast<T>(p.as_int()):static_cast<T>(p.as_double());
  else if constexpr(std::is_same_v<T,std::vector<int>>){auto a=p.as_integer_array();v.assign(a.begin(),a.end());}
  else v=p.get_value<T>();
 }
 public:explicit NodeHandle(const std::string& p=""):owner_(storage()){if(p!="~")prefix_=key(p);}
 NodeHandle(const NodeHandle& n,const std::string& p):owner_(n.owner_),prefix_(n.prefix_.empty()?key(p):n.prefix_+"."+key(p)){}
 rclcpp::Node::SharedPtr native()const{return owner_?owner_:node();}
 template<class T>bool getParam(const std::string& name,T& value)const{
  auto k=key(name);if(!prefix_.empty()&&name.front()!='/')k=prefix_+"."+k;
  if(!node()->has_parameter(k))throw std::runtime_error("Missing EPIC parameter: "+k);
  assign(node()->get_parameter(k),value);return true;
 }
 template<class T,class D>void param(const std::string& name,T& value,const D& def)const{
  auto k=key(name);if(!prefix_.empty()&&name.front()!='/')k=prefix_+"."+k;
  if(!node()->has_parameter(k)){value=static_cast<T>(def);return;}assign(node()->get_parameter(k),value);
 }
 template<class M>Publisher advertise(const std::string& topic,int depth,bool latch=false)const{
  auto qos=rclcpp::QoS(std::max(1,depth));if(latch)qos.transient_local();return Publisher(node()->create_publisher<M>(topic,qos));
 }
 template<class A,class C>Subscriber subscribe(const std::string& topic,int depth,void(C::*f)(A),C* object,TransportHints={})const{
  using Arg=std::remove_cv_t<std::remove_reference_t<A>>;using M=typename MessageArg<Arg>::type;
  auto qos=rclcpp::QoS(std::max(1,depth));if constexpr(std::is_same_v<M,nav_msgs::msg::Odometry>||std::is_same_v<M,sensor_msgs::msg::PointCloud2>)qos.best_effort();
  return node()->create_subscription<M>(topic,qos,[object,f](typename M::SharedPtr m){if constexpr(MessageArg<Arg>::shared)(object->*f)(m);else (object->*f)(*m);});
 }
 template<class A>Subscriber subscribe(const std::string& topic,int depth,void(*f)(A))const{
  using Arg=std::remove_cv_t<std::remove_reference_t<A>>;using M=typename MessageArg<Arg>::type;
  auto qos=rclcpp::QoS(std::max(1,depth));if constexpr(std::is_same_v<M,nav_msgs::msg::Odometry>)qos.best_effort();
  return node()->create_subscription<M>(topic,qos,[f](typename M::SharedPtr m){if constexpr(MessageArg<Arg>::shared)f(m);else f(*m);});
 }
 template<class C>Timer createTimer(Duration d,void(C::*f)(const TimerEvent&),C* o)const{
  return Timer(rclcpp::create_timer(node()->get_node_base_interface(),node()->get_node_timers_interface(),node()->get_clock(),std::chrono::nanoseconds(static_cast<int64_t>(d.value*1e9)),[o,f](){(o->*f)(TimerEvent{});}));
 }
 Timer createTimer(Duration d,void(*f)(const TimerEvent&))const{
  return Timer(rclcpp::create_timer(node()->get_node_base_interface(),node()->get_node_timers_interface(),node()->get_clock(),std::chrono::nanoseconds(static_cast<int64_t>(d.value*1e9)),[f](){f(TimerEvent{});}));
 }
};
}
#define ROS_INFO(...) RCLCPP_INFO(epic_ros::logger(),__VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(epic_ros::logger(),__VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(epic_ros::logger(),__VA_ARGS__)
#define ROS_ERROR_ONCE(...) RCLCPP_ERROR_ONCE(epic_ros::logger(),__VA_ARGS__)
#define ROS_WARN_THROTTLE(period,...) RCLCPP_WARN_THROTTLE(epic_ros::logger(),*epic_ros::node()->get_clock(),static_cast<int64_t>((period)*1000),__VA_ARGS__)
#define ROS_INFO_STREAM(msg) RCLCPP_INFO_STREAM(epic_ros::logger(),msg)
#define ROS_INFO_STREAM_THROTTLE(period,msg) RCLCPP_INFO_STREAM_THROTTLE(epic_ros::logger(),*epic_ros::node()->get_clock(),static_cast<int64_t>((period)*1000),msg)
#define ROS_ASSERT(expr) do{if(!(expr))throw std::runtime_error("EPIC assertion: " #expr);}while(false)
