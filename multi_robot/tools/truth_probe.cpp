// Independent evaluator only. Runtime ROS graph never receives these poses.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>
#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <cmath>
#include <thread>
std::atomic<bool> running{true};
int main(int argc,char**argv){
 std::ofstream file; if(argc>1) file.open(argv[1]); std::ostream& out=argc>1?file:std::cout;
 std::signal(SIGINT,[](int){running=false;});std::signal(SIGTERM,[](int){running=false;});
 ignition::transport::Node node; double last=-1.;
 std::function<void(const ignition::msgs::Pose_V&)> callback=[&](const ignition::msgs::Pose_V&m){
  const double t=m.header().stamp().sec()+1e-9*m.header().stamp().nsec();if(t-last<.1)return;last=t;
  std::string poses;bool found=false;
  out<<std::setprecision(10)<<"{\"sim_time\":"<<t<<",\"wall_time\":"<<std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count()<<",\"robots\":{";
  for(auto&p:m.pose())if(p.name()=="robot1"||p.name()=="robot2"||p.name()=="robot3"){
   if(found)out<<",";found=true;const auto&q=p.orientation();double yaw=std::atan2(2*(q.w()*q.z()+q.x()*q.y()),1-2*(q.y()*q.y()+q.z()*q.z()));
   out<<"\""<<p.name()<<"\":["<<p.position().x()<<","<<p.position().y()<<","<<yaw<<","<<p.position().z()<<"]";
  }
  out<<"}}\n";out.flush();
 };
 if(!node.Subscribe("/world/fleet_arena/dynamic_pose/info",callback)){std::cerr<<"subscribe failed\n";return 1;}
 while(running)std::this_thread::sleep_for(std::chrono::milliseconds(100));return 0;
}
