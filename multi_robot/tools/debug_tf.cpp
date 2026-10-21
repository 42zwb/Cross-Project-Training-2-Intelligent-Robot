#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <sstream>
#include <iostream>
extern "C" const char* dump_costmap(void* ptr){
 static std::string text;auto*c=reinterpret_cast<nav2_costmap_2d::Costmap2DROS*>(ptr);
 auto buffer=c->getTfBuffer();text=buffer->allFramesAsYAML();
 try{auto t=buffer->lookupTransform(c->getGlobalFrameID(),c->getBaseFrameID(),tf2::TimePointZero);std::ostringstream o;o<<"LOOKUP "<<t.header.frame_id<<" to "<<t.child_frame_id<<" stamp "<<t.header.stamp.sec<<"."<<t.header.stamp.nanosec<<" x "<<t.transform.translation.x<<" y "<<t.transform.translation.y<<"\n";text=o.str()+text;}catch(const std::exception&e){text=std::string(e.what())+text;}
 return text.c_str();
}
