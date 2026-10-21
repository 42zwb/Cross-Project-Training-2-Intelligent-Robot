// Deterministic reproduction of tf2_core / tf2_ros lock inversion.
#include <tf2/buffer_core.hpp>
#include <atomic>
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
int main(){
 tf2::BufferCore b;
 geometry_msgs::msg::TransformStamped t;t.header.frame_id="map";t.child_frame_id="base";t.transform.rotation.w=1.;t.header.stamp.sec=1;b.setTransform(t,"test");
 std::mutex application_lock;std::atomic<bool> callback_entered{false};
 b.addTransformableRequest([&](auto,auto,auto,auto,auto){callback_entered=true;std::lock_guard<std::mutex> lock(application_lock);},"map","base",tf2::TimePoint(std::chrono::seconds(2)));
 std::unique_lock<std::mutex> lock(application_lock);
 std::thread setter([&]{auto next=t;next.header.stamp.sec=2;b.setTransform(next,"test");});
 auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
 while(!callback_entered&&std::chrono::steady_clock::now()<until)std::this_thread::sleep_for(std::chrono::milliseconds(1));
 if(!callback_entered){lock.unlock();setter.join();return 2;}
 // The stock 0.25.23 core blocks here while retaining application_lock.
 b.addTransformableRequest([](auto,auto,auto,auto,auto){},"map","base",tf2::TimePoint(std::chrono::seconds(3)));
 lock.unlock();setter.join();std::cout<<"PASS: callbacks execute without holding transform-request mutexes\n";
}
