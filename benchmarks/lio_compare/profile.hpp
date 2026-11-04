#pragma once
#include <chrono>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>
namespace zbench {
inline double cpu_ms(){timespec t;clock_gettime(CLOCK_PROCESS_CPUTIME_ID,&t);return t.tv_sec*1000.+t.tv_nsec/1e6;}
inline double wall_ms(){return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();}
struct Frame {
 double stamp=0,start=0,last=0,cpu=0;size_t input=0,selected=0;bool active=false;
 std::map<std::string,double> stages;
 void begin(double s,size_t n){stamp=s;input=n;selected=0;stages.clear();start=last=wall_ms();cpu=cpu_ms();active=true;}
 void mark(const std::string& stage){if(active){double now=wall_ms();stages[stage]+=now-last;last=now;}}
 void end(){
  if(!active)return;
  mark("output_other");double total=wall_ms()-start,used=cpu_ms()-cpu;
  static std::ofstream out;
  if(!out.is_open()){
   const char* p=std::getenv("ZAV_BENCH_CSV");if(!p){active=false;return;}
   out.open(p);out<<"stamp,input_points,selected_points,imu_undistort_ms,downsample_ms,map_prepare_ms,estimate_ms,map_update_ms,output_other_ms,total_ms,process_cpu_ms\n";
  }
  out<<std::setprecision(12)<<stamp<<','<<input<<','<<selected;
  for(const auto& s:{"imu_undistort","downsample","map_prepare","estimate","map_update","output_other"})out<<','<<stages[s];
  out<<','<<total<<','<<used<<'\n';out.flush();active=false;
 }
};
inline Frame& frame(){static Frame value;return value;}
}
