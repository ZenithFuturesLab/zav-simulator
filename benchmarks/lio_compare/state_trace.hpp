#pragma once
#include <cstdlib>
#include <fstream>
#include <iomanip>
#define ZSTATE_TRACE(label) do { \
 const char* zp=std::getenv("ZAV_STATE_CSV"); \
 if(zp){static std::ofstream zo(zp, std::ios::app);zo<<std::setprecision(15)<<label<<','<<current_obs_time_<<','<<forward_time_; \
 for(const auto& zv:{p_,v_,ba_,bg_,g_,fw_p_,fw_v_})for(int zj=0;zj<3;++zj)zo<<','<<zv[zj]; \
 Eigen::Quaternion<scalar> zq(R_.R_);Eigen::Quaternion<scalar> zfq(fw_R_.R_);for(int zj=0;zj<4;++zj)zo<<','<<zq.coeffs()[zj]; \
 for(int zj=0;zj<4;++zj)zo<<','<<zfq.coeffs()[zj];zo<<'\n';zo.flush();} \
}while(false)
