#pragma once

#include "opencv2/opencv.hpp"
#include <jpeglib.h>
#include "data_structs.h"

class TofFilter
{
public:
  TofFilter(int width, int height, int kernelsize);
  void filterTof(cv::Mat& deep, cv::Mat& inst, cv::Mat& out, cv::Mat& kernel);
  std::vector<uint16_t> spatialFilter(const uint16_t* data_data, int filtertype);
  uint16_t* temporalFilter(uint16_t* datap);
  std::vector<uint16_t> flyingPointFilter(std::vector<uint16_t>& data, float threshold);
  std::vector<uint8_t> nV12Rgb(std::vector<uint8_t> data, int width, int height);
  std::vector<uint8_t> rgbRgba(std::vector<uint8_t> data, int width, int height);
  std::vector<uint16_t> tofCali(std::vector<uint16_t> deep, std::vector<uint16_t> status);
  std::vector<uint32_t> mapRgB2Tof(std::vector<uint16_t> deep, std::vector<uint8_t> rgb);
  std::vector<float> parseInfo(const InfoT* datap);
  void setCameraParm(const float* Rdata, const float* Tdata, const float* CMdata, const float* Ddata);
  void temporalFilterCfg(float times);
  void freeMem();
  void setKernelSize(int k);
  StackframeOldT decodePkg(uint8_t* deep_package);
  void setLut(const uint8_t* datap);

private:
  uint16_t depth_LUT_[256];
  uint16_t* bufferdata_;
  uint16_t* bufferspat_;
  float temple_alpha_;
  int kernel_size_;
  int imgw_;
  int imgh_;
  int correntid_;
  int parm_inited_;
  int temp_time_changed_;
  LensCoeff_t lens_;
  ModuleInfor_t info_;
  cv::Mat vail_map_;
  cv::Mat deepmap_x_;
  cv::Mat deepmap_y_;
  cv::Mat ToFcameraMatrix_;
  float R_data_[9];
  float T_data_[3];
  float RGB_CM_data_[9];
  float D_VEC_data_[5];
  std::vector<float> parms_to_fontend_;
};
