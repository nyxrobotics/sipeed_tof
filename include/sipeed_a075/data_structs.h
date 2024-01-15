#include <stdint.h>
struct __attribute__((packed)) AllConfigT
{
  uint8_t triggermode_;  // 0:STOP 1:AUTO 2:SINGLE
  uint8_t deepmode_;     // 0:16bit 1:8bit
  uint8_t deepshift_;    // for 8bit mode
  uint8_t irmode_;       // 0:16bit 1:8bit
  uint8_t statusmode_;   // 0:16bit 1:2bit 2:8bit 3:1bit
  uint8_t statusmask_;   // for 1bit mode 1:1 2:2 4:3
  uint8_t rgbmode_;      // 0:YUV 1:JPG 2:None
  uint8_t rgbres_;       // 0:800*600 1:1600*1200
  int32_t expose_time_;
  void checkAndFix()
  {
    if (triggermode_ > 2)
      triggermode_ = 0;
    if (deepmode_ > 1)
      deepmode_ = 0;
    if (deepshift_ > 11 && deepshift_ != 255)
      deepmode_ = 0;
    if (irmode_ > 1)
      irmode_ = 0;
    if (statusmode_ > 3)
      statusmask_ = 0;
    if (statusmask_ > 7)
      statusmask_ = 0;
    if (rgbmode_ > 3)
      rgbmode_ = 0;
    if (rgbres_ > 3)
      rgbres_ = 0;
  }
  int getDepthSize()
  {
    switch (deepmode_)
    {
      case 0:
        return 320 * 240 * 2;
      case 1:
        return 320 * 240;
    }
  }
  int getIrSize()
  {
    switch (irmode_)
    {
      case 0:
        return 320 * 240 * 2;
      case 1:
        return 320 * 240;
    }
  }
  int getStatusSize()
  {
    switch (statusmode_)
    {
      case 0:
        return 320 * 240 * 2;
      case 1:
        return 320 * 240 / 4;
      case 2:
        return 320 * 240;
      case 3:
        return 320 * 240 / 8;
    }
  };
};

#define PIXEL_ROWS (240)
#define PIXEL_COLS (320)
typedef uint16_t Image_t[PIXEL_ROWS][PIXEL_COLS];
struct StackframeOldT
{
  uint64_t frameid_;
  uint64_t framestamp_;
  Image_t depth_;
  Image_t ir_;
  Image_t status_;
  uint8_t rgb_[800 * 600 * 4];
};

struct __attribute__((packed)) StackframeT
{
  uint64_t frameid_;
  uint64_t framestamp_;
  AllConfigT config_;
  int deepsize_;
  int rgbsize_;
  uint8_t* depth_;
  uint8_t* ir_;
  uint8_t* status_;
  uint8_t* rgb_;
};

typedef struct
{
  uint8_t cali_mode_;
  float fx_;
  float fy_;
  float u0_;
  float v0_;
  float k1_;
  float k2_;
  float k3_;
  float k4_p1_;
  float k5_p2_;
  float skew_;
} __attribute__((packed)) LensCoeff_t;

typedef struct
{
  char sensor_pn_[9];  // Sensor part number
  char module_vendor_[2];
  char module_type_[2];
  char module_sn_[16];  // module serial number
  char vcsel_id_[4];
  char bin_version_[3];
  uint8_t cali_algo_ver_[2];  // Algorithm version for  this coefficient. X.Y.
                              // e.g 9.62
  uint8_t firmware_ver_[2];   // ISP firmware version
} __attribute__((packed)) ModuleInfor_t;

struct __attribute__((packed)) InfoT
{
  ModuleInfor_t module_info_;
  LensCoeff_t coeff_;
};
