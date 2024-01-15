#include <iostream>
#include <opencv2/opencv.hpp>

#include "cJSON.h"
#include "frame_struct.h"
#include "msa010.hpp"

int main(int /*argc*/, char const* /*argv*/[])
{
  auto a010 = std::make_unique<Msa010>();

  std::string s;
  while (1)
  {
    a010->keepConnect();

    a010 << "AT+DISP=1\r";
    do
    {
      a010 >> s;
    } while (!s.empty());

    a010 << "AT\r";
    a010 >> s;
    if (s.compare("OK\r\n"))
    {  // not this serial port
      continue;
    }

    a010 << "AT+COEFF?\r";
    a010 >> s;
    if (s.compare("+COEFF=1\r\nOK\r\n"))
    {  // not this serial port
      continue;
    }

    a010 >> s;
    if (s.empty())
    {  // not this serial port
      continue;
    }

    float uvf_parms[4];
    cJSON* cparms = cJSON_ParseWithLength((const char*)s.c_str(), s.length());
    uint32_t tmp;
    uvf_parms[0] = ((float)((cJSON_GetObjectItem(cparms, "fx")->valueint) / 262144.0f));
    uvf_parms[1] = ((float)((cJSON_GetObjectItem(cparms, "fy")->valueint) / 262144.0f));
    uvf_parms[2] = ((float)((cJSON_GetObjectItem(cparms, "u0")->valueint) / 262144.0f));
    uvf_parms[3] = ((float)((cJSON_GetObjectItem(cparms, "v0")->valueint) / 262144.0f));
    std::cout << "fx: " << uvf_parms[0] << std::endl;
    std::cout << "fy: " << uvf_parms[1] << std::endl;
    std::cout << "u0: " << uvf_parms[2] << std::endl;
    std::cout << "v0: " << uvf_parms[3] << std::endl;

    a010 << "AT+UNIT=2\r";
    a010 >> s;
    if (s.compare("OK\r\n"))
    {  // not this serial port
      continue;
    }

    std::this_thread::sleep_for(200ms);

    a010 << "AT+DISP=3\r";
    a010 >> s;
    if (s.compare("OK\r\n"))
    {  // not this serial port
      continue;
    }

    uint16_t times = 0;

    for (a010 >> s; !s.empty(); a010 >> s)
    {
      // for (a010->read_some(s); s.size(); a010->read_some(s)) {
      extern frame_t* handleProcess(const std::string& s);
      frame_t* f = handleProcess(s);
      if (!f)
        continue;

      cv::Mat src(f->frame_head_.resolution_rows_, f->frame_head_.resolution_cols_, CV_8UC1, f->payload_);

      std::stringstream fmt;
      fmt << "./" << times << ".jpg";
      cv::imwrite(fmt.str(), src);

      std::cout << f << " " << fmt.str() << std::endl;

      free(f);
      times += 1;

      if (times == 5)
      {
        a010 << "AT+DISP=1\r";
      }
    }
    std::cout << "end" << std::endl;

    while (a010->isConnected())
    {
      a010 >> s;
      if (!s.empty())
      {
        std::cerr << s;
        std::cerr << "--------------ONCE--------------" << std::endl;
      }
    }
  }
  return 0;
}
