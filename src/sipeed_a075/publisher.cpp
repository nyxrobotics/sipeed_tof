#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <semaphore.h>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"

#include <evhttp.h>
#include <event2/event.h>
#include <event2/http.h>
#include <event2/bufferevent.h>

#include "opencv2/opencv.hpp"

#include <cv_bridge/cv_bridge.h>

#include "cJSON.h"

#include "process.hpp"

using namespace std::chrono_literals;

const std::string g_HOST = "192.168.233.1";
int g_PORT = 80;
int g_IMAGECOUNT = 0;

#define DEBUG_PRINT 1

struct event_base* g_BASE;
struct evhttp_connection* g_CONN;
struct evhttp_request* g_REQ;

uint8_t g_REQUESTDATA[10 * 1024 * 1024];
int g_RETUESTDATA_SIZE = 0;
sem_t g_SEM_REQUEST;

void httpRequestDone(struct evhttp_request* req, void* arg)
{
  g_RETUESTDATA_SIZE = -1;
  int s = evbuffer_get_length(req->input_buffer);
  if (DEBUG_PRINT)
    printf("request got size:%d\n", s);
  if (s < sizeof(g_REQUESTDATA))
  {
    if (DEBUG_PRINT)
      printf("copy out\n");
    g_RETUESTDATA_SIZE = evbuffer_remove(req->input_buffer, g_REQUESTDATA, sizeof(g_REQUESTDATA));
    sem_post(&g_SEM_REQUEST);
  }
  // terminate event_base_dispatch()
  event_base_loopbreak((struct event_base*)arg);
}

void initLibevent()
{
  g_BASE = event_base_new();
  g_CONN = evhttp_connection_base_new(g_BASE, nullptr, g_HOST.c_str(), g_PORT);
  sem_init(&g_SEM_REQUEST, 0, -1);
}

int sendRequest(enum evhttp_cmd_type type, const char* url, void* data, int datasize)
{
  g_REQ = evhttp_request_new(httpRequestDone, g_BASE);
  evhttp_add_header(g_REQ->output_headers, "Host", g_HOST.c_str());
  // evhttp_add_header(req->output_headers, "Connection", "close");

  if (datasize)
  {
    // auto buf=evhttp_request_get_output_buffer(req);
    // char txtbuf[10];
    // sprintf(txtbuf,"%d",datasize);
    // evhttp_add_header(req->output_headers, "Content-Length", txtbuf);
    // printf("buf:%p,add size:%d\n",buf,datasize);
    evbuffer_add(g_REQ->output_buffer, data, datasize);
  }
  evhttp_make_request(g_CONN, g_REQ, type, url);
  evhttp_connection_set_timeout(g_REQ->evcon, 60);

  int ret = event_base_dispatch(g_BASE);
  sem_wait(&g_SEM_REQUEST);
  return ret;
}

ros::NodeHandle* g_LOCAL_NODE;
ros::Publisher g_PUBLISHER_PC;
ros::Publisher g_PUBLISHER_RGB;
ros::Publisher g_PUBLISHER_D;
ros::Publisher g_PUBLISHER_I;
ros::Publisher g_PUBLISHER_S;
size_t g_COUNT;
TofFilter* g_CALI_INST;
std::vector<float> g_UVF_PARMS;

void initPointCloudPublisher(ros::NodeHandle* n)
{
  g_LOCAL_NODE = n;
  g_CALI_INST = new TofFilter(320, 240, 7);
  g_CALI_INST->temporalFilterCfg(0.5);
  g_CALI_INST->setKernelSize(7);

  initLibevent();
  int ret = sendRequest(EVHTTP_REQ_GET, "/getinfo", nullptr, 0);
  // printf("get ret:%d size:%d\n",ret,retuestdata_size);
  InfoT info_all;
  memcpy(&info_all, g_REQUESTDATA, sizeof(InfoT));
  g_UVF_PARMS = g_CALI_INST->parseInfo(&info_all);

  static uint8_t lut_got[65536];
  ret = sendRequest(EVHTTP_REQ_GET, "/get_lut", nullptr, 0);
  printf("get ret:%d size:%d\n", ret, g_RETUESTDATA_SIZE);
  memcpy(&lut_got, g_REQUESTDATA, sizeof(lut_got));
  g_CALI_INST->setLut(lut_got);

  ret = sendRequest(EVHTTP_REQ_GET, "/CameraParms.json", nullptr, 0);
  printf("get ret:%d size:%d\n", ret, g_RETUESTDATA_SIZE);
  cJSON* cparms = cJSON_ParseWithLength((const char*)g_REQUESTDATA, g_RETUESTDATA_SIZE);
  // cparms.R_Matrix_data,cparms.T_Vec_data,cparms.Camera_Matrix_data,cparms.Distortion_Parm_data
  float r_data[9];
  float t_data[3];
  float rgb_cm_data[9];
  float d_vec_data[5];
  cJSON* tempobj = cJSON_GetObjectItem(cparms, "R_Matrix_data");
  for (int i = 0; i < 9; i++)
    r_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

  tempobj = cJSON_GetObjectItem(cparms, "T_Vec_data");
  for (int i = 0; i < 3; i++)
    t_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

  tempobj = cJSON_GetObjectItem(cparms, "Camera_Matrix_data");
  for (int i = 0; i < 9; i++)
    rgb_cm_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

  tempobj = cJSON_GetObjectItem(cparms, "Distortion_Parm_data");
  for (int i = 0; i < 5; i++)
    d_vec_data[i] = cJSON_GetArrayItem(tempobj, i)->valuedouble;

  g_CALI_INST->setCameraParm(r_data, t_data, rgb_cm_data, d_vec_data);

  AllConfigT config;
  config.triggermode_ = 1;  // 0:STOP 1:AUTO 2:SINGLE
  config.deepmode_ = 1;     // 0:16bit 1:8bit
  config.deepshift_ = 255;  // for 8bit mode
  config.irmode_ = 1;       // 0:16bit 1:8bit
  config.statusmode_ = 1;   // 0:16bit 1:2bit 2:8bit 3:1bit
  config.statusmask_ = 7;   // for 1bit mode 1:1 2:2 4:3
  config.rgbmode_ = 1;      // 0:YUV 1:JPG
  config.rgbres_ = 0;       // 0:800*600 1:1600*1200
  config.expose_time_ = 0;
  ret = sendRequest(EVHTTP_REQ_POST, "/set_cfg", &config, sizeof(AllConfigT));
  printf("get ret:%d size:%d\n", ret, g_RETUESTDATA_SIZE);

  g_PUBLISHER_PC = n->advertise<sensor_msgs::PointCloud2>("cloud", 10);
  g_PUBLISHER_RGB = n->advertise<sensor_msgs::Image>("rgb", 10);
  g_PUBLISHER_D = n->advertise<sensor_msgs::Image>("depth", 10);
  g_PUBLISHER_I = n->advertise<sensor_msgs::Image>("intensity", 10);
  g_PUBLISHER_S = n->advertise<sensor_msgs::Image>("status", 10);
}

void timerCallback()
{
  // printf("%s:%d\n",__FILE__,__LINE__);
  int ret = sendRequest(EVHTTP_REQ_GET, "/getdeep", nullptr, 0);
  // printf("get ret:%d size:%d\n",ret,retuestdata_size);
  StackframeOldT oldframe = g_CALI_INST->decodePkg(g_REQUESTDATA);
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto data_out1 = g_CALI_INST->temporalFilter((uint16_t*)oldframe.depth_);
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto data_out2 = g_CALI_INST->spatialFilter(data_out1, 0);
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto data_out3 = g_CALI_INST->flyingPointFilter(data_out2, 0.03);
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto tempcali =
      g_CALI_INST->tofCali(data_out3, std::vector<uint16_t>((uint16_t*)oldframe.status_,
                                                            (uint16_t*)((uint8_t*)oldframe.status_) + sizeof(Image_t)));
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto colormap_temp = g_CALI_INST->mapRgB2Tof(
      tempcali,
      std::vector<uint8_t>((uint8_t*)oldframe.rgb_, (uint8_t*)((uint8_t*)oldframe.rgb_) + sizeof(oldframe.rgb_)));
  // printf("%s:%d\n",__FILE__,__LINE__);
  auto tempcali_ir = g_CALI_INST->tofCali(
      std::vector<uint16_t>((uint16_t*)oldframe.ir_, (uint16_t*)((uint8_t*)oldframe.ir_) + sizeof(Image_t)),
      std::vector<uint16_t>((uint16_t*)oldframe.status_, (uint16_t*)((uint8_t*)oldframe.status_) + sizeof(Image_t)));

  std_msgs::Header header;
  header.stamp = ros::Time(oldframe.framestamp_ / 1000, (oldframe.framestamp_ % 1000) * 1000);

  header.frame_id = "tof";

  // auto rgbmsg = sensor_msgs::Image();
  Mat m_rgb(600, 800, CV_8UC4, (void*)oldframe.rgb_);
  Mat md(240, 320, CV_16UC1, (void*)oldframe.depth_);
  Mat mi(240, 320, CV_16UC1, (void*)oldframe.ir_);
  Mat ms(240, 320, CV_16UC1, (void*)oldframe.status_);
  Mat m_rgb_bgr;
  cvtColor(m_rgb, m_rgb_bgr, CV_RGBA2BGR);
  sensor_msgs::Image rgbmsg = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", m_rgb_bgr).toImageMsg().get();
  sensor_msgs::Image dmsg = *cv_bridge::CvImage(std_msgs::Header(), "mono16", md).toImageMsg().get();
  sensor_msgs::Image imsg = *cv_bridge::CvImage(std_msgs::Header(), "mono16", mi).toImageMsg().get();
  sensor_msgs::Image smsg = *cv_bridge::CvImage(std_msgs::Header(), "mono16", ms).toImageMsg().get();
  rgbmsg.header = header;
  dmsg.header = header;
  imsg.header = header;
  smsg.header = header;
  ROS_INFO("Publishing");
  g_PUBLISHER_RGB.publish(rgbmsg);
  g_PUBLISHER_D.publish(dmsg);
  g_PUBLISHER_I.publish(imsg);
  g_PUBLISHER_S.publish(smsg);

  sensor_msgs::PointCloud2 pcmsg;
  pcmsg.header = header;
  pcmsg.height = 240;
  pcmsg.width = 320;
  pcmsg.is_bigendian = false;
  pcmsg.point_step = 20;
  pcmsg.row_step = pcmsg.point_step * 320;
  pcmsg.is_dense = false;
  pcmsg.fields.resize(5);
  pcmsg.fields[0].name = "x";
  pcmsg.fields[0].offset = 0;
  pcmsg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pcmsg.fields[0].count = 1;
  pcmsg.fields[1].name = "y";
  pcmsg.fields[1].offset = 4;
  pcmsg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pcmsg.fields[1].count = 1;
  pcmsg.fields[2].name = "z";
  pcmsg.fields[2].offset = 8;
  pcmsg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pcmsg.fields[2].count = 1;
  pcmsg.fields[3].name = "rgb";
  pcmsg.fields[3].offset = 12;
  pcmsg.fields[3].datatype = sensor_msgs::PointField::UINT32;
  pcmsg.fields[3].count = 1;
  pcmsg.fields[4].name = "intensity";
  pcmsg.fields[4].offset = 16;
  pcmsg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  pcmsg.fields[4].count = 1;

  float fox = g_UVF_PARMS[0];
  float foy = g_UVF_PARMS[1];
  float u0 = g_UVF_PARMS[2];
  float v0 = g_UVF_PARMS[3];
  // printf("%f,%f,%f,%f\n",fox,foy,u0,v0);
  pcmsg.data.resize(320 * 240 * 20, 0x00);
  uint8_t* ptr = pcmsg.data.data();
  // for (int j=0;j<240;j++){
  //     for (int i=0;i<320;i++)
  //         std::cout<<tempcali[j*320+i]<<",";
  //         std::cout<<std::endl;
  // }

  for (int j = 0; j < 240; j++)
    for (int i = 0; i < 320; i++)
    {
      float cx = (((float)i) - u0) / fox;
      float cy = (((float)j) - v0) / foy;
      float dst = ((float)tempcali[j * 320 + i]) / 4.0 / 1000;
      float x = dst * cx;
      float y = dst * cy;
      float z = dst;

      *((float*)(ptr + 0)) = x;
      *((float*)(ptr + 4)) = y;
      *((float*)(ptr + 8)) = z;
      uint32_t color = colormap_temp[j * 320 + i];
      uint32_t color_r = color & 0xff;
      uint32_t color_g = (color & 0xff00) >> 8;
      uint32_t color_b = (color & 0xff0000) >> 16;
      *((uint32_t*)(ptr + 12)) = (color_r << 16) | (color_g << 8) | (color_b << 0);
      *((float*)(ptr + 16)) = tempcali_ir[j * 320 + i];
      ptr += 20;
    }
  g_PUBLISHER_PC.publish(pcmsg);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sipeed_tof");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  initPointCloudPublisher(&n);
  while (ros::ok())
  {
    timerCallback();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
