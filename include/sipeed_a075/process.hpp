
#include "opencv2/opencv.hpp"
#include <jpeglib.h>

#include "data_structs.h"
using namespace cv;

class TofFilter
{
public:
  TofFilter(int width, int height, int kernelsize)
  {
    bufferdata_ = (uint16_t*)malloc(width * height * 30 * 2);
    bufferspat_ = (uint16_t*)malloc(width * height * 2 * 3);
    memset(bufferdata_, 0, width * height * 2);
    memset(bufferspat_, 0, width * height * 2 * 3);
    imgw_ = width;
    imgh_ = height;
    kernel_size_ = kernelsize;
  }
  void filterTof(Mat& deep, Mat& inst, Mat& out, Mat& kernel)
  {
    out.create(deep.rows, deep.cols, CV_32FC1);
    int bs = kernel.size[0] / 2;
    int ks = kernel.size[0];
    //        printf("rows:%d,cols:%d\n",deep.rows,deep.cols);
    Mat buf1;
    Mat buf2;
    copyMakeBorder(deep, buf1, bs, bs, bs, bs, BORDER_REPLICATE);
    copyMakeBorder(inst, buf2, bs, bs, bs, bs, BORDER_REPLICATE);
    for (int xp = 0; xp < deep.cols; xp++)
      for (int yp = 0; yp < deep.rows; yp++)
      {
        Mat depi, iri;
        buf1(Rect(xp, yp, ks, ks)).copyTo(depi);
        buf2(Rect(xp, yp, ks, ks)).copyTo(iri);
        //                std::cout<<kernel<<std::endl;
        //                std::cout<<depi<<std::endl;
        //                std::cout<<iri<<std::endl;
        //                printf("mul1\n");
        Mat a = kernel.mul(depi);
        //                printf("mul2\n");
        Mat b = kernel.mul(iri);
        //                printf("mul3\n");
        Mat c = a.mul(b);
        //                printf("mul4\n");
        Mat d = c.mul(buf1(Rect(xp, yp, ks, ks)));
        //                printf("mul5\n");
        float num_out = sum(d)[0] / sum(c)[0];
        //                printf("mul6\n");
        out.at<float>(yp, xp) = num_out;
        //                printf("mul7\n");
      }
  }

  std::vector<uint16_t> spatialFilter(const uint16_t* data_data, int filtertype)
  {
    //        printf("vecsize:%lu\n", data.size());
    memcpy(bufferspat_, data_data, imgw_ * imgh_ * 2 * 2);
    Mat deep_img = Mat(240, 320, CV_16UC1, bufferspat_);
    Mat ir_img = Mat(240, 320, CV_16UC1, bufferspat_ + 320 * 240);
    deep_img.convertTo(deep_img, CV_32FC1);
    ir_img.convertTo(ir_img, CV_32FC1);
    int kernelsize = kernel_size_;
    Mat kernel = cv::getGaussianKernel(kernelsize, -1, CV_32F);
    Mat filter_out;
    if (filtertype == 1)
    {
      Mat kernel2_d = Mat(kernelsize, kernelsize, CV_32FC1);
      for (int i = 0; i < kernelsize; i++)
        for (int j = 0; j < kernelsize; j++)
        {
          kernel2_d.at<float>(i, j) = kernel.at<float>(i) * kernel.at<float>(j);
        }
      filterTof(deep_img, ir_img, filter_out, kernel2_d);
    }
    else
    {
      filter2D(deep_img, filter_out, CV_32FC1, kernel, Point(-1, -1), 0, BORDER_REPLICATE);
    }
    filter_out.convertTo(filter_out, CV_16UC1);
    std::vector<uint16_t> data_out(filter_out.begin<uint16_t>(), filter_out.end<uint16_t>());
    return data_out;
  }
  uint16_t* temporalFilter(uint16_t* datap)
  {
    //        printf("vecsize:%lu\n", data.size());
    if (temp_time_changed_)
    {
      temp_time_changed_ = 0;
      memcpy(bufferdata_, datap, imgw_ * imgh_ * 2);
    }
    Mat deep_img = Mat(240, 320, CV_16UC1);
    for (int xp = 0; xp < deep_img.cols; xp++)
      for (int yp = 0; yp < deep_img.rows; yp++)
      {
        uint32_t sum = 0;
        sum += float(datap[320 * yp + xp]) * temple_alpha_ + float(bufferdata_[320 * yp + xp]) * (1.0 - temple_alpha_);
        deep_img.at<uint16_t>(yp, xp) = sum;
      }
    memcpy(bufferdata_, deep_img.datastart, imgw_ * imgh_ * 2);
    // std::vector<uint16_t>data_out(deep_img.begin<uint16_t>(), deep_img.end<uint16_t>());
    return bufferdata_;
  }
  std::vector<uint16_t> flyingPointFilter(std::vector<uint16_t>& data, float threshold)
  {
    Mat deep_img = Mat(240, 320, CV_16UC1, (uint16_t*)data.data());
    Mat l_img, r_img, u_img, d_img;
    deep_img(Rect(0, 0, 319, 240)).copyTo(l_img);
    deep_img(Rect(1, 0, 319, 240)).copyTo(r_img);
    deep_img(Rect(0, 0, 320, 239)).copyTo(u_img);
    deep_img(Rect(0, 1, 320, 239)).copyTo(d_img);
    Mat abs_diff(240, 320, CV_16UC1, Scalar_<uint16_t>(0));
    Mat tmph, tmpv;
    Mat tmp1(240, 320, CV_16UC1, Scalar_<uint16_t>(0));
    Mat tmp2(240, 320, CV_16UC1, Scalar_<uint16_t>(0));
    Mat tmp3(240, 320, CV_16UC1, Scalar_<uint16_t>(0));
    Mat tmp4(240, 320, CV_16UC1, Scalar_<uint16_t>(0));
    cv::absdiff(l_img, r_img, tmph);
    cv::absdiff(u_img, d_img, tmpv);
    tmph.copyTo(tmp1(Rect(0, 0, 319, 240)));
    tmph.copyTo(tmp2(Rect(1, 0, 319, 240)));
    tmpv.copyTo(tmp3(Rect(0, 0, 320, 239)));
    tmpv.copyTo(tmp4(Rect(0, 1, 320, 239)));
    cv::max(tmp1, abs_diff, abs_diff);
    cv::max(tmp2, abs_diff, abs_diff);
    cv::max(tmp3, abs_diff, abs_diff);
    cv::max(tmp4, abs_diff, abs_diff);
    Mat absf, deepf;
    deep_img.convertTo(deepf, CV_32FC1);
    abs_diff.convertTo(absf, CV_32FC1);
    absf /= deepf;
    Mat mask = absf < threshold;
    Mat imgout;
    deep_img.copyTo(imgout, mask);
    //        std::cout<<mask<<std::endl;
    std::vector<uint16_t> data_out(imgout.begin<uint16_t>(), imgout.end<uint16_t>());
    return data_out;
  }
  std::vector<uint8_t> nV12Rgb(std::vector<uint8_t> data, int width, int height)
  {
    Mat m_yuv(height + height / 2, width, CV_8UC1, (void*)data.data());
    Mat m_rgb;  //(height, width, CV_8UC4);
    cvtColor(m_yuv, m_rgb, COLOR_YUV2RGBA_NV21);
    // flip(mRGB,mRGB,0);
    std::vector<uint8_t> data_out;
    data_out.assign((uint8_t*)m_rgb.datastart, (uint8_t*)m_rgb.dataend);
    //        printf("size:%d\n",data_out.size());
    auto ret = data_out;
    //        auto ret= emscripten::val(emscripten::typed_memory_view(5, "1111"));
    return ret;
  }
  std::vector<uint8_t> rgbRgba(std::vector<uint8_t> data, int width, int height)
  {
    Mat m_yuv(height, width, CV_8UC3, (void*)data.data());
    Mat m_rgb;  //(height, width, CV_8UC4);
    cvtColor(m_yuv, m_rgb, COLOR_RGB2RGBA);
    // flip(mRGB,mRGB,0);
    std::vector<uint8_t> data_out;
    data_out.assign((uint8_t*)m_rgb.datastart, (uint8_t*)m_rgb.dataend);
    //        printf("size:%d\n",data_out.size());
    auto ret = data_out;
    //        auto ret= emscripten::val(emscripten::typed_memory_view(5, "1111"));
    return ret;
  }
  std::vector<uint16_t> tofCali(std::vector<uint16_t> deep, std::vector<uint16_t> status)
  {
    if (!parm_inited_)
      return std::vector<uint16_t>({ 0 });
    Mat m_deep(240, 320, CV_16U, (void*)deep.data());
    Mat m_status(240, 320, CV_16U, (void*)status.data());
    Mat m_deep_out(240 * 2, 320, CV_16U);
    m_deep_out *= 0;
    m_deep_out(Rect(0, 240, 320, 240)) += 5;
    for (int i = 0; i < 320; i++)
      for (int j = 0; j < 240; j++)
      {
        if (!vail_map_.at<uint8_t>(j, i))
          continue;
        float x_pos = deepmap_x_.at<int16_t>(j, i);
        float y_pos = deepmap_y_.at<int16_t>(j, i);
        m_deep_out.at<uint16_t>(j, i) = m_deep.at<uint16_t>(y_pos, x_pos);
        m_deep_out.at<uint16_t>(j + 240, i) = m_status.at<uint16_t>(y_pos, x_pos);
      }
    std::vector<uint16_t> data_out(m_deep_out.begin<uint16_t>(), m_deep_out.end<uint16_t>());
    return data_out;
  }
  std::vector<uint32_t> mapRgB2Tof(std::vector<uint16_t> deep, std::vector<uint8_t> rgb)
  {
    if (!parm_inited_)
      return std::vector<uint32_t>();
    Mat m_deep(240, 320, CV_16U, (void*)deep.data());
    Mat m_rgb(600, 800, CV_8UC4, (void*)rgb.data());
    Mat m_dee_pf;
    m_deep.convertTo(m_dee_pf, CV_32FC1, 0.25);
    Mat rgb_out(240, 320, CV_8UC4);
    rgb_out *= 0;
    float u0 = ToFcameraMatrix_.at<float>(2);
    float v0 = ToFcameraMatrix_.at<float>(5);
    float fx = ToFcameraMatrix_.at<float>(0);
    float fy = ToFcameraMatrix_.at<float>(4);
    Mat pos(240 * 320, 3, CV_32F);
    for (int i = 0; i < 320; i++)
      for (int j = 0; j < 240; j++)
      {
        if (!vail_map_.at<uint8_t>(j, i))
          continue;
        float cx = (i - u0) / fx;
        float cy = (j - v0) / fy;
        float dst = m_dee_pf.at<float>(j, i);
        float x = dst * cx;
        float y = dst * cy;
        float z = dst;
        pos.at<float>(j * 320 + i, 0) = x;
        pos.at<float>(j * 320 + i, 1) = y;
        pos.at<float>(j * 320 + i, 2) = z;
      }
    //        float
    //        R_data[]={0.999009948014846,-0.035988877304778,-0.026151949788032,0.038600978837311,0.993464765304480,0.107413800469572,0.022115338572483,-0.108316946083506,0.993870389432630};
    Mat r_mat(3, 3, CV_32F, R_data_);
    //        float T_data[]={-13.764129748178371,12.774782478789565,-3.663691499294785};
    Mat t_mat(1, 3, CV_32F, T_data_);
    //        std::cout<<T_MAT<<std::endl;
    //        float
    //        RGB_CM_data[]={8.731012535254073e+02,-1.766830173252281,4.032462441863750e+02,0,8.738528440154463e+02,2.770907184013972e+02,0,0,1};
    Mat rgb_cm(3, 3, CV_32F, RGB_CM_data_);
    //        float
    //        D_VEC_data[]={0.039207535514683,0.108443361234459,0.005477497943480,-0.003425915814048,-0.782406374511559};
    Mat d_vec(1, 5, CV_32F, D_VEC_data_);
    Mat outpoints;
    Mat outpointsi;
    Mat r_mat_cv;
    //        T_MAT*=-1;
    Rodrigues(r_mat.t(), r_mat_cv);

    //            std::cout<<R_MAT<<std::endl;
    //            std::cout<<T_MAT<<std::endl;
    //            std::cout<<RGB_CM<<std::endl;
    //            std::cout<<D_VEC<<std::endl;
    projectPoints(pos, r_mat_cv, t_mat, rgb_cm, d_vec, outpoints);
    outpoints.convertTo(outpointsi, CV_16S);
    for (int i = 0; i < 320; i++)
      for (int j = 0; j < 240; j++)
      {
        auto pos = outpointsi.at<Point_<int16_t>>(j * 320 + i);
        if (pos.x < 0 || pos.x > 799 || pos.y < 0 || pos.y > 599)
          continue;
        rgb_out.at<Vec4b>(j, i) = m_rgb.at<Vec4b>(pos);
      }
    //        static int onceprint;
    //            if((onceprint++)==0)
    //        std::cout<<outpoints<<std::endl;
    //        std::cout<<map2<<std::endl;
    //        mRGB(Rect(0,0,320,240)).copyTo(RGB_out);
    std::vector<uint32_t> data_out;
    data_out.assign((uint32_t*)rgb_out.datastart, (uint32_t*)rgb_out.dataend);
    //        printf("size:%d\n",data_out.size());
    auto ret = data_out;
    return ret;
  }
  std::vector<float> parseInfo(const InfoT* datap)
  {
    info_ = datap->module_info_;
    lens_ = datap->coeff_;
    printf("PN:");
    for (char c : info_.sensor_pn_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("module_vendor:");
    for (char c : info_.module_vendor_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("module_type:");
    for (char c : info_.module_type_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("SN:");
    for (char c : info_.module_sn_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("vcsel_id:");
    for (char c : info_.vcsel_id_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("bin_version:");
    for (char c : info_.bin_version_)
    {
      printf("%c", c);
    }
    printf("\n");
    printf("cali_algo_ver:%d.%d\n", info_.cali_algo_ver_[0], info_.cali_algo_ver_[1]);
    printf("firmware_ver:%x.%x\n", info_.firmware_ver_[0], info_.firmware_ver_[1]);
    printf("Lenscoef:");
    printf("cali_mode:           %d\n", lens_.cali_mode_);
    printf("fx:           %e\n", lens_.fx_);
    printf("fy:           %e\n", lens_.fy_);
    printf("u0:           %e\n", lens_.u0_);
    printf("v0:           %e\n", lens_.v0_);
    printf("k1:           %e\n", lens_.k1_);
    printf("k2:           %e\n", lens_.k2_);
    printf("k3:           %e\n", lens_.k3_);
    printf("k4_p1:        %e\n", lens_.k4_p1_);
    printf("k5_p2:        %e\n", lens_.k5_p2_);
    printf("skew:         %e\n", lens_.skew_);

    float cam_matrix[9] = { lens_.fx_, 0, lens_.u0_, 0, lens_.fy_, lens_.v0_, 0, 0, 1 };
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, cam_matrix);
    camera_matrix.copyTo(ToFcameraMatrix_);
    float r_matrix[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    cv::Mat r_matrix_cv = cv::Mat(3, 3, CV_32F, r_matrix);
    float cam_dist_data[5] = { lens_.k1_, lens_.k2_, lens_.k4_p1_, lens_.k5_p2_, lens_.k3_ };
    cv::Mat cam_dist = cv::Mat(1, 5, CV_32F, cam_dist_data);
    cv::Mat map1, map2;
    initUndistortRectifyMap(camera_matrix, cam_dist, r_matrix_cv, camera_matrix, Size2i(320, 240), CV_16SC2, map1,
                            map2);
    vail_map_.create(240, 320, CV_8UC1);
    vail_map_ *= 0;
    deepmap_x_.create(240, 320, CV_16SC1);
    deepmap_y_.create(240, 320, CV_16SC1);
    std::cout << map1.cols << "," << map1.rows << std::endl;
    std::cout << map1.dims << std::endl;
    //        std::cout<<map1.at<Point_<uint16_t>>(0,0)<<std::endl;
    //        std::cout<<map1<< std::endl;
    for (int x = 0; x < 320; x++)
      for (int y = 0; y < 240; y++)
      {
        auto point = map1.at<Point_<int16_t>>(y, x);
        if (point.x < 0 || point.x >= 320 || point.y < 0 || point.y >= 240)
          continue;
        vail_map_.at<uint8_t>(y, x) = 1;
        deepmap_x_.at<int16_t>(y, x) = point.x;
        deepmap_y_.at<int16_t>(y, x) = point.y;
      }
    //        std::cout<<deepmap_x<< std::endl;
    //        std::cout<<deepmap_y<< std::endl;
    //        std::cout<<vail_map<< std::endl;
    parm_inited_ = 1;
    parms_to_fontend_ = std::vector<float>({ lens_.fx_, lens_.fy_, lens_.u0_, lens_.v0_ });
    auto ret = parms_to_fontend_;
    return ret;
  }
  void setCameraParm(const float* Rdata, const float* Tdata, const float* CMdata, const float* Ddata)
  {
    for (int i = 0; i < 9; i++)
      R_data_[i] = Rdata[i];
    for (int i = 0; i < 3; i++)
      T_data_[i] = Tdata[i] * -1;
    for (int i = 0; i < 9; i++)
      RGB_CM_data_[i] = CMdata[i];
    for (int i = 0; i < 5; i++)
      D_VEC_data_[i] = Ddata[i];
    Mat r_mat(3, 3, CV_32F, R_data_);
    Mat t_mat(1, 3, CV_32F, T_data_);
    Mat rgb_cm(3, 3, CV_32F, RGB_CM_data_);
    Mat d_vec(1, 5, CV_32F, D_VEC_data_);
    std::cout << "R_MAT:" << r_mat << std::endl;
    std::cout << "T_MAT:" << t_mat << std::endl;
    std::cout << "RGB_CM:" << rgb_cm << std::endl;
    std::cout << "D_VEC:" << d_vec << std::endl;
  }
  void temporalFilterCfg(float times)
  {
    temple_alpha_ = times > 1 ? 1 : times;
    temple_alpha_ = times < 0 ? 0 : times;
    printf("times:%f\n", times);
    temp_time_changed_ = 1;
  }
  void freeMem()
  {
    free(bufferdata_);
    free(bufferspat_);
  }
  void setKernelSize(int k)
  {
    kernel_size_ = k;
  }
  StackframeOldT decodePkg(uint8_t* deep_package)
  {
    StackframeT frame;
    memcpy(&frame, deep_package, sizeof(StackframeT) - 4 * sizeof(uint8_t*));
    frame.depth_ = (deep_package) + sizeof(StackframeT) - 4 * sizeof(uint8_t*);
    frame.ir_ = frame.depth_ + frame.config_.getDepthSize();
    frame.status_ = frame.ir_ + frame.config_.getIrSize();
    frame.rgb_ = frame.status_ + frame.config_.getStatusSize();
    static StackframeOldT frame_to_return;
    frame_to_return.frameid_ = frame.frameid_;
    frame_to_return.framestamp_ = frame.framestamp_;
    switch (frame.config_.deepmode_)
    {
      case 1:
        if (frame.config_.deepshift_ == 255)
          for (int i = 0; i < 320 * 240; i++)
            ((uint16_t*)frame_to_return.depth_)[i] = depth_LUT_[(((uint8_t*)frame.depth_)[i])];
        else
          for (int i = 0; i < 320 * 240; i++)
            ((uint16_t*)frame_to_return.depth_)[i] = ((uint16_t)(((uint8_t*)frame.depth_)[i]))
                                                     << frame.config_.deepshift_;
        break;
      case 0:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.depth_)[i] = ((uint16_t*)frame.depth_)[i];
        break;
      default:
        break;
    }
    switch (frame.config_.irmode_)
    {
      case 1:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.ir_)[i] = ((uint16_t)(((uint8_t*)frame.ir_)[i])) * 16;
        break;
      case 0:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.ir_)[i] = ((uint16_t*)frame.ir_)[i];
        break;
      default:
        break;
    }
    switch (frame.config_.statusmode_)
    {
      case 2:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.status_)[i] = ((uint16_t)(((uint8_t*)frame.status_)[i]));
        break;
      case 0:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.status_)[i] = ((uint16_t*)frame.status_)[i];
        break;
      case 1:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.status_)[i] = ((((uint8_t*)frame.status_)[i / 4]) >> (2 * (i % 4))) & 3;
        break;
      case 3:
        for (int i = 0; i < 320 * 240; i++)
          ((uint16_t*)frame_to_return.status_)[i] = ((((uint8_t*)frame.status_)[i / 8]) & 1) ? 3 : 0;
        break;
      default:
        break;
    }
    switch (frame.config_.rgbmode_)
    {
      case 0:
        memcpy(frame_to_return.rgb_, frame.rgb_, frame.rgbsize_);
        break;
      case 1:

        struct jpeg_decompress_struct info;  // for our jpeg info
        struct jpeg_error_mgr err;           // the error handler
        info.err = jpeg_std_error(&err);
        err.error_exit = [](j_common_ptr /*cinfo*/) {};
        jpeg_create_decompress(&info);  // fills info structure
        jpeg_mem_src(&info, frame.rgb_, frame.rgbsize_);
        if (JPEG_HEADER_OK == jpeg_read_header(&info, TRUE))
        {
          jpeg_start_decompress(&info);
          unsigned char* jdata = (unsigned char*)malloc(info.output_width * info.output_height * 3);
          while (info.output_scanline < info.output_height)  // loop
          {
            unsigned char* pt = jdata + 3 * info.output_width * info.output_scanline;
            jpeg_read_scanlines(&info, &pt, 1);
          }
          jpeg_finish_decompress(&info);  // finish decompressing
          Mat m_rgb(480, 640, CV_8UC3, (void*)jdata);
          Mat m_rgba;  //(height, width, CV_8UC4);
          cvtColor(m_rgb, m_rgba, COLOR_RGB2RGBA);
          resize(m_rgba, m_rgba, Size(800, 600));
          // flip(mRGB,mRGB,0);
          memcpy(frame_to_return.rgb_, m_rgba.datastart, m_rgba.dataend - m_rgba.datastart);
          free(jdata);
        }
        break;
      case 2:
        memset(frame_to_return.rgb_, 0, sizeof(frame_to_return.rgb_));
        break;
    }
    return frame_to_return;
  }
  void setLut(const uint8_t* datap)
  {
    float tempdata[256] = { 0 };
    float tempcounts[256] = { 0 };
    for (int i = 0; i < 65536; i++)
    {
      tempdata[datap[i]] += i;
      tempcounts[datap[i]] += 1;
    }
    for (int i = 0; i < 256; i++)
    {
      if (tempcounts[i] == 0)
        depth_LUT_[i] = 0;
      else
        depth_LUT_[i] = tempdata[i] / tempcounts[i];
    }
  }

  uint16_t depth_LUT_[256];
  uint16_t* bufferdata_ = nullptr;
  uint16_t* bufferspat_ = nullptr;
  float temple_alpha_ = 0.5;
  int kernel_size_ = 0;
  int imgw_;
  int imgh_;
  int correntid_ = 0;
  int parm_inited_ = 0;
  int temp_time_changed_ = 1;
  LensCoeff_t lens_;
  ModuleInfor_t info_;
  Mat vail_map_;
  Mat deepmap_x_;
  Mat deepmap_y_;
  Mat ToFcameraMatrix_;
  // When Calibration, RGB Camera is Camera1, ToF as Camera2
  // in MATLAB stereoParams.RotationOfCamera2' (here remember to transpose matrix)
  float R_data_[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
  // in MATLAB stereoParams.TranslationOfCamera2
  float T_data_[3] = { 0, 0, 0 };
  // in MATLAB stereoParams.CameraParameters1.IntrinsicMatrix' (here remember to transpose matrix)
  float RGB_CM_data_[9] = { 800, 0, 400, 0, 800, 300, 0, 0, 1 };
  // k1 k2 p1 p2 k3
  float D_VEC_data_[5] = { 0, 0, 0, 0, 0 };
  std::vector<float> parms_to_fontend_;
};
