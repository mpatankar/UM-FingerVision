//-------------------------------------------------------------------------------------------
/*! \file    visual_skin_node2.cpp
    \brief   Computer vision for visual skin version2.
             Improved marker (blob) tracking and proximity vision.
    \author  Akihiko Yamaguchi, info@akihikoy.net
    \version 0.1
    \date    Feb.19, 2017
*/
//-------------------------------------------------------------------------------------------
#include "lfd_vision/usb_stereo.h"
#include "lfd_vision/blob_tracker2.h"
#include "lfd_vision/prox_vision.h"
#include "lfd_vision/geom_util.h"
#include "lfd_vision/vision_util.h"
#include "lfd_vision/pcl_util.h"
//-------------------------------------------------------------------------------------------
#include "lfd_vision/BlobMoves.h"
#include "lfd_vision/ProxVision.h"
#include "lfd_vision/SetInt32.h"
//-------------------------------------------------------------------------------------------
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <algorithm>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
//-------------------------------------------------------------------------------------------
namespace trick
{
bool Running(true), Shutdown(false), DoCalibrate(false);
int FrameSkip(0);  // 0: no skip
std::string *CurrentWin(NULL);

std::string BlobCalibPrefix("blob_");
std::vector<TCameraInfo> CamInfo;
std::vector<TStereoInfo> StereoInfo;
std::vector<TStereo> Stereo;  // Standard stereo
std::vector<TStereo> StereoB;  // Stereo for blob
std::vector<TBlobTracker2> BlobTracker;  // Marker tracker
std::vector<TObjDetTrackBSP> ObjDetTracker;  // Proximity vision
std::vector<TCameraRectifier> SingleCamRectifier;
std::vector<boost::function<void(cv::Mat&)> > CamRectifier;  // Functions to rectify camera images.
void DummyRectify(cv::Mat&) {}  // Do nothing function

std::map<std::string, TEasyVideoOut> VideoOut;
struct TShowTrackbars
{
  bool Enabled;
  std::string Kind;
};
std::map<std::string, TShowTrackbars> ShowTrackbars;

std::vector<ros::Publisher> CloudPub;
std::vector<ros::Publisher> BlobPub;
std::vector<ros::Publisher> PXVPub;
std::vector<cv::Mat> Frame;
std::vector<int64_t> CapTime;
std::vector<boost::shared_ptr<boost::mutex> > MutCamCapture;  // Mutex for capture
std::vector<boost::shared_ptr<boost::mutex> > MutFrameCopy;  // Mutex for Frame
struct TIMShowStuff
{
  boost::shared_ptr<boost::mutex> Mutex;
  cv::Mat Frame;
};
std::map<std::string, TIMShowStuff> IMShowStuff;
}
//-------------------------------------------------------------------------------------------
using namespace std;
// using namespace boost;
using namespace trick;
//-------------------------------------------------------------------------------------------
// #define print(var) PrintContainer((var), #var"= ")
// #define print(var) std::cout<<#var"= "<<(var)<<std::endl
//-------------------------------------------------------------------------------------------

/*
  Right click: pause/resume
*/
void OnMouse(int event, int x, int y, int flags, void *data)
{
  if(event!=0)
  {
    CurrentWin= reinterpret_cast<std::string*>(data);
    std::cerr<<"CurrentWin: "<<*CurrentWin<<std::endl;
  }

  // if(flags!=0)  return;
  if(event == cv::EVENT_RBUTTONDOWN)
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  if(event == cv::EVENT_LBUTTONDOWN)
  {
    int i_cam(-1);
    TObjDetTrackBSP *ptracker(NULL);
    for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
      if(ObjDetTracker[i].Name==*CurrentWin)
      {
        ptracker= &ObjDetTracker[i];
        i_cam= i;
        break;
      }
    if(ptracker!=NULL)
    {
      cv::Mat frame;
      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
      }
      ptracker->AddToModel(frame, cv::Point(x,y));
    }
  }
}
//-------------------------------------------------------------------------------------------

// return if continue
bool HandleKeyEvent()
{
  // keyboard interface:
  char c(cv::waitKey(1));
  if(c=='\x1b'||c=='q') return false;
  else if(c=='W')
  {
    for(std::map<std::string, TEasyVideoOut>::iterator itr(VideoOut.begin()),itr_end(VideoOut.end()); itr!=itr_end; ++itr)
      itr->second.Switch();
  }
  else if(c==' ')
  {
    Running=!Running;
    std::cerr<<(Running?"Resume":"Pause (Hit space/R-click to resume)")<<std::endl;
  }
  else if(c=='c')
  {
    DoCalibrate= true;
  }
  else if(c=='s')
  {
    int i_cam(-1);
    TBlobTracker2 *ptracker(NULL);
    for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
      if(BlobTracker[i].Name==*CurrentWin)
      {
        ptracker= &BlobTracker[i];
        i_cam= i;
        break;
      }
    if(ptracker!=NULL)
    {
      ptracker->SaveCalib(BlobCalibPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Saved calibration data of "<<ptracker->Name<<" to "<<BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
    }
  }
  else if(c=='l')
  {
    int i_cam(-1);
    TBlobTracker2 *ptracker(NULL);
    for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
      if(BlobTracker[i].Name==*CurrentWin)
      {
        ptracker= &BlobTracker[i];
        i_cam= i;
        break;
      }
    if(ptracker!=NULL)
    {
      ptracker->LoadCalib(BlobCalibPrefix+CamInfo[i_cam].Name+".yaml");
      std::cerr<<"Loaded calibration data of "<<ptracker->Name<<" from "<<BlobCalibPrefix+CamInfo[i_cam].Name+".yaml"<<std::endl;
    }
  }
  else if(c=='C')
  {
    ShowTrackbars[*CurrentWin].Enabled= !ShowTrackbars[*CurrentWin].Enabled;
    if(ShowTrackbars[*CurrentWin].Enabled)
    {
      if(ShowTrackbars[*CurrentWin].Kind=="BlobTracker")
      {
        // int i_cam(-1);
        TBlobTracker2 *ptracker(NULL);
        for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
          if(BlobTracker[i].Name==*CurrentWin)
          {
            ptracker= &BlobTracker[i];
            // i_cam= i;
            break;
          }
        cv::createTrackbar("thresh_v", *CurrentWin, &ptracker->Params().ThreshV, 255, NULL);
      }
      else if(ShowTrackbars[*CurrentWin].Kind=="ObjDetTracker")
      {
        std::cerr<<"Not implemented yet. Show trackbars for ObjDetTracker"<<std::endl;
      }
    }
    else
    {
      // Remove trackbars from window.
      if(ShowTrackbars[*CurrentWin].Kind=="BlobTracker")
      {
        cv::destroyWindow(*CurrentWin);
        cv::namedWindow(*CurrentWin,1);
        cv::setMouseCallback(*CurrentWin, OnMouse, CurrentWin);
      }
      else if(ShowTrackbars[*CurrentWin].Kind=="ObjDetTracker")
      {
        cv::destroyWindow(*CurrentWin);
        cv::namedWindow(*CurrentWin,1);
        cv::setMouseCallback(*CurrentWin, OnMouse, CurrentWin);
      }
    }
  }

  return true;
}
//-------------------------------------------------------------------------------------------

bool Pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Paused..."<<std::endl;
  Running= false;
  return true;
}
//-------------------------------------------------------------------------------------------

bool Resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Resumed..."<<std::endl;
  Running= true;
  return true;
}
//-------------------------------------------------------------------------------------------

bool SetFrameSkip(lfd_vision::SetInt32::Request &req, lfd_vision::SetInt32::Response &res)
{
  std::cerr<<"Setting frame skip as "<<req.data<<"..."<<std::endl;
  FrameSkip= req.data;
  res.result= true;
  return true;
}
//-------------------------------------------------------------------------------------------

bool StopDetectObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Stopping object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StopDetect();
  return true;
}
//-------------------------------------------------------------------------------------------

bool StartDetectObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Starting object detection..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].StartDetect();
  return true;
}
//-------------------------------------------------------------------------------------------

bool ClearObj(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::cerr<<"Clearing object models..."<<std::endl;
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    ObjDetTracker[j].ClearObject();
  return true;
}
//-------------------------------------------------------------------------------------------

// Get point cloud by stereo
void ExecStereo(int i_stereo)
{
  TStereo &stereo(Stereo[i_stereo]);
  TStereoInfo &info(StereoInfo[i_stereo]);
  cv::Mat frame[2];
  cv::Mat disparity;
  while(!Shutdown)
  {
    if(Running)
    {
      {
        boost::mutex::scoped_lock lock1(*MutFrameCopy[info.CamL]);
        boost::mutex::scoped_lock lock2(*MutFrameCopy[info.CamR]);
        Frame[info.CamL].copyTo(frame[0]);
        Frame[info.CamR].copyTo(frame[1]);
      }
      stereo.Proc(frame[0],frame[1]);
      cv::normalize(stereo.Disparity(), disparity, 0, 255, CV_MINMAX, CV_8U);

      // 1:show, 0:hide; order=color1,color2,stereo1,stereo2,disparity,flow1,flow2
      // if(ImgWin[2]=='1')  cv::imshow("stereo_frame_l", stereo.FrameL());
      // if(ImgWin[3]=='1')  cv::imshow("stereo_frame_r", stereo.FrameR());
      // if(ImgWin[4]=='1')  cv::imshow("stereo_disparity", disparity);
      {
        // cv::imshow(info.Name, disparity);
        boost::mutex::scoped_lock lock(*IMShowStuff[info.Name].Mutex);
        disparity.copyTo(IMShowStuff[info.Name].Frame);
      }

      // Publish as point cloud.
      stereo.ReprojectTo3D();
      sensor_msgs::PointCloud2 cloud_msg;
      ConvertPointCloudToROSMsg<pcl::PointXYZRGB>(cloud_msg,
          ConvertXYZImageToPointCloud(stereo.XYZ(),stereo.RGB()),
          /*frame_id=*/CamInfo[info.CamL].Name);
      CloudPub[i_stereo].publish(cloud_msg);
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecBlobTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TBlobTracker2 &tracker(BlobTracker[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      CamRectifier[i_cam](frame);
      tracker.Step(frame);
      tracker.Draw(frame);

      VideoOut[tracker.Name].Step(frame);
      VideoOut[tracker.Name].VizRec(frame);

      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[tracker.Name].Mutex);
        frame.copyTo(IMShowStuff[tracker.Name].Frame);
      }

      // Publish as BlobMoves
      {
        const std::vector<TPointMove2> &data(tracker.Data());
        lfd_vision::BlobMoves blob_moves;
        blob_moves.camera_index= i_cam;
        blob_moves.camera_name= info.Name;
        blob_moves.width= info.Width;
        blob_moves.height= info.Height;
        blob_moves.data.resize(data.size());
        int i(0);
        for(std::vector<TPointMove2>::const_iterator itr(data.begin()),itr_end(data.end()); itr!=itr_end; ++itr,++i)
        {
          lfd_vision::BlobMove &m(blob_moves.data[i]);
          m.Pox= itr->Po.x;
          m.Poy= itr->Po.y;
          m.So = itr->So;
          m.DPx= itr->DP.x;
          m.DPy= itr->DP.y;
          m.DS = itr->DS;
        }
        BlobPub[i_cam].publish(blob_moves);
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

void ExecObjDetTrack(int i_cam)
{
  TCameraInfo &info(CamInfo[i_cam]);
  TObjDetTrackBSP &tracker(ObjDetTracker[i_cam]);
  cv::Mat frame;
  int64_t t_cap(0);
  while(!Shutdown)
  {
    if(Running)
    {
      if(CapTime[i_cam]==t_cap)
      {
        usleep(10*1000);
        continue;
      }

      {
        boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
        Frame[i_cam].copyTo(frame);
        t_cap= CapTime[i_cam];
      }
      CamRectifier[i_cam](frame);
      tracker.Step(frame);
      frame*= 0.3;
      tracker.Draw(frame);

      VideoOut[tracker.Name].Step(frame);
      VideoOut[tracker.Name].VizRec(frame);

      {
        // cv::imshow(info.Name, frame);
        boost::mutex::scoped_lock lock(*IMShowStuff[tracker.Name].Mutex);
        frame.copyTo(IMShowStuff[tracker.Name].Frame);
      }

      // Publish as PXVPub
      {
        const cv::Mat &objs(tracker.ObjS()), &mvs(tracker.MvS());
        const cv::Moments &om(tracker.ObjMoments());
        lfd_vision::ProxVision prox_vision;
        prox_vision.camera_index= i_cam;
        prox_vision.camera_name= info.Name;
        prox_vision.width= info.Width;
        prox_vision.height= info.Height;

        double m[]= {om.m00, om.m10, om.m01, om.m20, om.m11, om.m02, om.m30, om.m21, om.m12, om.m03};
        std::vector<float> vm(m,m+10);
        prox_vision.ObjM_m= vm;

        double mu[]= {om.mu20, om.mu11, om.mu02, om.mu30, om.mu21, om.mu12, om.mu03};
        std::vector<float> vmu(mu,mu+7);
        prox_vision.ObjM_mu= vmu;

        double nu[]= {om.nu20, om.nu11, om.nu02, om.nu30, om.nu21, om.nu12, om.nu03};
        std::vector<float> vnu(nu,nu+7);
        prox_vision.ObjM_nu= vnu;

        prox_vision.ObjS.resize(objs.rows*objs.cols);
        for(int r(0),rend(objs.rows),i(0);r<rend;++r) for(int c(0),cend(objs.cols);c<cend;++c,++i)
          prox_vision.ObjS[i]= objs.at<float>(r,c);

        prox_vision.MvS.resize(mvs.rows*mvs.cols);
        for(int r(0),rend(mvs.rows),i(0);r<rend;++r) for(int c(0),cend(mvs.cols);c<cend;++c,++i)
          prox_vision.MvS[i]= mvs.at<float>(r,c);

        PXVPub[i_cam].publish(prox_vision);
      }
      // usleep(10*1000);
    }  // Running
    else
    {
      usleep(200*1000);
    }
  }
}
//-------------------------------------------------------------------------------------------

cv::Mat Capture(cv::VideoCapture &cap, int i_cam, bool rectify)
{
  cv::Mat frame;
  {
    boost::mutex::scoped_lock lock(*MutCamCapture[i_cam]);
    while(!cap.read(frame))
    {
      if(CapWaitReopen(CamInfo[i_cam],cap)) continue;
      else  return cv::Mat();
    }
  }
  if(CamInfo[i_cam].CapWidth!=CamInfo[i_cam].Width || CamInfo[i_cam].CapHeight!=CamInfo[i_cam].Height)
    cv::resize(frame,frame,cv::Size(CamInfo[i_cam].Width,CamInfo[i_cam].Height));
  Rotate90N(frame,frame,CamInfo[i_cam].NRotate90);
  if(rectify)  CamRectifier[i_cam](frame);
  return frame;
}
//-------------------------------------------------------------------------------------------

int main(int argc, char**argv)
{
  ros::init(argc, argv, "visual_skin_node2");
  ros::NodeHandle node("~");
  std::string pkg_dir(".");
  std::string cam_config("config/usbcam4g1.yaml");
  std::string stereo_config("config/usbcam4g1.yaml");
  std::string blobtrack_config("config/usbcam4g1.yaml");
  std::string objdettrack_config("config/usbcam4g1.yaml");
  std::string blob_calib_prefix("blob_");
  std::string vout_base("/tmp/vout-");

  node.param("pkg_dir",pkg_dir,pkg_dir);
  node.param("cam_config",cam_config,cam_config);
  node.param("stereo_config",stereo_config,stereo_config);
  node.param("blobtrack_config",blobtrack_config,blobtrack_config);
  node.param("objdettrack_config",objdettrack_config,objdettrack_config);
  node.param("blob_calib_prefix",blob_calib_prefix,blob_calib_prefix);
  node.param("vout_base",vout_base,vout_base);
  node.param("frame_skip",FrameSkip,FrameSkip);
  std::cerr<<"pkg_dir: "<<pkg_dir<<std::endl;
  std::cerr<<"cam_config: "<<cam_config<<std::endl;
  std::cerr<<"stereo_config: "<<stereo_config<<std::endl;
  std::cerr<<"blobtrack_config: "<<blobtrack_config<<std::endl;
  std::cerr<<"objdettrack_config: "<<objdettrack_config<<std::endl;
  std::cerr<<"blob_calib_prefix: "<<blob_calib_prefix<<std::endl;

  std::vector<TBlobTracker2Params> blobtrack_info;
  std::vector<TObjDetTrackBSPParams> objdettrack_info;
  ReadFromYAML(CamInfo, pkg_dir+"/"+cam_config);
  ReadFromYAML(StereoInfo, pkg_dir+"/"+stereo_config);
  ReadFromYAML(blobtrack_info, pkg_dir+"/"+blobtrack_config);
  ReadFromYAML(objdettrack_info, pkg_dir+"/"+objdettrack_config);
  BlobCalibPrefix= pkg_dir+"/"+blob_calib_prefix;

  std::vector<cv::VideoCapture> cap(CamInfo.size());
  SingleCamRectifier.resize(CamInfo.size());
  CamRectifier.resize(CamInfo.size());
  Frame.resize(CamInfo.size());
  CapTime.resize(CamInfo.size());
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    TCameraInfo &info(CamInfo[i_cam]);
    if(!CapOpen(info, cap[i_cam]))  return -1;
    MutCamCapture.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));
    MutFrameCopy.push_back(boost::shared_ptr<boost::mutex>(new boost::mutex));

    if(info.Rectification)
    {
      // Setup rectification
      // NOTE: The rectification of StereoInfo overwrites this rectification.
      cv::Size size_in(info.Width,info.Height), size_out(info.Width,info.Height);
      SingleCamRectifier[i_cam].Setup(info.K, info.D, info.R, size_in, info.Alpha, size_out);
      CamRectifier[i_cam]= boost::bind(&TCameraRectifier::Rectify, SingleCamRectifier[i_cam], _1, /*border=*/cv::Scalar(0,0,0));
    }
    else
    {
      CamRectifier[i_cam]= &DummyRectify;
    }
  }
  std::cerr<<"Opened camera(s)"<<std::endl;

  Stereo.resize(StereoInfo.size());
  StereoB.resize(StereoInfo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
  {
    const TStereoInfo &info(StereoInfo[j]);
    Stereo[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    Stereo[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(info.Width,info.Height) );
    Stereo[j].SetRecommendedStereoParams();
    Stereo[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    Stereo[j].Init();
    cv::namedWindow(info.Name,1);
    cv::setMouseCallback(info.Name, OnMouse, const_cast<std::string*>(&info.Name));
    IMShowStuff[info.Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    VideoOut[info.Name].SetfilePrefix(vout_base+info.Name);
    ShowTrackbars[info.Name].Enabled= false;

    StereoB[j].LoadCameraParametersFromYAML(pkg_dir+"/"+info.StereoParam);
    StereoB[j].SetImageSize(
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height),
        cv::Size(CamInfo[info.CamL].Width,CamInfo[info.CamL].Height) );
    StereoB[j].SetRecommendedStereoParams();
    StereoB[j].LoadConfigurationsFromYAML(pkg_dir+"/"+info.StereoConfig);
    StereoB[j].Init();
    CamRectifier[info.CamL]= boost::bind(&TStereo::RectifyL, StereoB[j], _1, /*gray_scale=*/false);
    CamRectifier[info.CamR]= boost::bind(&TStereo::RectifyR, StereoB[j], _1, /*gray_scale=*/false);
  }

  BlobTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    BlobTracker[j].Name= CamInfo[j].Name+"-blob";
    BlobTracker[j].Params()= blobtrack_info[j];
    BlobTracker[j].Init();
    if(FileExists(BlobCalibPrefix+CamInfo[j].Name+".yaml"))
      BlobTracker[j].LoadCalib(BlobCalibPrefix+CamInfo[j].Name+".yaml");
    cv::namedWindow(BlobTracker[j].Name,1);
    cv::setMouseCallback(BlobTracker[j].Name, OnMouse, &BlobTracker[j].Name);
    IMShowStuff[BlobTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    VideoOut[BlobTracker[j].Name].SetfilePrefix(vout_base+BlobTracker[j].Name);
    ShowTrackbars[BlobTracker[j].Name].Enabled= false;
    ShowTrackbars[BlobTracker[j].Name].Kind= "BlobTracker";
  }

  ObjDetTracker.resize(CamInfo.size());
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
  {
    ObjDetTracker[j].Name= CamInfo[j].Name+"-pxv";
    ObjDetTracker[j].Params()= objdettrack_info[j];
    ObjDetTracker[j].Init();
    cv::namedWindow(ObjDetTracker[j].Name,1);
    cv::setMouseCallback(ObjDetTracker[j].Name, OnMouse, &ObjDetTracker[j].Name);
    IMShowStuff[ObjDetTracker[j].Name].Mutex= boost::shared_ptr<boost::mutex>(new boost::mutex);
    VideoOut[ObjDetTracker[j].Name].SetfilePrefix(vout_base+ObjDetTracker[j].Name);
    ShowTrackbars[ObjDetTracker[j].Name].Kind= "ObjDetTracker";
  }

  CloudPub.resize(Stereo.size());
  for(int j(0),j_end(Stereo.size());j<j_end;++j)
    CloudPub[j]= node.advertise<sensor_msgs::PointCloud2>(std::string("point_cloud_")+StereoInfo[j].Name, 1);

  BlobPub.resize(BlobTracker.size());
  for(int j(0),j_end(BlobTracker.size());j<j_end;++j)
    BlobPub[j]= node.advertise<lfd_vision::BlobMoves>(std::string("blob_moves_")+CamInfo[j].Name, 1);

  PXVPub.resize(ObjDetTracker.size());
  for(int j(0),j_end(ObjDetTracker.size());j<j_end;++j)
    PXVPub[j]= node.advertise<lfd_vision::ProxVision>(std::string("prox_vision_")+CamInfo[j].Name, 1);

  ros::ServiceServer srv_pause= node.advertiseService("pause", &Pause);
  ros::ServiceServer srv_resume= node.advertiseService("resume", &Resume);
  ros::ServiceServer srv_set_frame_skip= node.advertiseService("set_frame_skip", &SetFrameSkip);
  ros::ServiceServer srv_stop_detect_obj= node.advertiseService("stop_detect_obj", &StopDetectObj);
  ros::ServiceServer srv_start_detect_obj= node.advertiseService("start_detect_obj", &StartDetectObj);
  ros::ServiceServer srv_clear_obj= node.advertiseService("clear_obj", &ClearObj);

  int show_fps(0);

  // Dummy capture.
  for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
  {
    cap[i_cam] >> Frame[i_cam];
    CapTime[i_cam]= GetCurrentTimeL();
  }

  // Calibrate ObjDetTracker
  if(ObjDetTracker.size()>0)
  {
    std::vector<std::vector<cv::Mat> > frames(CamInfo.size());
    for(int i(0); i<ObjDetTracker[0].Params().NCalibBGFrames; ++i)
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
        frames[i_cam].push_back(Capture(cap[i_cam], i_cam, /*rectify=*/true));
    for(int j(0),j_end(ObjDetTracker.size()); j<j_end; ++j)
      ObjDetTracker[j].CalibBG(frames[j]);
  }

  std::vector<boost::shared_ptr<boost::thread> > th_blobtrack;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_blobtrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecBlobTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_objdettrack;
  for(int j(0),j_end(CamInfo.size());j<j_end;++j)
    th_objdettrack.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecObjDetTrack,j))));

  std::vector<boost::shared_ptr<boost::thread> > th_stereo;
  for(int j(0),j_end(StereoInfo.size());j<j_end;++j)
    th_stereo.push_back(boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(ExecStereo,j))));

  // ros::Rate loop_rate(5);  // 5 Hz
  for(int f(0);ros::ok();++f)
  {
    if(Running)
    {
      // Capture from cameras:
      for(int i_cam(0), i_cam_end(CamInfo.size()); i_cam<i_cam_end; ++i_cam)
      {
        cv::Mat frame= Capture(cap[i_cam], i_cam, /*rectify=*/false);
        if(FrameSkip<=0 || f%(FrameSkip+1)==0)
        {
          boost::mutex::scoped_lock lock(*MutFrameCopy[i_cam]);
          Frame[i_cam]= frame;
          CapTime[i_cam]= GetCurrentTimeL();
        }
      }

      // Show windows
      for(std::map<std::string, TIMShowStuff>::iterator itr(IMShowStuff.begin()),itr_end(IMShowStuff.end()); itr!=itr_end; ++itr)
      {
        boost::mutex::scoped_lock lock(*itr->second.Mutex);
        if(itr->second.Frame.total()>0)
          cv::imshow(itr->first, itr->second.Frame);
      }

      // Handle blob tracker calibration request
      if(DoCalibrate && BlobTracker.size()>0)
      {
        Running= false;
        int i_cam(-1);
        TBlobTracker2 *ptracker(NULL);
        for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
          if(BlobTracker[i].Name==*CurrentWin)
          {
            ptracker= &BlobTracker[i];
            i_cam= i;
            break;
          }
        if(ptracker!=NULL && i_cam!=-1)
        {
          std::vector<cv::Mat> frames;
          for(int i(0); i<ptracker->Params().NCalibPoints; ++i)
            frames.push_back(Capture(cap[i_cam], i_cam, /*rectify=*/true));
          ptracker->Calibrate(frames);
        }
        Running= true;
      }
      if(DoCalibrate && ObjDetTracker.size()>0)
      {
        Running= false;
        int i_cam(-1);
        TObjDetTrackBSP *ptracker(NULL);
        for(int i(0), i_end(CamInfo.size()); i<i_end; ++i)
          if(ObjDetTracker[i].Name==*CurrentWin)
          {
            ptracker= &ObjDetTracker[i];
            i_cam= i;
            break;
          }
        if(ptracker!=NULL && i_cam!=-1)
        {
          std::vector<cv::Mat> frames;
          for(int i(0); i<ptracker->Params().NCalibBGFrames; ++i)
            frames.push_back(Capture(cap[i_cam], i_cam, /*rectify=*/true));
          ptracker->CalibBG(frames);
        }
        Running= true;
      }
      DoCalibrate= false;

      // usleep(10*1000);
      if(show_fps==0)
      {
        std::cerr<<"FPS: "<<VideoOut.begin()->second.FPS()<<std::endl;
        show_fps=VideoOut.begin()->second.FPS()*4;
      }
      --show_fps;

    }  // Running
    else
    {
      usleep(200*1000);
    }

    if(!HandleKeyEvent())  break;

    ros::spinOnce();
  }
  Shutdown= true;
  for(int j(0),j_end(th_blobtrack.size());j<j_end;++j)
    th_blobtrack[j]->join();
  for(int j(0),j_end(th_objdettrack.size());j<j_end;++j)
    th_objdettrack[j]->join();
  for(int j(0),j_end(th_stereo.size());j<j_end;++j)
    th_stereo[j]->join();

  usleep(500*1000);

  return 0;
}
//-------------------------------------------------------------------------------------------
