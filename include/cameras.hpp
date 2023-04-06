#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_map>

#include "utils.h"

using namespace std;
using namespace cv;


namespace camera{

using camera_t = uint32_t;
enum class CMAERA_MODEL:int {
    SIMPLE_PINHOLE = 0,
    PINHOLE = 1,
    SIMPLE_RADIAL = 2,
    RADIAL = 3,
    OPENCV = 4,
    OPENCV_FISHEYE = 5,
    FULL_OPENCV = 6,
    FOV = 7,
    SIMPLE_RADIAL_FISHEYE = 8,
    RADIAL_FISHEYE = 9,
    THIN_PRISM_FISHEYE = 10
};
const static std::unordered_map<int, std::string> CAMERA_MODEL_ID_TO_NAME = {    // camera model name , model index, model param nums
        {static_cast<int>(CMAERA_MODEL::SIMPLE_PINHOLE), "SIMPLE_PINHOLE"}, 
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL), "SIMPLE_RADIAL"},  
        {static_cast<int>(CMAERA_MODEL::PINHOLE), "PINHOLE"},    
        {static_cast<int>(CMAERA_MODEL::RADIAL), "RADIAL"}, 
        {static_cast<int>(CMAERA_MODEL::OPENCV), "OPENCV"},     
        {static_cast<int>(CMAERA_MODEL::OPENCV_FISHEYE), "OPENCV_FISHEYE"},     
        {static_cast<int>(CMAERA_MODEL::FULL_OPENCV), "FULL_OPENCV"},       
        {static_cast<int>(CMAERA_MODEL::FOV), "FOV"},        
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE), "SIMPLE_RADIAL_FISHEYE"},      
        {static_cast<int>(CMAERA_MODEL::RADIAL_FISHEYE), "RADIAL_FISHEYE"},     
        {static_cast<int>(CMAERA_MODEL::THIN_PRISM_FISHEYE), "THIN_PRISM_FISHEYE"}
};
const static std::unordered_map<std::string, int> CAMERA_MODEL_NAME_TO_ID = {    // camera model name , model index, model param nums
        {"SIMPLE_PINHOLE", static_cast<int>(CMAERA_MODEL::SIMPLE_PINHOLE)}, 
        {"SIMPLE_RADIAL", static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL)},  
        {"PINHOLE", static_cast<int>(CMAERA_MODEL::PINHOLE)},    
        {"RADIAL", static_cast<int>(CMAERA_MODEL::RADIAL)}, 
        {"OPENCV", static_cast<int>(CMAERA_MODEL::OPENCV)},     
        {"OPENCV_FISHEYE", static_cast<int>(CMAERA_MODEL::OPENCV_FISHEYE)},     
        {"FULL_OPENCV", static_cast<int>(CMAERA_MODEL::FULL_OPENCV)},    
        {"FOV", static_cast<int>(CMAERA_MODEL::FOV)}, 
        {"SIMPLE_RADIAL_FISHEYE", static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE)},      
        {"RADIAL_FISHEYE", static_cast<int>(CMAERA_MODEL::RADIAL_FISHEYE)}, 
        {"THIN_PRISM_FISHEYE", static_cast<int>(CMAERA_MODEL::THIN_PRISM_FISHEYE)}     
};
const static std::unordered_map<int, int> CAMERA_MODEL_ID_TO_NUM_PARAMS = {    // camera model name , model index, model param nums
        {static_cast<int>(CMAERA_MODEL::SIMPLE_PINHOLE), 3},              //   f, cx, cy
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL), 4},               //    f, cx, cy, k
        {static_cast<int>(CMAERA_MODEL::PINHOLE), 4},                     //    fx, fy, cx, cy
        {static_cast<int>(CMAERA_MODEL::RADIAL), 5},                      //    f, cx, cy, k1, k2
        {static_cast<int>(CMAERA_MODEL::OPENCV), 8},                      //    fx, fy, cx, cy, k1, k2, p1, p2
        {static_cast<int>(CMAERA_MODEL::OPENCV_FISHEYE), 8},              //    fx, fy, cx, cy, k1, k2, k3, k4
        {static_cast<int>(CMAERA_MODEL::FULL_OPENCV), 12},                //    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
        {static_cast<int>(CMAERA_MODEL::FOV), 5},                         //    fx, fy, cx, cy, omega
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE), 4},       //    f, cx, cy, k
        {static_cast<int>(CMAERA_MODEL::RADIAL_FISHEYE), 5},              //    f, cx, cy, k1, k2
        {static_cast<int>(CMAERA_MODEL::THIN_PRISM_FISHEYE), 12}          //    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
};
const static std::unordered_map<int, int> CAMERA_MODEL_ID_TO_NUM_FocalLength = {    // camera model name , model index, model param nums
        {static_cast<int>(CMAERA_MODEL::SIMPLE_PINHOLE), 1},              //   f
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL), 1},               //    f
        {static_cast<int>(CMAERA_MODEL::PINHOLE), 2},                     //    fx, fy
        {static_cast<int>(CMAERA_MODEL::RADIAL), 1},                      //    f,
        {static_cast<int>(CMAERA_MODEL::OPENCV), 2},                      //    fx, fy,
        {static_cast<int>(CMAERA_MODEL::OPENCV_FISHEYE), 2},              //    fx, fy, 
        {static_cast<int>(CMAERA_MODEL::FULL_OPENCV), 2},                //    fx, fy,
        {static_cast<int>(CMAERA_MODEL::FOV), 2},                         //    fx, fy,
        {static_cast<int>(CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE), 1},       //    f, 
        {static_cast<int>(CMAERA_MODEL::RADIAL_FISHEYE), 1},              //    f, 
        {static_cast<int>(CMAERA_MODEL::THIN_PRISM_FISHEYE), 2}          //    fx, fy, 
};

int CameraModelNameToId(const std::string& model_name){
    const auto it = CAMERA_MODEL_NAME_TO_ID.find(model_name);
    if (it == CAMERA_MODEL_NAME_TO_ID.end()) {
      return -1;
    } else {
      return it->second;
    }
}

inline bool ExistsCameraModelWithName(const std::string& model_name){
    return CAMERA_MODEL_NAME_TO_ID.count(model_name) > 0;
}

inline bool ExistsCameraModelWithId(const int& model_id) {
    return CAMERA_MODEL_ID_TO_NAME.count(model_id) > 0;
}

size_t CameraModelNameToNumFocalLength(const int& model_id) {
  const auto it = CAMERA_MODEL_ID_TO_NUM_FocalLength.find(model_id);
  if (it == CAMERA_MODEL_ID_TO_NUM_FocalLength.end()) {
    return 0;
  } else {
    return it->second;
  }
}


size_t CameraModelNumParams(const int& model_id) {
  const auto it = CAMERA_MODEL_ID_TO_NUM_PARAMS.find(model_id);
  if (it == CAMERA_MODEL_ID_TO_NUM_PARAMS.end()) {
    return 0;
  } else {
    return it->second;
  }
}


class Camera{
public:
  Camera(std::string cameraTxtPath):colmapCameraTxtPath_(cameraTxtPath){
      std::cout<<"Load Colmap cameras.txt: "<< this->colmapCameraTxtPath_ << " ..." <<std::endl;
  };
  Camera(){};
  virtual ~Camera(){};
  virtual bool LoadCameraParamBase()=0;
  virtual camera_t LoadCameraParam(const std::string& line)=0;


  Eigen::Matrix3d CalibrationMatrixEigen() const {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    const size_t focalLengthNum = CameraModelNameToNumFocalLength(model_id_);
    if (focalLengthNum == 1) {
      K(0, 0) = params_[0];
      K(1, 1) = params_[0];
      K(0, 2) = params_[1];
      K(1, 2) = params_[2];
    } else if (focalLengthNum == 2) {
      K(0, 0) = params_[0];
      K(1, 1) = params_[1];
      K(0, 2) = params_[2];
      K(1, 2) = params_[3];
    } else {
      std::cerr
          << "Camera model must either have 1 or 2 focal length parameters."<<std::endl;
    }
    return K;
  }

  cv::Mat CalibrationMatrixCVMat() const {
    cv::Mat K = cv::Mat::eye(3, 3, CV_64FC1);
    const size_t focalLengthNum = CameraModelNameToNumFocalLength(model_id_);
    if (focalLengthNum == 1) {
      K.at<double>(0, 0) = params_[0];
      K.at<double>(1, 1) = params_[0];
      K.at<double>(0, 2) = params_[1];
      K.at<double>(1, 2) = params_[2];
    } else if (focalLengthNum == 2) {
      K.at<double>(0, 0) = params_[0];
      K.at<double>(1, 1) = params_[1];
      K.at<double>(0, 2) = params_[2];
      K.at<double>(1, 2) = params_[3];
    } else {
      std::cerr
          << "Camera model must either have 1 or 2 focal length parameters."<<std::endl;
    }
    return K;
  }

  Eigen::VectorXd DistortionVectorEigen() const {
    const size_t focalLengthNum = CameraModelNameToNumFocalLength(model_id_);
    const size_t distortionNum = params_.size()-focalLengthNum-2;
    if(distortionNum==0) return Eigen::VectorXd(0);

    Eigen::VectorXd D(distortionNum); // distortion param
    for(size_t i=0; i< distortionNum; ++i){
      D(i) = params_[i+focalLengthNum+2];
    }    
    return D;
  }
  cv::Mat DistortionVectorCVMat() const {
    const size_t focalLengthNum = CameraModelNameToNumFocalLength(model_id_);
    const size_t distortionNum = params_.size()-focalLengthNum-2;
    if(distortionNum==0) return cv::Mat();

    cv::Mat D(distortionNum, 1, CV_64FC1); // distortion param
    for(size_t i=0; i< distortionNum; ++i){
      D.at<double>(i, 0) = params_[i+focalLengthNum+2];
    }
    return D;
  }
  bool VerifyParams() const {
    return params_.size() == CameraModelNumParams(model_id_);
  }
  void printParams() const {
    std::cout << VectorToCSV(params_) <<std::endl;
  }
  inline camera_t CameraId() const { return camera_id_; }

  std::string colmapCameraTxtPath_;
  // The unique identifier of the camera. If the identifier is not specified
  // it is set to `kInvalidCameraId`.
  camera_t camera_id_;
  // The identifier of the camera model. If the camera model is not specified
  // the identifier is `kInvalidCameraModelId`.
  int model_id_;
  // The dimensions of the image, 0 if not initialized.
  size_t width_;
  size_t height_;
  // The focal length, principal point, and extra parameters. If the camera
  // model is not specified, this vector is empty.
  std::vector<double> params_;
  // Whether there is a safe prior for the focal length,
  // e.g. manually provided or extracted from EXIF
  bool prior_focal_length_;
};

class CamOPENCV final: public Camera
{
public:
    using Camera::Camera;           // 委托构造
    bool LoadCameraParamBase() override final{
        ifstream camerasFile(this->colmapCameraTxtPath_);
        if(!camerasFile.is_open()){
            std::cerr<<"Could not open "<< this->colmapCameraTxtPath_ << " file!" <<std::endl;
            exit(0);
        }
        // # Camera list with one line of data per camera:
        // #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
        // # Number of cameras: 37
        // OpenCV fish-eye camera model.
        //    fx, fy, cx, cy
        std::string line;
        std::string item;
        while (std::getline(camerasFile, line)) {
          StringTrim(&line);
          if (line.empty() || line[0] == '#') {
            continue;
          }
          std::stringstream line_stream(line);
          // ID
          std::getline(line_stream, item, ' ');
          camera_id_ = std::stoul(item);
          // MODEL
          std::getline(line_stream, item, ' ');
          model_id_ = CameraModelNameToId(item);
          // WIDTH
          std::getline(line_stream, item, ' ');
          width_ = std::stoul(item);
          // HEIGHT
          std::getline(line_stream, item, ' ');
          height_ = std::stoul(item);
          // PARAMS
          params_.clear();
          while (!line_stream.eof()) {
            std::getline(line_stream, item, ' ');
            params_.push_back(std::stold(item));
          }
        }
        assert(VerifyParams());
        return true;
    }
    camera_t LoadCameraParam(const std::string& line) override final{
      
      // # Camera list with one line of data per camera:
      // #   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]
      // # Number of cameras: 37
      // OpenCV fish-eye camera model.
      //    fx, fy, cx, cy
      std::string item;
      std::stringstream line_stream(line);
      // ID
      std::getline(line_stream, item, ' ');
      camera_id_ = std::stoul(item);
      // MODEL
      std::getline(line_stream, item, ' ');
      model_id_ = CameraModelNameToId(item);
      // WIDTH
      std::getline(line_stream, item, ' ');
      width_ = std::stoul(item);
      // HEIGHT
      std::getline(line_stream, item, ' ');
      height_ = std::stoul(item);
      // PARAMS
      params_.clear();
      while (!line_stream.eof()) {
        std::getline(line_stream, item, ' ');
        params_.push_back(std::stold(item));
      }
      assert(VerifyParams());
      return camera_id_;
    } 
};


decltype(auto) GetCamFromModelID(const int& camModelID){
  switch(static_cast<CMAERA_MODEL>(camModelID)){
    case CMAERA_MODEL::PINHOLE:
        // return std::make_unique<>
        break;
    case CMAERA_MODEL::OPENCV:
        return std::make_unique<CamOPENCV>();
        break;

    default:
        std::cerr<<"unknown camera model: "<< camModelID << " !" <<std::endl;
        exit(0);
        break;
  }
  return std::make_unique<CamOPENCV>();
}

};