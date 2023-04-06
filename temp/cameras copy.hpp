#pragma once

#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_map>


using namespace std;
using namespace cv;


using camera_t = uint32_t;
enum class CMAERA_MODEL {
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
        {CMAERA_MODEL::SIMPLE_PINHOLE, "SIMPLE_PINHOLE"}, 
        {CMAERA_MODEL::SIMPLE_RADIAL, "SIMPLE_RADIAL"},  
        {CMAERA_MODEL::PINHOLE, "PINHOLE"},    
        {CMAERA_MODEL::RADIAL, "RADIAL"}, 
        {CMAERA_MODEL::OPENCV, "OPENCV"},     
        {CMAERA_MODEL::OPENCV_FISHEYE, "OPENCV_FISHEYE"},     
        {CMAERA_MODEL::FULL_OPENCV, "FULL_OPENCV"},       
        {CMAERA_MODEL::FOV, "FOV"},        
        {CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE, "SIMPLE_RADIAL_FISHEYE"},      
        {CMAERA_MODEL::RADIAL_FISHEYE, "RADIAL_FISHEYE"},     
        {CMAERA_MODEL::THIN_PRISM_FISHEYE, "THIN_PRISM_FISHEYE"}        
};
const static std::unordered_map<int, std::string> CAMERA_MODEL_NAME_TO_ID = {    // camera model name , model index, model param nums
        {"SIMPLE_PINHOLE", CMAERA_MODEL::SIMPLE_PINHOLE}, 
        {"SIMPLE_RADIAL", CMAERA_MODEL::SIMPLE_RADIAL},  
        {"PINHOLE", CMAERA_MODEL::PINHOLE},    
        {"RADIAL", CMAERA_MODEL::RADIAL}, 
        {"OPENCV", CMAERA_MODEL::OPENCV},     
        {"OPENCV_FISHEYE", CMAERA_MODEL::OPENCV_FISHEYE},     
        {"FULL_OPENCV", CMAERA_MODEL::FULL_OPENCV},    
        {"FOV", CMAERA_MODEL::FOV}, 
        {"SIMPLE_RADIAL_FISHEYE", CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE},      
        {"RADIAL_FISHEYE", CMAERA_MODEL::RADIAL_FISHEYE}, 
        {"THIN_PRISM_FISHEYE", CMAERA_MODEL::THIN_PRISM_FISHEYE}     
};
const static std::unordered_map<int, int> CAMERA_MODEL_ID_TO_NUM_PARAMS = {    // camera model name , model index, model param nums
        {CMAERA_MODEL::SIMPLE_PINHOLE, 3},              //   f, cx, cy
        {CMAERA_MODEL::SIMPLE_RADIAL, 4},               //    f, cx, cy, k
        {CMAERA_MODEL::PINHOLE, 4},                     //    fx, fy, cx, cy
        {CMAERA_MODEL::RADIAL, 5},                      //    f, cx, cy, k1, k2
        {CMAERA_MODEL::OPENCV, 8},                      //    fx, fy, cx, cy, k1, k2, p1, p2
        {CMAERA_MODEL::OPENCV_FISHEYE, 8},              //    fx, fy, cx, cy, k1, k2, k3, k4
        {CMAERA_MODEL::FULL_OPENCV, 12},                //    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
        {CMAERA_MODEL::FOV, 5},                         //    fx, fy, cx, cy, omega
        {CMAERA_MODEL::SIMPLE_RADIAL_FISHEYE, 4},       //    f, cx, cy, k
        {CMAERA_MODEL::RADIAL_FISHEYE, 5},              //    f, cx, cy, k1, k2
        {CMAERA_MODEL::THIN_PRISM_FISHEYE, 12}          //    fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, sx1, sy1
};

struct cameraData{
    cameraData(const int& camera_ID, const std::string& camStrModel, const int &width, const int& height, \
    const vector<double>& camIntrins, const vector<double>& camDistort):
        camera_ID_(camera_ID), camera_model_(camStrModel), width_(width), height_(height)
    {
        // for(auto& c : camIntrins){
            std::cout << "camera_ID_:"<< camera_ID_<<endl;
        // }
    }
    //! pinhole params
    double fx_ = 0.0;
    double fy_ = 0.0;
    double cx_ = 0.0;
    double cy_ = 0.0;
    double fx_inv_ = 0.0;
    double fy_inv_ = 0.0;

    //! distortion params
    double k1_ = 0.0;
    double k2_ = 0.0;
    double p1_ = 0.0;
    double p2_ = 0.0;
    double k3_ = 0.0;
    double k4_ = 0.0;

    int camera_ID_;

    int width_;
    int height_;

    std::string camera_model_;

    //! camera matrix in OpenCV format
    cv::Mat cv_cam_matrix_;
    //! camera matrix in Eigen format
    Eigen::Matrix3d eigen_cam_matrix_;
    //! distortion params in OpenCV format
    cv::Mat cv_dist_params_;
    //! distortion params in Eigen format
    // Eigen::Matrix eigen_dist_params_;
};

class cameraBase{
public:
    cameraBase(std::string cameraTxtPath):colmapCameraTxtPath_(cameraTxtPath){
        std::cout<<"Load Colmap cameras.txt: "<< this->colmapCameraTxtPath_ << " ..." <<std::endl;
    };
    virtual ~cameraBase(){};

    virtual bool LoadCameraParam() = 0;

    std::string colmapCameraTxtPath_;
    std::unordered_map<int, cameraData> cameras;
};

class cameraOpenCV final: public cameraBase
{
public:
    using cameraBase::cameraBase;           // 委托构造

    bool LoadCameraParam() override final{
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
        double fx, fy, cx, cy, k1, k2, k3, k4;
        int camera_ID, width, height;
        std::string camStrModel;
        vector<double> camIntrins(4,0.0);
        vector<double> camDistort(4,0.0);
        std::string LineStr;
        while(getline(camerasFile, LineStr) && !LineStr.empty()){
            std::cout << "line:"<< LineStr<<std::endl;     

            if(LineStr[0] == '#') continue;   
            camerasFile >> camera_ID >> camStrModel>>  width >> height >> \
                std::setprecision(12) >> \
                camIntrins[0] >> camIntrins[1] >> camIntrins[2] >> camIntrins[3] >> \
                camDistort[0] >> camDistort[1]>> camDistort[2] >> camDistort[3];
            
            std::cout<<"camera id:"<< camera_ID <<std::endl;
            auto cameraTemp = cameraData(camera_ID, camStrModel, width, height, camIntrins, camDistort);

            this->cameras.insert(std::make_pair(camera_ID, cameraTemp));

        }
        return true;

    }


};





class camera{
public:
    Camera();

    // Access the unique identifier of the camera.
    inline camera_t CameraId() const;
    inline void SetCameraId(const camera_t camera_id);

    // Access the camera model.
    inline int ModelId() const;
    std::string ModelName() const;
    void SetModelId(const int model_id);
    void SetModelIdFromName(const std::string& model_name);

    // Access dimensions of the camera sensor.
    inline size_t Width() const;
    inline size_t Height() const;
    inline void SetWidth(const size_t width);
    inline void SetHeight(const size_t height);

    // Access focal length parameters.
    double MeanFocalLength() const;
    double FocalLength() const;
    double FocalLengthX() const;
    double FocalLengthY() const;
    void SetFocalLength(const double focal_length);
    void SetFocalLengthX(const double focal_length_x);
    void SetFocalLengthY(const double focal_length_y);

    // principal point parameters.
    double PrincipalPointX() const;
    double PrincipalPointY() const;
    void SetPrincipalPointX(const double ppx);
    void SetPrincipalPointY(const double ppy);

    // Get the indices of the parameter groups in the parameter vector.
    const std::vector<size_t>& FocalLengthIdxs() const;
    const std::vector<size_t>& PrincipalPointIdxs() const;
    const std::vector<size_t>& ExtraParamsIdxs() const;

private:
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




inline bool ExistsCameraModelWithName(const std::string& model_name) {
  return CAMERA_MODEL_NAME_TO_ID.count(model_name) > 0;
}

inline bool ExistsCameraModelWithId(const int model_id) {
  return CAMERA_MODEL_ID_TO_NAME.count(model_id) > 0;
}


int CameraModelNameToId(const std::string& model_name) {
  const auto it = CAMERA_MODEL_NAME_TO_ID.find(model_name);
  if (it == CAMERA_MODEL_NAME_TO_ID.end()) {
    return -1;
  } else {
    return it->second;
  }
}

size_t CameraModelNumParams(const int model_id) {
  const auto it = CAMERA_MODEL_ID_TO_NUM_PARAMS.find(model_id);
  if (it == CAMERA_MODEL_ID_TO_NUM_PARAMS.end()) {
    return 0;
  } else {
    return it->second;
  }
}




