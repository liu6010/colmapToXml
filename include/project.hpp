#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <unordered_map>

#include "utils.h"
#include "alignment.h"
#include "cameras.hpp"

using namespace std;
using namespace cv;

using camera::Camera;
using camera::camera_t;

#define EIGEN_STL_UMAP(KEY, VALUE)                                   \
  std::unordered_map<KEY, VALUE, std::hash<KEY>, std::equal_to<KEY>, \
                     Eigen::aligned_allocator<std::pair<KEY const, VALUE>>>

class System{

public:
    System(const std::string& CameraTxtPath, const std::string& ImagesTxtPath, const std::string& Points3DTxtPath)
        : colmapCameraTxtPath_(CameraTxtPath), colmapImagesTxtPath_(ImagesTxtPath), colmapPoints3DTxtPath_(Points3DTxtPath)
    {

    }

    void run(){
        size_t camera_num = InitCameras_();
        std::cout << "camera_num: " << camera_num << std::endl;
    }
    
private:
    size_t InitCameras_(){
        ifstream camerasFile(colmapCameraTxtPath_);
        if(!camerasFile.is_open()){
            std::cerr<<"Could not open "<< this->colmapCameraTxtPath_ << " file!" <<std::endl;
            exit(0);
        }
        std::string line;
        while (std::getline(camerasFile, line)) {
            StringTrim(&line);
            if (line.empty() || line[0] == '#') {
                continue;
            }
            int model_id;
            {
                std::string item;
                std::stringstream line_stream(line);
                std::getline(line_stream, item, ' ');
                // MODEL
                std::getline(line_stream, item, ' ');
                model_id = camera::CameraModelNameToId(item);
            }

            auto camera_ = camera::GetCamFromModelID(model_id);
            camera_t camera_ID = camera_->LoadCameraParam(line);
            cameras_.emplace(camera_ID, std::move(camera_));
            // cameras_[camera_ID] = std::move(camera_);
            // std::cout <<camera_->CalibrationMatrixCVMat()<<std::endl;    
            // std::cout <<cameras_[camera_ID]->CalibrationMatrixCVMat()<<std::endl; 
            // cameras_[camera_ID]->printParams();   
        }
        return cameras_.size();
    }

    std::string colmapCameraTxtPath_, colmapImagesTxtPath_, colmapPoints3DTxtPath_;
    EIGEN_STL_UMAP(camera_t, std::unique_ptr<Camera>) cameras_;
    // std::unordered_map<camera_t, std::unique_ptr<cameraOpenCV> > cameras_;
};



