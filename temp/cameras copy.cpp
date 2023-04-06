#include "cameras.hpp"

camera::Camera()
    : camera_id_(std::numeric_limits<camera_t>::max()),
      model_id_(-1),
      width_(0),
      height_(0),
      prior_focal_length_(false) {}

std::string Camera::ModelName() const {
  const auto iter = CAMERA_MODEL_ID_TO_NAME.find(model_id_);
  if(iter == CAMERA_MODEL_ID_TO_NAME.end()){
    return "";
  }else{
    return iter->second;
  }
}

void Camera::SetModelIdFromName(const std::string& model_name) {
  assert(ExistsCameraModelWithName(model_name));
  model_id_ = CameraModelNameToId(model_name);
  params_.resize(CameraModelNumParams(model_id_), 0);
}

// 设置参数

Eigen::Matrix3d Camera::CalibrationMatrix() const {
  Eigen::Matrix3d K = Eigen::Matrix3d::Identity();

  const std::vector<size_t>& idxs = FocalLengthIdxs();
  if (idxs.size() == 1) {
    K(0, 0) = params_[idxs[0]];
    K(1, 1) = params_[idxs[0]];
  } else if (idxs.size() == 2) {
    K(0, 0) = params_[idxs[0]];
    K(1, 1) = params_[idxs[1]];
  } else {
    LOG(FATAL)
        << "Camera model must either have 1 or 2 focal length parameters.";
  }

  K(0, 2) = PrincipalPointX();
  K(1, 2) = PrincipalPointY();

  return K;
}


