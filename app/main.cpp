#include <iostream>
#include "cameras.hpp"
#include "project.hpp"

using namespace std;

int main(int args, char *argv[]){
    std::string cameraTXTPath = "../data/0/cameras.txt";
    // auto pCamera = cameraOpenCV(cameraTXTPath);
    // auto flag = pCamera.LoadCameraParam();
    // std::cout << "flag:"<<flag<<std::endl;     
    // pCamera.printParams();
    // std::cout <<pCamera.CalibrationMatrixCVMat()<<std::endl;    
    // std::cout <<pCamera.DistortionVectorEigen()<<std::endl;    
    // std::cout <<pCamera.DistortionVectorCVMat()<<std::endl;   

    auto pCamera = System(cameraTXTPath, cameraTXTPath, cameraTXTPath);
    pCamera.run();
    
}