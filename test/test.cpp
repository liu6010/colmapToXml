#include <iostream>
#include <Eigen/Core>
#include <unordered_map>
using namespace std;
enum class CMAERA_MODEL: int {
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
// const static std::unordered_map<int, std::string> CAMERA_MODEL_ID_TO_NAME = {    // camera model name , model index, model param nums
//         {CMAERA_MODEL::SIMPLE_PINHOLE, "SIMPLE_PINHOLE"}, \
//         {CMAERA_MODEL::SIMPLE_RADIAL, "SIMPLE_RADIAL"}
// };
// g++ test.cpp -I /usr/local/include/eigen3 -o test
int main(){
    std::cout<< static_cast<int>(CMAERA_MODEL::FOV) << std::endl;
    int size =5;
    Eigen::VectorXd D(size);
    D(0) = 1.0;
    for(size_t i=0; i<5;i++){
        D(i) = 1.0;
      cout << D[i] << " ";

    }
    cout <<endl;



}