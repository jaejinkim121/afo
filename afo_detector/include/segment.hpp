#include <vector>
#include <queue>
#include <mutex>
#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <highs/Highs.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class Optimizer;

class ImuOptimizer{
    
    private:
        Optimizer opt_;
        SampleIMU zero_;
        std::vector<sampleTLA> seg_;
        std::queue<NormTLA> q_;
        NormTLA mean_;
        std::vector<double> tla_;
        std::vector<double> result_opt_;
        std::array<Matrix3d, 7> R_fromZero_;
        std::array<double, 7> linkLength_;
        bool isLeft_;
        bool isSetZero_;
};