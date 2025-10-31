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

using namespace Eigen;

const size_t N = 10;

struct SampleIMU{
    float t;
    std::array<float, 21> value;
};

struct SampleTLA{
    double t;
    double value;
};

struct NormTLA{
float cycle_time;
std::array<double, N+1> d;
};

class Optimizer{
    public:
        Optimizer();
        ~Optimizer() = default;
        void set_efficacy(const std::vector<double>& efficacy);
        void run_optimize(std::vector<double>& r);

    private:
        // Define Optimization problem parameters.
        static const unsigned int N_t = 99; // dt = 1 / (N_t+1). For this case, dt=10ms.
        double torque_upper_limit = 1.0; // Can be kHighSInf
        double jerk_absolute_upper_limit = 0.05;
        double torque_impulse_value = 0.1 * N_t;

        const int num_col = N_t;
        const int num_row = N_t + 2;

        // Define state-relevant variables.
        std::vector<double> col_cost;
        std::vector<double> col_lower;
        std::vector<double> col_upper;
        
        // 제약 하한/상한
        std::vector<double> row_lower;
        std::vector<double> row_upper;

        std::vector<HighsInt> Astart;     // size = num_col + 1
        std::vector<HighsInt> Aindex;  // row indices
        std::vector<double> Avalue;  // values
};

class ImuOptimizer{
    public:
        ImuOptimizer(bool isLeft);
        ~ImuOptimizer() = default;
        float push(float t, std::array<float, 21>& d);
        bool cut();
        bool mean();
        void setZeros(std::array<float, 21>& d);
        double getTLA(std::array<float, 21>& d);
        void optimize();
        void save();
        void getResult(std::vector<double>& target);

    private:
        Optimizer opt_;
        SampleIMU zero_;
        std::vector<SampleTLA> seg_;
        std::queue<NormTLA> q_;
        NormTLA mean_;
        std::vector<double> tla_;
        std::vector<double> result_opt_;
        std::array<Matrix3d, 7> R_fromZero_;
        std::array<double, 7> linkLength_;
        bool isLeft_;
        bool isSetZero_;
};