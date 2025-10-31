#include "../include/segment.hpp"

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
        Optimizer(){

            /// Optimization problem define.
            col_cost.reserve(num_col);
            col_cost.insert(col_cost.end(), 50, -1.0);
            col_cost.insert(col_cost.end(), N_t - 50, 1.0);
            col_lower.reserve(num_col);
            col_lower.insert(col_lower.end(), num_col, 0.0);
            col_upper.reserve(num_col);
            col_upper.insert(col_upper.end(), num_col, torque_upper_limit);

            row_lower.reserve(num_row);
            row_lower.insert(row_lower.end(), N_t + 1, -jerk_absolute_upper_limit);
            row_lower.push_back(torque_impulse_value);
            
            row_upper.reserve(num_row);
            row_upper.insert(row_upper.end(), N_t + 1, jerk_absolute_upper_limit);
            row_upper.push_back(torque_impulse_value);

            for (int i = 0; i <= num_col; i++){
                Astart.push_back(3 * i);
            }
            for (int i = 0; i < num_col; i++){
                Aindex.push_back(i);
                Aindex.push_back(i+1);
                Aindex.push_back(N_t + 1);

                Avalue.push_back(1.0);
                Avalue.push_back(-1.0);
                Avalue.push_back(1.0);
            }

            highs_.setOptionValue("output_flag", false);

        }

        ~Optimizer() = default;
        
        void set_efficacy(const std::vector<double>& efficacy){
            if (efficacy.size() != N_t){
                std::cout << "Optimizer - set_efficacy - Wrong size of input efficacy" << std::endl;
                return;
            }
            
            col_cost.clear();
            for (int i = 0; i < efficacy.size(); i++){
                col_cost.push_back(efficacy[i]);
            }
            return;
        }

        void run_optimize(std::vector<double>& r){
            if (r.size() != N_t+2){
                std::cout << "Optimizer - run_optimize - input result vector has wrong size" << std::endl;
            }
            lp_.sense_ = ObjSense::kMaximize;
            lp_.num_col_ = num_col;
            lp_.num_row_ = num_row;
            lp_.col_cost_  = col_cost;
            lp_.col_lower_ = col_lower;
            lp_.col_upper_ = col_upper;
            lp_.row_lower_ = row_lower;
            lp_.row_upper_ = row_upper;

            lp_.a_matrix_.format_ = MatrixFormat::kColwise;
            lp_.a_matrix_.start_  = Astart;
            lp_.a_matrix_.index_  = Aindex;
            lp_.a_matrix_.value_  = Avalue;

            highs_.passModel(lp_);
            highs_.run();

            const HighsSolution& sol = highs_.getSolution();
            for (int i = 0; i<num_col;i++) r[i+1] = sol.col_value[i];
        }

    private:
        // Define Optimization problem parameters.
        static const unsigned int N_t = 99; // dt = 1 / (N_t+1). For this case, dt=10ms.
        double torque_upper_limit = 1.0; // Can be kHighSInf
        double jerk_absolute_upper_limit = 0.05;
        double torque_impulse_value = 0.1 * N_t;

        // Define HiGHS Instances.
        HighsLp lp_;
        Highs highs_;
        HighsStatus return_status_;

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

class ImuOptimizer {
    public:
        ImuOptimizer(bool isLeft){
            isLeft_ = isLeft;
            isSetZero_ = false;
            seg_.reserve(1024);
            opt_ = Optimizer();
            tla_.reserve(101);
            tla_.insert(tla_.end(), 101, 0.0);
            result_opt_.reserve(101);
            result_opt_.insert(result_opt_.end(), 101, 0.0);

        }
        ~ImuOptimizer() = default;

        float push(float t, std::array<float, 21>& d){
            SampleTLA d_ = SampleTLA();
            double angle = getTLA(d);
            d_.value = angle;
            d_.t = t;
            seg_.emplace_back(std::move(d_));
            return angle;
        }

        bool cut(){
            const size_t n = seg_.size();
            if (n == 0 ) return false;
            if (n == 1) return false;
            const float ti = seg_.front().t;
            const float tf = seg_.back().t;
            float duration = tf - ti;

            NormTLA out;
            out.cycle_time = duration;

            int i = 0;
            for (int k = 0; k <= N; k++){
                const float s = static_cast<float>(k) / static_cast<float>(N);
                const float tau = ti + s * duration;

                while(i+1 < n && seg_[i+1].t < tau) i++;
                if (i + 1 >= n) i = n - 2;
                const float ta = seg_[i].t;
                const float tb = seg_[i+1].t;
                const float alpha = (tau - ta)/(tb - ta);
                const float va = seg_[i].value;
                const float vb = seg_[i+1].value;
                out.d[k] = va + (vb-va) * alpha;
            
            }
            q_.push(std::move(out));

            seg_.clear();
            seg_.reserve(1024);

            return true;
        }
        
        bool mean(){
            if (q_.empty()) return false;

            NormTLA sum;
            static constexpr int K = N+1;
            mean_.d[K] = {};
            sum.d[K] = {};
            sum.cycle_time = 0.0;
            size_t count = 0;
            while(!q_.empty()){
                NormTLA seg = std::move(q_.front());
                q_.pop();
                count++;

                sum.cycle_time += static_cast<float>(seg.cycle_time);
                for (int k = 0; k < K; k++){
                        sum.d[k] += static_cast<float>(seg.d[k]);
                    }
                }
            }
            const float invN = 1.0 / static_cast<float>(count);
            mean_.cycle_time = sum.cycle_time * invN;
            for (int k = 0; k < K; k++){
                mean_.d[k] = sum.d[k] * invN;
            }
            return true;
        }

        void setZero(std::array<float, 21>& d){
            zero_.t = 0.0;
            for (int i = 0 ; i < 21; i++){
                zero_.value[i] = d[i];
            }
            for (int i = 0; i < 7; i++){
                R_fromZero_[i] = AngleAxisd(d[3*i], Vector3d::UnitZ()) * AngleAxisd(d[3 * i + 1], Vector3d::UnitY()) * AngleAxisd(d[3 * i + 2], Vector3d::UnitX());
            }
            isSetZero_ = true;
            return;
        }

        double getTLA(std::array<float, 21>& d){
            if (!isSetZero_) return 0.0;
            float angle[3];
            Matrix3d R_fromData = AngleAxisd();
            std::array<Matrix3d, 7> R_;
            for (int i = 0 ; i < 7; i++){
                R_fromData = AngleAxisd(d[3*i], Vector3d::UnitZ()) * AngleAxisd(d[3 * i + 1], Vector3d::UnitY()) * AngleAxisd(d[3 * i + 2], Vector3d::UnitX());
                R_[i] = R_fromZero_[i] * R_fromData;
            }
            std::array<Vector3d, 7> p;

            for (int i = 1; i < 7; i++){
                p[i] = -R_[i] * Vector3d::UnitY();
            }
            p[0] = R_[0] * Vector3d::UnitX();
            
            double x,z;
            x = 0;
            z = 0;
            if (isLeft_){
                y = p[0].y() * linkLength_[0] * 0.5 + p[1].y() * linkLength_[1] + p[2].y() * linkLength_[2] - p[3].y() * linkLength_[3];  // Foot IMU direction 때문에 negative sign 포함.
                z = p[0].z() * linkLength_[0] * 0.5 + p[1].z() * linkLength_[1] + p[2].z() * linkLength_[2] - p[3].z() * linkLength_[3];  // Foot IMU direction 때문에 negative sign 포함.
            }
            else{
                y = -p[0].y() * linkLength_[0] * 0.5 + p[4].y() * linkLength_[1] + p[5].y() * linkLength_[2] - p[6].y() * linkLength_[3];  // Foot IMU direction 때문에 negative sign 포함.
                z = -p[0].z() * linkLength_[0] * 0.5 + p[4].z() * linkLength_[1] + p[5].z() * linkLength_[2] - p[6].z() * linkLength_[3];  // Foot IMU direction 때문에 negative sign 포함.
            }
            
            return std::atan2(z,y) * 180 / 3.141592; // Unit: [rad]

        }

        void optimize(){
            opt_.set_efficacy(mean_.d);
            opt_.run_optimize(result_opt_);
        }

        void save(){
            std::size_t idx_ = 0;
            while(idx_++ < 1){
                NormSampleIMU seg = mean_;
                
                std::ostringstream path;
                path << "./data/" << (++idx_) <<".csv";
                std::cout << path.str() << std::endl;
                std::ofstream ofs(path.str());
                if (!ofs){
                    std::cout << "file open error" << std::endl;
                    return;
                }

                ofs << std::fixed << std::setprecision(4);
                ofs << "duration," << seg.cycle_time << "\n";
                ofs << "s";
                ofs << ",ch" << 1;
                ofs << "\n";

                for (int k = 0; k <= N; k++){
                    float s = static_cast<float>(k) / static_cast<float>(N);
                    ofs << s;
                    ofs << "," << seg.d[k];
                    ofs <<"\n";
                }
                ofs.close();
            }
        }

        void getResult(std::vector<double>& target){
            target = std::copy(result_opt_);
            return;
        }
        

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
