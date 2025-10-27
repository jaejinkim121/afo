#pragma once
#include <vector>
#include <queue>
#include <mutex>
#include <array>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <highs/Highs.h>

const size_t N = 10;

struct SampleIMU{
    float t;
    std::array<float, 21> value;
};

struct NormSampleIMU{
float cycle_time;
std::array<std::array<float, 21>, N+1> d;
};


class Optimizer{
    public:
        Optimizer(){

            /// Optimization problem define.
            col_cost.reserve(num_col);
            col_cost.insert(col_cost.end(), 50, 0.0);
            col_cost.insert(col_cost.end(), 50, 1.0);
            col_lower.reserve(num_col);
            col_lower.insert(col_lower.end(), num_col, 0.0);
            col_upper.reserve(num_col);
            // Last state must be zero. To make it cyclic.
            col_upper.insert(col_upper.end(), num_col-1, torque_upper_limit);
            col_upper.push_back(0.0);

            row_lower.reserve(num_row);
            row_lower.insert(row_lower.end(), N_t, -jerk_absolute_upper_limit);
            row_lower.push_back(-jerk_absolute_upper_limit);
            row_lower.push_back(torque_impulse_value);
            
            row_upper.reserve(num_row);
            row_upper.insert(row_upper.end(), N_t, jerk_absolute_upper_limit);
            row_upper.push_back(jerk_absolute_upper_limit);
            row_upper.push_back(torque_impulse_value);

            for (int i = 0; i <= num_col; i++){
                Astart.push_back(3 * i);
            }
            for (int i = 0; i < num_col; i++){
                Aindex.push_back(i);
                Aindex.push_back(i+1);
                Aindex.push_back(num_col + 1);

                Avalue.push_back(1.0);
                Avalue.push_back(-1.0);
                Avalue.push_back(1.0);
            }

        }
        ~Optimizer() = default;
        
        void set_efficacy(){
        }

        void run_optimize(){
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
            std::cout << "x: ";
            for (int i = 0; i<num_col;i++) std::cout << sol.col_value[i] << ",";
            std::cout << std::endl;
        }

    private:
        // Define Optimization problem parameters.
        static const unsigned int N_t = 100; // dt = 1 / N. For this case, dt=10ms.
        double torque_upper_limit = 1.0; // Can be kHighSInf
        double jerk_absolute_upper_limit = 0.005;
        double torque_impulse_value = 0.3 * N_t;

        // Define HiGHS Instances.
        HighsLp lp_;
        Highs highs_;
        HighsStatus return_status_;

        const HighsInt num_col = N_t;
        const HighsInt num_row = N_t + 2;

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
        ImuOptimizer(){
            seg_.reserve(1024);
        }
        ~ImuOptimizer() = default;

        void push(float t, std::array<float, 21>& d){
            SampleIMU d_ = SampleIMU();
            d_.value = d;
            d_.t = t;
            seg_.emplace_back(std::move(d_));
        }

        bool cut(){
            const size_t n = seg_.size();
            if (n == 0 ) return false;
            if (n == 1) return false;

            const float ti = seg_.front().t;
            const float tf = seg_.back().t;
            float duration = tf - ti;

            NormSampleIMU out;
            out.cycle_time = duration;

            int i = 0;
            for (int k=0 ; k <= N; k++){
                const float s = static_cast<float>(k) / static_cast<float>(N);
                const float tau = ti + s * duration;

                while(i+1 < n && seg_[i+1].t < tau) i++;
                if (i + 1 >= n) i = n - 2;
                const float ta = seg_[i].t;
                const float tb = seg_[i+1].t;
                const float alpha = (tau - ta)/(tb - ta);

                for (int ch = 0; ch < 21; ch++){
                    const float va = seg_[i].value[ch];
                    const float vb = seg_[i+1].value[ch];
                    out.d[k][ch] = va + (vb-va) * alpha;
                }
            }
            q_.push(std::move(out));

            seg_.clear();
            seg_.reserve(1024);

            return true;
        }
        
        bool mean(){
            if (q_.empty()) return false;

            NormSampleIMU sum;
            static constexpr int K = N+1;
            static constexpr int C = 21;
            mean_.d[K][C] = {};
            sum.d[K][C] = {};
            sum.cycle_time = 0.0;
            size_t count = 0;
            while(!q_.empty()){
                NormSampleIMU seg = std::move(q_.front());
                q_.pop();
                count++;

                sum.cycle_time += static_cast<float>(seg.cycle_time);
                for (int k = 0; k < K; k++){
                    for (int ch = 0; ch < C; ch++){
                        sum.d[k][ch] += static_cast<float>(seg.d[k][ch]);
                    }
                }
            }
            const float invN = 1.0 / static_cast<float>(count);
            mean_.cycle_time = sum.cycle_time * invN;
            for (int k = 0; k < K; k++){
                for (int ch = 0; ch < C; ch++){
                    mean_.d[k][ch] = sum.d[k][ch] * invN;
                }
            }
            return true;
        }

        void setZero(float* d){
            zero_.t = 0.0;
            for (int i = 0 ; i < 21; i++){
                zero_.value[i] = d[i];
            }
            return;
        }
        void convertTLA(){
            // zero_와 mean_을 이용해서 TLA로 컨버트함.
            // link length 같은 것 재야 하는건지..
            // pelvis 위치를 정확히 알려면 link length 설정이 중요할 터인데
            
        }

        void optimize(){
            
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
                for (int ch = 0; ch < 21; ch++) ofs << ",ch" << ch;
                ofs << "\n";

                for (int k = 0; k <= N; k++){
                    float s = static_cast<float>(k) / static_cast<float>(N);
                    ofs << s;
                    for (int ch=0; ch < 21; ch++){
                        ofs << "," << seg.d[k][ch];
                    }
                    ofs <<"\n";
                }
                ofs.close();
            }
        }
        

    private:
        SampleIMU zero_;
        std::vector<SampleIMU> seg_;
        std::queue<NormSampleIMU> q_;
        NormSampleIMU mean_;
        std::array<float, 101> tla_;
        std::array<float, 101> result_opt;
};


int main(){
    Optimizer opt = Optimizer();
    opt.run_optimize();
    
    return 1;
    ImuOptimizer imuopt = ImuOptimizer();
    float t = 0;
    std::array<float, 21> d;
    std::ifstream input;
    input.open("./d.csv");
    std::string line;
    std::string field;
    int lineidx = 0;
    int fileidx = 0;
    while(std::getline(input, line)){
        if (lineidx++ == 0){continue;}
        std::array<float, 22> row;
        std::stringstream ss(line);
        for (int i = 0; i < 22; i++){
            std::getline(ss, field, ',');
            row[i] = std::stof(field);
        }
        t = row[0];
        for (int i =0;i<21;i++) d[i] = row[i+1];
        imuopt.push(t,d);
        if (lineidx % 200 == 0) {
            imuopt.cut();
            if (fileidx++ < 5) continue;
            else {
                std::cout << imuopt.mean() << std::endl;
                imuopt.save();
                std::cout << 1 << std::endl;
                break;
            }

        }
    }


    return 1;
}