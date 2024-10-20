#include "state_lattice_planner/lookup_table_utils.h"

namespace LookupTableUtils
{
    StateWithControlParams::StateWithControlParams(void)
    {

    }

    bool load_lookup_table(
      const std::string& lookup_table_file_name,
      LookupTable& lookup_table, std::vector<Eigen::Vector3d>& xyyaw_table)
    {
        lookup_table.clear();
        std::cout << "loading lookup table from " << lookup_table_file_name << std::endl;
        std::ifstream ifs(lookup_table_file_name);
        if(ifs){
            bool first_line_flag = true;
            Eigen::Vector3d err(0.05, 0.05, 0.05);
            while(!ifs.eof()){
                std::string data;
                std::getline(ifs, data);
                if(data == ""){
                    continue;
                }
                if(first_line_flag){
                    first_line_flag = false;
                    continue;
                }
                std::istringstream stream(data);
                std::vector<double> splitted_data;
                std::string buffer;
                while(std::getline(stream, buffer, ',')){
                    splitted_data.push_back(std::stod(buffer));
                }
                StateWithControlParams param;
                auto it = splitted_data.begin();
                double v0 = *(it);
                param.control.omega.k0 = *(++it);
                double x = *(++it);
                double y = *(++it);
                double yaw = *(++it);
                param.state << x, y, yaw;
                if (xyyaw_table.empty())
                  xyyaw_table.push_back(param.state);
                else if (fabs(xyyaw_table.back()(0)-x) < 0.05 || fabs(xyyaw_table.back()(1)-y) < 0.05)
                  xyyaw_table.push_back(param.state);
                param.control.omega.km = *(++it);
                param.control.omega.kf = *(++it);
                param.control.omega.sf = *(++it);
                lookup_table[v0][param.control.omega.k0].push_back(param);
            }
            ifs.close();
        }else{
            std::cout << "\033[91mERROR: cannot open file\033[00m" << std::endl;
            // exit(-1);
            return false;
        }
        std::cout << "x y yaw table size:" << xyyaw_table.size() << std::endl;
        return true;
    }

    void get_optimized_param_from_lookup_table(
      const LookupTable& lookup_table, const Eigen::Vector3d goal,
      const double v0, const double k0, MotionModelDiffDrive::ControlParams& param)
    {
        if(lookup_table.size() > 0){
            // 查表找与当前速度 v0 接近的速度
            double min_v_diff = 1e3;
            double v = 0;            
            for(const auto& v_data : lookup_table){
                double _v = v_data.first;
                double diff = fabs(_v - v0);
                if(diff < min_v_diff){
                    min_v_diff = diff;
                    v = _v;
                }
            }
            // 查表找与当前角速度 k0 接近的角速度
            double min_k_diff = 1e3;
            double k = 0;
            for(const auto& k_data : lookup_table.at(v)){
                double _k = k_data.first;
                double diff = fabs(_k - k0);
                if(diff < min_k_diff){
                    min_k_diff = diff;
                    k = _k;
                }
            }
            // 查表找与目标点 goal[x, y, yaw] 接近的坐标
            double min_cost = 1e3;
            StateWithControlParams _param;
            for(const auto& data : lookup_table.at(v).at(k)){
                // sqrt(x^2 + y^2 + yaw^2)
                double cost = (goal - data.state).norm();
                if(cost < min_cost){
                    min_cost = cost;
                    _param = data;
                }
            }
            param = _param.control;
        }else{
            param.omega.km = 0;
            param.omega.kf = 0;
            param.omega.sf = goal.segment(0, 2).norm();
        }
    }
}
