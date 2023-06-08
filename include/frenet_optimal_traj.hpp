/**
 * @file frenet_optimal_traj.hpp
 * @brief: This file contains the class definition of the FrenetOptimalTrajectory class. 
 * This class is used to calculate the optimal trajectory for the vehicle to follow.
 * @auther: Chandravaran Kunjeti
 * @date: 27th April 2023
 * @version: 1.0
 * @last_updated_on: 8th June 2023
*/
#include<iostream>
#include<limits>
#include<vector>
#include<sys/time.h>
#include <numeric>
#include"cubic_spline.h"
#include"frenet_path.h"
#include"quintic_polynomial.h"
#include"quartic_polynomial.h"

using namespace frenet;
using namespace std;

class FrenetOptimalTrajectory{

private:
  double max_speed_ = 6.0;
  double max_accel_ = 3.0;
  double max_curvature_ = 5.0;
  double max_road_width_ = 0.5;
  double d_road_w_ = 0.01;
  double dt_ = 0.1;
  double maxt_ = 1.5;
  double mint_ = 0.11;
  double target_speed_ = 2.0;
  double d_t_s_ = 0.5;
  int n_s_sample_ = 1;
  double robot_radius_ = 0.2;
  double di;

  double kj_ = 0.1;
  double kt_ = 0.1;
  double kd_ = 1.0;
  double klat_ = 10.0;
  double klon_ = 1.0;

  double s_max_ = 27.8478591615;
  double max_traj_len_ = 6;

public:
  FrenetOptimalTrajectory(double s_max){
    s_max_ = s_max;
  };

  FrenetOptimalTrajectory(){
  };  
  /**
  * @brief: calculate sum of power of a vector
  * @param: value_list: a vector of float
  * @return: sum of power of the vector
  */
  float sum_of_power(std::vector<float> value_list){
    float sum = 0;
    for(float item:value_list){
      sum += item*item;
    }
    return sum;
  };

  float wrap_s(float s){
    if (s > s_max_){
      s = fmod(s, s_max_);
    }    
    return s;
  };

  void set_max_s(float s){
    s_max_ = s;
  }

  float wrap_theta(float q){
    return fmod((q + M_PI),(2*M_PI)) - M_PI;
  }
  /**
  * @brief: calculate frenet paths
  * @param: c_speed: current speed
  * @param: c_d: current lateral position
  * @param: c_d_d: current lateral speed
  * @param: c_d_dd: current lateral acceleration
  * @param: s0: current course position
  * @return: frenet paths
  */
  Vec_Path calc_frenet_paths(
    float c_speed, float c_d, float c_d_d, float c_d_dd, float s0){
    std::vector<FrenetPath> fp_list;
    // for(float di=-max_road_width_; di<max_road_width_; di+=d_road_w_){
    // std::vector<double> di_list({-0.5, -0.45, -0.4, -0.3, 0, 0.3, 0.4, 0.45, 0.5});
    std::vector<double> di_list(50);
    std::iota(di_list.begin(), di_list.end(), 0);
    cout << "di_list.size() = " << di_list.size() << endl;
    for(int i=0; i<di_list.size(); i++){
      di = di_list[i];
      FrenetPath fp;
      QuinticPolynomial lat_qp(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, maxt_);
      for(float t=0; t<maxt_; t+=dt_){
        fp.t.push_back(t);
        fp.d.push_back(lat_qp.calc_point(t));
        fp.d_d.push_back(lat_qp.calc_first_derivative(t));
        fp.d_dd.push_back(lat_qp.calc_second_derivative(t));
        fp.d_ddd.push_back(lat_qp.calc_third_derivative(t));
      }

      QuarticPolynomial lon_qp(s0, c_speed, 0.0, target_speed_, 0.0, maxt_);

      fp.max_speed = std::numeric_limits<float>::min();
      fp.max_accel = std::numeric_limits<float>::min();
      for(float t_:fp.t){
        fp.s.push_back(lon_qp.calc_point(t_));
        fp.s_d.push_back(lon_qp.calc_first_derivative(t_));
        fp.s_dd.push_back(lon_qp.calc_second_derivative(t_));
        fp.s_ddd.push_back(lon_qp.calc_third_derivative(t_));
        if(fp.s_d.back() > fp.max_speed){
          fp.max_speed = fp.s_d.back();
        }
        if(fp.s_dd.back() > fp.max_accel){
          fp.max_accel = fp.s_dd.back();
        }
      }

      float Jp = sum_of_power(fp.d_ddd);
      float Js = sum_of_power(fp.s_ddd);
      float ds = (target_speed_ - fp.s_d.back());

      fp.cd = kj_ * Jp + kt_ * maxt_ + kd_ * std::pow(fp.d.back(), 2);
      fp.cv = kj_ * Js + kt_ * maxt_ + kd_ * ds;
      fp.cf = klat_ * fp.cd + klon_ * fp.cv;

      fp_list.push_back(fp);
    }
    return fp_list;
  }

  void calc_global_paths(Vec_Path & path_list, Spline2D csp){
    for (Vec_Path::iterator path_p=path_list.begin(); path_p!=path_list.end();path_p++){
      for(unsigned int i=0; i<path_p->s.size(); i++){
        if (path_p->s[i] >= csp.s.back()){
          path_p->s[i] -= csp.s.back();
        }
        std::array<float, 2> poi = csp.calc_postion(path_p->s[i]);
        float iyaw = csp.calc_yaw(path_p->s[i]);
        float di = path_p->d[i];
        float x = poi[0] + di * std::cos(iyaw + M_PI/2.0);
        float y = poi[1] + di * std::sin(iyaw + M_PI/2.0);
        path_p->x.push_back(x);
        path_p->y.push_back(y);
      }

      for(int i=0; i<path_p->x.size()-1; i++){
        float dx = path_p->x[i + 1] - path_p->x[i];
        float dy = path_p->y[i + 1] - path_p->y[i];
        path_p->yaw.push_back(std::atan2(dy, dx));
        path_p->ds.push_back(std::sqrt(dx * dx + dy * dy));
      }

      path_p->yaw.push_back(path_p->yaw.back());
      path_p->ds.push_back(path_p->ds.back());


      path_p->max_curvature = std::numeric_limits<float>::min();
      for(int i=0; i<path_p->x.size()-1; i++){
        path_p->c.push_back((path_p->yaw[i+1]-path_p->yaw[i])/path_p->ds[i]);
        if(path_p->c.back() > path_p->max_curvature){
          path_p->max_curvature = path_p->c.back();
        }
      }
    }
  }

  bool check_collision(FrenetPath path, const Vec_Pos ob){
    for(auto point:ob){
      for(unsigned int i=0; i<path.x.size(); i++){
        float dist = std::pow((path.x[i] - point[0]), 2) + std::pow((path.y[i] - point[1]), 2);
        if (dist <= robot_radius_ * robot_radius_){
          return false;
        }
      }
    }
    return true;
  }

  Vec_Path check_paths(Vec_Path path_list, const Vec_Pos ob){
  	Vec_Path output_fp_list;
    for(FrenetPath path:path_list){
      if (path.max_speed < max_speed_ && path.max_accel < max_accel_ && path.max_curvature < max_curvature_ && check_collision(path, ob)){
        output_fp_list.push_back(path);
      }
    }
    return output_fp_list;
  }

  FrenetPath frenet_optimal_planning( Spline2D csp, float s0, float c_speed,
      float c_d, float c_d_d, float c_d_dd, Vec_Pos ob){
    
    Vec_Path fp_list = calc_frenet_paths(c_speed, c_d, c_d_d, c_d_dd, s0);
    calc_global_paths(fp_list, csp);
    Vec_Path save_paths = check_paths(fp_list, ob);

    float min_cost = std::numeric_limits<float>::max();
    FrenetPath final_path;
    for(auto path:save_paths){
      if (min_cost >= path.cf){
        min_cost = path.cf;
        final_path = path;
      }
    }
    return final_path;
  }
};



