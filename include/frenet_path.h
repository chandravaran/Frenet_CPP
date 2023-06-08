/**
 * @file frenet_path.hpp
 * @brief: This file contains the class definition of a class definition of the FrenetPath class.
 * Most of this code is taken from the original cpp code written by LEI TAI 
 * @auther: Chandravaran Kunjeti
 * @date: 27th April 2023
 * @version: 1.0
 * @last_updated_on: 8th June 2023
*/
#ifndef _FRENET_PATH_H
#define _FRENET_PATH_H

#include<iostream>
#include<vector>
#include<array>
#include<string>
#include"frenet_types.h"

namespace frenet{

class FrenetPath{
public:
  float cd = 0.0;
  float cv = 0.0;
  float cf = 0.0;

  Vec_float t;
  Vec_float d;
  Vec_float d_d;
  Vec_float d_dd;
  Vec_float d_ddd;
  Vec_float s;
  Vec_float s_d;
  Vec_float s_dd;
  Vec_float s_ddd;

  Vec_float x;
  Vec_float y;
  Vec_float yaw;
  Vec_float ds;
  Vec_float c;

  float max_speed;
  float max_accel;
  float max_curvature;
};

using Vec_Path=std::vector<FrenetPath>;
}
#endif
