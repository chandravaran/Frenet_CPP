/**
 * @file frenet_types.hpp
 * @brief: This file contains the datatypes used for the frenet implementation.
 * Most of this code is taken from the original cpp code written by LEI TAI 
 * @auther: Chandravaran Kunjeti
 * @date: 27th April 2023
 * @version: 1.0
 * @last_updated_on: 8th June 2023
*/

#ifndef _FRENET_TYPES_H
#define _FRENET_TYPES_H

#include<iterator>
#include<vector>
#include<array>
#include<string>
#include<iostream>

namespace frenet{

using Vec_float=std::vector<float>;
using Pos_float=std::array<float, 2>;
using Vec_Pos=std::vector<Pos_float>;

};

#endif
