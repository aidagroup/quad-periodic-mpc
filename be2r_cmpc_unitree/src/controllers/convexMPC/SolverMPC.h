#pragma once

#include "common_types.h"
#include "convexMPC_interface.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <complex>
#include <fftw3.h>
#include <algorithm>
#include <numeric>
#include <tuple>
#include <std_msgs/Float32.h> 


using Eigen::Matrix;
using Eigen::Quaterniond;
using Eigen::Quaternionf;

template<class T>
void print_array(T* array, u16 rows, u16 cols)
{
  for (u16 r = 0; r < rows; r++)
  {
    for (u16 c = 0; c < cols; c++)
      std::cout << (fpt)array[c + r * cols] << " ";
    printf("\n");
  }
}

void crossMatrix(Eigen::MatrixXd& R, const Eigen::VectorXd& omega);

template<class T>
void print_named_array(const char* name, T* array, u16 rows, u16 cols)
{
  printf("%s:\n", name);
  print_array(array, rows, cols);
}

// print named variable
template<class T>
void pnv(const char* name, T v)
{
  printf("%s: ", name);
  std::cout << v << std::endl;
}

template<class T>
T t_min(T a, T b)
{
  if (a < b)
    return a;
  return b;
}

template<class T>
T sq(T a)
{
  return a * a;
}

void solve_mpc(update_data_t* update, problem_setup* setup);

void quat_to_rpy(Quaternionf q, Matrix<fpt, 3, 1>& rpy);
void ct_ss_mats(Matrix<fpt, 3, 3> I_world, fpt m, Matrix<fpt, 3, 4> r_feet, Matrix<fpt, 3, 3> R_yaw, Matrix<fpt, 13, 13>& A, Matrix<fpt, 13, 12>& B);
void resize_qp_mats(s16 horizon);
void c2qp(Matrix<fpt, 13, 13> Ac, Matrix<fpt, 13, 12> Bc, fpt dt, s16 horizon);
mfp* get_q_soln();


extern Eigen::Matrix<float, 6, 1> f_est;
extern Eigen::Matrix<float, 6, 1> f_est_smoothed;
extern Eigen::Matrix<float, 6, 1> f_est_static;

extern Eigen::Matrix<float, 6, 1> alpha;
extern Eigen::Matrix<float, 6, 1> beta;
// extern Eigen::Matrix<float, 6, 1> alpha_stationary;
// extern Eigen::Matrix<float, 6, 1> beta_stationary;

extern std::vector<float> time_history;
extern std::vector<float> diff_history;


bool estimate_disturbance(const std::vector<float>& tt, 
                          const std::vector<float>& diff, 
                          float dt,
                          float &stat_est, float &amp_est, 
                          float &freq_est, float &phase_est);


