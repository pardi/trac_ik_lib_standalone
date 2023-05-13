/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
Copyright (c) 2023, Tommaso Pardi
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef NLOPT_IK_HPP
#define NLOPT_IK_HPP

#include <trac_ik/kdl_tl.hpp>
#include <nlopt.hpp>


namespace trac_ik
{

enum class OptType { Joint, DualQuat, SumSq, L2 };


class NLOPT_IK
{
  friend class trac_ik::TRAC_IK;
  
  public:
  
  NLOPT_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double max_time = 0.005, double eps = 1e-3, OptType type = OptType::SumSq);

  ~NLOPT_IK() {};
  int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds = KDL::Twist::Zero(), const KDL::JntArray& q_desired = KDL::JntArray());

  double minJoints(const std::vector<double>& x, std::vector<double>& grad);
  
  //  void cartFourPointError(const std::vector<double>& x, double error[]);

  void cartSumSquaredError(const std::vector<double>& x, double error[]);
  void cartDQError(const std::vector<double>& x, double error[]);
  void cartL2NormError(const std::vector<double>& x, double error[]);

  inline void setMaxtime(double t)
  {
    max_time_ = t;
  }

  private:

  // Private member variable

  const KDL::Chain chain_;
  std::vector<double> l_bounds_;
  std::vector<double> u_bounds_;

  std::vector<double> des_;
  KDL::ChainFkSolverPos_recursive fksolver_;

  double max_time_;
  double eps_;
  int iter_counter_;
  OptType opt_type_;

  KDL::Frame target_pose_;
  KDL::Frame current_pose_;
  KDL::Frame z_up_;
  KDL::Frame x_out_;
  KDL::Frame y_out_;
  KDL::Frame z_target_;
  KDL::Frame x_target_;
  KDL::Frame y_target_;

  std::vector<KDL::BasicJointType> joint_types_;

  nlopt::opt opt_;

  std::vector<double> best_x_;
  
  int progress_;
  bool aborted_{false};

  KDL::Twist bounds_;

  inline void abort()
  {
    aborted_ = true;
  }

  inline void reset()
  {
    aborted_ = false;
  }

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

};

}

#endif
