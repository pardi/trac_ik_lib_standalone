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

#include <trac_ik/nlopt_ik.hpp>
#include <limits>
#include <boost/date_time.hpp>
#include <trac_ik/dual_quaternion.h>
#include <cmath>



using namespace trac_ik;

dual_quaternion targetDQ;

double minfunc(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Auxilory function to minimize (Sum of Squared joint angle error
  // from the requested configuration).  Because we wanted a Class
  // without static members, but NLOpt library does not support
  // passing methods of Classes, we use these auxilary functions.

  NLOPT_IK *c = (NLOPT_IK *) data;

  return c->minJoints(x, grad);
}

double minfuncDQ(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Auxilory function to minimize (Sum of Squared joint angle error
  // from the requested configuration).  Because we wanted a Class
  // without static members, but NLOpt library does not support
  // passing methods of Classes, we use these auxilary functions.
  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = std::numeric_limits<float>::epsilon();
  double result[1];
  c->cartDQError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartDQError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2 * jump);
    }
  }

  return result[0];
}


double minfuncSumSquared(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Auxilory function to minimize (Sum of Squared joint angle error
  // from the requested configuration).  Because we wanted a Class
  // without static members, but NLOpt library does not support
  // passing methods of Classes, we use these auxilary functions.

  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = std::numeric_limits<float>::epsilon();
  double result[1];
  c->cartSumSquaredError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartSumSquaredError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2.0 * jump);
    }
  }

  return result[0];
}


double minfuncL2(const std::vector<double>& x, std::vector<double>& grad, void* data)
{
  // Auxilory function to minimize (Sum of Squared joint angle error
  // from the requested configuration).  Because we wanted a Class
  // without static members, but NLOpt library does not support
  // passing methods of Classes, we use these auxilary functions.

  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(x);

  double jump = std::numeric_limits<float>::epsilon();
  double result[1];
  c->cartL2NormError(vals, result);

  if (!grad.empty())
  {
    double v1[1];
    for (uint i = 0; i < x.size(); i++)
    {
      double original = vals[i];

      vals[i] = original + jump;
      c->cartL2NormError(vals, v1);

      vals[i] = original;
      grad[i] = (v1[0] - result[0]) / (2.0 * jump);
    }
  }

  return result[0];
}



void constrainfuncm(uint m, double* result, uint n, const double* x, double* grad, void* data)
{
  //Equality constraint auxilary function for Euclidean distance .
  //This also uses a small walk to approximate the gradient of the
  //constraint function at the current joint angles.

  NLOPT_IK *c = (NLOPT_IK *) data;

  std::vector<double> vals(n);

  for (uint i = 0; i < n; i++)
  {
    vals[i] = x[i];
  }

  double jump = std::numeric_limits<float>::epsilon();

  c->cartSumSquaredError(vals, result);

  if (grad != NULL)
  {
    std::vector<double> v1(m);
    for (uint i = 0; i < n; i++)
    {
      double o = vals[i];
      vals[i] = o + jump;
      c->cartSumSquaredError(vals, v1.data());
      vals[i] = o;
      for (uint j = 0; j < m; j++)
      {
        grad[j * n + i] = (v1[j] - result[j]) / (2 * jump);
      }
    }
  }
}

// TODO: check eps
NLOPT_IK::NLOPT_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double max_time, double eps, OptType opt_type):
  chain_(chain), 
  fksolver_(chain), 
  max_time_(max_time),
  eps_(std::abs(eps)), 
  opt_type_(opt_type)
{
  assert(chain_.getNrOfJoints() == q_min.data.size());
  assert(chain_.getNrOfJoints() == q_max.data.size());

  //Constructor for an IK Class.  Takes in a Chain to operate on,
  //the min and max joint limits, an (optional) maximum number of
  //iterations, and an (optional) desired error.
  reset();

  if (chain_.getNrOfJoints() < 2)
  {
    std::cerr << "NLOpt_IK can only be run for chains of length 2 or more" << std::endl;
    return;
  }
  opt_ = nlopt::opt(nlopt::LD_SLSQP, chain.getNrOfJoints());

  for (uint i = 0; i < chain_.getNrOfJoints(); i++)
  {
    l_bounds_.push_back(q_min(i));
    u_bounds_.push_back(q_max(i));
  }

  for (uint i = 0; i < chain_.segments.size(); i++)
  {
    std::string joint_type = chain_.segments[i].getJoint().getTypeName();
    if (joint_type.find("Rot") != std::string::npos)
    {
      if (q_max(joint_types_.size()) >= std::numeric_limits<float>::max() &&
          q_min(joint_types_.size()) <= std::numeric_limits<float>::lowest()){
          joint_types_.push_back(KDL::BasicJointType::Continuous);
      }
      else{
        joint_types_.push_back(KDL::BasicJointType::RotJoint);
      }
    }
    else {
      if (joint_type.find("Trans") != std::string::npos){
        joint_types_.push_back(KDL::BasicJointType::TransJoint);
      }
    }
  }

  assert(joint_types_.size() == l_bounds_.size());

  std::vector<double> tolerance(1,  std::numeric_limits<float>::epsilon());
  opt_.set_xtol_abs(tolerance[0]);


  switch (opt_type_)
  {
  case OptType::Joint:
    opt_.set_min_objective(minfunc, this);
    opt_.add_equality_mconstraint(constrainfuncm, this, tolerance);
    break;
  case OptType::DualQuat:
    opt_.set_min_objective(minfuncDQ, this);
    break;
  case OptType::SumSq:
    opt_.set_min_objective(minfuncSumSquared, this);
    break;
  case OptType::L2:
    opt_.set_min_objective(minfuncL2, this);
    break;
  }
}


double NLOPT_IK::minJoints(const std::vector<double>& x, std::vector<double>& grad)
{
  // Actual function to compute the error between the current joint
  // configuration and the desired.  The SSE is easy to provide a
  // closed form gradient for.

  bool gradient = !grad.empty();

  double err = 0;
  for (uint i = 0; i < x.size(); i++)
  {
    err += pow(x[i] - des_[i], 2);
    if (gradient)
      grad[i] = 2.0 * (x[i] - des_[i]);
  }

  return err;

}


void NLOPT_IK::cartSumSquaredError(const std::vector<double>& x, double error[])
{
  // Actual function to compute Euclidean distance error.  This uses
  // the KDL Forward Kinematics solver to compute the Cartesian pose
  // of the current joint configuration and compares that to the
  // desired Cartesian pose for the IK solve.

  if (aborted_ || progress_ != -3)
  {
    opt_.force_stop();
    return;
  }


  KDL::JntArray q(x.size());

  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  int rc = fksolver_.JntToCart(q, current_pose_);

  if (rc < 0){ 
    std::cerr << "KDL FKSolver is failing: " << q.data << std::endl;
    // TODO: should it do something else?
  }

  if (std::isnan(current_pose_.p.x()))
  {
    std::cerr << "NaNs from NLOpt!!" << std::endl;
    error[0] = std::numeric_limits<float>::max();
    progress_ = -1;
    return;
  }

  KDL::Twist delta_twist = KDL::diffRelative(target_pose_, current_pose_);

  for (int i = 0; i < 6; i++)
  {
    if (std::abs(delta_twist[i]) <= std::abs(bounds_[i]))
      delta_twist[i] = 0.0;
  }

  error[0] = KDL::dot(delta_twist.vel, delta_twist.vel) + KDL::dot(delta_twist.rot, delta_twist.rot);

  if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_))
  {
    progress_ = 1;
    best_x_ = x;
    return;
  }
}



void NLOPT_IK::cartL2NormError(const std::vector<double>& x, double error[])
{
  // Actual function to compute Euclidean distance error.  This uses
  // the KDL Forward Kinematics solver to compute the Cartesian pose
  // of the current joint configuration and compares that to the
  // desired Cartesian pose for the IK solve.

  if (aborted_ || progress_ != -3)
  {
    opt_.force_stop();
    return;
  }

  KDL::JntArray q(x.size());

  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  int rc = fksolver_.JntToCart(q, current_pose_);

  if (rc < 0){
    std::cerr << "KDL FKSolver is failing: " << q.data << std::endl;
    // TODO: should it do something else?
  }

  if (std::isnan(current_pose_.p.x()))
  {
    std::cerr << "NaNs from NLOpt!!" << std::endl;
    error[0] = std::numeric_limits<float>::max();
    progress_ = -1;
    return;
  }

  KDL::Twist delta_twist = KDL::diffRelative(target_pose_, current_pose_);

  for (int i = 0; i < 6; i++)
  {
    if (std::abs(delta_twist[i]) <= std::abs(bounds_[i]))
      delta_twist[i] = 0.0;
  }

  error[0] = std::sqrt(KDL::dot(delta_twist.vel, delta_twist.vel) + KDL::dot(delta_twist.rot, delta_twist.rot));

  if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_))
  {
    progress_ = 1;
    best_x_ = x;
    return;
  }
}




void NLOPT_IK::cartDQError(const std::vector<double>& x, double error[])
{
  // Actual function to compute Euclidean distance error.  This uses
  // the KDL Forward Kinematics solver to compute the Cartesian pose
  // of the current joint configuration and compares that to the
  // desired Cartesian pose for the IK solve.

  if (aborted_ || progress_ != -3)
  {
    opt_.force_stop();
    return;
  }

  KDL::JntArray q(x.size());

  for (uint i = 0; i < x.size(); i++)
    q(i) = x[i];

  int rc = fksolver_.JntToCart(q, current_pose_);

  if (rc < 0){    
    std::cerr << "KDL FKSolver is failing: " << q.data << std::endl;
    // TODO: should it do something else?
  }

  if (std::isnan(current_pose_.p.x()))
  {
    std::cerr << "NaNs from NLOpt!!" << std::endl;
    error[0] = std::numeric_limits<float>::max();
    progress_ = -1;
    return;
  }

  KDL::Twist delta_twist = KDL::diffRelative(target_pose_, current_pose_);

  for (int i = 0; i < 6; i++)
  {
    if (std::abs(delta_twist[i]) <= std::abs(bounds_[i]))
      delta_twist[i] = 0.0;
  }

  math3d::matrix3x3<double> currentRotationMatrix(current_pose_.M.data);
  math3d::quaternion<double> currentQuaternion = math3d::rot_matrix_to_quaternion<double>(currentRotationMatrix);
  math3d::point3d currentTranslation(current_pose_.p.data);
  dual_quaternion currentDQ = dual_quaternion::rigid_transformation(currentQuaternion, currentTranslation);

  dual_quaternion errorDQ = (currentDQ * !targetDQ).normalize();
  errorDQ.log();
  error[0] = 4.0f * dot(errorDQ, errorDQ);


  if (KDL::Equal(delta_twist, KDL::Twist::Zero(), eps_))
  {
    progress_ = 1;
    best_x_ = x;
    return;
  }
}


int NLOPT_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist bounds, const KDL::JntArray& q_desired)
{
  // User command to start an IK solve.  Takes in a seed
  // configuration, a Cartesian pose, and (optional) a desired
  // configuration.  If the desired is not provided, the seed is
  // used.  Outputs the joint configuration found that solves the
  // IK.

  // Returns -3 if a configuration could not be found within the eps
  // set up in the constructor.

  boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration diff;

  bounds_ = bounds;
  q_out = q_init;

  if (chain_.getNrOfJoints() < 2)
  {
    std::cerr << "NLOpt_IK can only be run for chains of length 2 or more" << std::endl;
    return -3;
  }

  if (q_init.data.size() != joint_types_.size())
  {
    std::cout << "IK seeded with wrong number of joints.  Expected " << (int)joint_types_.size() << " but got " << (int)q_init.data.size() <<  std::endl;
    return -3;
  }

  opt_.set_maxtime(max_time_);

  double minf; /* the minimum objective value, upon return */

  target_pose_ = p_in;

  if (opt_type_ == OptType::DualQuat)   // DQ
  {
    math3d::matrix3x3<double> targetRotationMatrix(target_pose_.M.data);
    math3d::quaternion<double> targetQuaternion = math3d::rot_matrix_to_quaternion<double>(targetRotationMatrix);
    math3d::point3d targetTranslation(target_pose_.p.data);
    targetDQ = dual_quaternion::rigid_transformation(targetQuaternion, targetTranslation);
  }
  // else if (opt_type_ == 1)
  // {
  //   z_target = target_pose_*z_up;
  //   x_target = target_pose_*x_out;
  //   y_target = target_pose_*y_out;
  // }


  //    fksolver_.JntToCart(q_init, current_pose_);

  std::vector<double> x(chain_.getNrOfJoints());

  for (uint i = 0; i < x.size(); i++)
  {
    x[i] = q_init(i);

    if (joint_types_[i] == KDL::BasicJointType::Continuous)
      continue;

    if (joint_types_[i] == KDL::BasicJointType::TransJoint)
    {
      x[i] = std::min(x[i], u_bounds_[i]);
      x[i] = std::max(x[i], l_bounds_[i]);
    }
    else
    {

      // Below is to handle bad seeds outside of limits

      if (x[i] > u_bounds_[i])
      {
        //Find actual angle offset
        double diffangle = fmod(x[i] - u_bounds_[i], 2 * M_PI);
        // Add that to upper bound and go back a full rotation
        x[i] = u_bounds_[i] + diffangle - 2 * M_PI;
      }

      if (x[i] < l_bounds_[i])
      {
        //Find actual angle offset
        double diffangle = fmod(l_bounds_[i] - x[i], 2 * M_PI);
        // Subtract that from lower bound and go forward a full rotation
        x[i] = l_bounds_[i] - diffangle + 2 * M_PI;
      }

      if (x[i] > u_bounds_[i])
        x[i] = (u_bounds_[i] + l_bounds_[i]) / 2.0;
    }
  }

  best_x_ = x;
  progress_ = -3;

  std::vector<double> artificial_lower_limits(l_bounds_.size());

  for (uint i = 0; i < l_bounds_.size(); i++)
    if (joint_types_[i] == KDL::BasicJointType::Continuous)
      artificial_lower_limits[i] = best_x_[i] - 2 * M_PI;
    else if (joint_types_[i] == KDL::BasicJointType::TransJoint)
      artificial_lower_limits[i] = l_bounds_[i];
    else
      artificial_lower_limits[i] = std::max(l_bounds_[i], best_x_[i] - 2 * M_PI);

  opt_.set_lower_bounds(artificial_lower_limits);

  std::vector<double> artificial_upper_limits(l_bounds_.size());

  for (uint i = 0; i < u_bounds_.size(); i++)
    if (joint_types_[i] == KDL::BasicJointType::Continuous)
      artificial_upper_limits[i] = best_x_[i] + 2 * M_PI;
    else if (joint_types_[i] == KDL::BasicJointType::TransJoint)
      artificial_upper_limits[i] = u_bounds_[i];
    else
      artificial_upper_limits[i] = std::min(u_bounds_[i], best_x_[i] + 2 * M_PI);

  opt_.set_upper_bounds(artificial_upper_limits);

  if (q_desired.data.size() == 0)
  {
    des_ = x;
  }
  else
  {
    des_.resize(x.size());
    for (uint i = 0; i < des_.size(); i++){
      des_[i] = q_desired(i);
    }
  }

  try
  {
    opt_.optimize(x, minf);
  }
  catch (...)
  {
  }

  if (progress_ == -1) // Got NaNs
    progress_ = -3;


  if (!aborted_ && progress_ < 0)
  {

    double time_left;
    diff = boost::posix_time::microsec_clock::local_time() - start_time;
    time_left = max_time_ - diff.total_nanoseconds() / 1000000000.0;

    while (time_left > 0 && !aborted_ && progress_ < 0)
    {

      for (uint i = 0; i < x.size(); i++)
        x[i] = fRand(artificial_lower_limits[i], artificial_upper_limits[i]);

      opt_.set_maxtime(time_left);

      try
      {
        opt_.optimize(x, minf);
      }
      catch (...) {}

      if (progress_ == -1) // Got NaNs
        progress_ = -3;

      diff = boost::posix_time::microsec_clock::local_time() - start_time;
      time_left = max_time_ - diff.total_nanoseconds() / 1000000000.0;
    }
  }


  for (uint i = 0; i < x.size(); i++)
  {
    q_out(i) = best_x_[i];
  }

  return progress_;

}

