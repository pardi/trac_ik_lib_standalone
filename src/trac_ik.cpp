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


#include <trac_ik/trac_ik.hpp>
#include <boost/date_time.hpp>
#include <Eigen/Geometry>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

using namespace trac_ik;

TRAC_IK::TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_path, const double maxtime, const double eps, SolveType type) :
  eps_(eps),
  max_time_(maxtime),
  solve_type_(type)
{

  // READ URDF from path
	std::ifstream urdf_file(URDF_path); //taking file as inputstream
	std::string urdf_str;

	urdf::ModelInterfaceSharedPtr robot_model; 

	if(urdf_file) {
		std::ostringstream ss;

		ss << urdf_file.rdbuf(); 

		urdf_str = ss.str();

		robot_model = urdf::parseURDF(urdf_str);
	}
	else{
		std::cerr << "The URDF file does not exist!!!" << std::endl;
    return;
	}

  std::cout << "Reading joints and links from URDF" << std::endl;

  KDL::Tree tree;

  if (!kdl_parser::treeFromUrdfModel(*robot_model, tree)){
    std::cerr << "Failed to extract kdl tree from xml robot description" << std::endl;
    return;
  }
    
  if (!tree.getChain(base_link, tip_link, chain_)){
      std::cerr << "Couldn't find chain " << base_link.c_str() << " to " << tip_link.c_str() << std::endl;
      return;
  }

  std::vector<KDL::Segment> chain_segs = chain_.segments;

  urdf::JointConstSharedPtr jointSPrt;

  std::vector<double> l_bounds, u_bounds;

  l_bounds_.resize(chain_.getNrOfJoints());
  u_bounds_.resize(chain_.getNrOfJoints());

  size_t joint_num = 0;

  for (unsigned int i = 0; i < chain_segs.size(); ++i)
  {
    auto joint = robot_model->getJoint(chain_segs[i].getJoint().getName());
    if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED)
    {
      joint_num++;
      float lower, upper;
      int hasLimits;
      if (joint->type != urdf::Joint::CONTINUOUS)
      {
        if (joint->safety)
        {
          lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
          upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
        }
        else
        {
          lower = joint->limits->lower;
          upper = joint->limits->upper;
        }
        hasLimits = 1;
      }
      else
      {
        hasLimits = 0;
      }
      if (hasLimits)
      {
        l_bounds_(joint_num - 1) = lower;
        u_bounds_(joint_num - 1) = upper;
      }
      else
      {
        l_bounds_(joint_num - 1) = std::numeric_limits<float>::lowest();
        u_bounds_(joint_num - 1) = std::numeric_limits<float>::max();
      }

      std::cout << "IK Using joint " << joint->name << " " << l_bounds_(joint_num - 1) << " " << u_bounds_(joint_num - 1) << std::endl;
    }
  }

  initialize();
}


TRAC_IK::TRAC_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double maxtime, double eps, SolveType type):
  chain_(chain),
  l_bounds_(q_min),
  u_bounds_(q_max),
  eps_(eps),
  max_time_(maxtime),
  solve_type_(type)
{
  initialize();
}

void TRAC_IK::initialize()
{

  assert(chain_.getNrOfJoints() == l_bounds_.data.size());
  assert(chain_.getNrOfJoints() == u_bounds_.data.size());

  jacsolverUPtr_.reset(new KDL::ChainJntToJacSolver(chain_));
  nl_solverUPtr_.reset(new trac_ik::NLOPT_IK(chain_, l_bounds_, u_bounds_, max_time_, eps_, trac_ik::OptType::SumSq));
  iksolverUPtr_.reset(new KDL::ChainIkSolverPos_TL(chain_, l_bounds_, u_bounds_, max_time_, eps_, true, true));

  for (uint i = 0; i < chain_.segments.size(); i++)
  {
    std::string type = chain_.segments[i].getJoint().getTypeName();
    if (type.find("Rot") != std::string::npos)
    {
      if (u_bounds_(types_.size()) >= std::numeric_limits<float>::max() &&
          l_bounds_(types_.size()) <= std::numeric_limits<float>::lowest())
        types_.push_back(KDL::BasicJointType::Continuous);
      else
        types_.push_back(KDL::BasicJointType::RotJoint);
    }
    else if (type.find("Trans") != std::string::npos)
      types_.push_back(KDL::BasicJointType::TransJoint);
  }

  assert(types_.size() == l_bounds_.data.size());

  initialized_ = true;
}

bool TRAC_IK::unique_solution(const KDL::JntArray& sol)
{

  for (uint i = 0; i < solutions_.size(); i++)
    if (myEqual(sol, solutions_[i]))
      return false;
  return true;

}

inline void normalizeAngle(double& val, const double& min, const double& max)
{
  if (val > max)
  {
    //Find actual angle offset
    double diffangle = fmod(val - max, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = max + diffangle - 2 * M_PI;
  }

  if (val < min)
  {
    //Find actual angle offset
    double diffangle = fmod(min - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = min - diffangle + 2 * M_PI;
  }
}

inline void normalizeAngle(double& val, const double& target)
{
  double new_target = target + M_PI;
  if (val > new_target)
  {
    //Find actual angle offset
    double diffangle = fmod(val - new_target, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = new_target + diffangle - 2 * M_PI;
  }

  new_target = target - M_PI;
  if (val < new_target)
  {
    //Find actual angle offset
    double diffangle = fmod(new_target - val, 2 * M_PI);
    // Add that to upper bound and go back a full rotation
    val = new_target - diffangle + 2 * M_PI;
  }
}


template<typename T1, typename T2>
bool TRAC_IK::runSolver(T1& solver, T2& other_solver,
                        const KDL::JntArray &q_init,
                        const KDL::Frame &p_in)
{
  KDL::JntArray q_out;

  double fulltime = max_time_;
  KDL::JntArray seed = q_init;

  boost::posix_time::time_duration timediff;
  double time_left;

  while (true)
  {
    timediff = boost::posix_time::microsec_clock::local_time() - start_time_;
    time_left = fulltime - timediff.total_nanoseconds() / 1000000000.0;

    if (time_left <= 0)
      break;

    solver.setMaxtime(time_left);

    int RC = solver.CartToJnt(seed, p_in, q_out, bounds_);
    if (RC >= 0)
    {
      switch (solve_type_)
      {
      case SolveType::Manip1:
      case SolveType::Manip2:
        normalize_limits(q_init, q_out);
        break;
      default:
        normalize_seed(q_init, q_out);
        break;
      }
      mtx_.lock();
      if (unique_solution(q_out))
      {
        solutions_.push_back(q_out);
        uint curr_size = solutions_.size();
        errors_.resize(curr_size);
        mtx_.unlock();
        double err, penalty;
        switch (solve_type_)
        {
        case SolveType::Manip1:
          penalty = manipPenalty(q_out);
          err = penalty * TRAC_IK::ManipValue1(q_out);
          break;
        case SolveType::Manip2:
          penalty = manipPenalty(q_out);
          err = penalty * TRAC_IK::ManipValue2(q_out);
          break;
        default:
          err = TRAC_IK::JointErr(q_init, q_out);
          break;
        }
        mtx_.lock();
        errors_[curr_size - 1] = std::make_pair(err, curr_size - 1);
      }
      mtx_.unlock();
    }

    if (!solutions_.empty() && solve_type_ == SolveType::Speed)
      break;

    for (unsigned int j = 0; j < seed.data.size(); j++)
      if (types_[j] == KDL::BasicJointType::Continuous)
        seed(j) = fRand(q_init(j) - 2 * M_PI, q_init(j) + 2 * M_PI);
      else
        seed(j) = fRand(l_bounds_(j), u_bounds_(j));
  }
  other_solver.abort();

  solver.setMaxtime(fulltime);

  return true;
}


void TRAC_IK::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution)
{
  // Make sure rotational joint values are within 1 revolution of seed; then
  // ensure joint limits are met.

  bool improved = false;

  for (uint i = 0; i < l_bounds_.data.size(); i++)
  {

    if (types_[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle(val, target);

    if (types_[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, l_bounds_(i), u_bounds_(i));

    solution(i) = val;
  }
}

void TRAC_IK::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution)
{
  // Make sure rotational joint values are within 1 revolution of middle of
  // limits; then ensure joint limits are met.

  bool improved = false;

  for (uint i = 0; i < l_bounds_.data.size(); i++)
  {

    if (types_[i] == KDL::BasicJointType::TransJoint)
      continue;

    double target = seed(i);

    if (types_[i] == KDL::BasicJointType::RotJoint && types_[i] != KDL::BasicJointType::Continuous)
      target = (u_bounds_(i) + l_bounds_(i)) / 2.0;

    double val = solution(i);

    normalizeAngle(val, target);

    if (types_[i] == KDL::BasicJointType::Continuous)
    {
      solution(i) = val;
      continue;
    }

    normalizeAngle(val, l_bounds_(i), u_bounds_(i));

    solution(i) = val;
  }

}


double TRAC_IK::manipPenalty(const KDL::JntArray& arr)
{
  double penalty = 1.0;
  for (uint i = 0; i < arr.data.size(); i++)
  {
    if (types_[i] == KDL::BasicJointType::Continuous)
      continue;
    double range = u_bounds_(i) - l_bounds_(i);
    penalty *= ((arr(i) - l_bounds_(i)) * (u_bounds_(i) - arr(i)) / (range * range));
  }
  return std::max(0.0, 1.0 - exp(-1 * penalty));
}


double TRAC_IK::ManipValue1(const KDL::JntArray& arr)
{
  KDL::Jacobian jac(arr.data.size());

  jacsolverUPtr_->JntToJac(arr, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  double error = 1.0;
  for (unsigned int i = 0; i < singular_values.rows(); ++i)
    error *= singular_values(i, 0);
  return error;
}

double TRAC_IK::ManipValue2(const KDL::JntArray& arr)
{
  KDL::Jacobian jac(arr.data.size());

  jacsolverUPtr_->JntToJac(arr, jac);

  Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
  Eigen::MatrixXd singular_values = svdsolver.singularValues();

  return singular_values.minCoeff() / singular_values.maxCoeff();
}


int TRAC_IK::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds)
{

  if (!initialized_)
  {
    std::cerr << "TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed" << std::endl;
    return -1;
  }


  start_time_ = boost::posix_time::microsec_clock::local_time();

  nl_solverUPtr_->reset();
  iksolverUPtr_->reset();

  solutions_.clear();
  errors_.clear();

  bounds_ = _bounds;

  task1_ = std::thread(&TRAC_IK::runKDL, this, q_init, p_in);
  task2_ = std::thread(&TRAC_IK::runNLOPT, this, q_init, p_in);

  task1_.join();
  task2_.join();

  if (solutions_.empty())
  {
    q_out = q_init;
    return -3;
  }

  switch (solve_type_)
  {
  case SolveType::Manip1:
  case SolveType::Manip2:
    std::sort(errors_.rbegin(), errors_.rend()); // rbegin/rend to sort by max
    break;
  default:
    std::sort(errors_.begin(), errors_.end());
    break;
  }

  q_out = solutions_[errors_[0].second];

  return solutions_.size();
}


TRAC_IK::~TRAC_IK()
{
  if (task1_.joinable())
    task1_.join();
  if (task2_.joinable())
    task2_.join();
}
