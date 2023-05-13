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


#ifndef TRAC_IK_HPP
#define TRAC_IK_HPP

#include <trac_ik/nlopt_ik.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <thread>
#include <mutex>
#include <memory>
#include <boost/date_time.hpp>

namespace trac_ik
{

enum class SolveType { Speed, Distance, Manip1, Manip2 };

class TRAC_IK
{
public:
  TRAC_IK(const KDL::Chain& chain, const KDL::JntArray& q_min, const KDL::JntArray& q_max, double maxtime = 0.005, double eps = 1e-5, SolveType type = SolveType::Speed);

  TRAC_IK(const std::string& base_link, const std::string& tip_link, const std::string& URDF_path, double maxtime = 0.005, double eps = 1e-5, SolveType type = SolveType::Speed);

  ~TRAC_IK();

  bool getKDLChain(KDL::Chain& chain)
  {
    chain = chain_;
    return initialized_;
  }

  bool getKDLLimits(KDL::JntArray& l_bounds, KDL::JntArray& u_bounds)
  {
    l_bounds = l_bounds_;
    u_bounds = u_bounds_;

    return initialized_;
  }

  // Requires a previous call to CartToJnt()
  bool getSolutions(std::vector<KDL::JntArray>& solutions)
  {
    solutions = solutions_;
    return initialized_ && !solutions.empty();
  }

  bool getSolutions(std::vector<KDL::JntArray>& solutions, std::vector<std::pair<double, uint> >& errors)
  {
    errors = errors_;
    return getSolutions(solutions);
  }

  bool setKDLLimits(KDL::JntArray& l_bounds, KDL::JntArray& u_bounds)
  {
    l_bounds = l_bounds_;
    u_bounds = u_bounds_;
    // TODO check this!!!
    nl_solverUPtr_.reset(new trac_ik::NLOPT_IK(chain_, l_bounds_, u_bounds_, max_time_, eps_, OptType::SumSq));
    iksolverUPtr_.reset(new KDL::ChainIkSolverPos_TL(chain_, l_bounds_, u_bounds_, max_time_, eps_, true, true));
    return true;
  }

  static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2)
  {
    double err = 0;
    for (uint i = 0; i < arr1.data.size(); i++)
    {
      err += pow(arr1(i) - arr2(i), 2);
    }

    return err;
  }

  int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds = KDL::Twist::Zero());

  inline void SetSolveType(SolveType type)
  {
    solve_type_ = type;
  }

private:
  // Private Variables
  bool initialized_{false};
  KDL::Chain chain_;
  KDL::JntArray l_bounds_, u_bounds_;
  double eps_;
  double max_time_;
  SolveType solve_type_;

  std::unique_ptr<KDL::ChainJntToJacSolver> jacsolverUPtr_{nullptr};
  std::unique_ptr<trac_ik::NLOPT_IK> nl_solverUPtr_;
  std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolverUPtr_;

  boost::posix_time::ptime start_time_;

  std::mutex mtx_;
  std::vector<KDL::JntArray> solutions_;
  std::vector<std::pair<double, uint> >  errors_;

  std::thread task1_, task2_;
  KDL::Twist bounds_;
  std::vector<KDL::BasicJointType> types_;

  // TODO: Refactor Private Functions

  template<typename T1, typename T2>
  bool runSolver(T1& solver, T2& other_solver,
                 const KDL::JntArray &q_init,
                 const KDL::Frame &p_in);

  bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in);
  bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in);

  void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
  void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

  bool unique_solution(const KDL::JntArray& sol);

  inline static double fRand(double min, double max)
  {
    double f = (double)rand() / RAND_MAX;
    return min + f * (max - min);
  }

  /* @brief Manipulation metrics and penalties taken from "Workspace
  Geometric Characterization and Manipulability of Industrial Robots",
  Ming-June, Tsia, PhD Thesis, Ohio State University, 1986.
  https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
  */
  double manipPenalty(const KDL::JntArray&);
  double ManipValue1(const KDL::JntArray&);
  double ManipValue2(const KDL::JntArray&);

  inline bool myEqual(const KDL::JntArray& a, const KDL::JntArray& b)
  {
    return (a.data - b.data).isZero(1e-4);
  }

  void initialize();

};

inline bool TRAC_IK::runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*iksolverUPtr_.get(), *nl_solverUPtr_.get(), q_init, p_in);
}

inline bool TRAC_IK::runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in)
{
  return runSolver(*nl_solverUPtr_.get(), *iksolverUPtr_.get(), q_init, p_in);
}

}

#endif
