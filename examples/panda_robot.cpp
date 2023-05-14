#include <trac_ik/trac_ik.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl/chainfksolver.hpp>
#include <random>

using namespace trac_ik;

auto getRandomConf(KDL::JntArray& q, std::vector<double> l_bounds, std::vector<double> u_bounds, size_t size)
{
    /* initialize random seed: */
    for (size_t idx = 0; idx < size; ++idx){
        std::uniform_real_distribution<double> gen(l_bounds[idx], u_bounds[idx]); 
        std::default_random_engine re(time(0));

        q.data[idx] = gen(re);
    }
}

auto computeErrorAverage(const auto& desired_end_effector_pose, const auto& current_ee_pose){
    auto avg = 0.0;

    for (size_t idx = 0; idx < 3; ++idx){
        avg += std::fabs(desired_end_effector_pose.p.data[idx] - current_ee_pose.p.data[idx]);
    }

    for (size_t idx = 0; idx < 9; ++idx){
        avg += std::fabs(desired_end_effector_pose.M.data[idx] - current_ee_pose.M.data[idx]);
    }

    return avg / 12.0;
}

auto getChain(auto& chain, const std::string& URDF_path, const std::string& base_link, const std::string& tip_link){

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
        return 0;
    }

    std::cout << "Reading joints and links from URDF" << std::endl;

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(*robot_model, tree)){
        std::cerr << "Failed to extract kdl tree from urdf" << std::endl;
        return 0;
    }

    if (!tree.getChain(base_link, tip_link, chain)){
        std::cerr << "Couldn't find chain " << base_link.c_str() << " to " << tip_link.c_str() << std::endl;
        return -1;
    }
    return 0;
}

int main(){

    bool verbose = false;

    const std::string base_link = "panda_link0";
    const std::string tip_link = "panda_link8"; 
    const std::string URDF_path = "../../urdf/panda.urdf";
    double maxtime = 0.05;
    double eps = 1e-4;
    SolveType solve_type = SolveType::Speed;

    std::cout << "Instantiating the Trac_IK and FK algorithms" << std::endl;
    KDL::Chain chain;
    getChain(chain, URDF_path, base_link, tip_link);    

    trac_ik::TRAC_IK ik_solver(base_link, tip_link, URDF_path, maxtime, eps, solve_type);

    std::cout << "--------TESTS--------" << std::endl;

    constexpr auto max_iters = 100;
    constexpr auto joint_num = 7;
    constexpr auto tollerance = 1e-3;
    KDL::Twist tolerances;
    auto ik_successes = 0;
    auto total_avg = 0.0;
    
    tolerances.rot.data[0] = tollerance;
    tolerances.rot.data[1] = tollerance;
    tolerances.rot.data[2] = tollerance;

    tolerances.vel.data[0] = tollerance;
    tolerances.vel.data[1] = tollerance;
    tolerances.vel.data[2] = tollerance;

    // Collecting data
    std::vector<KDL::JntArray> qs_seed;
    std::vector<KDL::JntArray> qs_desired;

    for (auto iter = 0; iter < max_iters; ++iter){

        KDL::JntArray q_seed(joint_num);
        KDL::JntArray q_desired(joint_num);
        std::vector<double> l_bounds = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        std::vector<double> u_bounds = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
        getRandomConf(q_seed, l_bounds, u_bounds, joint_num);
        getRandomConf(q_desired, l_bounds, u_bounds, joint_num);

        qs_seed.push_back(q_seed);
        qs_desired.push_back(q_desired);
    }
    
    auto start = std::chrono::steady_clock::now();
    for (auto iter = 0; iter < max_iters; ++iter){

        KDL::Frame desired_end_effector_pose;
        KDL::JntArray q_out(joint_num);

        KDL::ChainFkSolverPos_recursive fksolver(chain);

        fksolver.JntToCart(qs_desired[iter], desired_end_effector_pose, joint_num);

        int res = ik_solver.CartToJnt(qs_seed[iter], desired_end_effector_pose, q_out, tolerances);

        KDL::Frame current_ee_pose;

        fksolver.JntToCart(q_out, current_ee_pose, joint_num);
        
        if (res == 1){
            ik_successes++;
        }

        if (verbose){
            auto avg = computeErrorAverage(desired_end_effector_pose, current_ee_pose);
            std::cout << "IK result: " << res << std::endl;
            std::cout << "Error on average: " << avg << std::endl;
            
        }
    }
    auto end = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    std::cout << "Success: " << ik_successes << "/" << max_iters << "(" << static_cast<double>(ik_successes/max_iters * 100.0) << "%)" << std::endl;
    std::cout << "Total time: " << elapsed_seconds.count() << "s | Average time per call: " << static_cast<double>(elapsed_seconds.count() / max_iters) << std::endl;

    return 0;
}