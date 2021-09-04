//
// Created by klevis on 1/12/18.
//

#include "ConfigParser.h"
#include "TrajData.h"
#include "Pagmo_traj_optimization.hpp"
#include "csv.h"
#include "CostComp.h"
#include "PseudoInv_Solver.h"

using std::vector;

PseudoInv_Solver::PseudoInv_Solver(const ConfigParser &configParser, const std::string &dataFile, const vector<vector<double>> &seeds, const Eigen::MatrixXd &tf):_configParser(configParser)
{
    //std::cout << "Parameters File: " << parametersFile << std::endl;
    //std::cout << "Trajectory File: " << dataFile << std::endl;
    //std::cout << "Seeds File: " << seedsFile << std::endl;
    //std::cout << "Toolframe: " << std::endl;
    //std::cout << tf << std::endl;

    // initialize kdl class
    _robot=RobotKDL(configParser.urdfFile(),configParser.baseName(),configParser.eeName());

    //read trajectory data
    _data=TrajData(dataFile, tf);

    //create the optimization problem
    Robot_mocap_opt_problem optProblem(_data, _robot, configParser.lowerCorner(), configParser.upperCorner(), configParser.velLimits());
    pagmo::problem prob{pagmo::Pagmo_traj_optimization{optProblem}};

    //set the tolerances
    vector<double> tol(optProblem.m_nec+optProblem.m_nic, configParser.tolerance());
    //these are the equality constraints - the first two are constraints on rotation about X and Y for the starting
    //position, the third is a constraint on overall positional and orientation accuracy
    tol[0]=configParser.rotTolerance();
    tol[1]=configParser.rotTolerance();
    tol[2]=configParser.tolerance();
    prob.set_c_tol(tol);

    //create the solving algorithm
    pagmo::nlopt nlopt{configParser.algorithm()};
    nlopt.set_maxeval(configParser.maxEval());
    nlopt.set_xtol_rel(configParser.xtol());
    nlopt.set_verbosity(10);
    _algo=pagmo::algorithm{nlopt};
    _algo.set_verbosity(10);
    _pop=pagmo::population{prob};

    //send the seeds to the algorithm
    for(auto seed : seeds)
    {
        vector<double> f_in=prob.fitness(seed);
        _pop.push_back(seed,f_in);
    }
}

void PseudoInv_Solver::printResults(const std::string &jointsFile) const {
    //get the best set of joint angles
    vector<double> x_best=_pop_result.champion_x();

    //from the initial joint angles, compute the rest of the joint angles
    CostComp finalSol(_data,_robot,_configParser.lowerCorner(),_configParser.upperCorner(),_configParser.velLimits());
    finalSol.computeDerived(x_best);

    //write out the joint space solution
    finalSol.writeTheta(jointsFile);
}

void PseudoInv_Solver::solve() {
    //solve the problem
    _pop_result = _algo.evolve(_pop);

    //print out the final cost
    std::cout<<"Final cost:"<< _pop_result.champion_f()[0]<<std::endl;
}
