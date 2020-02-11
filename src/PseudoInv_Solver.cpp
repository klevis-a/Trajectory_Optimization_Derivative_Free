//
// Created by klevis on 1/12/18.
//

#include <ConfigParser.h>
#include <TrajData.hpp>
#include <Pagmo_traj_optimization.hpp>
#include <csv.h>
#include <CostComp.hpp>
#include "PseudoInv_Solver.hpp"

using namespace std;
using namespace pagmo;
using namespace Eigen;

PseudoInv_Solver::PseudoInv_Solver(const ConfigParser &configParser, const string &dataFile, const vector<vector<double>> &seeds, const MatrixXd &tf):_configParser(configParser)
{
    //cout << "Parameters File: " << parametersFile << endl;
    //cout << "Trajectory File: " << dataFile << endl;
    //cout << "Seeds File: " << seedsFile << endl;
    //cout << "Toolframe: " << endl;
    //cout << tf << endl;

    // initialize kdl class
    _robot=RobotKDL(configParser.urdfFile(),configParser.baseName(),configParser.eeName());

    //read trajectory data
    _data=TrajData(dataFile, tf);

    //create the optimization problem
    opt_problems::Robot_mocap_opt_problem optProblem(_data, _robot, configParser.lowerCorner(), configParser.upperCorner(), configParser.velLimits());
    problem prob{Pagmo_traj_optimization{optProblem}};

    //set the tolerances
    vector<double> tol(optProblem.m_nec+optProblem.m_nic, configParser.tolerance());
    //these are the equality constraints - the first two are constraints on rotation about X and Y for the starting
    //position, the third is a constraint on overall positional and orientation accuracy
    tol[0]=configParser.rotTolerance();
    tol[1]=configParser.rotTolerance();
    tol[2]=configParser.tolerance();
    prob.set_c_tol(tol);

    //create the solving algorithm
    nlopt nlopt{configParser.algorithm()};
    nlopt.set_maxeval(configParser.maxEval());
    nlopt.set_xtol_rel(configParser.xtol());
    nlopt.set_verbosity(10);
    _algo=algorithm{nlopt};
    _algo.set_verbosity(10);
    _pop=population{prob};

    //send the seeds to the algorithm
    for(auto seed : seeds)
    {
        vector<double> f_in=prob.fitness(seed);
        _pop.push_back(seed,f_in);
    }
}

void PseudoInv_Solver::printResults(const string &jointsFile) const {
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
    cerr<<"Final cost:"<< _pop_result.champion_f()[0]<<endl;
}
