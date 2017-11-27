#include <iostream>
#include <TrajData.hpp>
#include <csv.h>
#include <trajectory_optimization.hpp>

using namespace std;
using namespace pagmo;

void printVector(vector<double> vec)
{
    for(int i=0; i<vec.size(); i++)
    {
        cout << vec[i] << " ";
    }

    cout << endl;
}

int main(int argc, char** argv)
{
    string dataFile = argv[1];
    string initialGuess = argv[2];
    string outFile = argv[3];
    string outFileJ = argv[4];
    double stopValue = stod(argv[5]);

	// urdf file location:
    string urdf_file = "/home/klevis/Desktop/URDF/m20iag.urdf";
    vector<string> ee_names={"tool0"};
    vector<string> base_names={"base"};
    vector<double> g_vec={0.0,0.0,-9.8};
    
	// initialize kdl class
    manipulator_kdl::robotKDL m20ia(urdf_file,base_names,ee_names,g_vec);
    m20ia.urdfParser();

    //read initial guess
    io::CSVReader<7> inputJ(initialGuess);
    double j1,j2,j3,j4,j5,j6;
    int timeIndex;
    inputJ.read_row(timeIndex,j1,j2,j3,j4,j5,j6);
    vector<double> firstGuess{j1,j2,j3,j4,j5,j6};

    TrajData data(dataFile, m20ia);

    opt_problems::optProblem optProblem(data);
    problem prob{trajectory_optimization{optProblem}};

    //set the tolerances
    vector<double> tol(optProblem.m_nic, 1e-3);
    prob.set_c_tol(tol);

    //start optimization
    nlopt nlopt{"cobyla"};
    nlopt.set_xtol_rel(0);
    nlopt.set_ftol_abs(0);
    nlopt.set_stopval(stopValue);
    algorithm algo{nlopt};
    population pop{prob};

    cout << "First guess" << endl;
    printVector(firstGuess);

    vector<double> f_in=prob.fitness(firstGuess);
    pop.push_back(firstGuess,f_in);
    population pop_res = algo.evolve(pop);

    cerr<<"Final cost:"<<pop_res.champion_f()[0]<<endl;

    vector<double> x_best=pop_res.champion_x();

    cerr<<"Best candidate: " << endl;
    printVector(x_best);

    cout << "Cartesian position: " << endl;
    printVector(m20ia.getFK(0,x_best,true));

    vector<double> zeros(6,0.0);
    cout << "Zero: " << endl;
    printVector(m20ia.getFK(0,zeros,true));

	return 0;
}
