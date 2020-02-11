#include <iostream>
#include <PseudoInv_Solver.hpp>
#include <csv.h>
#include "InputParser.h"
#include <boost/filesystem.hpp>
#include <regex>

using namespace std;
using namespace Eigen;
using namespace boost::filesystem;

int main(int argc, char** argv)
{
    ConfigParser configParser(argv[1]);
    InputParser inputParser(argv[2]);
    path framesPath(inputParser.trajectoryFile());
    auto framesDir = framesPath.parent_path();
    path outputDir(inputParser.outputDir());
    auto framesFilePath = framesPath.filename().string();

    //read in the trajectory
    io::CSVReader<16> input(inputParser.toolframesFile());
    double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
    input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33);
    MatrixXd currentFrame(4,4);
    currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

    //read in seeds
    io::CSVReader<6> inputS(inputParser.seedsFile());
    vector<vector<double>> seeds;
    double j1s,j2s,j3s,j4s,j5s,j6s;
    while(inputS.read_row(j1s,j2s,j3s,j4s,j5s,j6s))
    {
        vector<double> currentSeed{j1s,j2s,j3s,j4s,j5s,j6s};
        seeds.push_back(currentSeed);
    }

    for(int n=0; n<seeds.size(); n++) {
        vector<vector<double>> passedSeeds;
        passedSeeds.push_back(seeds[n]);
        auto newJointsFileName = regex_replace(framesFilePath, regex("smoothFrames"), "jointsS" + to_string(n + 1));
        auto jointsFilePath = outputDir / path(newJointsFileName);

        PseudoInv_Solver solver(configParser, inputParser.trajectoryFile(), passedSeeds, currentFrame);
        solver.solve();
        solver.printResults(jointsFilePath.string());
    }

	return 0;
}
