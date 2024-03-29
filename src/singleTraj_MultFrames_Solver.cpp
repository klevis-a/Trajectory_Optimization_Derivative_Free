#include <iostream>
#include <regex>

#include <boost/filesystem.hpp>

#include "PseudoInv_Solver.h"
#include "csv.h"
#include "InputParser.h"


using std::vector;
using boost::filesystem::path;


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

    //read in seeds
    io::CSVReader<6> inputS(inputParser.seedsFile());
    vector<vector<double>> seeds;
    double j1s,j2s,j3s,j4s,j5s,j6s;
    while(inputS.read_row(j1s,j2s,j3s,j4s,j5s,j6s))
    {
        vector<double> currentSeed{j1s,j2s,j3s,j4s,j5s,j6s};
        seeds.push_back(currentSeed);
    }

    int rowNum=1;
    while(input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33))
    {
        auto newJointsFileName = std::regex_replace(framesFilePath, std::regex("smoothFrames"), "jointsTF" + std::to_string(rowNum));
        auto newCartesianFileName = std::regex_replace(framesFilePath, std::regex("smoothFrames"), "cartTF" + std::to_string(rowNum));
        auto jointsFilePath = outputDir / path(newJointsFileName);
        auto cartesianFilePath = outputDir / path(newCartesianFileName);

        Eigen::MatrixXd currentFrame(4,4);
        currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
        PseudoInv_Solver solver(configParser, inputParser.trajectoryFile(), seeds, currentFrame);
        solver.solve();
        solver.printResults(jointsFilePath.string());
        rowNum++;
    }

    return 0;
}
