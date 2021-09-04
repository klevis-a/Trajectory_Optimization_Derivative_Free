#include <iostream>
#include <regex>

#include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/filesystem.hpp>

#include "PseudoInv_Solver.h"
#include "InputParser.h"
#include "csv.h"

using std::cout;
using std::endl;
using std::vector;
using boost::filesystem::path;


int main(int argc, char** argv)
{
    ConfigParser configParser(argv[1]);
    InputParser inputParser(argv[2]);
    path p(inputParser.trajectoryFolder());
    path outputDir(inputParser.outputDir());
    const boost::regex sfFilter(".*smoothFrames\\.txt");

    //read in seeds
    io::CSVReader<6> inputS(inputParser.seedsFile());
    vector<vector<double>> seeds;
    double j1s,j2s,j3s,j4s,j5s,j6s;
    while(inputS.read_row(j1s,j2s,j3s,j4s,j5s,j6s))
    {
        vector<double> currentSeed{j1s,j2s,j3s,j4s,j5s,j6s};
        seeds.push_back(currentSeed);
    }

    for(auto i=boost::filesystem::directory_iterator(p); i!=boost::filesystem::directory_iterator(); i++)
    {
        // Skip if not a file
        if(!boost::filesystem::is_regular_file(i->status())) continue;

        boost::smatch match;
        //skip if the file does not match the filter
        if(!boost::regex_match( i->path().string(), match, sfFilter)) continue;

        auto filePath = i->path();
        auto dir = filePath.parent_path();
        auto fileName = filePath.filename().string();

        //read in the trajectory
        io::CSVReader<16> input(inputParser.toolframesFile());
        double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

        int rowNum=1;

        while(input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33))
        {
            auto jointsFile = std::regex_replace(fileName, std::regex("smoothFrames"), "jointsTF" + std::to_string(rowNum));
            auto cartesianFile = std::regex_replace(fileName, std::regex("smoothFrames"), "cartesianTF" + std::to_string(rowNum));
            auto jointsFilePath = outputDir / path(jointsFile);
            auto cartesianFilePath = outputDir / path(cartesianFile);

            cout << "Processing file: " << fileName << endl;
            cout << "Joints file: " << jointsFilePath.string() << endl;
            cout << "Cartesian file: " << cartesianFilePath.string() << endl;
            cout << " .... " << endl;

            Eigen::MatrixXd currentFrame(4,4);
            currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

            PseudoInv_Solver solver(configParser, filePath.string(), seeds, currentFrame);
            solver.solve();
            solver.printResults(jointsFilePath.string());

            rowNum++;
        }
    }
}
