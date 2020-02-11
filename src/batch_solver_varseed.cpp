//
// Created by klevis on 1/13/18.
//

#include <ConfigParser.h>
#include <PseudoInv_Solver.hpp>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <regex>
#include <csv.h>
#include "InputParser.h"

using namespace std;
using namespace boost::filesystem;
using namespace Eigen;

int main(int argc, char** argv)
{
    InputParser inputParser(argv[2]);

    io::CSVReader<16> input(inputParser.toolframesFile());
    double t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;
    input.read_row(t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33);
    MatrixXd currentFrame(4,4);
    currentFrame << t00,t01,t02,t03,t10,t11,t12,t13,t20,t21,t22,t23,t30,t31,t32,t33;

    ConfigParser configParser(argv[1]);

    path p(inputParser.trajectoryFolder());
    const boost::regex my_filter(".*smoothFrames\\.txt");
    for(auto i=directory_iterator(p); i!=directory_iterator(); i++)
    {
        if(is_directory(i->path()))
        {
            for(auto j=directory_iterator(i->path()); j!=directory_iterator(); j++)
            {
                // Skip if not a file
                if(!is_regular_file( j->status() ) ) continue;

                boost::smatch what;
                //skip if the file does not match the filter
                if(!boost::regex_match( j->path().string(), what, my_filter ) ) continue;

                auto filePath = j->path();
                auto dir = filePath.parent_path();
                auto fileName = filePath.filename().string();
                auto jointsFile = regex_replace(fileName, regex("smoothFrames"), "joints");
                auto jointsFilePath = dir / path(jointsFile);
                auto seedsFile = regex_replace(fileName, regex("smoothFrames"), "seedJoint");
                auto seedsFilePath = dir / path(seedsFile);

                //read in seeds
                vector<vector<double>> seeds;
                io::CSVReader<6> inputS(seedsFilePath.string());
                double j1s,j2s,j3s,j4s,j5s,j6s;
                inputS.read_row(j1s,j2s,j3s,j4s,j5s,j6s);
                vector<double> seed{j1s,j2s,j3s,j4s,j5s,j6s};
                seeds.push_back(seed);


                cout << "Processing file: " << filePath.string() << endl;
                cout << "Joints file: " << jointsFilePath.string() << endl;
                cout << " .... " << endl;

                PseudoInv_Solver solver(configParser, filePath.string(), seeds, currentFrame);
                solver.solve();
                solver.printResults(jointsFilePath.string());
            }
        }
    }
}