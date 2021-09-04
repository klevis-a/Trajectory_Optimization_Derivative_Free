//
// Created by klevis on 4/18/18.
//

#include "InputParser.h"

using std::string;

InputParser::InputParser(const char *fileName) : XMLParser(fileName)  {
    readParameters();
}

void InputParser::readParameters() {
    //desired trajectory
    _trajectoryFile = parseText("TrajectoryFile");
    //joint combos file
    _seedsFile = parseText("SeedsFile");
    //toolframes file
    _toolframesFile = parseText("Toolframes");
    //trajectory folder
    _trajectoryFolder = parseText("TrajectoryFolder");
    //output directory
    _outputDir = parseText("OutputDirectory");
}

const string &InputParser::trajectoryFile() const {
    return _trajectoryFile;
}

const string &InputParser::seedsFile() const {
    return _seedsFile;
}

const string &InputParser::toolframesFile() const {
    return _toolframesFile;
}

const string &InputParser::trajectoryFolder() const {
    return _trajectoryFolder;
}

const string &InputParser::outputDir() const {
    return _outputDir;
}
