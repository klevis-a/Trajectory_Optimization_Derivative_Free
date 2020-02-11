//
// Created by klevis on 12/7/17.
//

#include "ConfigParser.h"

using namespace std;
using namespace tinyxml2;


ConfigParser::ConfigParser(const char *fileName) : XMLParser(fileName) {
    readParameters();
}

const string &ConfigParser::urdfFile() const {
    return _urdfFile;
}

const vector<double> &ConfigParser::velLimits() const {
    return _velLimits;
}

double ConfigParser::tolerance() const {
    return _tolerance;
}

const string &ConfigParser::algorithm() const {
    return _algorithm;
}

double ConfigParser::rotTolerance() const {
    return _rottolerance;
}

double ConfigParser::xtol() const {
    return _xtol;
}

const string &ConfigParser::baseName() const {
    return _baseName;
}

const string &ConfigParser::eeName() const {
    return _eeName;
}

unsigned int ConfigParser::maxEval() const {
    return _maxEval;
}

const vector<double> &ConfigParser::lowerCorner() const {
    return _lowerCorner;
}

const vector<double> &ConfigParser::upperCorner() const {
    return _upperCorner;
}

void ConfigParser::readParameters() {
    //URDF file
    _urdfFile = string(parseText("URDF_File"));
    //velocity limits
    _velLimits = stringToVector(parseText("VelocityLimits"));
    //tolerance
    _tolerance = parseDouble("Tolerance");
    //rotational tolerance
    _rottolerance = parseDouble("RotTolerance");
    //algorithm
    _algorithm = parseText("Algorithm");
    //xtol
    _xtol = parseDouble("XTol");
    //base name
    _baseName = parseText("BaseName");
    //end effector name
    _eeName = parseText("EndEffectorName");
    //max evaluations
    _maxEval = parseUnsignedInt("MaxEval");
    //lower corner
    _lowerCorner = stringToVector(parseText("WorkspaceLowerCorner"));
    //upper corner
    _upperCorner = stringToVector(parseText("WorkspaceUpperCorner"));
}




