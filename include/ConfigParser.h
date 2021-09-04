//
// Created by klevis on 12/7/17.
//

#ifndef M20IA_PINV_OPT_CONFIGPARSER_H
#define M20IA_PINV_OPT_CONFIGPARSER_H

#include <string>
#include "XMLParser.h"

class ConfigParser : XMLParser {
public:
    ConfigParser(const char* fileName);
    const std::string &urdfFile() const;
    const std::vector<double> &velLimits() const;
    double tolerance() const;
    double rotTolerance() const;
    double xtol() const;
    unsigned int maxEval() const;
    const std::string &algorithm() const;
    const std::string &baseName() const;
    const std::string &eeName() const;
    const std::vector<double> &lowerCorner() const;
    const std::vector<double> &upperCorner() const;

private:
    //extracted parameters
    std::string _urdfFile;
    std::string _baseName;
    std::string _eeName;
    std::vector<double> _velLimits;
    double _tolerance;
    double _rottolerance;
    std::string _algorithm;
    double _xtol;
    unsigned int _maxEval;
    std::vector<double> _lowerCorner;
    std::vector<double> _upperCorner;

private:
    void readParameters();
};


#endif //M20IA_PINV_OPT_CONFIGPARSER_H
