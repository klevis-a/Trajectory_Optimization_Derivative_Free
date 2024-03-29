//
// Created by klevis on 12/9/17.
//

#include <fstream>

#include "CsvWriter.h"

CsvWriter::CsvWriter(const std::vector<std::vector<double>> &matrix):_matrix(matrix) {
}

void CsvWriter::writeToFile(const std::string &file) const {
    std::ofstream csvFile;
    csvFile.open(file);

    for(auto row : _matrix)
    {
        for(auto cell=row.begin(); cell!=row.end(); ++cell)
        {
            csvFile << *cell;
            if(cell == (row.end()-1))
            {
                csvFile << std::endl;
            }
            else {
                csvFile << ",";
            }
        }
    }
}
