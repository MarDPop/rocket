#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace util {

    std::vector< std::string > split( const std::string& line) {
        std::vector< std::string > out;
        int i = 0;
        const int nChar = line.size();
        while(i < nChar){
            while(i < nChar && line[i] == ' '){
                i++;
            }
            int i_last = i;
            while(i < nChar && line[i] != ' '){
                i++;
            }
            out.push_back(line.substr(i_last,i));
        }
        return out;
    }

    unsigned int lowerbound(const double* x, const double& val, unsigned int hi){
        unsigned int lo = 0;
        unsigned int mid = (lo + hi)/2;
        while(lo != mid){
            if(x[mid] > val){
                lo = mid;
            } else {
                hi = mid;
            }
            mid = (lo + hi)/2;
        }
        return mid;
    }

    void print_table(std::string fn, std::vector<double> x, std::vector< std::vector< double > > data) {
       std::ofstream file(fn);
       int nData = x.size();
       for(int i = 0; i < nData;i++){
            file << x[i];
            for(int j = 0; j < data[i].size();j++){
                file << " " << data[i][j];
            }
            file << std::endl;
       }
       file.close();
    }

    void print_table(std::string fn, const std::vector<double>& x, const std::vector< double >& data) {
       std::ofstream file(fn);
       int nData = x.size();
       for(int i = 0; i < nData;i++){
            file << x[i] << " " << data[i] << std::endl;
       }
       file.close();
    }

}