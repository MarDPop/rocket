#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <cmath>

namespace util {

    inline std::vector< std::string > split( const std::string& line) {
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

    inline std::string get_extension(std::string fn)
    {
        if(fn.size() < 2)
        {
            return fn;
        }
        std::string ext;
        unsigned idx = fn.size()-1;
        while(idx-- != 0 && fn[idx] != '.') {}
        while(++idx != fn.size()){
            ext.push_back(fn[idx]);
        }
        return ext;
    }

    constexpr double POW10[16] = {1.0,0.1,0.01,0.001,1e-4,1e-5,1e-6,1e-7,1e-8,1e-9,1e-10,1e-11,1e-12,1e-13,1e-14,1e-15};

    inline double fast_stod(const char* buffer, int start_idx, const int end_idx) {
      bool sign = buffer[start_idx] == '-';
      int decimal_place = 0;
      long val = 0;
      while(start_idx++ < end_idx) {
        if(buffer[start_idx] == '.') {
          decimal_place = start_idx;
          continue;
        }
        val = val*10 + (buffer[start_idx] - '0');
      }

      if(sign) {
        return -val*POW10[end_idx - decimal_place];
      } else {
        return val*POW10[end_idx - decimal_place];
      }
    }

    inline unsigned search(const double* x, const double key, unsigned length)
    {
        // perform exponential search first
        unsigned advance = 1;
        while(advance < length && x[advance] < key) { advance <<= 1; }
        advance >>= 1;
        x += advance;
        length -= advance;

        advance = 1;
        while(advance < length && x[length - advance] > key) { advance <<= 1; }
        advance >>= 1;
        length -= advance;

        advance = 0;
        unsigned mid = length >> 1;
        while(advance != mid)
        {
            if(key > x[mid])
            {
                advance = mid;
            }
            else
            {
                length = mid;
            }
            mid = (advance + length) >> 1;
        }
        return mid;
    }

    inline void print_table(std::string fn, std::vector<double> x, std::vector< std::vector< double > > data) {
       std::ofstream file(fn);
       int nData = x.size();
       for(int i = 0; i < nData;i++){
            file << x[i];
            for(unsigned int j = 0; j < data[i].size();j++){
                file << " " << data[i][j];
            }
            file << std::endl;
       }
       file.close();
    }

    inline void print_table(std::string fn, const std::vector<double>& x, const std::vector< double >& data) {
       std::ofstream file(fn);
       int nData = x.size();
       for(int i = 0; i < nData;i++){
            file << x[i] << " " << data[i] << std::endl;
       }
       file.close();
    }

    inline std::string to_string(const double* u, int n) {
        std::string out = "[";
        for(int i = 0; i < n;i++) {
            out += std::to_string(u[i]) + " ";
        }
        out += "]";
        return out;
    }



}
