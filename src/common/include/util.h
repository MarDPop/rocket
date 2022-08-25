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

    inline unsigned int bisection_search(const double* x, double val, unsigned int hi){
        unsigned int lo = 0;
        unsigned int mid = (lo + hi) >> 1;
        while(lo != mid){
            if(val > x[mid]){
                lo = mid;
            } else {
                hi = mid;
            }
            mid = (lo + hi) >> 1;
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

    inline double dot3(const double* u, const double* v) {
        return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    }

    inline void rotation_between(const double* u, const double* v, float* q) {
        //https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
        q[1] = u[1]*v[2] - u[2]*v[1];
        q[2] = u[2]*v[0] - u[0]*v[2];
        q[3] = u[0]*v[1] - u[1]*v[0];
        q[0] = 1 + dot3(u,v);

        double recipNorm = 1.0 / sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

    inline void q2rotm(const double* q, double* rotm) {
        double qi2 = q[1]*2;
        double qj2 = q[2]*2;
        double qk2 = q[3]*2;
        rotm[0] = 1 - (qj2*q[2] + qk2*q[3]);
        rotm[4] = 1 - (qi2*q[1] + qk2*q[3]);
        rotm[8] = 1 - (qi2*q[1] + qj2*q[2]);
        rotm[1] = qi2*q[2] - qk2*q[0];
        rotm[2] = qi2*q[3] + qj2*q[0];
        rotm[3] = qi2*q[2] + qk2*q[0];
        rotm[5] = qj2*q[3] - qi2*q[0];
        rotm[6] = qi2*q[3] - qj2*q[0];
        rotm[7] = qj2*q[3] + qi2*q[0];
    }


    inline void rotm2q( const double* rotm, double* q ) {
        double s = rotm[0] + rotm[4] + rotm[8];
        if( s > 0 ) {
            s = 0.5 / sqrt(s + 1.0);
            q[0] = 0.25 / s;
            q[1] = ( rotm[7] - rotm[5] ) * s;
            q[2] = ( rotm[2] - rotm[6] ) * s;
            q[3] = ( rotm[3] - rotm[1] ) * s;
        } else {
            if ( rotm[0] > rotm[4] && rotm[0] > rotm[8] ) {
              s = 0.5 / sqrt( 1.0f + rotm[0] - rotm[4] - rotm[8]);
              q[0] = (rotm[7] - rotm[5] ) * s;
              q[1] = 0.25 / s;
              q[2] = (rotm[1] + rotm[3] ) * s;
              q[3] = (rotm[2] + rotm[6] ) * s;
            } else if (rotm[4] > rotm[8]) {
              s = 0.5f / sqrt( 1.0f + rotm[4] - rotm[0] - rotm[8]);
              q[0] = (rotm[2] - rotm[6] ) * s;
              q[1] = (rotm[1] + rotm[3] ) * s;
              q[2] = 0.25 / s;
              q[3] = (rotm[5] + rotm[7] ) * s;
            } else {
              s = 0.5 / sqrt( 1.0f + rotm[8] - rotm[0] - rotm[4] );
              q[0] = (rotm[3] - rotm[1] ) * s;
              q[1] = (rotm[2] + rotm[6] ) * s;
              q[2] = (rotm[5] + rotm[7] ) * s;
              q[3] = 0.25 / s;
            }
        }
    }

}
