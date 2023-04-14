#ifndef __MATH__H
#define __MATH__H

#include "string.h"
#include <array>
#include <vector>
#include <cmath>
#include <sstream>

#define TWOPI 6.283185307179586476925286766559
#define DEG2RAD 1.7453292519943295769236907685e-2

namespace Math {

	inline double SQ(const double& x) {
		return x*x;
	}

	inline double CUB(const double& x) {
		return x*x*x;
	}

	inline double dot(const std::array<double,3>& u, const std::array<double,3>& v) {
		return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
	}

    inline double norm(const std::array<double,3>& u) {
        return sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
    }

	inline std::array<double,3> cross(const std::array<double,3>& u, const std::array<double,3>& v) {
		std::array<double,3> w = {u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]};
		return w;
	}

    inline double distance(const std::array<double,3>& u, const std::array<double,3>& v) {
        double dx = u[0] - v[0];
        double dy = u[1] - v[1];
        double dz = u[2] - v[2];
		return sqrt(dx*dx + dy*dy + dz*dz);
	}

	inline std::array< std::array<double,3>, 3 > transpose(const std::array< std::array<double,3>, 3 >& A) {
		std::array< std::array<double,3>, 3 > AT;
		for(std::size_t i = 0;i < 3; i++){
			for(std::size_t j = 0;j < 3; j++){
				AT[j][i] = A[i][j];
			}
		}
		return AT;
	}

	inline std::array<double,3> mult(const std::array< std::array<double,3>, 3 >& A, const std::array<double,3>& x) {
		std::array<double,3> y;
		y[0] = dot(A[0],x);
		y[1] = dot(A[1],x);
		y[2] = dot(A[2],x);
		return y;
	}

	inline double** eye(const std::size_t&n) {
		std::size_t bytes = n*sizeof(double);
		double** I = new double*[n];
		for(std::size_t i = 0;i < n; i++){
			I[i] = new double[n];
			memset(I[i], 0, bytes);
			I[i][i] = 1;
		}
		return I;
	}

	inline double** zeros(const std::size_t&n) {
		std::size_t bytes = n*sizeof(double);
		double** A = new double*[n];
		for(std::size_t i = 0;i < n; i++){
			A[i] = new double[n];
			memset(A[i], 0, bytes);
		}
		return A;
	}

	inline double** copy(double** A,const std::size_t&n) {
		double** B = new double*[n];
		for(std::size_t i = 0;i < n; i++){
			double* a = A[i];
			double* b = new double[n];
			for(std::size_t j = 0;j < n; i++){
				b[j] = a[j];
			}
			B[i] = b;
		}
		return B;
	}

	inline void clear(double** A, const std::size_t& n) {
		std::size_t bytes = n*sizeof(double);
		for(std::size_t i = 0;i < n; i++){
			memset(A[i], 0, bytes);
		}
	}

	inline void del(double** A, const std::size_t& n){
		for(std::size_t i = 0;i < n; i++){
			delete[] A[i];
		}
		delete[] A;
	}

	inline void identity(double** A, const std::size_t& n){
		clear(A,n);
		for(std::size_t i = 0;i < n; i++){
			A[i][i] = 1;
		}
	}

	inline double dot(const double* u, const double* v, const std::size_t& N) {
		double sum = 0;
		for(std::size_t i = 0; i < N; i++){
			sum += u[i]*v[i];
		}
		return sum;
	}

	inline void mult(double** A, double** B, double** C, const std::size_t& n) {
		for (std::size_t i = 0; i < n; i++) {
			double* A_row = A[i];
			double* C_row = C[i];
            for (std::size_t j = 0; j < n; j++) {
                double s = 0;
				for (std::size_t k = 0; k < n; k++)
					s += A_row[k]*B[k][j];
                C_row[j] = s;
            }
        }
	}

	inline void multMat(double **A, double **B, double **C, const uint_fast16_t& n){
		clear(C,n);
        uint_fast16_t i,j,k;
        for (i = 0; i < n; i++) {
            double* a_row = A[i];
            double* c_row = C[i];
            for (k = 0; k < n; k++) { // inner dimension
                double a_val = a_row[k];
                double* b_row = B[k];
                for (j = 0; j < n; j++) { // hopefully -O3 vectorized
                    c_row[j] += a_val*b_row[j];
                }
            }
        }
    }

	inline void mult(double** A, double b, double** C, const std::size_t& n) {
		for (std::size_t i = 0; i < n; i++) {
			double* A_row = A[i];
			double* C_row = C[i];
            for (std::size_t j = 0; j < n; j++) {
                C_row[j] = A_row[j]*b;
            }
        }
	}

	inline void mult(double** A, double b, const std::size_t& n) {
		for (std::size_t i = 0; i < n; i++) {
			double* A_row = A[i];
            for (std::size_t j = 0; j < n; j++) {
                A_row[j] *= b;
            }
        }
	}

	inline void add(double** A, double** B, double** C, const std::size_t& n) {
		for (std::size_t i = 0; i < n; i++) {
			double* A_row = A[i];
			double* B_row = B[i];
			double* C_row = C[i];
            for (std::size_t j = 0; j < n; j++) {
                C_row[j] = A_row[j] + B_row[j];
            }
        }
	}

	inline void add(double** A, double** B, const std::size_t& n) {
		for (std::size_t i = 0; i < n; i++) {
			double* A_row = A[i];
			double* B_row = B[i];
            for (std::size_t j = 0; j < n; j++) {
                A_row[j] += B_row[j];
            }
        }
	}

	template<std::size_t N>
	inline std::array< std::array<double,N>, N> mult(const std::array< std::array<double,N>, N>& A, const std::array< std::array<double,N>, N>& B) {
		std::array< std::array<double,N>, N> C;
		for (std::size_t i = 0; i < N; i++) {
            for (std::size_t j = 0; j < N; j++) {
                double s = 0;
				for (std::size_t k = 0; k < N; k++)
					s += A[i][k]*B[k][j];
                C[i][j] = s;
            }
        }
		return C;
	}

	inline void mult(double* A, double* B, double* C, const std::size_t& n) {
		memset(C, 0, n*sizeof(double));
		for (std::size_t i = 0; i < n; i++) {
			std::size_t row = i*n;
            for (std::size_t k = 0; k < n; k++) {
				double a = A[row + k];
				std::size_t b_row = k*n;
				for (std::size_t j = 0; j < n; j++)
					C[row + j] += a*B[b_row + j];
            }
        }
	}

	inline void LUPSolve(double** A, double* b, const std::size_t& n) {
        std::size_t i, j, k, i_max;
        double max,absA;

        for (i = 0; i < n; i++) {
            max = 0;
            i_max = i;

            for (k = i; k < n; k++)
                if ((absA = fabs(A[k][i])) > max) {
                    max = absA;
                    i_max = k;
                }

            if (max < 1e-300){
                throw "degenerate Matrix";
            }

            if (i_max != i) {
                //pivoting A
                double *rowPtr = std::move(A[i]);
                A[i] = std::move(A[i_max]);
                A[i_max] = std::move(rowPtr);

                //pivoting rows of b
                double tmp = std::move(b[i]);
                b[i] = std::move(b[i_max]);
                b[i_max] = std::move(tmp);
            }

            for (j = i+1; j < n; j++) {
                A[j][i] /= A[i][i];
                for (k = i+1; k < n; k++) {
                    A[j][k] -= A[j][i] * A[i][k];
                }
            }
        }

        i=0;
        while(i < n) {
            for (j = 0; j < i; j++){
                b[i] -= A[i][j]*b[j];
            }
            i++;
        }

        while(i > 0) {
            i--;
            for (j = i+1; j < n; j++) {
                b[i] -= A[i][j]*b[j];
            }
            b[i] /= A[i][i];
        }
    }

	inline void triDiagonalSolve(double a[], double b[], double c[], double x[], const uint_fast16_t& nDiag){
        int i1 = 0;
        int i = 1;
        // forward replace
        while(i < nDiag){
            double w = a[i]/b[i1];
            b[i] -= w*c[i1];
            x[i] -= w*x[i1];
            i1 = i++;
        }
        // backwards solve
        x[i] /= b[i];
        while(i > 0){
            i--;
            x[i] = (x[i]-c[i]*x[i+1])/b[i];
        }
    }

	inline void LTS(double ** L, double * b, const uint_fast16_t & n){
        //lower triangular solver by forward substitution
        for(uint_fast16_t i = 0; i < n;i++){
            for(uint_fast16_t j = 0; j < i;j++){
                b[i] -= L[i][j]*b[j];
            }
            b[i] /= L[i][i];
        }
    }

    inline void UTS(double ** U, double * b, const uint_fast16_t & n){
        //lower triangular solver by backward substitution
        for(uint_fast16_t i = n-1; i != 0; i--){
            for(uint_fast16_t j = i+1; j < n; j++){
                b[i] -= U[i][j]*b[j];
            }
            b[i] /= U[i][i];
        }
    }

    inline std::array<double**,2> getLU(double** A, const uint_fast16_t& n){
        // returns L and U
        double** L = eye(n);
        double** U = copy(A,n);

        uint_fast16_t i,j,k;

        for(k = 0; k < n-1;k++){
            for(i = k+1; i < n;i++){
                L[i][k] = U[i][k]/U[k][k];
            }
            for(i = k+1; i < n;i++){
                for( j = k+1; j < n;j++){
                    U[i][j] -= L[i][k]*U[k][j];
                }
            }
        }

        return std::array<double**,2>{L,U};
    }

	inline double* cholesky(double* A, const uint_fast16_t& n) {
        double* L = (double*)calloc(n * n, sizeof(double));

        for (int i = 0; i < n; i++) {
			int i_row =  i*n;
            for (int j = 0; j < (i+1); j++) {
				int j_row = j*n;
                double s = 0;
                for (int k = 0; k < j; k++) {
                    s += L[i_row + k] * L[j_row + k];
                }
                L[i_row + j] = (i == j) ? sqrt(A[i_row + i] - s) : (1.0 / L[j_row + j] * (A[i_row + j] - s));
            }
        }
        return L;
    }

	constexpr unsigned long long factorial[] = {1,1,2,6,24,120,720,5040,40320,362880,3628800,39916800,479001600,6227020800,87178291200,1307674368000,20922789888000,355687428096000,6402373705728000,121645100408832000,2432902008176640000};

    constexpr int arithmetics_primes[] = {2,3,5,7,11,13,17,19,23,29,31,37,41,43,47,53,59,61,67,71,73,79,83,89,97,101,103,107,109,113,127,131,137,139,149,151,157,163,167,173,0};

    constexpr int C[11][11] = {{1},{1,1},{1,2,1},{1,3,3,1},{1,4,6,4,1},{1,5,10,10,5,1},{1,6,15,20,15,6,1},{1,7,21,35,35,21,7,1},{1,8,28,56,70,56,28,8,1},{1,9,36,84,126,126,36,9,1},{1,10,45,120,210,252,210,120,45,10,1}};

	inline double generalBinomial(double alpha, int k){
        // this can be further optimized for half values required by legendre
        double res = 1;
        for (int i = 1; i <= k; ++i)
            res = res * (alpha - (k + i)) / i;
        return res;
    }

    inline unsigned long binomial(unsigned long& n, unsigned long& k) {
        unsigned long c = 1, i;

        if (k > n-k) // take advantage of symmetry
            k = n-k;

        for (i = 1; i <= k; i++, n--) {
            if (c/i > 4294967295/n) // return 0 on overflow
                return 0;

            c = c / i * n + c % i * n / i;  // split c * n / i into (c / i * i + c % i) * n / i
        }

        return c;
    }

    inline int combination(const int& n, const int& k) {
        if(n <= 10) {
            return C[n][k];
        }
        if(k > n/2){
            return combination(n,n-k);
        }
        int num = n;
        int den = k;
        //vectorizable
        for(int i = 1; i < k; i++){
            den *= i;
            num *= (n-i);
        }

        return num/den;
    }

	inline double legendrePoly(const int n, const double x){
        if (n == 0)
            return 1;
        if (n == 1)
            return x;

        double sums = 0;

        for (int k = 0; k < n; k++) {
            if (k > 3){
                sums += pow(x,k) * (combination(n,k) * generalBinomial((n+k-1)*0.5,n));
            } else {
                if(k == 0) {
                    sums += generalBinomial((n+k-1)*0.5,n);
                } else {
                    if(k == 1) {
                        sums += x * n * generalBinomial((n+k-1)*0.5,n);
                    } else {
                        sums += x * n * generalBinomial((n+k-1)*0.5,n);
                    }
                }
            }
        }
        return (1<<n) * sums;
    }

    inline double assocLegendrePoly(int l, int m, double x){
        int sums = 0;
        for (int k = m; k <= l; k++) {
            int prod = k;
            for (int j = m; m < k; m++)
                prod *= j;
            sums += prod* pow(x,k-m) * combination(l,k) * generalBinomial((l+k-1)*0.5,l);
        }
        if (m % 2 == 0)
            return (1<<l) * pow((1-x*x),m/2) *sums;
        else
            return -1 * (1<<l) * pow((1-x*x),m*0.5) *sums;
    }

	inline double evalPoly(const double coef[], const int&n, const double& x){
        int idx = n-1;
        double y = coef[idx];
        while(idx-- > 0){
            y = y*x + coef[idx];
        }
        return y;
    }

	inline void polyFit(double* x, double* y, const uint_fast16_t& n, const uint_fast16_t& deg, double* coef){
        const uint_fast16_t m = deg+1;
        const uint_fast16_t mm = m+m;
        uint_fast16_t i,j;

        double** A = zeros(m);
        double* sums = new double[mm];

        std::fill_n(coef,m,0.0);
        std::fill_n(sums,mm,0.0);
        sums[0] = n;
        for(i = 0; i < n; i++){
            coef[0] += y[i];
            double xn = x[i];
            for(j = 1; j < m;j++){
                sums[j] += xn;
                coef[j] += xn*y[i];
                xn *= x[i];
            }

            for(; j < mm; j++){
                sums[j] += xn;
                xn *= x[i];
            }
        }

        for(i = 0; i < m; i++){
            for(j = 0; j < m;j++){
                A[i][j] = sums[i+j];
            }
        }

        LUPSolve(A,coef,m);

        del(A,m);
        delete[] sums;
    }

    inline void weightedPolyFit(const double x[], const double y[], const double w[], const uint_fast16_t& n, const uint_fast16_t& deg, double* coef){
        const uint_fast16_t m = deg+1;
        const uint_fast16_t mm = m+m;
        uint_fast16_t i,j;

        double** A = zeros(m);
        double* sums = new double[mm];

        std::fill_n(coef,m,0.0);
        std::fill_n(sums,mm,0.0);

        for(i = 0; i < n; i++){
            sums[0] += w[i];
            coef[0] += w[i]*y[i];
            double xn = x[i];
            for(j = 1; j < m;j++){
                double tmp = xn*w[i];
                sums[j] += tmp;
                coef[j] += tmp*y[i];
                xn *= x[i];
            }

            for(; j < mm; j++){
                sums[j] += xn*w[j];
                xn *= x[i];
            }
        }

        for(i = 0; i < m; i++){
            for(j = 0; j < m;j++){
                A[i][j] = sums[i+j];
            }
        }

        LUPSolve(A,coef,m);

        del(A,m);
        delete[] sums;
    }

}

#endif
