#pragma once

#include <stdint.h>
#include <cmath>
#include <cstring>
#include <string>

#define OUTPUT_FORMAT " %+.8e"
// #define OUTPUT_FORMAT "%E "

struct Dimension {
	const uint_fast16_t value;
	Dimension(uint_fast16_t val) : value(val) {}
};

template <uint_fast16_t N>
struct Vector {

	alignas(32) double data[N];
	static const uint_fast16_t bytes = sizeof(data);

	Vector(){}
	
	Vector(const char& c) {
		memset(this->data, c, bytes);
	}
	
	Vector(double x[N]) : data(x) {}
	
	Vector(const Vector<N>& v){
		std::memcpy(this->data, v.data, bytes);
	}
	
	~Vector(){}

	void set_zero() {
		std::memset(this->data,0,bytes);
	}
	
	void set(const uint_fast16_t i, double val) {
		this->data[i] = val;
	}
	
	void operator=(const Vector<N>& v) {
		std::copy(v.data, v.data + N, this->data);
	}
	
	double& operator[](const uint_fast16_t& i) {
		return this->data[i];
	}
	
	double& operator()(const uint_fast16_t& i) {
		return this->data[i];
	}
	
	double operator()(const uint_fast16_t& i) const {
		return this->data[i];
	}
	
	double operator*(const Vector<N>& v) const {
		double sum = 0;
		for(uint_fast16_t i = 0; i < N; i++){
			sum += v[i]*this->data[i];
		}
		return sum;
	}
	
	Vector<N> operator+(const Vector<N>& v) const {
		Vector<N> out;
		for(uint_fast16_t i = 0; i < N; i++){
			out[i] = v.data[i] + this->data[i];
		}
		return out;
	}
	
	Vector<N> operator-(const Vector<N>& v) const {
		Vector<N> out;
		for(uint_fast16_t i = 0; i < N; i++){
			out[i] = v.data[i] - this->data[i];
		}
		return out;
	}
	
	Vector<N> operator*(const double& a) const {
		Vector<N> out;
		for(uint_fast16_t i = 0; i < N; i++){
			out[i] = a*this->data[i];
		}
		return out;
	}
	
	Vector<N> operator=(const Vector<N>& a) const {
		Vector<N> out;
		for(uint_fast16_t i = 0; i < N; i++){
			out[i] = this->data[i];
		}
		return out;
	}
	
	void operator*=(const double& a) {
		for(uint_fast16_t i = 0; i < N; i++){
			this->data[i] *= a;
		}
	}
	
	double norm() const {
		double sum = 0;
		for(uint_fast16_t i = 0; i < N; i++){
			sum += data[i]*data[i];
		}
		return sqrt(sum);
	}
	
	std::string to_string(){
		std::string out;
		for(uint_fast16_t j = 0; j < N; j++){
			char buf[16];
			sprintf(buf,OUTPUT_FORMAT,this->data[j]);
			out += buf;
		}
		return out;
	}
};

template <uint_fast16_t R,uint_fast16_t C = R>
struct Matrix {
	static const uint_fast32_t M = R*C;
	alignas(32) double data[M];
	double* rows[R];
	static const uint_fast32_t bytes = sizeof(data);

	Matrix(){
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
	}
	
	Matrix(const bool& eye){
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
		memset(this->data, 0, bytes);
		if(eye) {
			uint_fast16_t i = 0;
			const uint_fast16_t n = C+1;
			while(i < M) {
				this->data[i] = 1;
				i += n;
			}
		} 
	}
	
	Matrix(const char& val){
		memset(this->data, val, bytes);
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
	}
	
	Matrix(double* x) : data(x){
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
	}
	
	Matrix(const double& a){
		for(uint_fast16_t i = 0; i < M; i++){
			data[i] = a;
		}
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
	}
	
	Matrix(const Matrix<R,C>& A){
		std::memcpy(this->data, A.data, bytes);
		for(uint_fast16_t row = 0; row < R; row++){
			this->rows[row] = &(this->data[row*C]);
		}
	}
	
	~Matrix(){}
	
	void set(const uint_fast16_t& i, const uint_fast16_t& j, const double& val) {
		this->rows[i][j] = val;
	}
	
	void set_zero(){
		memset(this->data, 0, bytes);
	}
	
	Matrix<C,R> get_transpose() const {
		Matrix<C,R> A;
		for(uint_fast16_t i = 0; i < R; i++){
			const double* row = this->rows[i];
			for(uint_fast16_t j = 0; j < C; j++){
				A[j][i] = row[j];
			}
		}
		return A;
	}
	
	void operator=(const Matrix<R,C>& A) {
		std::memcpy(this->data, A.data, bytes);
	}
	
	double* operator[](const uint_fast16_t& i) {
		return this->rows[i];
	}
	
	const double* operator[](const uint_fast16_t& i) const {
		return this->rows[i];
	}
	
	double& operator()(const uint_fast16_t& i,const uint_fast16_t& j) {
		return this->rows[i][j];
	}
	
	double operator()(const uint_fast16_t& i,const uint_fast16_t& j) const {
		return this->rows[i][j];
	}
	
	void operator+=(const double& a) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] += a;
		}
	}
	
	void operator-=(const double& a) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] -= a;
		}
	}
	
	void operator*=(const double& a) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] *= a;
		}
	}
	
	void operator/=(const double& a) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] /= a;
		}
	}
	
	void operator+=(const Matrix<R,C>& A) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] += A.data[i];
		}
	}
	
	void operator-=(const Matrix<R,C>& A) {
		for(uint_fast16_t i = 0; i < M; i++){
			this->data[i] -= A.data[i];
		}
	}
	
	Matrix<R,C> operator+(const Matrix<R,C>& A) const {
		Matrix<R,C> out;
		for(uint_fast16_t i = 0; i < M; i++){
			out.data[i] = this->data[i] + A.data[i];
		}
		return out;
	}
	
	Matrix<R,C> operator-(const Matrix<R,C>& A) const {
		Matrix<R,C> out;
		for(uint_fast16_t i = 0; i < M; i++){
			out.data[i] = this->data[i] - A.data[i];
		}
		return out;
	}
	
	Matrix<R,C> operator*(const double& a) const {
		Matrix<R,C> out;
		for(uint_fast16_t i = 0; i < M; i++){
			out.data[i] = this->data[i]*a;
		}
		return out;
	}
	
	Matrix<R,C> operator/(const double& a) const {
		Matrix<R,C> out;
		for(uint_fast16_t i = 0; i < M; i++){
			out.data[i] = this->data[i]/a;
		}
		return out;
	}
	

	template<uint_fast16_t K>
	Matrix<R,K> operator*(const Matrix<C,K>& A) const {
		Matrix<R,K> B(false);
        uint_fast16_t i,j,k;
        for (i = 0; i < R; i++) {
            double* b_row = B[i];
            for (k = 0; k < C; k++) { // inner dimension
                const double& val = this->rows[i][k];
                const double* a_row = A[k];
                for (j = 0; j < K; j++) { // hopefully -O3 vectorized
                    b_row[j] += val*a_row[j];
                }
            }
        }
		return B;
	}
	
	Vector<R> operator*(const Vector<C>& b) const {
		Vector<R> x;
        uint_fast16_t i,j;
        for (i = 0; i < R; i++) {
			const double* a_row = this->rows[i];
			x.data[i] = 0;
            for (j = 0; j < C; j++) { 
                x.data[i] += a_row[j]*b.data[j];
            }
        }
		return x;
	}

	template<uint_fast16_t N>
	static void solve(Matrix<N,N>& A, Vector<N>& b){
		uint_fast16_t i, j, k, i_max; 
        double max,absA;
		
		double *_A[N];
		for (i = 0; i < N; i++) {
			_A[i] = A[i];
		}
		//uint_fast16_t N1 = N + 1;
        for (i = 0; i < N; i++) {
            i_max = i;
			absA = fabs(A(i,i));
			max = absA;
            for (k = i + 1; k < N; k++)
                if ((absA = fabs(A(k,i))) > max) { 
                    max = absA;
                    i_max = k;
                }

            if (max < 1e-300){
                throw "degenerate Matrix";
            }
			
            if (i_max != i) {
                //pivoting A
                _A[i] = A[i_max];
                _A[i_max] = A[i];

                //pivoting rows of b
                double tmp = std::move(b.data[i]);
                b.data[i] = std::move(b.data[i_max]);
                b.data[i_max] = std::move(tmp);
            }
			
			const double* rowi = _A[i];
            for (j = i+1; j < N; j++) {
				double* rowj = _A[j];
                rowj[i] /= rowi[i];
				const double& a = rowj[i];
                for (k = i+1; k < N; k++) {
                    rowj[k] -= a * rowi[k];
                }
            }
        }
        
        i=0;
        while(i < N) {
			const double* rowi = _A[i];
            for (j = 0; j < i; j++){
                b[i] -= rowi[j]*b[j];
            }
            i++;
        }

        while(i > 0) {
            i--;
			const double* rowi = _A[i];
            for (j = i+1; j < N; j++) {
                b[i] -= rowi[j]*b[j];
            }
            b[i] /= rowi[i];
        }
	}
	
	template<uint_fast16_t R2, uint_fast16_t C2>
	Matrix<R2,C2> slice(const uint_fast16_t& row,const uint_fast16_t& col) {
		/*
		if( row + R2 > R || col + C2 > C){
			throw "invalid dimensions or index";
		}
		*/
		
		Matrix<R2,C2> A;
		for(uint_fast16_t i = 0; i < R2; i++) {
			double* r1 = this->rows[row + i];
			double* r2 = A[i];
			for(uint_fast16_t j = 0; j < C2; j++) {
				r2[j] = r1[col + j];
			}
		}
		return A;
	}
	
	void set_identity(){
		memset(this->data, 0, bytes);
		uint_fast16_t i = 0;
		uint_fast16_t n = C+1;
		while(i < M) {
			this->data[i] = 1;
			i += n;
		}
	}

	void qr( Matrix<R,R>& q, Matrix<R,C>& r){
		q.set_identity();
		std::memcpy(this->data, r.data, bytes);
		Vector<R> u;
		for(uint_fast16_t k = 0; k < C; k++){
			double sum = 0;
			for(uint_fast16_t j = k; j < R; j++){
				u.data[j] = this->rows[j][k];
				sum += (u.data[j]*u.data[j]);
			}
		}
	}
	
	std::string to_string(){
		std::string out;
		for(uint_fast16_t i = 0; i < R; i++){
			const double* row = this->rows[i];
			for(uint_fast16_t j = 0; j < C; j++){
				char buf[17];
				sprintf(buf,OUTPUT_FORMAT,row[j]);
				out += buf;
			}
			out += "\n";
		}
		return out;
	}
	
};


struct MatrixX {
	const uint_fast16_t nRows;
	const uint_fast16_t nCols;
	const uint_fast32_t nElements;
	double* const data;
	double** const rows;
	const uint_fast16_t bytes;

	MatrixX(const uint_fast16_t& r, const uint_fast16_t& c) : nRows(r), nCols(c), nElements(c*r), data(new double[nElements]), rows(new double*[r]), bytes(nElements*sizeof(double)) {
		for(uint_fast16_t row = 0; row < r; row++){
			this->rows[row] = &(this->data[row*this->nCols]);
		}
	}
	
	MatrixX(const uint_fast16_t& r, const uint_fast16_t& c, const char& val) : nRows(r), nCols(c), nElements(c*r), data(new double[nElements]), rows(new double*[r]), bytes(nElements*sizeof(double)) {
		memset(this->data, val, bytes);
		for(uint_fast16_t row = 0; row < this->nRows; row++){
			this->rows[row] = &(this->data[row*this->nCols]);
		}
	}
	
	~MatrixX(){
		delete[] this->rows;
		delete[] this->data;
	}
	
	void set(const uint_fast16_t& i, const uint_fast16_t& j, const double& val) {
		this->rows[i][j] = val;
	}
	
	void set_zero(){
		memset(this->data, 0, bytes);
	}
	
	MatrixX get_transpose() const {
		MatrixX A(this->nCols,this->nRows);
		for(uint_fast16_t i = 0; i < this->nRows; i++){
			const double* row = this->rows[i];
			for(uint_fast16_t j = 0; j < this->nCols; j++){
				A[j][i] = row[j];
			}
		}
		return A;
	}
	
	void operator=(const MatrixX& A) {
		std::memcpy(this->data, A.data, bytes);
	}
	
	double* operator[](const uint_fast16_t& i) {
		return this->rows[i];
	}
	
	const double* operator[](const uint_fast16_t& i) const {
		return this->rows[i];
	}
	
	double& operator()(const uint_fast16_t& i,const uint_fast16_t& j) {
		return this->rows[i][j];
	}
	
	double operator()(const uint_fast16_t& i,const uint_fast16_t& j) const {
		return this->rows[i][j];
	}
	
	void operator+=(const double& a) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] += a;
		}
	}
	
	void operator-=(const double& a) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] -= a;
		}
	}
	
	void operator*=(const double& a) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] *= a;
		}
	}
	
	void operator/=(const double& a) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] /= a;
		}
	}
	
	void operator+=(const MatrixX& A) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] += A.data[i];
		}
	}
	
	void operator-=(const MatrixX& A) {
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			this->data[i] -= A.data[i];
		}
	}
	
	MatrixX operator+(const MatrixX& A) const {
		MatrixX out(this->nRows,this->nCols);
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			out.data[i] = this->data[i] + A.data[i];
		}
		return out;
	}
	
	MatrixX operator-(const MatrixX& A) const {
		MatrixX out(this->nRows,this->nCols);
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			out.data[i] = this->data[i] - A.data[i];
		}
		return out;
	}
	
	MatrixX operator*(const double& a) const {
		MatrixX out(this->nRows,this->nCols);
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			out.data[i] = this->data[i]*a;
		}
		return out;
	}
	
	MatrixX operator/(const double& a) const {
		MatrixX out(this->nRows,this->nCols);
		for(uint_fast16_t i = 0; i < this->nElements; i++){
			out.data[i] = this->data[i]/a;
		}
		return out;
	}
	

	MatrixX operator*(const MatrixX& A) const {
		MatrixX B(this->nRows,A.nCols, (char)0);
        uint_fast16_t i,j,k;
        for (i = 0; i < this->nRows; i++) {
            double* b_row = B[i];
            for (k = 0; k < this->nCols; k++) { // inner dimension
                const double& val = this->rows[i][k];
                const double* a_row = A[k];
                for (j = 0; j < A.nCols; j++) { // hopefully -O3 vectorized
                    b_row[j] += val*a_row[j];
                }
            }
        }
		return B;
	}
	
	static Matrix<3,3> invert(const Matrix<3,3>& A) {
		Matrix<3,3> Inv;
		
        Inv.data[0] = A[1][1]*A[2][2] - A[1][2]*A[2][1];
        Inv.data[1] = A[0][2]*A[2][1] - A[0][1]*A[2][2];
        Inv.data[2] = A[0][1]*A[1][2] - A[0][2]*A[1][1];
        
        Inv.data[3] = A[1][2]*A[2][0] - A[1][0]*A[2][2];
        Inv.data[4] = A[0][0]*A[2][2] - A[0][2]*A[2][0];
        Inv.data[5] = A[0][2]*A[1][0] - A[0][0]*A[1][2];
        
        Inv.data[6] = A[1][0]*A[2][1] - A[1][1]*A[2][0];
        Inv.data[7] = A[0][1]*A[2][0] - A[0][0]*A[2][1];
        Inv.data[8] = A[0][0]*A[1][1] - A[0][1]*A[1][0];
		
		double det = 1/(A.data[0]*Inv.data[0] + A.data[1]*Inv.data[3] + A.data[2]*Inv.data[6]);

		Inv *= det;

        return Inv;
	}
	
	template<uint_fast16_t N>
	static Matrix<N,N> LUPInvert(Matrix<N,N>& A) { // should make constant
		uint_fast16_t i, j, k, imax; 
		double maxA, *ptr, absA;
		Matrix<N,N> IA;
		double P[N];
		double *_A[N];
		for (i = 0; i < N; i++) {
			P[i] = i; 
			_A[i] = A[i];
		}

		for (i = 0; i < N; i++) {
			maxA = 0.0;
			imax = i;

			for (k = i; k < N; k++) {
				if ((absA = fabs(A[k][i])) > maxA) { 
					maxA = absA;
					imax = k;
				}
			}

			if (imax != i) {
				j = P[i];
				P[i] = P[imax];
				P[imax] = j;

				ptr = _A[i];
				_A[i] = _A[imax];
				_A[imax] = ptr;
			}

			for (j = i + 1; j < N; j++) {
				_A[j][i] /= _A[i][i];
				for (k = i + 1; k < N; k++) {
					_A[j][k] -= _A[j][i] * _A[i][k];
				}
			}
		}

		for (j = 0; j < N; j++) {
			for (i = 0; i < N; i++) {
				IA[i][j] = (P[i] == j) ? 1.0 : 0.0;

				for (k = 0; k < i; k++) {
					IA[i][j] -= _A[i][k] * IA[k][j];
				}
			}

			for (int i2 = N - 1; i2 >= 0; i2--) {
				for (k = i2 + 1; k < N; k++) {
					IA[i2][j] -= _A[i2][k] * IA[k][j];
				}

				IA[i2][j] /= _A[i2][i2];
			}
		}
		
		return IA;
	}
	
	/*
	MatrixX* QR(MatrixX& A){
		
	}
	*/
	
	std::string to_string(){
		std::string out;
		for(uint_fast16_t i = 0; i < this->nRows; i++) {
			std::string rowS;
			const double* row = this->rows[i];
			for(uint_fast16_t j = 0; j < this->nCols; j++) {
				char buf[16];
				sprintf(buf,OUTPUT_FORMAT,row[j]);
				rowS += buf;
			}
			out += rowS + "\n";
		}
		return out;
	}
	
};
