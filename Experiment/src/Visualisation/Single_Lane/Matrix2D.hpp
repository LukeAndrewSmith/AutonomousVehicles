#include <vector>
#include <cstdlib>

// simple classe pour un tableau 2D
template<class T>
class Matrix2D {
    std::vector<T> vector;
    size_t n_cols;
public:
    Matrix2D() : vector(0), n_cols(0) {}
    Matrix2D(size_t rows, size_t cols) : vector (rows*cols), n_cols(cols) {}
    Matrix2D(size_t rows, size_t cols, T init) : vector (rows*cols, init), n_cols(cols) {}
    T& operator()(size_t row, size_t col) {
        return vector.at((row*n_cols)+col);
    }
    T* data(){ return vector.data(); }
    size_t get_n_cols(){ return n_cols;  }
    size_t get_n_rows(){ return vector.size()/n_cols;  }
};
