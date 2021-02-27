#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <vector>

struct Matrix{
    unsigned int rows = 0;
    unsigned int cols = 0;
    std::vector<float> elem;

    void makeIdentity(int rows, int cols){
        elem.resize(rows * cols);
        for(int r = 0; r < rows; r++)
            for(int c = 0; c < cols; c++)
                if(r == c)
                    elem[c + r*cols] = 1.f;
                else
                    elem[c + r*cols] = 0.f;
    }

    void makeZero(int rows, int cols){
        elem.resize(rows * cols);
        for(int r = 0; r < rows; r++)
            for(int c = 0; c < cols; c++)
                elem[c + r*cols] = 0.f;
    }

    void set(int row, int col, float val)
    {
        elem[col + row *cols] = val;
    }

};

Matrix multiplyMatrices(const Matrix& lm, const Matrix& rm){
    Matrix out;
    out.makeZero(lm.rows, rm.cols);


    for(unsigned int r = 0; r < out.rows; r++)
        for(unsigned int c = 0; c < out.cols; c++)
            for(unsigned int i = 0; i < lm.cols; i++)
                out.elem[c + r*out.cols] += lm.elem[i + r*lm.cols] * rm.elem[c*rm.rows + i];

    return out;
}

Matrix transpose(const Matrix& m){
    Matrix out;
    out.makeZero(m.cols, m.rows);

    for(unsigned int r = 0; r < m.rows; r++)
        for(unsigned int c = 0; c < m.cols; c++)
            out.elem[c*out.cols + r] = m.elem[c + r*m.cols];

    return out;
}

Matrix add(const Matrix& lm, const Matrix& rm){
    Matrix out;
    out.makeZero(lm.cols, rm.rows);

    for(unsigned int r = 0; r < out.rows; r++)
        for(unsigned int c = 0; c < out.cols; c++)
            out.elem[c + r*out.cols] += lm.elem[c + r*out.cols] + rm.elem[c + r*out.cols];

    return out;
}

class KalmanFilter
{
public:
    KalmanFilter();

    void init(const Matrix& systemState, float variance);
protected:
    void predict(float dt);
    void update();

private:
    // [x,y,v_x,v_y]
    Matrix systemState;
    Matrix transition;
    Matrix covariance;
    Matrix processNoiseCovariance;
    Matrix observationMatrix;

};

#endif // KALMANFILTER_H
