#include "kalmanfilter.h"

KalmanFilter::KalmanFilter()
{

}

void KalmanFilter::init(const Matrix &systemState, float variance)
{
    this->systemState = systemState;

    transition.makeIdentity(4,4);
    transition.set(0, 2, 1.f); //set time
    transition.set(1, 3, 1.f);

    observationMatrix.makeZero(2, 2);
}

void KalmanFilter::predict(float dt)
{
    // model
    systemState = multiplyMatrices(transition, systemState);

    covariance = multiplyMatrices(multiplyMatrices(transition, covariance), transpose(transition));
    covariance = add(covariance, processNoiseCovariance);
}

void KalmanFilter::update()
{
    Matrix kalmanGain = multiplyMatrices(covariance, transpose(observationMatrix));
    //kalmanGain = multiplyMatrices(kalmanGain, )
    //systemState = multiplyMatrices(kalmanGain, )
}
