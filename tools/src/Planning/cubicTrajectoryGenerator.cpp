//
// Created by tina on 23. 11. 6.
//

#include "Planning/cubicTrajectoryGenerator.hpp"

void cubicTrajectoryGenerator::updateTrajectory(double currentPosition, double Reference_pose, double currentTime, double timeDuration) {
    mFunctionValue << currentPosition, Reference_pose, 0.0, 3.0;
    mReferenceTime = currentTime;
    mTimeDuration = timeDuration;
    calculateCoefficient();
}

void cubicTrajectoryGenerator::calculateCoefficient() {
    mCoefficient = mMatrixA * mFunctionValue;
}

double cubicTrajectoryGenerator::getPositionTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return mCoefficient(0,0) * pow(normalizedTime, 3.0) + mCoefficient(1,0) * pow(normalizedTime, 2.0) + mCoefficient(2,0) * normalizedTime + mCoefficient(3, 0);
}

double cubicTrajectoryGenerator::getVelocityTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (3.0 *mCoefficient(0,0) * pow(normalizedTime, 2.0) + 2.0 * mCoefficient(1,0) * normalizedTime + mCoefficient(2,0)) / mTimeDuration;
}

double cubicTrajectoryGenerator::getAccelerationTrajectory(double currentTime) {
    if(currentTime>mReferenceTime + mTimeDuration) currentTime = mReferenceTime + mTimeDuration;
    if(currentTime<mReferenceTime) currentTime = mReferenceTime;
    double normalizedTime = (currentTime - mReferenceTime) / mTimeDuration;
    return (6.0 *mCoefficient(0,0) * normalizedTime + 2.0 * mCoefficient(1,0)) / pow(mTimeDuration, 2.0);
}
