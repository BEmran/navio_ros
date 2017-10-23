#include "DynSys.h"

/*
void integDyn(float* y, float* x, float* xdot, float* u, float t) {
    xdot = u;
    y = x;
}
*/
DynSys::DynSys(int size, void (*Fun)(float*, float*, float*, float*, float)) {
    size_ = size;
    dynFun = Fun;
    for (int i = 0; i < size_; i++) {
        x_.push_back(0.0);
        y_.push_back(0.0);
        xdot_.push_back(0.0);
        xdot_old_.push_back(0.0);
    }
}

DynSys::DynSys(int size, void (*Fun)(float*, float*, float*, float*, float), float* x0) {
    size_ = size;
    dynFun = Fun;
    for (int i = 0; i < size_; i++) {
        x_.push_back(x0[i]);
        y_.push_back(0.0);
        xdot_.push_back(0.0);
        xdot_old_.push_back(0.0);
    }
}

void DynSys::setX(float* x0) {
    for (int i = 0; i < x_.size(); i++) {
        x_.at(i) = x0[i];
    }
}

const float* DynSys::getX(void) {
    return &x_[0];
}

const float*  DynSys::getY(void) {
    return &y_[0];
}

int DynSys::getSize(void)const {
    return size_;
}

DynSys::~DynSys() {
}

const float* DynSys::calcDynamic(float* u, float t) {
    float xdot[size_];
    dynFun(&y_[0], &x_[0], xdot, u, t);
    for (int i = 0; i < size_; i++) {
        xdot_.at(i) = xdot[i];
    }
    return &xdot_[0];
}

const float* DynSys::integration(float dt) {
    for (int i = 0; i < size_; i++) {
        x_.at(i) = x_.at(i) + (xdot_.at(i) * 1.5 - xdot_old_.at(i) * 0.5) * dt;
        xdot_old_.at(i) = xdot_.at(i);
    }
    return &x_[0];
}

const float* DynSys::update(float* u, float t,float dt) {
    calcDynamic(u, t);
    integration(dt);
    return &y_[0];
}

