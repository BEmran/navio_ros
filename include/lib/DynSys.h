#ifndef DYNSYS_H
#define DYNSYS_H

#include <iostream>
#include <vector>

//void integDyn(float*, float*, float*, float*, float);

class DynSys {
public:
    DynSys() {};
//    DynSys(int size, void (*Fun)(float*, float*, float*, float*, float)= *integDyn);
    DynSys(int size, void (*Fun)(float*, float*, float*, float*, float));
    DynSys(int size, void (*Fun)(float*, float*, float*, float*, float),float* x0);    
    virtual ~DynSys();
    int getSize(void)const;
    void setX(float *);
    const float* getX(void);
    const float* getY(void);
    const float* integration(float dt);
    const float* calcDynamic(float* u, float t);
    const float* update(float* u, float t,float dt);

private:
    int size_;
    std::vector<float> x_, y_;
    std::vector<float> xdot_, xdot_old_;
    void (*dynFun)(float*, float*, float*, float*, float);
};

#endif /* DYNSYS_H */

