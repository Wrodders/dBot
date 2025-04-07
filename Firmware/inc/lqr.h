#ifndef LQR_H
#define LQR_H
#include "common/common.h"

struct LQR{
    float K[4]; // LQR Gain Matrix
    float u;    // Control Input
    struct{
        float  *const x[4]; // State Vector pointers
    }state; 
};
#endif // LQR_H