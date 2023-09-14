#pragma once

//#include "ftype.h"
#include "vector3.h"
#include "vector2.h"
#include "matrix3.h"

class SE23 {
public:
    // Default constructor
    SE23() 
        : _rot(), _x(), _w(), _alpha(0.0f) 
    {

    }

    SE23(const Matrix3f& rot, const Vector3f& x, const Vector3f& w, float alpha)
        : _rot(rot), _x(x), _w(w), _alpha(alpha) 
    {
        
    }
    
    //SE23(const Matrix3f& rot, const Vector3f& x, const Vector3f& w, float alpha);
    // Matrix multiplication
    SE23 operator *(const SE23& rhs);
    //Matrix Exponential
    static SE23 exponential(const Vector3f &S, const Vector3f &x,const Vector3f &w, float alpha); //needs to be static 
    //Inverse of the ZHat Matrix used in CINS
    static SE23 inverse_ZHat(const SE23& ZHat);
    const Matrix3f& rot();
    const Vector3f& x();
    const Vector3f& w();
    float alpha();

private:
    Matrix3f _rot;
    Vector3f _x;
    Vector3f _w;
    float _alpha;
};

typedef SE23 SE23f;
