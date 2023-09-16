#pragma once

#include "AP_Math.h"
#include "ftype.h"
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

    SE23(const Matrix3F& rot, const Vector3F& x, const Vector3F& w, ftype alpha)
        : _rot(rot), _x(x), _w(w), _alpha(alpha) 
    {
        
    }
    
    //SE23(const Matrix3F& rot, const Vector3F& x, const Vector3F& w, ftype alpha);
    // Matrix multiplication
    SE23 operator *(const SE23& rhs);
    //Matrix Exponential
    static SE23 exponential(const Vector3F &S, const Vector3F &x,const Vector3F &w, ftype alpha); //needs to be static 
    //Inverse of the ZHat Matrix used in CINS
    static SE23 inverse_ZHat(const SE23& ZHat);
    const Matrix3F& rot();
    const Vector3F& x();
    const Vector3F& w();
    ftype alpha();

    void init(Matrix3F &rot) {
        _rot = rot;
    }

private:
    Matrix3F _rot;
    Vector3F _x;
    Vector3F _w;
    ftype _alpha;
};

typedef SE23 SE23F;
