#ifndef __CORE_h__
#define __CORE_h__

#include <iostream>
#include <cmath>

#define EPSILON 0.00001

struct Vec2 {
    double x, y;

    Vec2(double x, double y) : x(x), y(y) {}

    inline Vec2   operator + (const Vec2& o)  const { return Vec2 (x+o.x, y+o.y); }
    inline Vec2   operator - ()               const { return Vec2 (-x, -y); };
    inline Vec2   operator - (const Vec2& o)  const { return Vec2 (x-o.x, y-o.y); }
    inline Vec2   operator * (const double o) const { return Vec2 (x*o, y*o); }
    inline double operator * (const Vec2& o)  const { return x*o.x + y*o.y; }
    inline bool   operator ==(const Vec2& o)  const { return std::fabs(x - o.x) < EPSILON && std::fabs(y - o.y) < EPSILON; }

    inline double mag () const { return std::sqrt(x*x + y*y); }
    inline bool isNorm () const { return std::fabs(x * x + y * y - 1) < EPSILON; };
    Vec2 norm () const {
        double m = mag();
        return Vec2(x / m, y / m);
    }
    Vec2& normalize () {
        *this = norm();
        return *this;
    }

    Vec2 rotate (double rad) {
        double c = std::cos(rad);
        double s = std::sin(rad);
        return Vec2(c*x - s*y,
                    s*x + c*y);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec2& o) { os << "(" << o.x << ", " << o.y << ")"; return os; }
};

struct Vec3 {
    double x, y, z;

    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3(Vec2 v) : x(v.x), y(v.y), z(0) {}

    inline Vec3   operator + (const Vec3& o)  const { return Vec3 (x+o.x, y+o.y, z+o.z); }
    inline Vec3   operator - ()               const { return Vec3 (-x, -y, -z); };
    inline Vec3   operator - (const Vec3& o)  const { return Vec3 (x-o.x, y-o.y, z-o.z); }
    inline Vec3   operator * (const double o) const { return Vec3 (x*o, y*o, z*o); }
    inline double operator * (const Vec3& o)  const { return x*o.x + y*o.y + z*o.z; }
    inline bool   operator ==(const Vec3& o)  const { return x == o.x && y == o.y && z == o.z; }

    Vec3   operator ^ (const Vec3& o)  const {
        return Vec3(
            y * o.z - z * o.y,
          -(x * o.z - z * o.x),
            x * o.y - y * o.x
        );
    }

    inline bool isNorm () const { return std::fabs( x * x + y * y + z * z - 1) < EPSILON; };
    Vec3 norm () const {
        double m = mag();
        return Vec3(
                x / m,
                y / m,
                z / m);
    }
    Vec3& normalize () {
        *this = norm();
        return *this;
    }

    inline double mag () const { return std::sqrt(x*x + y*y + z*z); }

    friend std::ostream& operator<<(std::ostream& os, const Vec3& o) { os << "(" << o.x << ", " << o.y << ", " << o.z << ")"; return os; }
};

struct Vec12 {
    double v[12];

    Vec12(double* nv) {
        for (int i=0; i<12; i++) v[i] = nv[i];
    }
    Vec12(Vec3 a, Vec3 b, Vec3 c, Vec3 d) {
        v[0] = a.x;
        v[1] = a.y;
        v[2] = a.z;
        v[3] = b.x;
        v[4] = b.y;
        v[5] = b.z;
        v[6] = c.x;
        v[7] = c.y;
        v[8] = c.z;
        v[9] = d.x;
        v[10] = d.y;
        v[11] = d.z;
    }

    inline Vec12  operator * (const double o) const {
        Vec12 nv = Vec12(*this);
        for (int i=0; i<12; i++) nv.v[i] *= o;
        return nv;
    }
    inline double operator * (const Vec12& o)  const { 
        double s = 0;
        for (int i=0; i<12; i++) {
            s += v[i] * o.v[i];
            /*std::cout << s << "\t";*/
        }
        /*std::cout << std::endl;*/
        return s;
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec12& o) {
        os << "( " << o.v[0];
        for (int i=1; i<12; i++) os << ", " << o.v[i];
        os << " )";
        return os;
    }
};

#endif
