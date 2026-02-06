#include "core.h"
#include "environment.h"
#include "constraint.h"
#include "app.h"
#include "view.h"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <list>
#include <optional>
#include <memory>
#include <ranges>


struct Edge {
    Vec2 e;
    Vec2 n;
    Vec2 v1;
    Vec2 v2;
    Edge (const Vec2& v1, const Vec2& v2) : e((v1-v2).normalize()), n(Vec2(e.y, -e.x).normalize()), v1(v1), v2(v2) {};
};

// Boardphase
std::vector<std::shared_ptr<Body>>& boardphase(Environment& env) {
    // TODO
    return env.getBodiesMut();
}

// Given two vectors, find a normalized vector orthogonal to v1, in the direction of v2.
Vec2 orthogonalTowards(const Vec2& _v1, const Vec2& _v2) {

    // if v1 and v2 lie on the same vector, you cannot select an orthogonal vector towards v2.
    if (_v1.norm() == _v2.norm() || _v1.norm() == -_v2.norm()) {
        assert(false);
    }

    Vec3 v1 (_v1.x, _v1.y, 0);
    Vec3 v2 (_v2.x, _v2.y, 0);

    Vec3 o = v1 ^ v2 ^ v1;
    if (o.z != 0) {
        std::cout << "Error :" << v1 << ", " << v2 << "\tout: " << o << std::endl;
        assert(false);
    }

    return Vec2(o.x, o.y).normalize();
}



std::tuple<double, Vec2, double, Vec2> getMinMax(const Body& body, const Vec2& axis) {
    if (!axis.isNorm()) {
        std::cout << axis << std::endl;
        assert(false);
    }

    auto min = std::nan("0");
    auto minv = Vec2(0,0);
    auto max = std::nan("0");
    auto maxv = Vec2(0,0);

    for (const Vec2& p : body.getPointsGlobal())
    {
        double v = p * axis;

        if (std::isnan(min) || v < min) { min = v; minv = p; }
        if (std::isnan(max) || v > max) { max = v; maxv = p; }
    }
    return std::make_tuple(min, minv, max, maxv);
}

std::pair<Edge, Edge> getBodyEdgesWithPoint (const Body& b, const Vec2& p) {
    const auto& v = b.getPointsGlobal();

    int out = 0;
    for (int i=0; i<v.size(); i++) {
        if ((v[i] - p).mag() < (p - v[out]).mag()) {
            out = i;
        }
    }
    Vec2 v1 = v[out];
    Edge e1 (v1, v[out == 0 ? v.size()-1 : out-1]);
    Edge e2 (v1, v[out == v.size()-1 ? 0 : out+1]);

    return std::make_pair(e1, e2);
}


Vec2 findContactPoints (const Edge& e1, const Edge& e2, const Vec2& separationAxis);
std::optional<std::tuple<double, Vec2, Vec2>> detectCollisionSAT(const Body& b1, const Body& b2) {

    // Find normals
    std::vector<Vec2> norms;
    {
        const auto& v = b1.getPointsLocal();
        for (int i=0; i<v.size(); i++) {
            Vec2 e = (v[i] - v[i == 0 ? v.size()-1 : i-1]).normalize();
            norms.push_back(Vec2(-e.y, e.x));
        }
    }{
        const auto& v = b2.getPointsLocal();
        for (int i=0; i<v.size(); i++) {
            Vec2 e = (v[i] - v[i == 0 ? v.size()-1 : i-1]).normalize();
            norms.push_back(Vec2(-e.y, e.x));
        }
    }


    double overlap = std::nan("0");
    Vec2 separationNorm(norms[0]);
    Vec2 va = Vec2(0,0);
    Vec2 vb = Vec2(0,0);
    
    // For each normal
    for (const Vec2& axis: norms)
    {
        // Get min & max projection
        auto [min1, minv1, max1, maxv1] = getMinMax(b1, axis);
        auto [min2, minv2, max2, maxv2] = getMinMax(b2, axis);

        // If no overlap
        if (max1 < min2 || max2 < min1) 
        {
            return std::nullopt;
        }
        else 
        // Check overlap to find MTV
        {
            if (std::isnan(overlap) ||  abs(min1-max2) < abs(overlap)) {
                overlap = min1-max2;
                separationNorm = axis;
                va = minv1;
                vb = maxv2;
            }
            if (abs(min2-max1) < abs(overlap)) {
                overlap = min2-max1;
                separationNorm = axis;
                va = maxv1;
                vb = minv2;
            }
        }
    }
    
    auto f = [&](const Body& body, Vec2 vertex){
        auto [a, b] = getBodyEdgesWithPoint(body, vertex);
        return abs(a.n * separationNorm) > abs(b.n * separationNorm) ? a : b;
    };

    auto e1 = f(b1, va);
    auto e2 = f(b2, vb);

    auto contactP = findContactPoints(e1, e2, separationNorm);

    //injectDebugEffect(std::make_shared<PointEffect>(va, Red));
    //injectDebugEffect(std::make_shared<PointEffect>(vb, Red));

    return std::make_optional(std::make_tuple(overlap, separationNorm, contactP));
}


int clip (Vec2 out[2], Vec2 a, Vec2 b, double o, Vec2 norm) {
    double da = a * norm - o;
    double db = b * norm - o;
    int i = 0;

    // DEBUG
    // std::cout << "clipping seg\t" << a << ":" << da << ", " << b << ":" << db << "\tagainst plane " << o << "\t" << norm << std::endl;
    
    // Past clipping plane
    if (da >= 0) {
        out[i++] = a;
        //std::cout << "a past\n";
    }
    if (db >= 0) { 
        out[i++] = b;
        //std::cout << "b past\n";
    }

    // Opposing sides of plane
    if (da * db < 0) 
    {
        // std::cout << "opposing sides. ";
        // assert(i == 1);

        double r = da / (da - db);
        Vec2 v = (b - a) * r;
        out[i++] = v + a;

        // std::cout << "interpolated to:\t" << out[i-1] << std::endl;
    }

    // Both behind
    else if (da < 0 && db < 0) 
    {
        // std::cout << "both behind. ";
        double r = da / (da - db);
        Vec2 v = (b - a) * -r;
        out[i++] = v + a;
        out[i++] = v + a;

        // std::cout << "interpolated to:\t" << out[i-1] << std::endl;
    }
    assert(i == 2);
    return i;
}


Vec2 findContactPoints (const Edge& e1, const Edge& e2, const Vec2& separationAxis) {

    // Get Edges
    /*injectDebugEffect(std::make_shared<PointEffect>(e1.v1, Blue));*/
    /*injectDebugEffect(std::make_shared<PointEffect>(e1.v2, Blue));*/
    /*injectDebugEffect(std::make_shared<PointEffect>(e2.v1, Blue));*/
    /*injectDebugEffect(std::make_shared<PointEffect>(e2.v2, Blue));*/

    // Identify Reference & Incident Edge 
    Edge ref = e2;
    Vec2 inc[] = { e1.v1, e1.v2 };
    Vec2 norm = orthogonalTowards(ref.e, separationAxis);

    // if e1.norm is more parallel to norm. ref -> e1
    if (std::abs(e1.n * separationAxis) > std::abs(e2.n * separationAxis)) {
        ref = e1;
        norm = orthogonalTowards(ref.e,-separationAxis);
        inc[0] = e2.v1;
        inc[1] = e2.v2;
    }
    // DEBUG
    /*
    std::cout << "norm: " << separationAxis << std::endl;
    std::cout << "ref:  " << ref.v1 << ", " << ref.v2 << std::endl;
    std::cout << "inc:  " << inc[0] << ", " << inc[1] << std::endl;
    */
    

    // Adjacent Clip 
    // v1 -- v2
    if (ref.v1 * ref.e < ref.v2 * ref.e) 
    {
        clip(inc, inc[0], inc[1], ref.v1 * ref.e, ref.e);
        clip(inc, inc[0], inc[1], ref.v2 *-ref.e,-ref.e);
    }
    // v2 -- v1
    else 
    {
        clip(inc, inc[0], inc[1], ref.v1 *-ref.e,-ref.e);
        clip(inc, inc[0], inc[1], ref.v2 * ref.e, ref.e);
    }

    // Normal Clip
    clip(inc, inc[0], inc[1], ref.v1 * norm, norm);


    // Select Point with higher overlap
    Vec2 p = inc[1];
    if (inc[0] * norm > inc[1] * norm) 
    {
        p = inc[0];
    }

    return p; 
}


void ContactConstraint::resolve (const double dt) {
    // http://www.mft-spirit.nl/files/articles/ImpulseSolverBrief.pdf
    // https://raphaelpriatama.medium.com/sequential-impulses-explained-from-the-perspective-of-a-game-physics-beginner-72a37f6fea05
    // (Pa - Pb) * n = 0
    // n * d/dt (Pa - Pb) + (Pa - Pb) * dn/dt = 0
    // n * (Vrel) = 0
    // n * (va + wa x ra - vb - wb x rb) = 0  
    // (n  n x ra  -n  -n x rb) * (Va - Vb + dV)
    // lamb = J V + B / J M-1 J^t dt
    // P = J^t lamb

    Body& b1 = *this->b1;
    Body& b2 = *this->b2;
    Vec3 n = Vec3(this->norm);
    Vec2 p = this->point;

    // Calculate Radius
    Vec3 r1 = Vec3(p - b1.getPos());
    Vec3 r2 = Vec3(p - b2.getPos());

    Vec12 J = Vec12(-n, -n^r1, n, n^r2);
    Vec12 V = Vec12(Vec3(b1.getVelo()), Vec3(0, 0, b1.getAngVelo()), Vec3(b2.getVelo()), Vec3(0, 0, b2.getAngVelo()));
    Vec12 MinvJt = Vec12(
            -n * b1.getInvMass(),
            -n^r1 * b1.getInvInertia(),
            n * b2.getInvMass(),
            n^r2 * b2.getInvInertia());
    double b = 0;

    double lambda;
    {
        //double num = -n*b1.getVelo() -n*(Vec2(-r1.y, r2.x) * b1.getAngVelo()) + n*b2.getVelo() +n*(Vec2(-r2.y, r2.x) * b2.getAngVelo());
        double num = J * V;
        double den = dt * (J * MinvJt);

        std::cout << num << '\t' << den << std::endl;

        lambda = - num / den;
    }

    // std::cout << lambda * dt << std::endl;

    // Apply Impulse
    const Vec2 impulseN = this->norm * dt * lambda;

    const Vec2 tan = Vec2(this->norm.y, this->norm.x);
    const Vec2 vrel = b1.getVelo() + Vec2( -r1.y, r1.x) * b1.getAngVelo() -
                      b2.getVelo() - Vec2( -r2.y, r2.x) * b2.getAngVelo();
    const double mu = sqrt(b1.getFriction() * b2.getFriction());
    const Vec2 impulseT = tan * (tan * vrel * mu);

    const Vec2 impulse = impulseN ;//+ impulseT;
    b1.impulse(-impulse, Vec2(r1.x, r1.y));
    b2.impulse( impulse, Vec2(r2.x, r2.y));

    std::cout << "impulse mag: " << impulse.mag() << std::endl;

    /*
    // (Sink Prevention) Positional Correction, Linear Projection
    const double percent = 0.2;     // usually 20% to 80% 
    const double slop    = 0.01;    // usually 0.01 to 0.1 
    double correction = percent * std::max( this->depth - slop, (double) 0 ) / (b1.getInvMass() + b2.getInvMass());
    Vec2 correctionV = this->norm * correction;

    b1.setPos(b1.getPos() - correctionV * b1.getInvMass());
    b2.setPos(b2.getPos() + correctionV * b2.getInvMass());
    */
}

bool ContactConstraint::converged(const int iteration) {

    if (iteration >= 4) 
        return true;

    // C: contact points touching
    Vec2 pa = this->point;
    Vec2 pb = this->point - this->norm * this->depth;
    injectDebugEffect(std::make_shared<PointEffect>(pb, Red)); 
    injectDebugEffect(std::make_shared<PointEffect>(pa, Green));
    injectDebugEffect(std::make_shared<VectorEffect>(pa, this->norm * 30, Red));

    Vec2 del = pa - pb;

    std::cout << "del mag: " << del.mag() << std::endl;

    return del.mag() < 1;
}
        
// Checks for collisions and resolves accordingly.
void Environment::collide(const double dt) {
        
    // Prune with boardphase
    auto& bodyList = boardphase(*this);

    // Find collisions
    std::list<std::shared_ptr<Constraint>> constraints;
    for (int i=0; i<bodyList.size(); i++) 
    {
        for (int j=i+1; j<bodyList.size(); j++) 
        {
            // Check for collision
            auto opt = detectCollisionSAT(*bodyList[i], *bodyList[j]);
            if (!opt) continue;
            auto [overlap, separationAxis, contactP] = *opt;
            std::shared_ptr<Constraint> contact = 
                std::make_shared<ContactConstraint>(overlap, separationAxis, contactP, bodyList[i], bodyList[j]);
            constraints.push_back(contact);

        }
    }

    // sequential impulse
    int iteration = 0;
    while (!constraints.empty()) 
    {
        std::list<std::shared_ptr<Constraint>> nconstraints;
        for (auto& constraint: constraints) {
            // Resolve collision
            constraint->resolve(dt);

            // Update impulses
            constraint->b1->update(dt);
            constraint->b2->update(dt);

            // Remove if converged
            if (!constraint->converged(iteration)) {
                nconstraints.push_back(constraint);
            }
        }

        constraints = std::move(nconstraints);
        iteration++;
    }
    if (iteration)
        std::cout << "iterations: " << iteration << std::endl;
}
