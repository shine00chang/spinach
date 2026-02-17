#include "core.h"
#include "environment.h"
#include "constraint.h"
#include "view.h"

#include <cmath>
#include <utility>
#include <vector>
#include <list>
#include <optional>
#include <memory>


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
    if (!axis.isNorm()) assert(false);

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


// e: contact edge where normal is pointed outwards, b: penetrating body
// returns: list of (edge point, penetrating point)
// Finds all points of B behind edge E.
// Accomplished by taking the projection of the difference between the point and a point on the edge.
std::list<std::pair<Vec2, Vec2>> findContactPoints (const Edge& e, const Body& b) {

    std::list<std::pair<Vec2, Vec2>> out;
    for (auto& p : b.getPointsGlobal()) {
        if ((p-e.v1) * e.n > 0) continue; 

        Vec2 p_edge = p - e.n * ((p-e.v1) * e.n);
        out.push_back(std::make_pair(p_edge, p));
    }
    return out;
}

std::optional<std::list<std::shared_ptr<Constraint>>> detectCollisionSAT(std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) {

    // all data used to create contact constraint
    double overlap = std::nan("0");
    Edge contact_edge (Vec2(0,0), Vec2(0,0));
    std::list<std::pair<Vec2, Vec2>> contacts;
    std::shared_ptr<Body> b_edge = b1;
    std::shared_ptr<Body> b_penetrating = b2;

    auto check = [&](std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) -> bool 
    {
        // Construct edges where normals are pointed outwards.
        std::vector<Edge> edges;
        const auto& v = b1->getPointsGlobal();
        for (int i=0; i<v.size(); i++) {
            Edge e = Edge(v[i], v[i == 0 ? v.size()-1 : i-1]);

            // If the projection of position on normal is positive, normal is pointing inwards.
            if ((v[i]-b1->getPos()) * e.n < 0) e.n = -e.n;

            edges.push_back(e);
        }
        
        bool changed = false;
        // For each edge, determine overlap. Find and store minimum overlap.
        for (const Edge& edge: edges)
        {
            auto [min1, minv1, max1, maxv1] = getMinMax(*b1, edge.n);
            auto [min2, minv2, max2, maxv2] = getMinMax(*b2, edge.n);

            // If the furthest point on body1 along this normal is not one of the points on this edge.
            // This should only happen when 1) the shape is concave or 2) the normal is not outwards.
            if (max1 - (edge.v1 * edge.n) > 0.01) {
                // TODO: Add debug dump
                std::cout << "furthest point on body1 along this normal is not one of the points on this edge." << std::endl;
                assert(false);
                continue;
            }

            // If no overlap, there is no collision.
            if (max1 < min2 || max2 < min1) 
            {
                std::cout << "exiting\n";
                return false;
            }
            // Find minimum overlap write. store edge as contact edge.
            // Technically removing the abs should be fine...?
            if (std::isnan(overlap) ||  
               abs(max1-min2) < abs(overlap)) 
            {
                overlap = max1-min2;
                contact_edge = edge;
                changed = true;
            }
        }

        // If minum has changed, find and store contact points and body order
        if (changed) 
        {
            contacts = findContactPoints(contact_edge, *b2);
            b_edge = b1;
            b_penetrating = b2;
        }
        return true;
    };

    if (!check(b1, b2)) return std::nullopt;
    if (!check(b2, b1)) return std::nullopt;

    // TODO: Add debug dump
    if (overlap < 0) {
        std::cout << "negative overlap.. weird: " << overlap << std::endl;   
    }
    std::cout << "overlap: " << overlap << std::endl;

    // Make constraint out of every contact
    std::list<std::shared_ptr<Constraint>> constraints;
    for (auto& [p_edge, p_penetrating] : contacts) {
        constraints.push_back(std::make_shared<ContactConstraint>(
            contact_edge.n,
            p_edge,
            p_penetrating,
            b_edge,
            b_penetrating));

        injectDebugEffect(std::make_shared<PointEffect>(p_edge, Red)); 
        injectDebugEffect(std::make_shared<PointEffect>(p_penetrating, Blue));
        injectDebugEffect(std::make_shared<VectorEffect>(p_edge, contact_edge.n * 30, Green));
    }
    std::cout << "contacts found: " << contacts.size() << std::endl;

    return std::make_optional(constraints);
}

// Returns impulse. Positive for penetrating body (b2)
Vec2 ContactConstraint::resolve (const double dt) {
    // http://www.mft-spirit.nl/files/articles/ImpulseSolverBrief.pdf
    // https://raphaelpriatama.medium.com/sequential-impulses-explained-from-the-perspective-of-a-game-physics-beginner-72a37f6fea05
    // (Pa - Pb) * n = 0
    // n * d/dt (Pa - Pb) + (Pa - Pb) * dn/dt = 0
    // n * (Vrel) = 0
    // n * (va + wa x ra - vb - wb x rb) = 0  
    // (n  n x ra  -n  -n x rb) * (V + dV) = 0 V includes both A and B
    // dV = J * lamb, that is, dV is in the direction of N
    // J = (n  n x ra  -n  -n x rb)
    // J * V + J * dV = 0
    // J * V + J * dV = 0
    // lamb = J V / J J^t dt
    // P = J^t lamb

    Body& b1 = *this->b1;
    Body& b2 = *this->b2;
    Vec3 n = Vec3(this->norm);

    // Calculate Radius
    Vec3 r1 = Vec3(this->v1 - b1.getPos());
    Vec3 r2 = Vec3(this->v2 - b2.getPos());

    // Construct Jacobian
    const Vec12 J = Vec12(-n, n^r1, n, -n^r2);
    const Vec12 V = Vec12(Vec3(b1.getVelo()), Vec3(0, 0, b1.getAngVelo()), Vec3(b2.getVelo()), Vec3(0, 0, b2.getAngVelo()));
    const Vec12 MinvJ_tp = Vec12(
            -n * b1.getInvMass(),
            n^r1 * b1.getInvInertia(),
            n * b2.getInvMass(),
            -n^r2 * b2.getInvInertia());

    // Calculate Bias
    double b = 0;
    {   // Baumgarte Stabilization
        // adds energy to produce impulse to separate penetration.
        // energy is proportional to the positional penetration. 
        // allows set bit of penetration (slop) so as to not introduce energy when the system is near stable.
        // NOTE: not sure why its negative.. isn't it adding energy? might be the sign depending on which body its applied to
        double Baumgarte = 0.2;
        double d = (v1-v2) * norm;
        double slopallowance = 2;
        d = std::max(d-slopallowance, 0.0);
        b += - Baumgarte * d / dt;
    }
    {   // Restitution
        // adds energy proportional to the relevative normal velocity, to produce 'bounce'
        double restitution = 0.15;
        double vreln = J * V;
        double slopallowance = 5;
        vreln = std::max(vreln-slopallowance, 0.0);
        b += restitution * vreln;
    }

    // Calculate resolution
    Vec2 impulseN;
    {
        double num = J * V + b;
        double den = dt * (J * MinvJ_tp);
        double lambda = - num / den;
        impulseN = this->norm * dt * lambda;
    }

    // Calculate Friction
    // Friction Impulse is a fraction of the relative tangent velocity 
    // clamped by the normal impulse times coeffecient of friction
    Vec2 impulseT;
    {
        const Vec3 t = Vec3(-this->norm.y, this->norm.x, 0);
        const Vec12 Jt = Vec12(-t, t^r1, t, -t^r2);
        const Vec12 MinvJt_tp = Vec12(
            -t * b1.getInvMass(),
            t^r1 * b1.getInvInertia(),
            t * b2.getInvMass(),
            -t^r2 * b2.getInvInertia());

        double num = Jt * V;
        double den = dt * (Jt * MinvJt_tp);
        double lambda = - num / den;

        double mu = 0.5;
        double mag = std::min(mu * impulseN.mag(), dt * lambda);
        impulseT = Vec2(t.x * mag, t.y * mag);
    }

    const Vec2 impulse = impulseN + impulseT;
    std::cout << "impulse:\t" << impulse << std::endl;
    
    return impulse;
}

bool ContactConstraint::converged(const int iteration, Vec2 J1, double L1, Vec2 J2, double L2) {

    if (iteration >= 4) 
        return true;

    // C: contact points touching

    // Calculate contact points after impulse
    // nv = v + dp
    // rotate nv by angvelo * dt 


    auto pointVelocity = [](Vec2 p, Vec2 v, double angV, Vec2 bp) {
        Vec3 r = p-bp;
        Vec3 av = Vec3(0,0,angV) ^ r;
        return v + Vec2(av.x, av.y);
    };

    auto applyImpulse = [](const Body& body, Vec2 J, double L) {
        Vec2 nv = body.getVelo() + (J * body.getInvMass());
        double nw = body.getAngVelo() + body.getInvInertia() * L;
        return std::make_pair(nv,nw);
    };


    auto [nv1, nw1] = applyImpulse(*b1, J1, L1);
    auto [nv2, nw2] = applyImpulse(*b2, J2, L2);

    std::cout << "velocity, angvelocity:\t" << b1->getVelo() << ",\t" << b1->getAngVelo() << std::endl;
    std::cout << "velocity, angvelocity:\t" << b2->getVelo() << ",\t" << b2->getAngVelo() << std::endl;
    std::cout << "velocity, angvelocity, v1:\t" << nv1 << ",\t" << nw1 << ",\t" << (v1-b1->getPos()) << std::endl;
    std::cout << "velocity, angvelocity, v2:\t" << nv2 << ",\t" << nw2 << ",\t" << (v2-b2->getPos()) << std::endl;

    Vec2 delP = v1-v2;
    Vec2 delV = pointVelocity(v1, nv1, nw1, b1->getPos()) + pointVelocity(v2, nv2, nw2, b2->getPos());

    injectDebugEffect(std::make_shared<VectorEffect>(v1, pointVelocity(v1, nv1, nw1, b1->getPos()) * 0.1, Red));
    injectDebugEffect(std::make_shared<VectorEffect>(v2, pointVelocity(v2, nv2, nw2, b2->getPos()) * 0.1, Red));

    std::cout << "delP mag: " << delP.mag() << "\tdelV * n: " << (delV * norm) << std::endl;

    //return del.mag() < 1;
    return true;
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
            auto opt = detectCollisionSAT(bodyList[i], bodyList[j]);
            if (!opt) continue;
            auto contacts = *opt;

            constraints.splice(constraints.end(), contacts);
        }
    }

    // sequential impulse
    // do not accmulate impulse into velocity
    // accmulate impulse for each body
    // then apply impulse onto velocity
    // convergence checker needs to apply the accumulated velocity and check from there.
    // we do not apply it in collision
    
    // Body*, Translational impulse J, Rotational impulse L
    std::map<Body*, std::pair<Vec2, double>> impulses;
    auto accumulateJ = [&](std::shared_ptr<Body> b, Vec2 v, Vec2 impulse) {
        auto [J, L] = impulses[b.get()];
        Vec3 r = v-b->getPos();
        Vec3 dL = r ^ impulse;
        impulses[b.get()] = std::make_pair(J + impulse, L + dL.z);
    };
    int iteration = 0;

    while (!constraints.empty()) 
    {
        std::list<std::shared_ptr<Constraint>> nconstraints;

        // Resolve collisions: accumulate impulse
        for (auto& constraint: constraints) {
            auto impulse = constraint->resolve(dt);
            accumulateJ(constraint->b1, constraint->v1, -impulse);
            accumulateJ(constraint->b2, constraint->v2,  impulse);
        }

        // Check Convergence with current accumulated impulse
        // Remove if converged
        for (auto& constraint: constraints) {
            if (!constraint->converged(iteration,
                    impulses[constraint->b1.get()].first, impulses[constraint->b1.get()].second,
                    impulses[constraint->b2.get()].first, impulses[constraint->b2.get()].second)) {
                nconstraints.push_back(constraint);
            }
        }

        constraints = std::move(nconstraints);
        iteration++;
    }

    // Apply impulse now
    for (auto [body, impulse] : impulses) {
        auto [J, L] = impulse;
        std::cout << "J:\t" << J << "\tL:\t" << L << std::endl;
        body->impulse(J, L);
    }

    if (iteration)
        std::cout << "iterations: " << iteration << std::endl;
}
