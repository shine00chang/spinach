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
                std::cout << "overlap: " << overlap << std::endl;
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
    // (n  n x ra  -n  -n x rb) * (Va - Vb + dV)
    // lamb = J V + B / J M-1 J^t dt
    // P = J^t lamb

    Body& b1 = *this->b1;
    Body& b2 = *this->b2;
    Vec3 n = Vec3(this->norm);

    // Calculate Radius
    Vec3 r1 = Vec3(this->v1 - b1.getPos());
    Vec3 r2 = Vec3(this->v2 - b2.getPos());

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
        double num = J * V;
        double den = dt * (J * MinvJt);
        lambda = - num / den;
    }
    // Apply Impulse
    const Vec2 impulseN = this->norm * dt * lambda;

    const Vec2 tan = Vec2(this->norm.y, this->norm.x);
    const Vec2 vrel = b1.getVelo() + Vec2( -r1.y, r1.x) * b1.getAngVelo() -
                      b2.getVelo() - Vec2( -r2.y, r2.x) * b2.getAngVelo();
    const double mu = sqrt(b1.getFriction() * b2.getFriction());
    const Vec2 impulseT = tan * (tan * vrel * mu);

    const Vec2 impulse = impulseN ;//+ impulseT;
    /*b1.impulse(-impulse, Vec2(r1.x, r1.y));*/
    /*b2.impulse( impulse, Vec2(r2.x, r2.y));*/

    std::cout << "impulse mag: " << impulse.mag() << std::endl;
    
    return impulse;

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

    // Calculate contact points after impulse
    // nv = v + dp
    // rotate nv by angvelo * dt 

    /*
    injectDebugEffect(std::make_shared<PointEffect>(v1, Red)); 
    injectDebugEffect(std::make_shared<PointEffect>(v2, Green));
    injectDebugEffect(std::make_shared<VectorEffect>(v1, this->norm * 30, Red));
    */

    Vec2 del = v1-v2;

    std::cout << "del mag: " << del.mag() << std::endl;

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
        impulses[b.get()] = std::make_pair(J + impulse, L + Vec2(-v.y, v.x) * impulse);
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
            if (!constraint->converged(iteration)) {
                nconstraints.push_back(constraint);
            }
        }

        constraints = std::move(nconstraints);
        iteration++;
    }

    // Apply impulse now
    for (auto [body, impulse] : impulses) {
        auto [J, L] = impulse;
        body->impulse(J, L);
    }

    if (iteration)
        std::cout << "iterations: " << iteration << std::endl;
}
