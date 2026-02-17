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
Bodies& boardphase(Environment& env) {
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

// SAT helper: find the min and max projection of a body on an axis
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

std::optional<Constraints> detectCollisionSAT(std::shared_ptr<Body> b1, std::shared_ptr<Body> b2) {

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
    Constraints constraints;
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


        

// Checks for collisions and resolves accordingly.
Constraints collide(Environment& env, const double dt) 
{
    // Prune with boardphase
    auto& bodyList = boardphase(env);

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
    return constraints;
}

