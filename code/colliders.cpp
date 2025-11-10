#include "colliders.h"
#include <cmath>


/*
 * Generic function for collision response from contact plane
 */
void Collider::resolveCollision(Particle* p, const Collision& col, double kElastic, double kFriction) const
{
    double h = (p->pos - col.position).dot(col.normal);
    p->pos = p->pos - 2.0 * h * col.normal + col.normal * (p->radius + 1e-4);

    Vec3 v_n = col.normal * p->vel.dot(col.normal);
    Vec3 v_t = p->vel - v_n;
    v_n = -kElastic * v_n;
    v_t = (1.0 - kFriction) * v_t;
    p->vel = v_n + v_t;
}



/*
 * Plane
 */
bool ColliderPlane::isInside(const Particle* p) const {
    return planeN.dot(p->pos) + planeD <= p->radius;
}


bool ColliderPlane::testCollision(const Particle* p, Collision& colInfo) const {
    double d0 = planeN.dot(p->prevPos) + planeD;
    double d1 = planeN.dot(p->pos) + planeD;
    if (d0 * d1 > 0.0) return false;

    double lambda = d0 / (d0 - d1);

    colInfo.position = p->prevPos + lambda * (p->pos - p->prevPos);
    colInfo.normal = planeN;
    return true;
}



/*
 * Sphere
 */
bool ColliderSphere::isInside(const Particle* p) const {
    return (p->pos - center).norm() < radius + p->radius;
}


bool ColliderSphere::testCollision(const Particle* p, Collision& colInfo) const {
    Vec3 v = p->pos - p->prevPos;
    double r = p->radius + radius;
    Vec3 p_c = p->prevPos - center;

    double a = v.dot(v);
    if (a == 0)
        return false;
    double b = 2.0 * v.dot(p_c);
    double c = (p_c).dot(p_c) - r * r;
    double delta = b * b - 4 * a * c;
    if (delta < 0.0)
        return false;
    double lambda1 = (-b - std::sqrt(delta)) / (2.0 * a);
    double lambda2 = (-b + std::sqrt(delta)) / (2.0 * a);
    double lambda = -1.0;
    if (lambda1 >= 0.0 && lambda1 <= 1.0)
        lambda = lambda1;
    else if (lambda2 >= 0.0 && lambda2 <= 1.0)
        lambda = lambda2;
    if (lambda < 0.0)
        return false;
    Vec3 p_lambda = p->prevPos + lambda * v;

    Vec3 n = (p_lambda - center).normalized();
    colInfo.normal = n;
    colInfo.position = center + n * (radius + p->radius);
    return true;
}


/*
 * AABB
 */
bool ColliderAABB::isInside(const Particle* p) const {
    double r = p->radius;
    Vec3 pos = p->pos;

    if (pos.x() + r < bmin.x() || pos.x() - r > bmax.x()) return false;
    if (pos.y() + r < bmin.y() || pos.y() - r > bmax.y()) return false;
    if (pos.z() + r < bmin.z() || pos.z() - r > bmax.z()) return false;

    return true;
}



bool ColliderAABB::testCollision(const Particle* p, Collision& colInfo) const
{
    Vec3 dir = p->pos - p->prevPos;
    Vec3 minR = bmin;
    Vec3 maxR = bmax;

    double tmin = 0.0;
    double tmax = 1.0;
    int hitAxis = -1;

    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < 1e-8) {
            if (p->prevPos[i] < minR[i] || p->prevPos[i] > maxR[i])
                return false;
            continue;
        }

        double t1 = (minR[i] - p->prevPos[i]) / dir[i];
        double t2 = (maxR[i] - p->prevPos[i]) / dir[i];
        if (t1 > t2) std::swap(t1, t2);

        if (t1 > tmin) {
            tmin = t1;
            hitAxis = i;
        }
        if (t2 < tmax) {
            tmax = t2;
        }

        if (tmin > tmax) return false;
    }

    if (tmin < 0.0 || tmin > 1.0) return false;

    colInfo.position = p->prevPos + tmin * dir;
    colInfo.normal = Vec3(0,0,0);

    Vec3 dirN = dir.normalized();
    if (hitAxis == 0) colInfo.normal.x() = (dirN.x() > 0 ? -1 : 1);
    if (hitAxis == 1) colInfo.normal.y() = (dirN.y() > 0 ? -1 : 1);
    if (hitAxis == 2) colInfo.normal.z() = (dirN.z() > 0 ? -1 : 1);

    return true;
}


GridId ColliderParticle::getGridId(const Vec3 &pos) const {
    return { (int)std::floor(pos.x()/cellSize),
            (int)std::floor(pos.y()/cellSize),
            (int)std::floor(pos.z()/cellSize) };
}

void ColliderParticle::resolveParticleCollision(Particle* p1, Particle* p2, double kElastic) {
    double d = (p2->pos - p1->pos).norm();
    double minDist = p1->radius + p2->radius;
    if (d <= 0.0 || d >= minDist)
        return;

    Vec3 n = (p2->pos - p1->pos) / d;
    // Particle correction if they bump into each other
    double penetration = minDist - d;
    p1->pos -= 0.5 * penetration * n;
    p2->pos += 0.5 * penetration * n;

    Vec3 v = p1->vel - p2->vel;
    double vn = v.dot(n);
    if (vn > 0) return;

    double j = -(1 + kElastic) * vn * 0.5;
    p1->vel += j * n;
    p2->vel -= j * n;
}

void ColliderParticle::resolveAllCollisions(ParticleSystem& system, double kElastic) {
    grid.clear();
    auto& particles = system.getParticles();
    // Do the mapping between each particle and the grid cell
    for (auto* p : particles)
        grid[getGridId(p->pos)].push_back(p);

    int dirs[3] = {-1, 0, 1};
    for (auto& cell : grid) {
        const GridId& id = cell.first;
        for (int dx : dirs)
            for (int dy : dirs)
                for (int dz : dirs) {
                    GridId nId = {id.x + dx, id.y + dy, id.z + dz};
                    // If not out of bounds
                    if (grid.find(nId) == grid.end()) continue;

                    auto& A = cell.second;
                    auto& B = grid[nId];
                    for (auto* p1 : A)
                        for (auto* p2 : B)
                            if (p1 < p2)
                                resolveParticleCollision(p1, p2, kElastic);
                }
    }
}
