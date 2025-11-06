#include "forces.h"

void ForceConstAcceleration::apply() {
    for (Particle* p : particles) {
        p->force += p->mass * acceleration;
    }
}


void ForceDrag::apply() {
    for (Particle* p : particles) {
        Vec3 v = p->vel;
        p->force -= klinear * v + kquadratic * v * v.norm();
    }
}

void ForceSpring::apply() {
    if (particles.size() < 2) return;

    Particle* p1 = getParticle1();
    Particle* p2 = getParticle2();

    double d = (p2->pos - p1->pos).norm();
    if (d < 1e-8) return;

    Vec3 dv = p2->vel - p1->vel;
    double relVel = dv.dot((p2->pos - p1->pos).normalized());

    Vec3 F = ks * (d - L) * (p2->pos - p1->pos).normalized()
             + kd * relVel * (p2->pos - p1->pos).normalized();

    p1->force += F;
    p2->force -= F;
}

void ForceGravitation::apply() {
    if (!attractor) return;

    for (Particle* p : particles) {
        if (p == attractor) continue;

        double d = (attractor->pos - p->pos).norm();
        if (d < 1e-8) continue;

        double smoothFactor = (2.0 / (1.0 + std::exp(-a * (d*d) / (b*b)))) - 1.0;
        Vec3 F = G * (p->mass * attractor->mass) * smoothFactor /
                 (d*d) * (attractor->pos - p->pos).normalized();

        p->force += F;
    }
}
