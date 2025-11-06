#include "integrators.h"


void IntegratorEuler::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();
    Vecd x1 = x0 + dt*dx;
    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces();
}


void IntegratorSymplecticEuler::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();

    // First compute the new velocity
    Vecd v0 = system.getVelocities();
    Vecd a0 = system.getAccelerations();
    Vecd v1 = v0 + a0 * dt;

    // Then move
    Vecd p0 = system.getPositions();
    Vecd p1 = p0 + v1 * dt;

    system.setVelocities(v1);
    system.setPositions(p1);
    system.setTime(t0+dt);
    system.updateForces();
}


void IntegratorMidpoint::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();

    // Compute the state (pos, vel) at midpoint
    Vecd xmid = x0 + dx * dt / 2;
    system.setState(xmid);
    system.updateForces();
    Vecd dxmid = system.getDerivative();
    Vecd x1 = x0 + dxmid * dt;

    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces();
}


void IntegratorRK2::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd dx = system.getDerivative();

    Vecd xtrial = x0 + dx * dt;
    system.setState(xtrial);
    system.updateForces();
    Vecd dtrial = system.getDerivative();

    Vecd x1 = x0 + dt * (dx + dtrial)/2;

    system.setState(x1);
    system.setTime(t0+dt);
    system.updateForces(); }


void IntegratorRK4::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd x0 = system.getState();
    Vecd k1 = system.getDerivative();

    Vecd trial = x0 + (dt/2) * k1;
    system.setState(trial);
    system.updateForces();
    Vecd k2 = system.getDerivative();

    trial = x0 + (dt/2) * k2;
    system.setState(trial);
    system.updateForces();
    Vecd k3 = system.getDerivative();

    trial = x0 + dt * k3;
    system.setState(trial);
    system.updateForces();
    Vecd k4 = system.getDerivative();

    Vecd x1 = x0 + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);

    system.setState(x1);
    system.setTime(t0 + dt);
    system.updateForces();
}


void IntegratorVerlet::step(ParticleSystem &system, double dt) {
    double t0 = system.getTime();
    Vecd a_current = system.getAccelerations();

    Vecd p_current = system.getPositions();
    Vecd p_previous = system.getPreviousPositions();

    Vecd p_next = 2.0 * p_current - p_previous + dt * dt * a_current;
    Vecd v_next = (p_next - p_previous) / (2.0 * dt);

    system.setPreviousPositions(p_current);
    system.setPositions(p_next);
    system.setVelocities(v_next);

    system.setTime(t0 + dt);
    system.updateForces();
}

