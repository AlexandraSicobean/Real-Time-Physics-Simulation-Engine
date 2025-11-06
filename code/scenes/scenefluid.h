#ifndef SCENEFLUID_H
#define SCENEFLUID_H

#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>
#include <list>
#include "scene.h"
#include "widgetfluid.h"
#include "particlesystem.h"
#include "integrators.h"
#include "colliders.h"

class SceneFluid : public Scene
{
    Q_OBJECT

public:
    SceneFluid();
    virtual ~SceneFluid();

    // Scene overrides
    virtual void initialize() override;
    virtual void reset() override;
    virtual void update(double dt) override;
    virtual void paint(const Camera& cam) override;

    virtual void mousePressed(const QMouseEvent* e, const Camera& cam) override;
    virtual void mouseMoved(const QMouseEvent* e, const Camera& cam) override;
    virtual void mouseReleased(const QMouseEvent* e, const Camera& cam) override;

    virtual void getSceneBounds(Vec3& bmin, Vec3& bmax) override {
        bmin = Vec3(-110, -10, -110);
        bmax = Vec3( 110, 100, 110);
    }

    virtual unsigned int getNumParticles() override { return system.getNumParticles(); }
    virtual QWidget* sceneUI() override { return widget; }

public slots:
    void updateSimParams();

protected:
    WidgetFluid* widget = nullptr;

    QOpenGLShaderProgram* shader = nullptr;
    QOpenGLVertexArrayObject* vaoSphere = nullptr;
    unsigned int numFacesSphere = 0;

    ParticleSystem system;
    IntegratorSymplecticEuler integrator;
    ForceConstAcceleration* fGravity = nullptr;

    double restDensity      = 1000.0;
    double viscosity        = 0.001;
    double speedOfSound     = 5.0;
    double particleRadius   = 0.02;
    double spacing          = particleRadius * 2.1;
    double smoothingRadius  = spacing * 2.0;
    double particleMass     = 0.8 * restDensity * pow(spacing, 3);

    double sceneScale       = 100.0;
    double halfSize         = 0.25;
    double fillHeight       = 0.25;

    // Colliders
    QOpenGLVertexArrayObject* vaoCube = nullptr;
    ColliderAABB  colliderBox;
    ColliderPlane colliderBottom;
    ColliderPlane colliderLeft;
    ColliderPlane colliderRight;
    ColliderPlane colliderFront;
    ColliderPlane colliderBack;

    double maxParticleLife;
    std::list<Particle*> deadParticles;

    int mouseX = 0, mouseY = 0;

    void computeDensityAndPressure();
    void computeForces();
};

#endif // SCENEFLUID_H
