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

    double restDensity     = 1000.0;
    double viscosity       = 0.02;
    double speedOfSound    = 2.0;

    double particleRadius  = 0.00366;
    double spacing         = 0.0058;
    double smoothingRadius = 4.0 * particleRadius;

    double particleMass = restDensity * pow(2.0 * particleRadius, 3);
    double halfSize        = 0.08;
    double fillHeight      = 0.08;
    double sceneScale      = 500.0;


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
    double poly6(double d);
    Vec3 spikyGrad(Vec3 dir, double d);
    double viscosityLapl(double d);
};

#endif // SCENEFLUID_H
