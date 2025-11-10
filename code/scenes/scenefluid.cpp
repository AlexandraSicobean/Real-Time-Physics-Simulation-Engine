#include "scenefluid.h"
#include "glutils.h"
#include "model.h"
#include <QOpenGLFunctions_3_3_Core>

SceneFluid::SceneFluid()
{
    widget = new WidgetFluid();
    connect(widget, SIGNAL(updatedParameters()), this, SLOT(updateSimParams()));
}

SceneFluid::~SceneFluid()
{
    if (widget) delete widget;
    if (shader) delete shader;
    if (vaoSphere) delete vaoSphere;
    if (fGravity) delete fGravity;
}

void SceneFluid::initialize()
{
    // Load shader
    shader = glutils::loadShaderProgram(":/shaders/phong.vert", ":/shaders/phong.frag");

    // Create a sphere VAO for particles
    Model sphere = Model::createIcosphere(2);
    vaoSphere = glutils::createVAO(shader, &sphere, buffers);
    numFacesSphere = sphere.numFaces();

    // Forces
    fGravity = new ForceConstAcceleration();
    system.addForce(fGravity);

    // Create cube (bath tub)
    Model cube = Model::createOpenCube();
    vaoCube = glutils::createVAO(shader, &cube, buffers);
    glutils::checkGLError();

    // Actual colliders of the 5 faces cube
    colliderBottom.setPlane(Vec3(0, 1, 0), 0.0);
    colliderLeft.setPlane(Vec3(1, 0, 0), halfSize);
    colliderRight.setPlane(Vec3(-1, 0, 0), halfSize);
    colliderFront.setPlane(Vec3(0, 0, 1), halfSize);
    colliderBack.setPlane(Vec3(0, 0, -1), halfSize);

    reset();
}

void SceneFluid::reset() {

    system.deleteParticles();
    fGravity->clearInfluencedParticles();

    double step = 2.0 * particleRadius;
    smoothingRadius = 2.0 * step;
    particleMass    = restDensity * pow(step, 3);

    for (double x = -halfSize + step; x <= halfSize; x += step)
        for (double y = step; y <= fillHeight; y += step)
            for (double z = -halfSize + step; z <= halfSize; z += step) {
                Particle* p = new Particle();
                p->pos = Vec3(x, y, z);
                p->prevPos = p->pos;
                p->vel = Vec3(0, 0, 0);
                p->radius = particleRadius;
                p->color = Vec3(0.2, 0.4, 1.0);
                p->mass = particleMass;
                p->life = maxParticleLife;
                system.addParticle(p);
                fGravity->addInfluencedParticle(p);
            }
    std::cout << "Created " << system.getParticles().size() << " particles.\n";
}

void SceneFluid::updateSimParams()
{
    // Get values from widget
    // fGravity->setAcceleration(Vec3(0, -widget->getGravity(), 0));

    // restDensity = widget->getRestDensity();
    // stiffness   = widget->getStiffness();
    // viscosity   = widget->getViscosity();
    // smoothingRadius = 24.0;
    // particleMass = widget->getParticleMass();
}

void SceneFluid::update(double dt)
{
    computeDensityAndPressure();
    computeForces();

    Vecd posPrev = system.getPositions();
    integrator.step(system, dt);
    system.setPreviousPositions(posPrev);

    // Handle wall collisions
    Collision col;
    for (Particle* p : system.getParticles()) {
        if (colliderBottom.testCollision(p, col))
            colliderBottom.resolveCollision(p, col, 0.05, 0.3);
        if (colliderLeft.testCollision(p, col))
            colliderLeft.resolveCollision(p, col, 0.05, 0.3);
        if (colliderRight.testCollision(p, col))
            colliderRight.resolveCollision(p, col, 0.05, 0.3);
        if (colliderFront.testCollision(p, col))
            colliderFront.resolveCollision(p, col, 0.05, 0.3);
        if (colliderBack.testCollision(p, col))
            colliderBack.resolveCollision(p, col, 0.05, 0.3);
    }

    for (Particle* p : system.getParticles()) {
        if (p->life > 0) {
            p->life -= dt;
            if (p->life < 0)
                deadParticles.push_back(p);
        }
    }
}


void SceneFluid::paint(const Camera& cam)
{
    QOpenGLFunctions* glFuncs = nullptr;
    glFuncs = QOpenGLContext::currentContext()->functions();

    shader->bind();

    // camera matrices
    QMatrix4x4 camProj = cam.getPerspectiveMatrix();
    QMatrix4x4 camView = cam.getViewMatrix();
    shader->setUniformValue("ProjMatrix", camProj);
    shader->setUniformValue("ViewMatrix", camView);

    // lighting
    const int numLights = 1;
    const QVector3D lightPosWorld[numLights] = { QVector3D(80, 80, 80) };
    const QVector3D lightColor[numLights]    = { QVector3D(1, 1, 1) };
    QVector3D lightPosCam[numLights];
    for (int i = 0; i < numLights; i++) {
        lightPosCam[i] = camView.map(lightPosWorld[i]);
    }
    shader->setUniformValue("numLights", numLights);
    shader->setUniformValueArray("lightPos", lightPosCam, numLights);
    shader->setUniformValueArray("lightColor", lightColor, numLights);

    vaoCube->bind();

    glFuncs->glEnable(GL_BLEND);
    glFuncs->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glFuncs->glDisable(GL_CULL_FACE);  // show inner walls too

    // tub walls material
    shader->setUniformValue("matdiff", 0.6f, 0.8f, 1.0f);  // lighter blue
    shader->setUniformValue("matspec", 0.2f, 0.2f, 0.2f);
    shader->setUniformValue("matshin", 30.0f);


    // transform the cube
    QMatrix4x4 modelMat;
    modelMat.translate(0, halfSize * sceneScale, 0);   // center of the box
    modelMat.scale(halfSize * sceneScale, halfSize * sceneScale, halfSize * sceneScale);   // make it wide and low
    shader->setUniformValue("ModelMatrix", modelMat);

    // draw all faces
    glFuncs->glDrawElements(GL_TRIANGLES, 3 * 2 * 5, GL_UNSIGNED_INT, 0);

    glFuncs->glEnable(GL_CULL_FACE);
    glFuncs->glDisable(GL_BLEND);
    vaoCube->release();

    // draw particles
    vaoSphere->bind();
    for (Particle* p : system.getParticles()) {
        QMatrix4x4 modelMat;
        modelMat.translate(p->pos.x() * sceneScale,
                           p->pos.y() * sceneScale,
                           p->pos.z() * sceneScale);
        modelMat.scale(p->radius * sceneScale);

        shader->setUniformValue("ModelMatrix", modelMat);

        // Normalize deviation around rest density
        double rel = (p->density - restDensity) / restDensity;

        // Clamp to [-0.5, 0.5] to avoid extreme values
        rel = std::max(-0.5, std::min(0.5, rel));

        // Map to color: blue (low density) → white (medium) → red (high density)
        float r = float(0.5 + rel * 2.0);
        float g = float(0.5 - fabs(rel) * 1.0);
        float b = float(1.0 - rel * 2.0);

        shader->setUniformValue("matdiff", r, g, b);

        shader->setUniformValue("matspec", 1.0f, 1.0f, 1.0f);
        shader->setUniformValue("matshin", 80.0f);

        glFuncs->glDrawElements(GL_TRIANGLES, 3 * numFacesSphere, GL_UNSIGNED_INT, 0);
    }

    vaoSphere->release();
    shader->release();
}

void SceneFluid::mousePressed(const QMouseEvent*, const Camera&) {}
void SceneFluid::mouseMoved(const QMouseEvent*, const Camera&) {}
void SceneFluid::mouseReleased(const QMouseEvent*, const Camera&) {}

double SceneFluid::poly6(double d) {
    if (d >= smoothingRadius)
        return 0.0;
    double diff2 = smoothingRadius * smoothingRadius - d * d;
    if (diff2 <= 0.0)
        return 0.0;
    return (315.0 / (64.0 * M_PI * pow(smoothingRadius, 9))) * pow(diff2, 3);
}


void SceneFluid::computeDensityAndPressure() {
    auto &particles = system.getParticles();

    // Debug variables
    double totalDensity = 0.0, minDensity = 1e9, maxDensity = 0.0;
    double totalPressure = 0.0, minPressure = 1e9, maxPressure = 0.0;
    int count = 0;

    for (Particle *p1 : particles) {
        p1->density = p1->mass * poly6(0.0);
        for (Particle *p2 : particles) {
            double d = (p1->pos - p2->pos).norm();
            // Compute density
            p1->density += p2->mass * poly6(d);
        }

        // Compute pressure
        p1->pressure = speedOfSound * speedOfSound * (p1->density - restDensity);

        // Debug prints
        totalDensity += p1->density;
        totalPressure += p1->pressure;
        minDensity = std::min(minDensity, p1->density);
        maxDensity = std::max(maxDensity, p1->density);
        minPressure = std::min(minPressure, p1->pressure);
        maxPressure = std::max(maxPressure, p1->pressure);
        count++;
    }

    double totalMass = particleMass * particles.size();
    if (count > 0) {
        std::cout << std::fixed
                  << "[SPH] Avg density: " << (totalDensity / count)
                  << "  Min: " << minDensity
                  << "  Max: " << maxDensity
                  << "  (Rest: " << restDensity << ")\n"
                  << "[SPH] Avg pressure: " << (totalPressure / count)
                  << "  Min: " << minPressure
                  << "  Max: " << maxPressure << '\n'
                  << "[SPH] Total mass: " << totalMass
                  << " for nr of particles:" << particles.size() << '\n';
    }
}


Vec3 SceneFluid::spikyGrad(Vec3 dir, double d) {
    return -dir * (45 / (M_PI * std::pow(smoothingRadius, 6) * d))
           * std::pow((smoothingRadius - d), 2);
}

double SceneFluid::viscosityLapl(double d){
    return (45 / (M_PI * std::pow(smoothingRadius, 5)))*(1 - (d / smoothingRadius));
}


void SceneFluid::computeForces() {
    auto &particles = system.getParticles();

    for (auto *p1 : particles) {
        Vec3 acc_pressure = Vec3(0, 0, 0);
        Vec3 acc_viscosity = Vec3(0, 0, 0);

        for (auto *p2 : particles) {
            if (p1->id == p2->id)
                continue;
            Vec3 dir = p1->pos - p2->pos;
            double d = std::max(dir.norm(), 1e-6);

            if (d > smoothingRadius)
                continue;

            double pij = (p1->pressure / (p1->density * p1->density)) + (p2->pressure / (p2->density * p2->density));
            pij = std::min(pij, 10.0);

            acc_pressure += -p2->mass * pij * spikyGrad(dir, d);
            Vec3 vij = (p2->vel - p1->vel)/(p1->density * p2->density);
            acc_viscosity += viscosity * p2->mass * vij * viscosityLapl(d);
        }
        Vec3 total_acc = acc_pressure + acc_viscosity + Vec3(0, -9.81, 0);
        p1->force = total_acc * p1->mass;
    }
}
