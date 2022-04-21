#include <World.h>

#include <GL/glut.h>

#include <chrono>
#include <thread>

using namespace utad;

static const float dt = 0.01;
static World world;

int Rand(int min, int max)
{
    return rand() % max + min;
}

void DisplayCallback()
{
    static float t = 0;
    static float tNext = 0;
    if (t >= tNext) {
        // Add soft body
        const float h = world.m_ParticleSystem.m_Radius * 2.0F;

        auto sb = world.CreateSoftBody();
        sb->m_ParticleMass = 1.0F;

        const Vector3f x0(0, 2, 0);
        const Quaternionf q0(Quaternionf::UnitRandom());
        const Vector3f sz(h * Rand(2, 2), h * Rand(2, 2), h * Rand(6, 8));
        //const Vector3f sz(h * Rand(2, 3), h * Rand(2, 2), h * Rand(2, 4));

        initSoftBodyFromCube(sb, x0, q0, sz);

        tNext += 1;
    }

    world.Simulate(dt);

    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    world.Draw();

    glutSwapBuffers();

    if (tNext < 4) {
        t += dt;
    }
}

void SetupView()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0F, 1.333F, 0.1F, 100.0F);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(-0.5F, 1.5F, 3.5F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F);
}

void SetupWorld()
{
    // Do nothing
}

int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(1024, 768);
    glutCreateWindow("UTAD-SIM03-PBD-SoftBody");
    glutDisplayFunc(DisplayCallback);
    glutIdleFunc(DisplayCallback);

    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);

    SetupView();
    SetupWorld();

    glutMainLoop();

    return 0;
}