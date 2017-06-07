#include "dewidget.h"

void DEWidget::InitializePhysics() {
    // create the collision configuration
    m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
    // create the dispatcher
    m_pDispatcher = new btCollisionDispatcher(m_pCollisionConfiguration);
    // create the broadphase
    m_pBroadphase = new btDbvtBroadphase();
    // create the constraint solver
    m_pSolver = new btSequentialImpulseConstraintSolver();
    // create the world
    m_pWorld = new btDiscreteDynamicsWorld(m_pDispatcher, m_pBroadphase, m_pSolver, m_pCollisionConfiguration);

    // create our scene's physics objects
    CreateObjects();
}

void DEWidget::ShutdownPhysics() {
    delete m_pWorld;
    delete m_pSolver;
    delete m_pBroadphase;
    delete m_pDispatcher;
    delete m_pCollisionConfiguration;
}

void DEWidget::CreateObjects()
{

// create a ground plane
CreateGameObject(new btBoxShape(btVector3(50,1,50)), 0, btVector3(0.2f, 0.6f, 0.6f), btVector3(0.0f, 0.0f, 0.0f));

// create our original red box
CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(0.0f, 5.0f, 0.0f));

// create a blue box
CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 10.0f, 0.0f));

// create a green box
CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.2f, 0.8f, 0.2f), btVector3(0.75f, 15.0f, 0.0f));

}
