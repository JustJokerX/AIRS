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
//    // create a box shape of size (1,1,1)
//    btBoxShape* pBoxShape = new btBoxShape(btVector3(1.0f, 1.0f, 1.0f));
//    // give our box an initial position of (0,0,0)
//    btTransform transform;
//    transform.setIdentity();
//    transform.setOrigin(btVector3(0.0f, 0.0f, 0.0f));
//    // create a motion state
//    m_pMotionState = new OpenGLMotionState(transform);
//    // create the rigid body construction info object, giving it a
//    // mass of 1, the motion state, and the shape
//    btRigidBody::btRigidBodyConstructionInfo rbInfo(1.0f, m_pMotionState, pBoxShape);
//    btRigidBody* pRigidBody = new btRigidBody(rbInfo);
//    // inform our world that we just created a new rigid body for
//    // it to manage
//    m_pWorld->addRigidBody(pRigidBody);

// create a ground plane
CreateGameObject(new btBoxShape(btVector3(1,100,100)), 0, btVector3(0.2f, 0.6f, 0.6f), btVector3(0.0f, 0.0f, 0.0f));

//// create our original red box
//CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(0.0f, 5.0f, 0.0f));

//// create a blue box
//CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 10.0f, 0.0f));

//// create a green box
//CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.2f, 0.8f, 0.2f), btVector3(0.75f, 15.0f, 0.0f));

for(int i=0;i<10;++i){
    for(int j=0;j<10;++j){
        for(int k=0;k<10;++k){
            if((i+j+k)%2==0){
                CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(-5.0f+2*i, 1000.0f+2*j, 0.0f+2*k));
            }else{
                CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.2f, 0.2f, 1.0f), btVector3(-5.0f+2*i, 1000.0f+2*j, 0.0f+2*k));
            }

        }
    }
}

}
