#include "dewidget.h"
#include "b3ResourcePath.h"
#include "LoadMeshFromSTL.h"
#include "LoadMeshFromObj.h"
#include <Bullet3Common/b3FileUtils.h>
DEWidget::DEWidget():
OGLWidget(),
m_bApplyForce(false),
m_pExplosion(0),
m_bCanExplode(true)
{

}

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
    //CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(0.0f, 5.0f, 0.0f));

    // create a blue box
    //CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 10.0f, 0.0f));

    // create a green box
    //CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.2f, 0.8f, 0.2f), btVector3(0.75f, 15.0f, 0.0f));

    // create our red box, but store the pointer for future usage
    m_pBox = CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(1.0f, 0.2f, 0.2f), btVector3(0.0f, 10.0f, 0.0f));
    // create a second box
    CreateGameObject(new btBoxShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.2f, 0.8f), btVector3(1.25f, 20.0f, 0.0f));
    // create a trigger volume
    m_pTrigger = new btCollisionObject();
    // create a box for the trigger's shape
    m_pTrigger->setCollisionShape(new btBoxShape(btVector3(1,0.25,1)));
    // set the trigger's position
    btTransform triggerTrans;
    triggerTrans.setIdentity();
    triggerTrans.setOrigin(btVector3(0,1.5,0));
    m_pTrigger->setWorldTransform(triggerTrans);
    // flag the trigger to ignore contact responses
    m_pTrigger->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
    // add the trigger to our world
    m_pWorld->addCollisionObject(m_pTrigger);
    // create a yellow sphere
    CreateGameObject(new btSphereShape(1.0f), 1.0, btVector3(0.7f, 0.7f, 0.0f), btVector3(-5.0, 10.0f, 0.0f));
    // create a green cylinder
    CreateGameObject(new btCylinderShape(btVector3(1,1,1)), 1.0, btVector3(0.0f, 0.7f, 0.0f), btVector3(-2, 10.0f, 0.0f));

    // create a vertex cloud defining a square-based pyramid
    btVector3 points[5] = {btVector3(-0.5,1,1),
                           btVector3(-0.5,1,-1),
                           btVector3(-0.5,-1,1),
                           btVector3(-0.5,-1,-1),
                           btVector3(1,0,0)};
    // create our convex hull
    btConvexHullShape* pShape = new btConvexHullShape(&points[0].getX(),5);
    // initialize the object as a polyhedron
    pShape->initializePolyhedralFeatures();
    // create the game object using our convex hull shape
    CreateGameObject(pShape, 1.0, btVector3(1,1,1), btVector3(5, 15, 0));

    // create two shapes for the rod and the load
    btCollisionShape* pRod = new btBoxShape(btVector3(1.5f, 0.2f, 0.2f));
    btCollisionShape* pLoad = new btSphereShape(0.5f);
    // create a transform we'll use to set each object's position
    btTransform trans;
    trans.setIdentity();
    // create our compound shape
    btCompoundShape* pCompound = new btCompoundShape();
    // add the rod
    pCompound->addChildShape(trans, pRod);
    trans.setOrigin(btVector3(-1.75f, 0.0f, 0.0f));
    // add the top load
    pCompound->addChildShape(trans, pLoad);
    trans.setIdentity();
    // add the bottom load
    trans.setOrigin(btVector3(1.75f, 0.0f, 0.0f));
    pCompound->addChildShape(trans, pLoad);
    // create a game object using the compound shape
    CreateGameObject(pCompound, 2.0f, btVector3(0.8,0.4,0.1), btVector3(-4, 10.0f, 0.0f));

    //load stl mesh
    const char* fileName = "wheel.stl";
    char relativeFileName[1024];
    if (b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024))
    {
        char pathPrefix[1024];
        b3FileUtils::extractPath(relativeFileName, pathPrefix, 1024);
    }
    GLInstanceGraphicsShape *glmesh_stl = LoadMeshFromSTL(relativeFileName);
    printf("[INFO] STL loaded: Extracted %d verticed from stl file [%s]\n", glmesh_stl->m_numvertices, fileName);
    const GLInstanceVertex& v_stl = glmesh_stl->m_vertices->at(0);
    btConvexHullShape* pShape_stl = new btConvexHullShape((const btScalar*)(&(v_stl.xyzw[0])), glmesh_stl->m_numvertices, sizeof(GLInstanceVertex));
    float scaling_stl[4] = {10,10,10,1};
    glmesh_stl->m_scaling[0]=scaling_stl[0];
    glmesh_stl->m_scaling[1]=scaling_stl[1];
    glmesh_stl->m_scaling[2]=scaling_stl[2];

    btVector3 localScaling(scaling_stl[0],scaling_stl[1],scaling_stl[2]);
    pShape_stl->setLocalScaling(localScaling);
    // initialize the object as a polyhedron
    pShape_stl->initializePolyhedralFeatures();
    // create the game object using our convex hull shape
    GameObject* pObject_stl =CreateGameObject(pShape_stl, 1.0, btVector3(1,1,1), btVector3(5, 15, 0));
    // bind the game object with GL mesh object
    pObject_stl->bindGLShape(glmesh_stl);
    // set render the glmesh object to be true
    pObject_stl->SetGLShapeRender(true);

    //load obj mesh
    const char* fileName_obj = "teddy.obj";//sphere8.obj";//sponza_closed.obj";//sphere8.obj";
    char relativeFileName_obj[1024];
    if (b3ResourcePath::findResourcePath(fileName_obj, relativeFileName_obj, 1024))
    {
        char pathPrefix[1024];
        b3FileUtils::extractPath(relativeFileName_obj, pathPrefix, 1024);
    }
    GLInstanceGraphicsShape* glmesh_obj = LoadMeshFromObj(relativeFileName_obj, "");
    printf("[INFO] Obj loaded: Extracted %d verticed from obj file [%s]\n", glmesh_obj->m_numvertices, fileName_obj);

    const GLInstanceVertex& v_obj = glmesh_obj->m_vertices->at(0);
    btConvexHullShape* pShape_obj = new btConvexHullShape((const btScalar*)(&(v_obj.xyzw[0])), glmesh_obj->m_numvertices, sizeof(GLInstanceVertex));

    float scaling_obj[4] = {0.1,0.1,0.1,1};

    glmesh_obj->m_scaling[0]=scaling_obj[0];
    glmesh_obj->m_scaling[1]=scaling_obj[1];
    glmesh_obj->m_scaling[2]=scaling_obj[2];
    btVector3 localScaling_obj(scaling_obj[0],scaling_obj[1],scaling_obj[2]);
    pShape_obj->setLocalScaling(localScaling_obj);
    pShape_obj->optimizeConvexHull();
    // initialize the object as a polyhedron
    pShape_obj->initializePolyhedralFeatures();
    // create the game object using our convex hull shape
    GameObject* pObject_obj = CreateGameObject(pShape_obj, 1.0, btVector3(1,1,1), btVector3(0, 5, 0));
    // bind the game object with GL mesh object
    pObject_obj->bindGLShape(glmesh_obj);
    // set render the glmesh object to be true
    pObject_obj->SetGLShapeRender(true);
}

void DEWidget::CollisionEvent(btRigidBody *pBody0, btRigidBody *pBody1) {
    // did the box collide with the trigger?
    if ((pBody0 == m_pBox->GetRigidBody() && pBody1 == m_pTrigger) || (pBody1 == m_pBox->GetRigidBody() && pBody0 == m_pTrigger)) {
        // if yes, create a big green box nearby
        CreateGameObject(new btBoxShape(btVector3(2,2,2)), 2.0, btVector3(0.3, 0.7, 0.3), btVector3(5, 10, 0));
    }

    // Impulse testing
    if (pBody0 == m_pExplosion || pBody1 == m_pExplosion) {
        // get the pointer of the other object
        btRigidBody* pOther;
        pBody0 == m_pExplosion ? pOther = (btRigidBody*)pBody1 : pOther = (btRigidBody*)pBody0;
        // wake the object up
        pOther->setActivationState(ACTIVE_TAG);
        // calculate the vector between the object and
        // the center of the explosion
        btVector3 dir = pOther->getWorldTransform().getOrigin() - m_pExplosion->getWorldTransform().getOrigin();
        // get the distance
        float dist = dir.length();
        // calculate the impulse strength
        float strength = EXPLOSION_STRENGTH;
        // follow an inverse-distance rule
        if (dist != 0.0) strength /= dist;
        // normalize the direction vector
        dir.normalize();
        // apply the impulse
        pOther->applyCentralImpulse(dir * strength);
    }
}



void DEWidget::keyPressEvent(QKeyEvent *event) {
    OGLWidget::keyPressEvent(event);

    switch (event->key()) {

        case Qt::Key_G:
        {
            // if 'g' is held down, apply a force
            m_bApplyForce = true;
            // prevent the box from deactivating
            m_pBox->GetRigidBody()->setActivationState(DISABLE_DEACTIVATION);
            break;
        }

        case Qt::Key_E:
        {
            // don't create a new explosion if one already exists
            // or we haven't released the key, yet
            if (m_pExplosion || !m_bCanExplode) break;
            // don't let us create another explosion until the key is released
            m_bCanExplode = false;
            // create a collision object for our explosion
            m_pExplosion = new btCollisionObject();
            m_pExplosion->setCollisionShape(new btSphereShape(3.0f));
            // get the position that we clicked
            RayResult result;
            Raycast(m_cameraPosition, GetPickingRay(m_move_x, m_move_y), result, true);
            // create a transform from the hit point
            btTransform explodeTrans;
            explodeTrans.setIdentity();
            explodeTrans.setOrigin(result.hitPoint);
            m_pExplosion->setWorldTransform(explodeTrans);
            // set the collision flag
            m_pExplosion->setCollisionFlags(btCollisionObject::CF_NO_CONTACT_RESPONSE);
            // add the explosion trigger to our world
            m_pWorld->addCollisionObject(m_pExplosion);
            break;
        }

    }
}

void DEWidget::keyReleaseEvent(QKeyEvent *event) {
    OGLWidget::keyReleaseEvent(event);
    switch (event->key()){
        case Qt::Key_G:
        {
            // if 'g' is let go, stop applying the force
            m_bApplyForce = false;
            // allow the object to deactivate again
            m_pBox->GetRigidBody()->forceActivationState(ACTIVE_TAG);
            break;
        }

        case Qt::Key_E:
        {
            m_bCanExplode = true;
            break;
        }

    }

}

void DEWidget::UpdateScene(float dt) {
    OGLWidget::UpdateScene(dt);
    // Force testing
    if (m_bApplyForce) {
        if (!m_pBox) return;
        // apply a central upwards force that exceeds gravity
        m_pBox->GetRigidBody()->applyCentralForce(btVector3(0, 20, 0));
    }
    // Impulse testing
    if (m_pExplosion and m_bCanExplode == true) {
        // destroy the explosion object after one iteration
        m_pWorld->removeCollisionObject(m_pExplosion);
        delete m_pExplosion;
        m_pExplosion = 0;
    }
}


void DEWidget::mousePressEvent(QMouseEvent *event) {
    OGLWidget::mousePressEvent(event);
}

void DEWidget::mouseMoveEvent(QMouseEvent *event) {
    OGLWidget::mouseMoveEvent(event);
}

void DEWidget::mouseReleaseEvent(QMouseEvent *event) {
    OGLWidget::mouseReleaseEvent(event);
}



