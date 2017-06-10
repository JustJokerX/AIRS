#include "GameObject.h"
GameObject::GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition, const btQuaternion &initialRotation,GLInstanceGraphicsShape* pGLShape) {
    // store the shape for later usage
    m_pShape = pShape;

    // store the shape for GL graphics usage
    m_pGLShape = pGLShape;

    // whether to render th GL graphics shape
    m_bGLShapeRender=false;

    // store the color
    m_color = color;

    // create the initial transform
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(initialPosition);
    transform.setRotation(initialRotation);

    // create the motion state from the
    // initial transform
    m_pMotionState = new btDefaultMotionState(transform);
    // calculate the local inertia
    btVector3 localInertia(0,0,0);

    // objects of infinite mass can't
    // move or rotate
    if (mass != 0.0f)
        pShape->calculateLocalInertia(mass, localInertia);

    // create the rigid body construction
    // info using the mass, motion state
    // and shape
    btRigidBody::btRigidBodyConstructionInfo cInfo(mass, m_pMotionState, pShape, localInertia);

    // create the rigid body
    m_pBody = new btRigidBody(cInfo);
}

GameObject::~GameObject() {
    delete m_pBody;
    delete m_pMotionState;
    delete m_pShape;
    delete m_pGLShape;
}

void GameObject::GetTransform(btScalar *transform) {
    if(m_pMotionState) {
        btTransform trans; // used to maintain orientation and position
        m_pMotionState->getWorldTransform(trans); // right multiply,NOTE: OldState * transform = NewState
        trans.getOpenGLMatrix(transform); //Do nothing weired,but turns the data to OpenGLMatrix type
    }
}

bool GameObject::bindGLShape(GLInstanceGraphicsShape *pGLShape , bool bDelOldShape) {
    if (nullptr == pGLShape){
        b3Warning("Error: Binding GLInstanceGraphicsShape with NULLPTR\n");
        return false;
    }
    if (nullptr == m_pGLShape){
        b3Printf("[INFO] GL shape bind success\n");
        m_pGLShape = pGLShape;
    }else{
        if (false == bDelOldShape){
            b3Printf("[INFO] GL shape bind success, replacing without delete the old shape\n");
            m_pGLShape = pGLShape;
        } else {
            b3Printf("[INFO] GL shape bind success, replacing and delete the old shape\n");
            delete (m_pGLShape);
            m_pGLShape = pGLShape;
        }
    }
    return true;
}

bool GameObject::SetGLShapeRender(bool bGLShapeRender) {
    if (nullptr == m_pGLShape){
        b3Warning("Error: set GL shape render failed\n");
        return false;
    }else{
        m_bGLShapeRender = bGLShapeRender;
    }
    return true;
}
