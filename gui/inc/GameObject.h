#ifndef _GAMEOBJECT_H_
#define _GAMEOBJECT_H_

#include <bullet/btBulletDynamicsCommon.h>
#include "GLInstanceGraphicsShape.h"
class GameObject {
public:
    GameObject(btCollisionShape* pShape, float mass, const btVector3 &color, const btVector3 &initialPosition = btVector3(0,0,0),
               const btQuaternion &initialRotation = btQuaternion(0,0,1,1), GLInstanceGraphicsShape* pGLShape= nullptr);
    ~GameObject();

    // accessors
    btCollisionShape* GetShape() { return m_pShape; }

    GLInstanceGraphicsShape* GetGLShape(){ return m_pGLShape;}

    btRigidBody* GetRigidBody() { return m_pBody; }

    btMotionState* GetMotionState() { return m_pMotionState; }

    void GetTransform(btScalar* transform);

    btVector3 GetColor() { return m_color; }

    void SetColor(const btVector3 &color) { m_color = color; }

    bool bindGLShape(GLInstanceGraphicsShape* pGLShape, bool bDelOldShape= true);

    bool SetGLShapeRender(bool bGLShapeRender= true);

    bool GetGLShapeRender() { return m_bGLShapeRender; }

protected:
    btCollisionShape*  m_pShape;
    GLInstanceGraphicsShape *  m_pGLShape;
    btRigidBody*  m_pBody;
    btDefaultMotionState*  m_pMotionState;
    btVector3     m_color;
    bool          m_bGLShapeRender;
};
#endif
