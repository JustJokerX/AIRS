#ifndef DEWIDGET_H
#define DEWIDGET_H
#include "oglwidget.h"

class DEWidget : public OGLWidget
{
public:
    virtual void InitializePhysics() override;
    virtual void ShutdownPhysics() override;
            void CreateObjects();
    virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;
protected:
    // our box to lift
    GameObject* m_pBox;

    // a simple trigger volume
    btCollisionObject* m_pTrigger;
};

#endif // DEWIDGET_H
