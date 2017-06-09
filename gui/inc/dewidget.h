#ifndef DEWIDGET_H
#define DEWIDGET_H
#include "oglwidget.h"
#include <btBulletDynamicsCommon.h>

#define EXPLOSION_STRENGTH 50.0f

class DEWidget : public OGLWidget
{
public:
    DEWidget();

    virtual void InitializePhysics() override;
    virtual void ShutdownPhysics() override;
            void CreateObjects();

    virtual void UpdateScene(float dt) override;
    virtual void keyPressEvent(QKeyEvent *event) override;
    virtual void keyReleaseEvent(QKeyEvent *event) override;
    virtual void mousePressEvent(QMouseEvent *event) override;
    virtual void mouseMoveEvent(QMouseEvent *event) override;
    virtual void mouseReleaseEvent(QMouseEvent *event) override;



    virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody* pBody1) override;

protected:
    // our box to lift
    GameObject* m_pBox;

    // a simple trigger volume
    btCollisionObject* m_pTrigger;

    // keeps track of whether we're holding down the 'g' key
    bool m_bApplyForce;

    // explosion variables
    btCollisionObject* m_pExplosion;
    bool m_bCanExplode;
};

#endif // DEWIDGET_H
