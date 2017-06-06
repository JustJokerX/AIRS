#ifndef OGLWIDGET_H
#define OGLWIDGET_H
#include <QDebug>
#include <QWidget>
#include <QKeyEvent>
#include <QOpenGLWidget>
#include <GL/glu.h>
#include <GL/gl.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/btBulletCollisionCommon.h>
// include our custom Motion State object
#include "openglmotionstate.h"

#include "GameObject.h"
#include <vector>

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;

class OGLWidget : public QOpenGLWidget
{
public:
    OGLWidget(QWidget *parent = 0);
    ~OGLWidget();
    // camera functions
    void keyPressEvent(QKeyEvent *event);           //处理键盘按下事件
    void initializeGL();

    // rendering. Can be overrideen by derived classes
    virtual void RenderScene();

    // scene updating. Can be overridden by derived classes
    virtual void UpdateScene(float dt);

    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    // physics functions. Can be overriden by derived classes (like BasicDemo)
    virtual void InitializePhysics() {}
    virtual void ShutdownPhysics() {}

    // camera functions
    void UpdateCamera();
    void RotateCamera(float &angle, float value);
    void ZoomCamera(float distance);

    // drawing functions
    void DrawBox(const btVector3 &halfSize);
    void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color);

    // object functions
    GameObject *CreateGameObject(btCollisionShape* pShape,
            const float &mass,
            const btVector3 &color = btVector3(1.0f,1.0f,1.0f),
            const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f),
            const btQuaternion &initialRotation = btQuaternion(0,0,1,1));
protected:
    float m_cameraDistance; // distance from the camera to its target
    float m_cameraPitch; // pitch of the camera
    float m_cameraYaw; // yaw of the camera

    bool m_Light;

    // camera control
    btVector3 m_cameraPosition; // the camera's current position
    btVector3 m_cameraTarget;	 // the camera's lookAt target
    float m_nearPlane; // minimum distance the camera will render
    float m_farPlane; // farthest distance the camera will render
    btVector3 m_upVector; // keeps the camera rotated correctly

    int m_screenWidth;
    int m_screenHeight;

    // core Bullet components
    btBroadphaseInterface* m_pBroadphase;
    btCollisionConfiguration* m_pCollisionConfiguration;
    btCollisionDispatcher* m_pDispatcher;
    btConstraintSolver* m_pSolver;
    btDynamicsWorld* m_pWorld;

    // our custom motion state
//    OpenGLMotionState* m_pMotionState;

    // a simple clock for counting time
    btClock m_clock;

    // an array of our game objects
    GameObjects m_objects;
};

#endif // OGLWIDGET_H
