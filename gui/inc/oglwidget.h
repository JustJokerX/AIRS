#ifndef OGLWIDGET_H
#define OGLWIDGET_H
#include <vector>
#include <QDebug>
#include <QWidget>
#include <QKeyEvent>
#include <QOpenGLWidget>
#include <QtGui/QPainter>
#include <GL/glu.h>
#include <GL/gl.h>
#include <bullet/btBulletDynamicsCommon.h>
#include <bullet/btBulletCollisionCommon.h>
#include <bullet/BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <bullet/BulletCollision/CollisionShapes/btShapeHull.h>

// include our custom Motion State object
#include "GameObject.h"
#include "DebugDrawer.h"

#include <set>
#include <iterator>
#include <algorithm>

// a convenient typedef to reference an STL vector of GameObjects
typedef std::vector<GameObject*> GameObjects;

// convenient typedefs for collision events
typedef std::pair<const btRigidBody*, const btRigidBody*> CollisionPair;
typedef std::set<CollisionPair> CollisionPairs;

// struct to store our raycasting results
struct RayResult{
    btRigidBody* pBody;
    btVector3 hitPoint;
};

class OGLWidget : public QOpenGLWidget
{
public:
    OGLWidget(QWidget *parent = 0);
    ~OGLWidget();
    // camera functions
    virtual void keyPressEvent(QKeyEvent *event);           //处理键盘按下事件
    virtual void keyReleaseEvent(QKeyEvent *event);         //处理键盘弹起事件
    virtual void mousePressEvent(QMouseEvent *event);       //鼠标按下事件
    virtual void mouseMoveEvent(QMouseEvent *event);        //鼠标移动事件
    virtual void mouseReleaseEvent(QMouseEvent *event);     //鼠标释放事件

    // do init
    virtual void initializeGL();

    // rendering. Can be overrideen by derived classes
    virtual void RenderScene();

    // scene updating. Can be overridden by derived classes
    virtual void UpdateScene(float dt);

    virtual void resizeGL(int w, int h);
    virtual void paintGL();

    // physics functions. Can be overriden by derived classes (like DeWidget)
    virtual void InitializePhysics() {}
    virtual void ShutdownPhysics() {}




    // camera functions
    void UpdateCamera();
    void RotateCamera(float &angle, float value);
    void ZoomCamera(float distance);

    // drawing functions
    void DrawBox(const btVector3 &halfSize);
    void DrawSphere(const btScalar &radius);
    void DrawCylinder(const btScalar &radius, const btScalar &halfHeight);
    void DrawConvexHull(const btCollisionShape* shape);
    void DrawMesh(const GLInstanceGraphicsShape* shape);
    void DrawShape(btScalar* transform, const btCollisionShape* pShape, const btVector3 &color);
    void DrawMeshShape(btScalar* transform, const GLInstanceGraphicsShape* pGLShape, const btVector3 &color);
    // object functions
    GameObject *CreateGameObject(btCollisionShape* pShape,
            const float &mass,
            const btVector3 &color = btVector3(1.0f,1.0f,1.0f),
            const btVector3 &initialPosition = btVector3(0.0f,0.0f,0.0f),
            const btQuaternion &initialRotation = btQuaternion(0,0,1,0),
             GLInstanceGraphicsShape *pGLShape = nullptr);

    void ShootBox(const btVector3 &direction);
    void DestroyGameObject(btRigidBody* pBody);
    GameObject* FindGameObject(btRigidBody* pBody);

    // picking functions
    btVector3 GetPickingRay(int x, int y);

    //bool Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output);
    bool Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output, bool includeStatic = false);

    // constraint functions
    void CreatePickingConstraint(int x, int y);
    void RemovePickingConstraint();
    virtual void Motion(int x, int y);

    // collision event functions
    void CheckForCollisionEvents();
    virtual void CollisionEvent(btRigidBody* pBody0, btRigidBody * pBody1);
    virtual void SeparationEvent(btRigidBody * pBody0, btRigidBody * pBody1);

protected:
    float m_cameraDistance; // distance from the camera to its target
    float m_cameraPitch; // pitch of the camera
    float m_cameraYaw; // yaw of the camera

    bool m_Light;

    //bool m_fullscreen;

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

    // a simple clock for counting time
    btClock m_clock;
    // an array of our game objects
    GameObjects m_objects;

    // debug renderer
    DebugDrawer* m_pDebugDrawer;

    // constraint variables
    btRigidBody* m_pPickedBody;				// the body we picked up
    btTypedConstraint*  m_pPickConstraint;	// the constraint the body is attached to
    btScalar m_oldPickingDist;				// the distance from the camera to the hit point (so we can move the object up, down, left and right from our view)

    // collision event variables
    CollisionPairs m_pairsLastUpdate;

    // pick bool
    bool m_bpick;
    // pick point
    float m_pick_x;
    float m_pick_y;

    // mouse now point
    float m_move_x;
    float m_move_y;

    // fps
    float m_fps;

    // 1/fps
    float m_1_fps;

    // now fps
    float FPS;
};

#endif // OGLWIDGET_H
