#include "oglwidget.h"

// Some constants for 3D math and the camera speed
#define RADIANS_PER_DEGREE 0.01745329f
#define CAMERA_STEP_SIZE 5.0f

OGLWidget::OGLWidget(QWidget *parent)
    : QOpenGLWidget(parent),
    m_cameraPosition(0.0f, 5.0f, 0.0f),
    m_cameraTarget(0.0f, 0.0f, 0.0f),
    m_cameraDistance(20.0f),
    m_cameraPitch(20.0f),
    m_cameraYaw(0.0f),
    m_upVector(0.0f, 1.0f, 0.0f),
    m_nearPlane(1.0f),
    m_farPlane(1000.0f),
    m_Light(false),
    m_pBroadphase(0),
    m_pCollisionConfiguration(0),
    m_pDispatcher(0),
    m_pSolver(0),
    m_pWorld(0),
    m_fullscreen(false)
{

}

OGLWidget::~OGLWidget()
{
    // shutdown the physics system
    ShutdownPhysics();
}



void OGLWidget::initializeGL()
{
    makeCurrent();
    // create some floats for our ambient, diffuse, specular and position
    GLfloat ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f }; // dark grey
    GLfloat diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
    GLfloat specular[] = { 1.0f, 1.0f, 1.0f, 1.0f }; // white
    GLfloat position[] = { 5.0f, 10.0f, 1.0f, 0.0f };

    // set the ambient, diffuse, specular and position for LIGHT0
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular);
    glLightfv(GL_LIGHT0, GL_POSITION, position);

    glEnable(GL_LIGHTING); // enables lighting
    glEnable(GL_LIGHT0); // enables the 0th light
    glEnable(GL_COLOR_MATERIAL); // colors materials when lighting is enabled

    // enable specular lighting via materials
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular);
    glMateriali(GL_FRONT, GL_SHININESS, 15);

    // enable smooth shading
    glShadeModel(GL_SMOOTH);

    // enable depth testing to be 'less than'
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    // doneCurrent();
    // set the backbuffer clearing color to a lightish blue
    glClearColor(0.6, 0.65, 0.85, 0);
    // initialize the physics system
    InitializePhysics();
	// create the debug drawer
    m_pDebugDrawer = new DebugDrawer();
    // set the initial debug level to 0
    m_pDebugDrawer->setDebugMode(0);
    // add the debug drawer to the world
    m_pWorld->setDebugDrawer(m_pDebugDrawer);
}

void OGLWidget::paintGL()
{
    // this function is called frequently, whenever FreeGlut
    // isn't busy processing its own events. It should be used
    // to perform any updating and rendering tasks

    // clear the backbuffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // get the time since the last iteration
    float dt = m_clock.getTimeMilliseconds();
    // reset the clock to 0
    m_clock.reset();
    // update the scene (convert ms to s)
    UpdateScene(dt / 100.0f);
    // update the camera
    UpdateCamera();

//    // draw a simple box of size 1
//    // also draw it red
//    DrawBox(btVector3(1, 1, 1), btVector3(1.0f, 0.2f, 0.2f));

    // render the scene
    RenderScene();
    update();
    //parentWidget()->update();
}



void OGLWidget::resizeGL(int w, int h)
{
    m_screenWidth = w;
    m_screenHeight = h;
    // exit in erroneous situations
    if (m_screenWidth == 0 && m_screenHeight == 0)
        return;

    // select the projection matrix
    glMatrixMode(GL_PROJECTION);
    // set it to the matrix-equivalent of 1
    glLoadIdentity();
    // determine the aspect ratio of the screen
    float aspectRatio = m_screenWidth / (float)m_screenHeight;
    // create a viewing frustum based on the aspect ratio and the
    // boundaries of the camera
    glFrustum (-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
    // the projection matrix is now set

    // select the view matrix
    glMatrixMode(GL_MODELVIEW);
    // set it to '1'
    glLoadIdentity();
    // create a view matrix based on the camera's position and where it's
    // looking
    gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], m_cameraTarget[0], m_cameraTarget[1], m_cameraTarget[2], m_upVector.getX(), m_upVector.getY(), m_upVector.getZ());
    // the view matrix is now set
}

void OGLWidget::UpdateCamera()
{
    // exit in erroneous situations
    if (m_screenWidth == 0 && m_screenHeight == 0)
        return;

    // select the projection matrix
    glMatrixMode(GL_PROJECTION);
    // set it to the matrix-equivalent of 1
    glLoadIdentity();
    // determine the aspect ratio of the screen
    float aspectRatio = m_screenWidth / (float)m_screenHeight;
    // create a viewing frustum based on the aspect ratio and the
    // boundaries of the camera
    glFrustum (-aspectRatio * m_nearPlane, aspectRatio * m_nearPlane, -m_nearPlane, m_nearPlane, m_nearPlane, m_farPlane);
    // the projection matrix is now set

    // select the view matrix
    glMatrixMode(GL_MODELVIEW);
    // set it to '1'
    glLoadIdentity();

    // our values represent the angles in degrees, but 3D
    // math typically demands angular values are in radians.
    float pitch = m_cameraPitch * RADIANS_PER_DEGREE;
    float yaw = m_cameraYaw * RADIANS_PER_DEGREE;

    // create a quaternion defining the angular rotation
    // around the up vector
    btQuaternion rotation(m_upVector, yaw);

    // set the camera's position to 0,0,0, then move the 'z'
    // position to the current value of m_cameraDistance.
    btVector3 cameraPosition(0,0,0);
    cameraPosition[2] = -m_cameraDistance;

    // create a Bullet Vector3 to represent the camera
    // position and scale it up if its value is too small.
    btVector3 forward(cameraPosition[0], cameraPosition[1], cameraPosition[2]);
    if (forward.length2() < SIMD_EPSILON) {
        forward.setValue(1.f,0.f,0.f);
    }

    // figure out the 'right' vector by using the cross
    // product on the 'forward' and 'up' vectors
    btVector3 right = m_upVector.cross(forward);

    // create a quaternion that represents the camera's roll
    btQuaternion roll(right, - pitch);

    // turn the rotation (around the Y-axis) and roll (around
    // the forward axis) into transformation matrices and
    // apply them to the camera position. This gives us the
    // final position
    cameraPosition = btMatrix3x3(rotation) * btMatrix3x3(roll) * cameraPosition;

    // save our new position in the member variable, and
    // shift it relative to the target position (so that we
    // orbit it)
    m_cameraPosition[0] = cameraPosition.getX();
    m_cameraPosition[1] = cameraPosition.getY();
    m_cameraPosition[2] = cameraPosition.getZ();
    m_cameraPosition += m_cameraTarget;
    // create a view matrix based on the camera's position and where it's
    // looking
    gluLookAt(m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], m_cameraTarget[0], m_cameraTarget[1], m_cameraTarget[2], m_upVector.getX(), m_upVector.getY(), m_upVector.getZ());
    // the view matrix is now set
}

void OGLWidget::RotateCamera(float &angle, float value)
{
    // change the value (it is passed by reference, so we
    // can edit it here)
    angle -= value;
    // keep the value within bounds
    if (angle < 0) angle += 360;
    if (angle >= 360) angle -= 360;
    // update the camera since we changed the angular value
    UpdateCamera();
}

void OGLWidget::ZoomCamera(float distance)
{
    // change the distance value
    m_cameraDistance -= distance;
    // prevent it from zooming in too far
    if (m_cameraDistance < 0.1f) m_cameraDistance = 0.1f;
    // update the camera since we changed the zoom distance
    UpdateCamera();
}


void OGLWidget::keyPressEvent(QKeyEvent *event)
{
    switch (event->key())
    {
    case Qt::Key_L:                                     //L为开启关闭光源的切换键
        m_Light = !m_Light;
        if (m_Light)
        {
            glEnable(GL_LIGHTING);                      //开启光源
        }
        else
        {
            glDisable(GL_LIGHTING);                     //关闭光源
        }
        break;

    //OpenGL Widget fullscreen
    case Qt::Key_F2:
        m_fullscreen=!m_fullscreen;
        if (m_fullscreen){
            setWindowFlags(Qt::Window);
            showFullScreen();
        } else{
            setWindowFlags(Qt::SubWindow);
            showNormal();
        }
        break;

    case Qt::Key_Z:
        ZoomCamera(+CAMERA_STEP_SIZE);
        break;

    case Qt::Key_X:
        ZoomCamera(-CAMERA_STEP_SIZE);
        break;

    case Qt::Key_W:
        m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawWireframe);
        break;

    case Qt::Key_B:
        m_pDebugDrawer->ToggleDebugFlag(btIDebugDraw::DBG_DrawAabb);
        break;

    case Qt::Key_Left:
        RotateCamera(m_cameraYaw, +CAMERA_STEP_SIZE);
        break;

    case Qt::Key_Right:
        RotateCamera(m_cameraYaw, -CAMERA_STEP_SIZE);
        break;

    case Qt::Key_Up:
        RotateCamera(m_cameraPitch, +CAMERA_STEP_SIZE);
        break;

    case Qt::Key_Down:
        RotateCamera(m_cameraPitch, -CAMERA_STEP_SIZE);
        break;

    }
}


//void OGLWidget::DrawBox(const btVector3 &halfSize, const btVector3 &color) {
void OGLWidget::DrawBox(const btVector3 &halfSize) {

    // push the transform onto the stack
    //glPushMatrix();
    //glMultMatrixf(transform);

    float halfWidth = halfSize.x();
    float halfHeight = halfSize.y();
    float halfDepth = halfSize.z();

    // set the object's color

    //glColor3f(color.x(), color.y(), color.z());

    // create the vertex positions
    btVector3 vertices[8]={
    btVector3(halfWidth,halfHeight,halfDepth),
    btVector3(-halfWidth,halfHeight,halfDepth),
    btVector3(halfWidth,-halfHeight,halfDepth),
    btVector3(-halfWidth,-halfHeight,halfDepth),
    btVector3(halfWidth,halfHeight,-halfDepth),
    btVector3(-halfWidth,halfHeight,-halfDepth),
    btVector3(halfWidth,-halfHeight,-halfDepth),
    btVector3(-halfWidth,-halfHeight,-halfDepth)};

    // create the indexes for each triangle, using the
    // vertices above. Make it static so we don't waste
    // processing time recreating it over and over again
    static int indices[36] = {
        0,1,2,
        3,2,1,
        4,0,6,
        6,0,2,
        5,1,4,
        4,1,0,
        7,3,1,
        7,1,5,
        5,4,7,
        7,4,6,
        7,2,3,
        7,6,2};

    // start processing vertices as triangles
    glBegin (GL_TRIANGLES);

    // increment the loop by 3 each time since we create a
    // triangle with 3 vertices at a time.

    for (int i = 0; i < 36; i += 3) {
        // get the three vertices for the triangle based
        // on the index values set above
        // use const references so we don't copy the object
        // (a good rule of thumb is to never allocate/deallocate
        // memory during *every* render/update call. This should
        // only happen sporadically)
        const btVector3 &vert1 = vertices[indices[i]];
        const btVector3 &vert2 = vertices[indices[i+1]];
        const btVector3 &vert3 = vertices[indices[i+2]];

        // create a normal that is perpendicular to the
        // face (use the cross product)
        btVector3 normal = (vert3-vert1).cross(vert2-vert1);
        normal.normalize ();

        // set the normal for the subsequent vertices
        glNormal3f(normal.getX(),normal.getY(),normal.getZ());

        // create the vertices
        glVertex3f (vert1.x(), vert1.y(), vert1.z());
        glVertex3f (vert2.x(), vert2.y(), vert2.z());
        glVertex3f (vert3.x(), vert3.y(), vert3.z());
    }

    // stop processing vertices
    glEnd();
    // pop the transform from the stack in preparation
    // for the next object
    //glPopMatrix();
}



void OGLWidget::RenderScene() {
    // create an array of 16 floats (representing a 4x4 matrix)
    btScalar transform[16];
    // iterate through all of the objects in our world
    for(GameObjects::iterator i = m_objects.begin(); i != m_objects.end(); ++i) {
        // get the object from the iterator
        GameObject* pObj = *i;

        // read the transform
        pObj->GetTransform(transform);

        // get data from the object and draw it
        DrawShape(transform, pObj->GetShape(), pObj->GetColor());
    }

    // after rendering all game objects, perform debug rendering
    // Bullet will figure out what needs to be drawn then call to
    // our DebugDrawer class to do the rendering for us
    m_pWorld->debugDrawWorld();
}

void OGLWidget::UpdateScene(float dt) {
    // check if the world object exists
    if (m_pWorld) {
        // step the simulation through time. This is called
        // every update and the amount of elasped time was
        // determined back in ::paintGL() by our clock object.
        m_pWorld->stepSimulation(dt);
    }
}

void OGLWidget::DrawShape(btScalar *transform, const btCollisionShape *pShape, const btVector3 &color)
{
    // set the color
    glColor3f(color.x(), color.y(), color.z());

    // push the matrix stack
    glPushMatrix();
    glMultMatrixf(transform);

    // make a different draw call based on the object type
    switch(pShape->getShapeType()) {
        // an internal enum used by Bullet for boxes
    case BOX_SHAPE_PROXYTYPE:
    {
        // assume the shape is a box, and typecast it
        const btBoxShape* box = static_cast<const btBoxShape*>(pShape);
        // get the 'halfSize' of the box
        btVector3 halfSize = box->getHalfExtentsWithMargin();
        // draw the box
        DrawBox(halfSize);
        break;
    }
    default:
        // unsupported type
        break;
    }

    // pop the stack
    glPopMatrix();
}

GameObject *OGLWidget::CreateGameObject(btCollisionShape *pShape, const float &mass, const btVector3 &color, const btVector3 &initialPosition, const btQuaternion &initialRotation)
{
    GameObject* pObject = new GameObject(pShape, mass, color, initialPosition, initialRotation);

    // push it to the back of the list
    m_objects.push_back(pObject);

    // check if the world object is valid
    if (m_pWorld) {
        // add the object's rigid body to the world
        m_pWorld->addRigidBody(pObject->GetRigidBody());
    }
    return pObject;
}
