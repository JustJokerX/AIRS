#include <iostream>
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
    m_fullscreen(false),
    m_pick_x(0),
    m_pick_y(0),
    m_move_x(0),
    m_move_y(0),
    m_fps(60.0),
    m_1_fps(0.16666),
    m_pPickedBody(0),
    m_pPickConstraint(0),
    m_bpick(false)
{
    this->setMouseTracking(true);
    m_1_fps=1.0f/m_fps;
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
    float dt = m_clock.getTimeMicroseconds()/1.0e6; //turns us to s

    // reset the clock to 0
    if(dt>m_1_fps){
        // update the scene (convert ms to s)
        UpdateScene(dt);
        FPS=1./dt;
        m_clock.reset();
    }

    // update the camera
    UpdateCamera();

    // render the scene
    RenderScene();

    glDisable(GL_DEPTH_TEST);
    QPainter painter;
    painter.begin(this);
    QPen pen;
    pen.setColor(Qt::red);
    painter.setPen(pen);

    painter.drawText(10,20,"FPS:"+QString::number(FPS,'f',1)+"|"+QString::number(m_fps,'f',1));

    painter.end();
    glEnable(GL_DEPTH_TEST);

    update();
    //std::cout<<"update"<<std::endl;
    //parentWidget()->update();
    //std::cout<<"parent->update"<<std::endl;

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
    // boundaries of the cameara
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

void OGLWidget::mouseMoveEvent(QMouseEvent *event) {
    m_move_x=event->pos().x();
    m_move_y=event->pos().y();
    // Only effective when it is picked
    Motion(m_move_x,m_move_y);
}

void OGLWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button()==Qt::RightButton){
        ShootBox(GetPickingRay(event->pos().x(),event->pos().y()));
    }
    else if(event->button()==Qt::LeftButton){
        // create the picking constraint when we click the LMB
        CreatePickingConstraint(event->pos().x(),event->pos().y());
        m_bpick = true;
    }
}

void OGLWidget::mouseReleaseEvent(QMouseEvent *event) {
    if((event->button()==Qt::LeftButton) and (true == m_bpick) ){
        // remove the picking constraint when we release the LMB
        RemovePickingConstraint();
        m_bpick= false;
    }
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

    //OpenGL Widget fullscreen // have some bugs
//    case Qt::Key_F2:
//        m_fullscreen=!m_fullscreen;
//        if (m_fullscreen){
//            setWindowFlags(Qt::Window);
//            showFullScreen();
//        }
//        else{
//            setWindowFlags(Qt::SubWindow);
//            showNormal();
//        }
//        break;

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

    case Qt::Key_D:
        {
            // create a temp object to store the raycast result
            RayResult result;
            // perform the raycast
            if (!Raycast(m_cameraPosition,GetPickingRay(m_move_x,m_move_y),result))
                return;
            // destroy the corresponding game object
            DestroyGameObject(result.pBody);
            break;
        }
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
        int sub_step=m_pWorld->stepSimulation(dt,1,m_1_fps);
        // std::cout<<"substeps:"<<sub_step<<std::endl;
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

void OGLWidget::ShootBox(const btVector3 &direction) {
// create a new box object
    GameObject* pObject = CreateGameObject(new btBoxShape(btVector3(1, 1, 1)), 1, btVector3(0.4f, 0.f, 0.4f), m_cameraPosition);

// calculate the velocity
    btVector3 velocity = direction;
    velocity.normalize();
    velocity *= 25.0f;

// set the linear velocity of the box
    pObject->GetRigidBody()->setLinearVelocity(velocity);
}

void OGLWidget::DestroyGameObject(btRigidBody *pBody) {
// we need to search through the objects in order to
// find the corresponding iterator (can only erase from
// an std::vector by passing an iterator)
    for (GameObjects::iterator iter = m_objects.begin(); iter != m_objects.end(); ++iter) {
        if ((*iter)->GetRigidBody() == pBody) {
            GameObject* pObject = *iter;
            // remove the rigid body from the world
            m_pWorld->removeRigidBody(pObject->GetRigidBody());
            // erase the object from the list
            m_objects.erase(iter);
            // delete the object from memory
            delete pObject;
            // done
            return;
        }
    }
}

btVector3 OGLWidget::GetPickingRay(int x, int y) {
    // calculate the field-of-view
    float tanFov = 1.0f / m_nearPlane;
    float fov = btScalar(2.0) * btAtan(tanFov);
// get a ray pointing forward from the
// camera and extend it to the far plane
    btVector3 rayFrom = m_cameraPosition;
    btVector3 rayForward = (m_cameraTarget - m_cameraPosition);
    rayForward.normalize();
    rayForward*= m_farPlane;
// find the horizontal and vertical vectors
// relative to the current camera view
    btVector3 ver = m_upVector;
    btVector3 hor = rayForward.cross(ver);
    hor.normalize();
    ver = hor.cross(rayForward);
    ver.normalize();
    hor *= 2.f * m_farPlane * tanFov;
    ver *= 2.f * m_farPlane * tanFov;
// calculate the aspect ratio
    btScalar aspect = m_screenWidth / (btScalar)m_screenHeight;
// adjust the forward-ray based on
// the X/Y coordinates that were clicked
    hor*=aspect;
    btVector3 rayToCenter = rayFrom + rayForward;
    btVector3 dHor = hor * 1.f/float(m_screenWidth);
    btVector3 dVert = ver * 1.f/float(m_screenHeight);
    btVector3 rayTo = rayToCenter - 0.5f * hor + 0.5f * ver;
    rayTo += btScalar(x) * dHor;
    rayTo -= btScalar(y) * dVert;
// return the final result
    return rayTo;
}

bool OGLWidget::Raycast(const btVector3 &startPosition, const btVector3 &direction, RayResult &output) {
    if (!m_pWorld)
        return false;

// get the picking ray from where we clicked
    btVector3 rayTo = direction;
    btVector3 rayFrom = m_cameraPosition;

// create our raycast callback object
    btCollisionWorld::ClosestRayResultCallback rayCallback(rayFrom,rayTo);

// perform the raycast
    m_pWorld->rayTest(rayFrom,rayTo,rayCallback);

// did we hit something?
    if (rayCallback.hasHit())
    {
        // if so, get the rigid body we hit
        btRigidBody* pBody = (btRigidBody*)btRigidBody::upcast(rayCallback.m_collisionObject);
        if (!pBody)
            return false;

        // prevent us from picking objects
        // like the ground plane
        if (pBody->isStaticObject() || pBody->isKinematicObject())
            return false;

        // set the result data
        output.pBody = pBody;
        output.hitPoint = rayCallback.m_hitPointWorld;
        return true;
    }
// we didn't hit anything
    return false;
}

void OGLWidget::CreatePickingConstraint(int x, int y) {
    if (!m_pWorld)
        return;
    // perform a raycast and return if it fails
    RayResult output;
    if (!Raycast(m_cameraPosition, GetPickingRay(x, y), output))
        return;
    // store the body for future reference
    m_pPickedBody = output.pBody;
    // prevent the picked object from falling asleep
    m_pPickedBody->setActivationState(DISABLE_DEACTIVATION);
    // get the hit position relative to the body we hit
    btVector3 localPivot = m_pPickedBody->getCenterOfMassTransform().inverse() * output.hitPoint;
    // create a transform for the pivot point
    btTransform pivot;
    pivot.setIdentity();
    pivot.setOrigin(localPivot);
    // create our constraint object
    btGeneric6DofConstraint* dof6 = new btGeneric6DofConstraint(*m_pPickedBody, pivot, true);
    bool bLimitAngularMotion = true;
    if (bLimitAngularMotion) {
        dof6->setAngularLowerLimit(btVector3(0,0,0));
        dof6->setAngularUpperLimit(btVector3(0,0,0));
    }
    // add the constraint to the world
    m_pWorld->addConstraint(dof6,true);
    // store a pointer to our constraint
    m_pPickConstraint = dof6;
    // define the 'strength' of our constraint (each axis)
    float cfm = 0.5f;
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,0);
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,1);
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,2);
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,3);
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,4);
    dof6->setParam(BT_CONSTRAINT_STOP_CFM,cfm,5);
    // define the 'error reduction' of our constraint (each axis)
    float erp = 0.5f;
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,0);
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,1);
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,2);
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,3);
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,4);
    dof6->setParam(BT_CONSTRAINT_STOP_ERP,erp,5);
    // save this data for future reference
    m_oldPickingDist  = (output.hitPoint - m_cameraPosition).length();
}

void OGLWidget::RemovePickingConstraint() {
    // exit in erroneous situations
    if (!m_pPickConstraint || !m_pWorld)
        return;
    // remove the constraint from the world
    m_pWorld->removeConstraint(m_pPickConstraint);
    // delete the constraint object
    delete m_pPickConstraint;
    // reactivate the body
    m_pPickedBody->forceActivationState(ACTIVE_TAG);
    m_pPickedBody->setDeactivationTime( 0.f );
    // clear the pointers
    m_pPickConstraint = 0;
}


void OGLWidget::Motion(int x, int y) {
    // did we pick a body with the LMB?
    if (m_pPickedBody) {
        btGeneric6DofConstraint* pickCon = static_cast<btGeneric6DofConstraint*>(m_pPickConstraint);
        if (!pickCon)
            return;

        // use another picking ray to get the target direction
        btVector3 dir = GetPickingRay(x,y) - m_cameraPosition;
        dir.normalize();

        // use the same distance as when we originally picked the object
        dir *= m_oldPickingDist;
        btVector3 newPivot = m_cameraPosition + dir;

        // set the position of the constraint
        pickCon->getFrameOffsetA().setOrigin(newPivot);
    }
}


