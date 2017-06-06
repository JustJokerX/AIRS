#ifndef DEWIDGET_H
#define DEWIDGET_H
#include "oglwidget.h"

class DEWidget : public OGLWidget
{
public:
    virtual void InitializePhysics() override;
    virtual void ShutdownPhysics() override;
            void CreateObjects();
};

#endif // DEWIDGET_H
