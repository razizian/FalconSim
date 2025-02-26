#include "Flight3DView.hpp"

#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>
#include <cmath>

Flight3DView::Flight3DView(QWidget *parent)
    : QOpenGLWidget(parent)
{
    // Set focus policy to enable keyboard input
    setFocusPolicy(Qt::StrongFocus);
    
    // Enable mouse tracking for smooth camera movement
    setMouseTracking(true);
}

Flight3DView::~Flight3DView()
{
    // Clean up OpenGL resources if needed
}

void Flight3DView::updateAircraftState(const double position[3], const double orientation[3])
{
    // Convert double arrays to Qt types
    m_aircraftPosition = QVector3D(
        static_cast<float>(position[0]),    // North
        static_cast<float>(position[1]),    // East
        static_cast<float>(-position[2])    // Down -> Up (invert for OpenGL)
    );
    
    // Convert Euler angles to quaternion
    // Note: Order matters here - we're converting from aerospace conventions (roll, pitch, yaw)
    // to OpenGL coordinate system
    QQuaternion rollQuat = QQuaternion::fromAxisAndAngle(
        QVector3D(1.0f, 0.0f, 0.0f),
        static_cast<float>(orientation[0] * 180.0f / M_PI)
    );
    
    QQuaternion pitchQuat = QQuaternion::fromAxisAndAngle(
        QVector3D(0.0f, 1.0f, 0.0f),
        static_cast<float>(orientation[1] * 180.0f / M_PI)
    );
    
    QQuaternion yawQuat = QQuaternion::fromAxisAndAngle(
        QVector3D(0.0f, 0.0f, 1.0f),
        static_cast<float>(orientation[2] * 180.0f / M_PI)
    );
    
    // Combine rotations (note order: yaw, pitch, roll)
    m_aircraftOrientation = yawQuat * pitchQuat * rollQuat;
    
    // Update the view to show new position
    update();
}

void Flight3DView::initializeGL()
{
    // Initialize OpenGL functions
    auto *f = QOpenGLContext::currentContext()->functions();
    f->glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
    
    // Enable depth testing for 3D rendering
    f->glEnable(GL_DEPTH_TEST);
    
    // Initialize matrices
    updateMatrices();
}

void Flight3DView::paintGL()
{
    // Get OpenGL functions
    auto *f = QOpenGLContext::currentContext()->functions();
    
    // Clear the screen
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Update matrices before drawing
    updateMatrices();
    
    // Draw grid for reference
    drawGrid();
    
    // Draw coordinate axes
    drawAxes();
    
    // Draw aircraft
    drawAircraft();
}

void Flight3DView::resizeGL(int width, int height)
{
    // Update projection matrix for new aspect ratio
    float aspect = static_cast<float>(width) / static_cast<float>(height);
    m_projectionMatrix.setToIdentity();
    m_projectionMatrix.perspective(m_cameraFOV, aspect, 0.1f, 1000.0f);
}

void Flight3DView::mousePressEvent(QMouseEvent *event)
{
    m_lastMousePosition = event->pos();
    
    if (event->button() == Qt::LeftButton) {
        m_rotating = true;
    } else if (event->button() == Qt::RightButton) {
        m_panning = true;
    }
}

void Flight3DView::mouseMoveEvent(QMouseEvent *event)
{
    if (!m_rotating && !m_panning) {
        return;
    }
    
    QPoint delta = event->pos() - m_lastMousePosition;
    m_lastMousePosition = event->pos();
    
    const float rotationSpeed = 0.5f;
    const float panSpeed = 0.01f;
    
    if (m_rotating) {
        // Orbit camera around target
        QVector3D cameraDirection = m_cameraTarget - m_cameraPosition;
        float distance = cameraDirection.length();
        cameraDirection.normalize();
        
        // Horizontal rotation (around up vector)
        QQuaternion yawRotation = QQuaternion::fromAxisAndAngle(
            m_cameraUp,
            -delta.x() * rotationSpeed
        );
        
        // Vertical rotation (around right vector)
        QVector3D right = QVector3D::crossProduct(cameraDirection, m_cameraUp);
        right.normalize();
        QQuaternion pitchRotation = QQuaternion::fromAxisAndAngle(
            right,
            -delta.y() * rotationSpeed
        );
        
        // Apply rotations
        cameraDirection = pitchRotation.rotatedVector(
            yawRotation.rotatedVector(cameraDirection)
        );
        
        // Update camera position while keeping distance
        m_cameraPosition = m_cameraTarget - (cameraDirection * distance);
        
        update();
    } else if (m_panning) {
        // Move target and camera together (panning)
        QVector3D cameraDirection = m_cameraTarget - m_cameraPosition;
        cameraDirection.normalize();
        
        QVector3D right = QVector3D::crossProduct(cameraDirection, m_cameraUp);
        right.normalize();
        
        QVector3D movementX = right * (delta.x() * panSpeed);
        QVector3D movementY = m_cameraUp * (-delta.y() * panSpeed);
        
        QVector3D totalMovement = movementX + movementY;
        
        m_cameraPosition += totalMovement;
        m_cameraTarget += totalMovement;
        
        update();
    }
}

void Flight3DView::wheelEvent(QWheelEvent *event)
{
    // Zoom in/out with mouse wheel
    const float zoomSpeed = 0.001f;
    float zoomFactor = 1.0f + (event->angleDelta().y() * zoomSpeed);
    
    QVector3D cameraDirection = m_cameraTarget - m_cameraPosition;
    float distance = cameraDirection.length();
    
    // Limit minimum and maximum zoom
    distance = qBound(1.0f, distance / zoomFactor, 50.0f);
    
    cameraDirection.normalize();
    m_cameraPosition = m_cameraTarget - (cameraDirection * distance);
    
    update();
}

void Flight3DView::updateMatrices()
{
    // Update view matrix based on camera position
    m_viewMatrix.setToIdentity();
    m_viewMatrix.lookAt(m_cameraPosition, m_cameraTarget, m_cameraUp);
    
    // Update model matrix based on aircraft position and orientation
    m_modelMatrix.setToIdentity();
    m_modelMatrix.translate(m_aircraftPosition);
    m_modelMatrix.rotate(m_aircraftOrientation);
}

void Flight3DView::drawAircraft()
{
    // Basic aircraft model using OpenGL primitives
    auto *f = QOpenGLContext::currentContext()->functions();
    
    // Save current matrix
    f->glPushMatrix();
    
    // Apply model matrix for aircraft position and orientation
    QMatrix4x4 mvMatrix = m_viewMatrix * m_modelMatrix;
    f->glLoadMatrixf(mvMatrix.constData());
    
    // Draw a simple aircraft model
    // This is a simplified aircraft model using basic OpenGL
    // In a real application, you would use proper 3D models
    
    // Body (fuselage) - blue
    f->glColor3f(0.2f, 0.4f, 0.8f);
    f->glBegin(GL_QUADS);
    // Top
    f->glVertex3f(-0.5f, 1.0f, 0.25f);
    f->glVertex3f(0.5f, 1.0f, 0.25f);
    f->glVertex3f(0.5f, -1.0f, 0.25f);
    f->glVertex3f(-0.5f, -1.0f, 0.25f);
    
    // Bottom
    f->glVertex3f(-0.5f, -1.0f, -0.25f);
    f->glVertex3f(0.5f, -1.0f, -0.25f);
    f->glVertex3f(0.5f, 1.0f, -0.25f);
    f->glVertex3f(-0.5f, 1.0f, -0.25f);
    
    // Left
    f->glVertex3f(-0.5f, 1.0f, 0.25f);
    f->glVertex3f(-0.5f, -1.0f, 0.25f);
    f->glVertex3f(-0.5f, -1.0f, -0.25f);
    f->glVertex3f(-0.5f, 1.0f, -0.25f);
    
    // Right
    f->glVertex3f(0.5f, 1.0f, -0.25f);
    f->glVertex3f(0.5f, -1.0f, -0.25f);
    f->glVertex3f(0.5f, -1.0f, 0.25f);
    f->glVertex3f(0.5f, 1.0f, 0.25f);
    
    // Front
    f->glVertex3f(-0.5f, -1.0f, 0.25f);
    f->glVertex3f(0.5f, -1.0f, 0.25f);
    f->glVertex3f(0.5f, -1.0f, -0.25f);
    f->glVertex3f(-0.5f, -1.0f, -0.25f);
    
    // Back
    f->glVertex3f(-0.5f, 1.0f, -0.25f);
    f->glVertex3f(0.5f, 1.0f, -0.25f);
    f->glVertex3f(0.5f, 1.0f, 0.25f);
    f->glVertex3f(-0.5f, 1.0f, 0.25f);
    f->glEnd();
    
    // Wings - gray
    f->glColor3f(0.7f, 0.7f, 0.7f);
    f->glBegin(GL_QUADS);
    // Main wing
    f->glVertex3f(-3.0f, -0.2f, 0.0f);
    f->glVertex3f(3.0f, -0.2f, 0.0f);
    f->glVertex3f(3.0f, 0.2f, 0.0f);
    f->glVertex3f(-3.0f, 0.2f, 0.0f);
    
    // Tail horizontal stabilizer
    f->glVertex3f(-1.0f, 0.9f, 0.0f);
    f->glVertex3f(1.0f, 0.9f, 0.0f);
    f->glVertex3f(1.0f, 1.1f, 0.0f);
    f->glVertex3f(-1.0f, 1.1f, 0.0f);
    f->glEnd();
    
    // Tail vertical stabilizer - red
    f->glColor3f(0.8f, 0.2f, 0.2f);
    f->glBegin(GL_TRIANGLES);
    f->glVertex3f(0.0f, 0.9f, 0.0f);
    f->glVertex3f(0.0f, 1.1f, 0.0f);
    f->glVertex3f(0.0f, 1.0f, 0.5f);
    f->glEnd();
    
    // Restore matrix
    f->glPopMatrix();
}

void Flight3DView::drawGrid()
{
    auto *f = QOpenGLContext::currentContext()->functions();
    
    // Save current matrix
    f->glPushMatrix();
    
    // Apply view matrix
    f->glLoadMatrixf(m_viewMatrix.constData());
    
    // Draw a grid on the ground plane
    f->glColor3f(0.5f, 0.5f, 0.5f);
    f->glBegin(GL_LINES);
    
    const float gridSize = 20.0f;
    const float gridStep = 1.0f;
    
    for (float i = -gridSize; i <= gridSize; i += gridStep) {
        // Lines along North axis
        f->glVertex3f(i, -gridSize, 0.0f);
        f->glVertex3f(i, gridSize, 0.0f);
        
        // Lines along East axis
        f->glVertex3f(-gridSize, i, 0.0f);
        f->glVertex3f(gridSize, i, 0.0f);
    }
    
    f->glEnd();
    
    // Restore matrix
    f->glPopMatrix();
}

void Flight3DView::drawAxes()
{
    auto *f = QOpenGLContext::currentContext()->functions();
    
    // Save current matrix
    f->glPushMatrix();
    
    // Apply view matrix
    f->glLoadMatrixf(m_viewMatrix.constData());
    
    // Draw coordinate axes
    f->glBegin(GL_LINES);
    
    // X axis - red
    f->glColor3f(1.0f, 0.0f, 0.0f);
    f->glVertex3f(0.0f, 0.0f, 0.0f);
    f->glVertex3f(1.0f, 0.0f, 0.0f);
    
    // Y axis - green
    f->glColor3f(0.0f, 1.0f, 0.0f);
    f->glVertex3f(0.0f, 0.0f, 0.0f);
    f->glVertex3f(0.0f, 1.0f, 0.0f);
    
    // Z axis - blue
    f->glColor3f(0.0f, 0.0f, 1.0f);
    f->glVertex3f(0.0f, 0.0f, 0.0f);
    f->glVertex3f(0.0f, 0.0f, 1.0f);
    
    f->glEnd();
    
    // Restore matrix
    f->glPopMatrix();
}

QVector3D Flight3DView::colorToVector(const QColor& color) const
{
    return QVector3D(
        color.redF(),
        color.greenF(),
        color.blueF()
    );
} 