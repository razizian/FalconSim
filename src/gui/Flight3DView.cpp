#include "Flight3DView.hpp"

#include <QOpenGLFunctions>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QColor>
#include <QOpenGLVersionFunctionsFactory>
#include <cmath>

Flight3DView::Flight3DView(QWidget *parent)
    : QOpenGLWidget(parent), m_glFunctions(nullptr)
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
    m_glFunctions = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(QOpenGLContext::currentContext());
    if (!m_glFunctions) {
        qWarning("Could not obtain OpenGL 2.1 functions");
        return;
    }
    
    m_glFunctions->initializeOpenGLFunctions();
    
    // Set clear color (dark blue-gray)
    m_glFunctions->glClearColor(0.2f, 0.2f, 0.3f, 1.0f);
    
    // Enable depth testing for 3D rendering
    m_glFunctions->glEnable(GL_DEPTH_TEST);
    
    // Initialize matrices
    updateMatrices();
}

void Flight3DView::paintGL()
{
    if (!m_glFunctions) return;

    // Clear the color and depth buffers
    m_glFunctions->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Update matrices based on current state
    updateMatrices();
    
    // Set up view and projection matrices
    m_glFunctions->glMatrixMode(GL_PROJECTION);
    m_glFunctions->glLoadMatrixf(m_projectionMatrix.constData());
    
    m_glFunctions->glMatrixMode(GL_MODELVIEW);
    m_glFunctions->glLoadMatrixf(m_viewMatrix.constData());
    
    // Draw the aircraft, grid, and axes
    drawGrid();
    drawAxes();
    drawAircraft();
}

void Flight3DView::resizeGL(int width, int height)
{
    if (!m_glFunctions) return;

    // Update viewport
    m_glFunctions->glViewport(0, 0, width, height);
    
    // Update projection matrix with new aspect ratio
    float aspect = static_cast<float>(width) / static_cast<float>(height);
    m_projectionMatrix.setToIdentity();
    m_projectionMatrix.perspective(m_cameraFOV, aspect, 0.1f, 100.0f);
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

void Flight3DView::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        m_rotating = false;
    } else if (event->button() == Qt::RightButton) {
        m_panning = false;
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
    if (!m_glFunctions) return;
    
    // Basic aircraft model using OpenGL primitives
    // Save current matrix
    m_glFunctions->glPushMatrix();
    
    // Apply model matrix for aircraft position and orientation
    QMatrix4x4 mvMatrix = m_viewMatrix * m_modelMatrix;
    m_glFunctions->glLoadMatrixf(mvMatrix.constData());
    
    // Draw a simple aircraft model
    // This is a simplified aircraft model using basic OpenGL
    // In a real application, you would use proper 3D models
    
    // Body (fuselage) - blue
    m_glFunctions->glColor3f(0.2f, 0.4f, 0.8f);
    m_glFunctions->glBegin(GL_QUADS);
    // Top
    m_glFunctions->glVertex3f(-0.5f, 1.0f, 0.25f);
    m_glFunctions->glVertex3f(0.5f, 1.0f, 0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, 0.25f);
    m_glFunctions->glVertex3f(-0.5f, -1.0f, 0.25f);
    
    // Bottom
    m_glFunctions->glVertex3f(-0.5f, -1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, 1.0f, -0.25f);
    m_glFunctions->glVertex3f(-0.5f, 1.0f, -0.25f);
    
    // Left
    m_glFunctions->glVertex3f(-0.5f, 1.0f, 0.25f);
    m_glFunctions->glVertex3f(-0.5f, -1.0f, 0.25f);
    m_glFunctions->glVertex3f(-0.5f, -1.0f, -0.25f);
    m_glFunctions->glVertex3f(-0.5f, 1.0f, -0.25f);
    
    // Right
    m_glFunctions->glVertex3f(0.5f, 1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, 0.25f);
    m_glFunctions->glVertex3f(0.5f, 1.0f, 0.25f);
    
    // Front
    m_glFunctions->glVertex3f(-0.5f, -1.0f, 0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, 0.25f);
    m_glFunctions->glVertex3f(0.5f, -1.0f, -0.25f);
    m_glFunctions->glVertex3f(-0.5f, -1.0f, -0.25f);
    
    // Back
    m_glFunctions->glVertex3f(-0.5f, 1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, 1.0f, -0.25f);
    m_glFunctions->glVertex3f(0.5f, 1.0f, 0.25f);
    m_glFunctions->glVertex3f(-0.5f, 1.0f, 0.25f);
    m_glFunctions->glEnd();
    
    // Wings - gray
    m_glFunctions->glColor3f(0.7f, 0.7f, 0.7f);
    m_glFunctions->glBegin(GL_QUADS);
    // Main wing
    m_glFunctions->glVertex3f(-3.0f, -0.2f, 0.0f);
    m_glFunctions->glVertex3f(3.0f, -0.2f, 0.0f);
    m_glFunctions->glVertex3f(3.0f, 0.2f, 0.0f);
    m_glFunctions->glVertex3f(-3.0f, 0.2f, 0.0f);
    
    // Tail horizontal stabilizer
    m_glFunctions->glVertex3f(-1.0f, 0.9f, 0.0f);
    m_glFunctions->glVertex3f(1.0f, 0.9f, 0.0f);
    m_glFunctions->glVertex3f(1.0f, 1.1f, 0.0f);
    m_glFunctions->glVertex3f(-1.0f, 1.1f, 0.0f);
    m_glFunctions->glEnd();
    
    // Tail vertical stabilizer - red
    m_glFunctions->glColor3f(0.8f, 0.2f, 0.2f);
    m_glFunctions->glBegin(GL_TRIANGLES);
    m_glFunctions->glVertex3f(0.0f, 0.9f, 0.0f);
    m_glFunctions->glVertex3f(0.0f, 1.1f, 0.0f);
    m_glFunctions->glVertex3f(0.0f, 1.0f, 0.5f);
    m_glFunctions->glEnd();
    
    // Restore matrix
    m_glFunctions->glPopMatrix();
}

void Flight3DView::drawGrid()
{
    if (!m_glFunctions) return;

    // Draw a reference grid
    m_glFunctions->glPushMatrix();
    
    // Set grid color (light gray)
    m_glFunctions->glColor3f(0.7f, 0.7f, 0.7f);
    
    // Draw grid lines
    m_glFunctions->glBegin(GL_LINES);
    
    // Grid size and spacing
    const float gridSize = 10.0f;
    const float gridStep = 1.0f;
    
    // Draw north-south lines (along X axis)
    for (float x = -gridSize; x <= gridSize; x += gridStep) {
        m_glFunctions->glVertex3f(x, -gridSize, 0.0f);
        m_glFunctions->glVertex3f(x, gridSize, 0.0f);
    }
    
    // Draw east-west lines (along Y axis)
    for (float y = -gridSize; y <= gridSize; y += gridStep) {
        m_glFunctions->glVertex3f(-gridSize, y, 0.0f);
        m_glFunctions->glVertex3f(gridSize, y, 0.0f);
    }
    
    m_glFunctions->glEnd();
    m_glFunctions->glPopMatrix();
}

void Flight3DView::drawAxes()
{
    if (!m_glFunctions) return;

    // Draw coordinate axes
    m_glFunctions->glPushMatrix();
    
    // X axis - red (North)
    m_glFunctions->glColor3f(1.0f, 0.0f, 0.0f);
    m_glFunctions->glBegin(GL_LINES);
    m_glFunctions->glVertex3f(0.0f, 0.0f, 0.0f);
    m_glFunctions->glVertex3f(1.0f, 0.0f, 0.0f);
    m_glFunctions->glEnd();
    
    // Y axis - green (East)
    m_glFunctions->glColor3f(0.0f, 1.0f, 0.0f);
    m_glFunctions->glBegin(GL_LINES);
    m_glFunctions->glVertex3f(0.0f, 0.0f, 0.0f);
    m_glFunctions->glVertex3f(0.0f, 1.0f, 0.0f);
    m_glFunctions->glEnd();
    
    // Z axis - blue (Up)
    m_glFunctions->glColor3f(0.0f, 0.0f, 1.0f);
    m_glFunctions->glBegin(GL_LINES);
    m_glFunctions->glVertex3f(0.0f, 0.0f, 0.0f);
    m_glFunctions->glVertex3f(0.0f, 0.0f, 1.0f);
    m_glFunctions->glEnd();
    
    m_glFunctions->glPopMatrix();
}

QVector3D Flight3DView::colorToVector(const QColor& color) const
{
    return QVector3D(
        color.redF(),
        color.greenF(),
        color.blueF()
    );
} 