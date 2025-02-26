#pragma once

#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QVector3D>
#include <QQuaternion>
#include <QTimer>
#include <memory>

/**
 * @brief 3D visualization widget for the UAV
 * 
 * This widget provides a 3D visualization of the UAV using OpenGL.
 * It shows the aircraft model, attitude, and position in 3D space.
 */
class Flight3DView : public QOpenGLWidget {
    Q_OBJECT

public:
    explicit Flight3DView(QWidget *parent = nullptr);
    ~Flight3DView();
    
    /**
     * Update the aircraft state
     * @param position Array of 3 doubles [north, east, down]
     * @param orientation Array of 3 doubles [roll, pitch, yaw] in radians
     */
    void updateAircraftState(const double position[3], const double orientation[3]);

protected:
    // OpenGL initialization and rendering
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int width, int height) override;
    
    // Mouse interaction
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    // Aircraft state
    QVector3D m_aircraftPosition{0.0f, 0.0f, 0.0f};
    QQuaternion m_aircraftOrientation{};
    
    // Camera state
    QVector3D m_cameraPosition{0.0f, -10.0f, 2.0f};
    QVector3D m_cameraTarget{0.0f, 0.0f, 0.0f};
    QVector3D m_cameraUp{0.0f, 0.0f, 1.0f};
    float m_cameraFOV{45.0f};
    
    // Mouse interaction
    QPoint m_lastMousePosition{};
    bool m_rotating{false};
    bool m_panning{false};
    
    // Rendering matrices
    QMatrix4x4 m_modelMatrix{};
    QMatrix4x4 m_viewMatrix{};
    QMatrix4x4 m_projectionMatrix{};
    
    // Helper methods
    void updateMatrices();
    void drawAircraft();
    void drawGrid();
    void drawAxes();
    
    // Color helpers
    QVector3D colorToVector(const QColor& color) const;
}; 