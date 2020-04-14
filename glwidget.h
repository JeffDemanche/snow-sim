#ifndef GLWIDGET_H
#define GLWIDGET_H
#include "GL/glew.h"
#include <QGLWidget>
#include <QTimer>
#include <QTime>
#include "openglshape.h"
#ifdef __APPLE__
#include <glu.h>
#else
#include <GL/glu.h>
#endif

#define GLM_FORCE_RADIANS
#include "glm/glm.hpp"            // glm::vec*, mat*, and basic glm functions
#include "glm/gtx/transform.hpp"  // glm::translate, scale, rotate
#include "glm/gtc/type_ptr.hpp"   // glm::value_ptr

#include <Eigen/Dense>
#include "mpm.h"


class GLWidget : public QGLWidget {
    Q_OBJECT

public:
    GLWidget(QGLFormat format, QWidget *parent = 0);
    ~GLWidget();

protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int h);
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent *e);

private:
    void rebuildMatrices();
    void initGround();
    void renderPoints();

    /** ID for the shader program. */
    GLuint m_program;
    GLuint m_particleProgram;

    glm::mat4 m_model, m_view, m_projection;

    /** For mouse interaction. */
    float m_angleX, m_angleY, m_zoom;
    QPoint m_prevMousePos;

    std::unique_ptr<OpenGLShape> m_ground;
    float m_groundHeight;

    std::unique_ptr<OpenGLShape> m_grid;
    void initGrid(std::pair<Eigen::Vector3f, Eigen::Vector3f> gridBounds);

    std::vector<glm::vec3> m_samplePoints;
    std::vector<OpenGLShape*> m_points;
    void initPoints(std::vector<Eigen::Vector3f> positions);

    MPM m_MPM;
    QTime m_time;
    QTimer m_timer;

    int m_maxParticles = 2500;

private slots:
    void tick();
};

#endif // GLWIDGET_H
