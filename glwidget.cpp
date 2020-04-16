#include "glwidget.h"
#include <QApplication>
#include "util/resourceloader.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <iostream>
#include "gl/shaders/ShaderAttribLocations.h"

#define PI 3.14159265f

GLWidget::GLWidget(QGLFormat format, QWidget *parent)
    : QGLWidget(format, parent), m_angleX(0), m_angleY(0.5f), m_zoom(10.f)
{
    m_maxParticles = 2500;
    m_groundHeight = -2.f;
    srand(time(NULL));

    QStringList args = QApplication::arguments();
    if (args.length() < 2) {
        cerr << "Error: Wrong number of arguments" << endl;
    }
    QString infile = args[1];
    int numParticles = args[2].toInt();
    Mesh m;
    m.loadFromFile(infile.toStdString());

    MPM mpm = MPM(m, numParticles);
    m_MPM = mpm;

    // The game loop is implemented using a timer
    connect(&m_timer, SIGNAL(timeout()), this, SLOT(tick()));
}

GLWidget::~GLWidget()
{
    for (int i = 0; i < m_points.size(); i++) {
        delete m_points[i];
    }
}

void GLWidget::initializeGL() {
    ResourceLoader::initializeGlew();
    resizeGL(width(), height());

    glEnable(GL_DEPTH_TEST);
    //glEnable(GL_CULL_FACE);

    // Set the color to set the screen when the color buffer is cleared.
    glClearColor(240/255.f, 240/255.f, 240/255.f, 0.0f);

    m_program = ResourceLoader::createShaderProgram("shaders/shader.vert", "shaders/shader.frag");
    initGround();

    m_particleProgram = ResourceLoader::createShaderProgram("shaders/particle.vert", "shaders/particle.frag");
    initPoints(m_MPM.getPositions());

    initGrid(m_MPM.getGridBounds());

    rebuildMatrices();

    m_time.start();
    m_timer.start(1000 / 60);
}

void GLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Bind shader program.
    glUseProgram(m_program);

    // Set uniforms.
    glUniformMatrix4fv(glGetUniformLocation(m_program, "model"), 1, GL_FALSE, glm::value_ptr(m_model));
    glUniformMatrix4fv(glGetUniformLocation(m_program, "view"), 1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_program, "projection"), 1, GL_FALSE, glm::value_ptr(m_projection));

    // Draw
    m_ground->draw();
    // Unbind shader program.
    glUseProgram(0);

    glUseProgram(m_particleProgram);
    // Set uniforms.
    glUniformMatrix4fv(glGetUniformLocation(m_particleProgram, "model"), 1, GL_FALSE, glm::value_ptr(m_model));
    glUniformMatrix4fv(glGetUniformLocation(m_particleProgram, "view"), 1, GL_FALSE, glm::value_ptr(m_view));
    glUniformMatrix4fv(glGetUniformLocation(m_particleProgram, "projection"), 1, GL_FALSE, glm::value_ptr(m_projection));
    for (int i = 0; i < m_points.size(); i++) {
        m_points[i]->draw();
    }

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    m_grid->draw();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glUseProgram(0);
}

void GLWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
    rebuildMatrices();
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
    m_prevMousePos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
    m_angleX += 10 * (event->x() - m_prevMousePos.x()) / (float) width();
    m_angleY += 10 * (event->y() - m_prevMousePos.y()) / (float) height();
    m_prevMousePos = event->pos();
    rebuildMatrices();
}

void GLWidget::wheelEvent(QWheelEvent *event) {
    m_zoom -= event->delta() / 100.f;
    rebuildMatrices();
}

void GLWidget::rebuildMatrices() {
    m_model = glm::mat4(1.f);
    m_view = glm::translate(glm::vec3(0, 0, -m_zoom)) *
             glm::rotate(m_angleY, glm::vec3(1,0,0)) *
             glm::rotate(m_angleX, glm::vec3(0,1,0));
    m_projection = glm::perspective(0.8f, (float)width()/height(), 0.1f, 100.f);
    update();
}

void GLWidget::initGround() {
    int numVertices = 4;
    std::vector<glm::vec3> data(numVertices);

    data[0] = glm::vec3(5, m_groundHeight, 5);
    data[1] = glm::vec3(5, m_groundHeight, -5);
    data[2] = glm::vec3(-5, m_groundHeight, 5);
    data[3] = glm::vec3(-5, m_groundHeight, -5);

    m_ground = std::make_unique<OpenGLShape>();
    m_ground->setVertexData(&data[0][0], data.size() * 3, VBO::GEOMETRY_LAYOUT::LAYOUT_TRIANGLE_STRIP, numVertices);
    m_ground->setAttribute(ShaderAttrib::POSITION, 3, 0, VBOAttribMarker::DATA_TYPE::FLOAT, false);
    m_ground->buildVAO();
}

void GLWidget::initPoints(std::vector<Eigen::Vector3f> positions) {
    m_points.clear();

    std::vector<Eigen::Vector3f> renderablePoints;
    // If there are more particles than the visualizer can handle
    if (positions.size() > m_maxParticles) {
        int step = positions.size() / m_maxParticles;
        // then step through and uniformly sample the maximum number of points
        for (int i = 0; i < m_maxParticles; i++) {
            if (true) {
                renderablePoints.push_back(positions[i*step]);
            }
        }
    } else {
        renderablePoints = positions;
    }

    int numVertices = 3;
    std::vector<glm::vec3> data(numVertices);
    float halfWidth = 0.01;
    float halfHeight = 0.01;

    for (int i = 0; i < renderablePoints.size(); i++) {
        Eigen::Vector3f point = positions[i];
        data[0] = glm::vec3(point(0) + halfWidth, point(1) - halfHeight, point(2));
        data[1] = glm::vec3(point(0) + halfWidth, point(1) + halfHeight, point(2));
        data[2] = glm::vec3(point(0) - halfWidth, point(1) - halfHeight, point(2));

        OpenGLShape* tri = new OpenGLShape;
        tri->setVertexData(&data[0][0], data.size() * 3, VBO::GEOMETRY_LAYOUT::LAYOUT_TRIANGLE_STRIP, numVertices);
        tri->setAttribute(ShaderAttrib::POSITION, 3, 0, VBOAttribMarker::DATA_TYPE::FLOAT, false);
        tri->buildVAO();

        m_points.push_back(tri);
    }
}

void GLWidget::initGrid(std::pair<Eigen::Vector3f, Eigen::Vector3f> gridBounds) {
    int numTriangles = 8;
    std::vector<glm::vec3> data(numTriangles*3);
    Eigen::Vector3f min = gridBounds.first;
    Eigen::Vector3f max = gridBounds.second;

    glm::vec3 v1 = glm::vec3(min.x(), min.y(), min.z());
    glm::vec3 v2 = glm::vec3(max.x(), min.y(), min.z());
    glm::vec3 v3 = glm::vec3(min.x(), max.y(), min.z());
    glm::vec3 v4 = glm::vec3(max.x(), max.y(), min.z());
    glm::vec3 v5 = glm::vec3(min.x(), min.y(), max.z());
    glm::vec3 v6 = glm::vec3(max.x(), min.y(), max.z());
    glm::vec3 v7 = glm::vec3(max.x(), max.y(), max.z());
    glm::vec3 v8 = glm::vec3(min.x(), max.y(), max.z());

    // Top face
    data[0] = v8;
    data[1] = v4;
    data[2] = v7;
    data[3] = v3;
    data[4] = v4;
    data[5] = v8;

    // Right face
    data[6] = v4;
    data[7] = v7;
    data[8] = v6;
    data[9] = v6;
    data[10] = v2;
    data[11] = v4;

    // Bottom face
    data[12] = v1;
    data[13] = v2;
    data[14] = v6;
    data[15] = v6;
    data[16] = v5;
    data[17] = v1;

    // Left face
    data[18] = v8;
    data[19] = v3;
    data[20] = v5;
    data[21] = v1;
    data[22] = v3;
    data[23] = v5;

//    // Back face
//    data[24] = v1;
//    data[25] = v2;
//    data[26] = v3;
//    data[27] = v3;
//    data[28] = v2;
//    data[29] = v4;

//    // Front face
//    data[30] = v8;
//    data[31] = v6;
//    data[32] = v5;
//    data[33] = v8;
//    data[34] = v7;
//    data[35] = v6;

    m_grid = std::make_unique<OpenGLShape>();
    m_grid->setVertexData(&data[0][0], data.size() * 3, VBO::GEOMETRY_LAYOUT::LAYOUT_TRIANGLES, numTriangles*3);
    m_grid->setAttribute(ShaderAttrib::POSITION, 3, 0, VBOAttribMarker::DATA_TYPE::FLOAT, false);
    m_grid->buildVAO();
}

void GLWidget::tick()
{
    float seconds = m_time.restart() * 0.001f;
    std::vector<Eigen::Vector3f> newPoints = m_MPM.update(seconds);
    initPoints(newPoints);
    update();
}

