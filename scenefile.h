#ifndef SCENEFILE_H
#define SCENEFILE_H

#include <string>
#include <Eigen/Core>

using namespace Eigen;
using namespace std;

struct GridInfo {
    Vector3f gridMin;
    Vector3f gridMax;
    float gridSpacing;
};



class SceneFile
{
public:
    SceneFile(string sceneFile);

    string getObject();
    string getOutput();
    int getParticles();
    static int defaultParticles();
    int getFrames();
    static int defaultFrames();
    float getStepLength();
    static float defaultStepLength();
    bool getDebugStepTimes();
    int getDebugParticles();

    GridInfo getGridInfo();
    static GridInfo defaultGridInfo();

private:
    string m_object;
    string m_output;
    int m_particles;
    int m_frames;
    float m_stepLength;
    bool m_debugStepTimes;
    int m_debugParticles;

    GridInfo m_gridInfo;

};

#endif // SCENEFILE_H
