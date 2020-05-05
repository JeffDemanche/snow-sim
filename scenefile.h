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
    Vector3f initialVelocity;
    Vector3f gravity;
    float groundHeight;
};

struct HyperparameterInfo {
    float criticalCompression;
    float criticalStretch;
    float youngsModulus;
    float hardeningCoefficient;
    float poissonsRatio;
    float density;
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
    static GridInfo defaultGridInfo() {
        return GridInfo {
            Vector3f::Zero(), Vector3f::Zero(), 0.035, Vector3f(0, 0, 0), Vector3f(0, 0, 0), -0.5
        };
    };

    HyperparameterInfo getHyperparameterInfo();
    static HyperparameterInfo defaultHyperparameterInfo() {
        return HyperparameterInfo {
            2.5E-2, 7.5E-3, 1.4E5, 10, 0.2, 400
        };
    };

private:
    string m_object;
    string m_output;
    int m_particles;
    int m_frames;
    float m_stepLength;
    bool m_debugStepTimes;
    int m_debugParticles;

    GridInfo m_gridInfo;
    HyperparameterInfo m_hyperparameterInfo;

};

#endif // SCENEFILE_H
