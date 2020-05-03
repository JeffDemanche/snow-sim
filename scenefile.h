#ifndef SCENEFILE_H
#define SCENEFILE_H

#include <string>

using namespace std;

class SceneFile
{
public:
    SceneFile(string sceneFile);

    string getObject();
    string getOutput();
    int getParticles();
    int getFrames();
    float getStepLength();
    bool getDebugStepTimes();
    int getDebugParticles();

private:
    string m_object;
    string m_output;
    int m_particles;
    int m_frames;
    float m_stepLength;
    bool m_debugStepTimes;
    int m_debugParticles;

};

#endif // SCENEFILE_H
