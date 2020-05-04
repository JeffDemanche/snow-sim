#include "scenefile.h"

#include "rapidjson/document.h"

#include <iostream>
#include <fstream>
#include <streambuf>

using namespace std;
using namespace rapidjson;

SceneFile::SceneFile(string sceneFile)
{
    ifstream in(sceneFile.c_str());
    string json((istreambuf_iterator<char>(in)), istreambuf_iterator<char>());

    Document jsonDoc;
    jsonDoc.Parse(json.c_str());

    assert(jsonDoc.IsObject());

    m_object = jsonDoc["object"].GetString();
    m_output = jsonDoc["output"].GetString();
    m_particles = jsonDoc.HasMember("particles") ? jsonDoc["particles"].GetInt() : defaultParticles();
    m_frames = jsonDoc.HasMember("frames") ? jsonDoc["frames"].GetInt() : defaultFrames();
    m_stepLength = jsonDoc.HasMember("stepLength") ? jsonDoc["step_length"].GetFloat() : defaultStepLength();
    m_debugStepTimes = jsonDoc.HasMember("debugStepTimes") ? jsonDoc["debugStepTimes"].GetBool() : false;
    m_debugParticles = jsonDoc.HasMember("debugParticles") ? jsonDoc["debugParticles"].GetInt() : 0;

    if(jsonDoc.HasMember("grid")) {
        Value gridObj = jsonDoc["grid"].GetObject();

        float gridSpacing = gridObj["gridSpacing"].GetFloat();
        if (!gridObj.HasMember("gridMin") || !gridObj.HasMember("gridMax")) {
            m_gridInfo = GridInfo {
                Vector3f::Zero(), Vector3f::Zero(), gridSpacing
            };
        }
        else {
            Vector3f gridMin(gridObj["gridMin"].GetArray()[0].GetFloat(),
                             gridObj["gridMin"].GetArray()[1].GetFloat(),
                             gridObj["gridMin"].GetArray()[2].GetFloat());
            Vector3f gridMax(gridObj["gridMax"].GetArray()[0].GetFloat(),
                             gridObj["gridMax"].GetArray()[1].GetFloat(),
                             gridObj["gridMax"].GetArray()[2].GetFloat());
            m_gridInfo = GridInfo {
                gridMin, gridMax, gridSpacing
            };
        }
    } else {
        m_gridInfo = defaultGridInfo();
    }
}

string SceneFile::getObject() {
    return m_object;
}

string SceneFile::getOutput() {
    return m_output;
}

int SceneFile::getParticles() {
    return m_particles;
}

int SceneFile::defaultParticles() {
    return 1000;
}

int SceneFile::getFrames() {
    return m_frames;
}

int SceneFile::defaultFrames() {
    return 60;
}

float SceneFile::getStepLength() {
    return m_stepLength;
}

float SceneFile::defaultStepLength() {
    return 1.0 / 24.0;
}

bool SceneFile::getDebugStepTimes() {
    return m_debugStepTimes;
}

int SceneFile::getDebugParticles() {
    return m_debugParticles;
}

GridInfo SceneFile::getGridInfo() {
    return m_gridInfo;
}

GridInfo SceneFile::defaultGridInfo() {
    return GridInfo {
        Vector3f::Zero(), Vector3f::Zero(), 0.035
    };
}
