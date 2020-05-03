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
    m_particles = jsonDoc.HasMember("particles") ? jsonDoc["particles"].GetInt() : 1000;
    m_frames = jsonDoc.HasMember("frames") ? jsonDoc["frames"].GetInt() : 60;
    m_stepLength = jsonDoc.HasMember("stepLength") ? jsonDoc["step_length"].GetFloat() : 1.0 / 24.0;
    m_debugStepTimes = jsonDoc.HasMember("debugStepTimes") ? jsonDoc["debugStepTimes"].GetBool() : false;
    m_debugParticles = jsonDoc.HasMember("debugParticles") ? jsonDoc["debugParticles"].GetInt() : 0;
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

int SceneFile::getFrames() {
    return m_frames;
}

float SceneFile::getStepLength() {
    return m_stepLength;
}

bool SceneFile::getDebugStepTimes() {
    return m_debugStepTimes;
}

int SceneFile::getDebugParticles() {
    return m_debugParticles;
}
