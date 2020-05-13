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
    m_stepLength = jsonDoc.HasMember("stepLength") ? jsonDoc["stepLength"].GetFloat() : defaultStepLength();
    m_debugStepTimes = jsonDoc.HasMember("debugStepTimes") ? jsonDoc["debugStepTimes"].GetBool() : false;
    m_debugParticles = jsonDoc.HasMember("debugParticles") ? jsonDoc["debugParticles"].GetInt() : 0;

    if(jsonDoc.HasMember("grid")) {
        Value gridObj = jsonDoc["grid"].GetObject();

        Vector3f gridMin(gridObj["gridMin"].GetArray()[0].GetFloat(),
                         gridObj["gridMin"].GetArray()[1].GetFloat(),
                         gridObj["gridMin"].GetArray()[2].GetFloat());
        Vector3f gridMax(gridObj["gridMax"].GetArray()[0].GetFloat(),
                         gridObj["gridMax"].GetArray()[1].GetFloat(),
                         gridObj["gridMax"].GetArray()[2].GetFloat());
        float gridSpacing = gridObj["gridSpacing"].GetFloat();
        Vector3f initialVelocity(gridObj["initialVelocity"].GetArray()[0].GetFloat(),
                                 gridObj["initialVelocity"].GetArray()[1].GetFloat(),
                                 gridObj["initialVelocity"].GetArray()[2].GetFloat());
        Vector3f gravity(gridObj["gravity"].GetArray()[0].GetFloat(),
                         gridObj["gravity"].GetArray()[1].GetFloat(),
                         gridObj["gravity"].GetArray()[2].GetFloat());
        float groundHeight = gridObj["groundHeight"].GetFloat();

        m_gridInfo = GridInfo {
            gridMin, gridMax, gridSpacing, initialVelocity, gravity, groundHeight
        };
    } else {
        m_gridInfo = defaultGridInfo();
    }

    if (jsonDoc.HasMember("hyperparameters")) {
        Value hypObj = jsonDoc["hyperparameters"].GetObject();

        m_hyperparameterInfo = HyperparameterInfo {
            hypObj["criticalCompression"].GetFloat(),
            hypObj["criticalStretch"].GetFloat(),
            hypObj["youngsModulus"].GetFloat(),
            hypObj["hardeningCoefficient"].GetFloat(),
            hypObj["poissonsRatio"].GetFloat(),
            hypObj["density"].GetFloat(),
            hypObj["particleSize"].GetFloat()
        };
    } else {
        m_hyperparameterInfo = defaultHyperparameterInfo();
    }

    if (jsonDoc.HasMember("collisions")) {
        Value colObj = jsonDoc["collisions"].GetObject();

        Vector3f center(colObj["center"].GetArray()[0].GetFloat(),
                        colObj["center"].GetArray()[1].GetFloat(),
                        colObj["center"].GetArray()[2].GetFloat());
        Vector3f velocity(colObj["velocity"].GetArray()[0].GetFloat(),
                          colObj["velocity"].GetArray()[1].GetFloat(),
                          colObj["velocity"].GetArray()[2].GetFloat());
        Vector3f scale(colObj["scale"].GetArray()[0].GetFloat(),
                       colObj["scale"].GetArray()[1].GetFloat(),
                       colObj["scale"].GetArray()[2].GetFloat());

        m_collisionInfo = CollisionInfo {
            colObj["type"].GetString(), center, velocity, colObj["coefficientOfFriction"].GetFloat(), colObj["rotZ"].GetFloat(), scale
        };

    } else {
        m_collisionInfo = defaultCollisionInfo();
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

HyperparameterInfo SceneFile::getHyperparameterInfo() {
    return m_hyperparameterInfo;
}

CollisionInfo SceneFile::getCollisionInfo() {
    return m_collisionInfo;
}
