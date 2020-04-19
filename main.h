#ifndef MAIN_H
#define MAIN_H

#include<QString>

struct AppArgs {
    QString infile;
    bool vizualize;
    int numParticles;
    int numFrames;
    float stepLength;
};

AppArgs snowSimParseArgs();

#endif // MAIN_H
