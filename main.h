#ifndef MAIN_H
#define MAIN_H

#include<QString>
#include<string>

struct AppArgs {
    QString infile;
    QString outDir;
    bool vizualize;
    int numParticles;
    int numFrames;
    float stepLength;
};

AppArgs snowSimParseArgs();

#endif // MAIN_H
