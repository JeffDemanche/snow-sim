#include <main.h>

#include <QCoreApplication>
#include <QApplication>
#include <QCommandLineParser>
#include <QDir>
#include <QCommandLineOption>

#include <iostream>
#include <chrono>

#include "mesh.h"
#include "mpm.h"
#include "mainwindow.h"

using namespace std;
using namespace std::chrono;

AppArgs snowSimParseArgs() {
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("snowmesh", "Mesh file to use as snow");
    parser.addPositionalArgument("outdir", "Directory to export volumetric data to");
    parser.addPositionalArgument("numparticles", "Number of particles to create within snowmesh");
    parser.addPositionalArgument("numframes", "Number of simulation timesteps to calculate");

    QCommandLineOption s({"s", "steplength"}, "Amount of seconds per simulation step", "steplength", "default");
    parser.addOption(s);

    QCommandLineOption visualize("viz");
    parser.addOption(visualize);

    parser.process(QApplication::arguments());

    const QStringList args = parser.positionalArguments();
    if(args.size() < 4) {
        cerr << "Error: Wrong number of arguments" << endl;
        QApplication::exit(1);
    }

    float stepLength = parser.value("steplength").toStdString() == "default"
                        ? 1.0 / 24.0
                        : parser.value("steplength").toFloat();

    return AppArgs{
        args[0],
        args[1],
        parser.isSet(visualize),
        args[2].toInt(),
        args[3].toInt(),
        stepLength
    };
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    QCommandLineParser parser;
    QCommandLineOption f({"f", "scenefile"}, "Scene file to use", "scenefile", "");
    parser.addOption(f);
    parser.process(QApplication::arguments());

    auto t0 = high_resolution_clock::now();

    if (parser.isSet(f)) {
        MPM mpm = MPM(parser.value("scenefile").toStdString());
        mpm.runSimulation();
    }
    else {
        AppArgs args = snowSimParseArgs();

        // Add --viz to command to run GUI.
        if (args.vizualize) {
            MainWindow w;
            w.show();
            return a.exec();
        } else {
            Mesh m;
            m.loadFromFile(args.infile.toStdString());
            MPM mpm = MPM(m, args.outDir, args.numParticles, args.numFrames, args.stepLength);
            mpm.runSimulation();
        }
    }

    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();

    ////////////////////////////////////////////////////////////////////////////////

    cout << "Total simulation takes: " << duration << " milliseconds." <<endl;

    a.exit();
}
