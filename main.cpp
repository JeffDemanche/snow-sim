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
    parser.addPositionalArgument("numparticles", "Number of particles to create within snowmesh");
    parser.addPositionalArgument("numframes", "Number of simulation timesteps to calculate");

    QCommandLineOption s({"s", "steplength"}, "Amount of seconds per simulation step", "steplength", "default");
    parser.addOption(s);

    QCommandLineOption visualize("viz");
    parser.addOption(visualize);

    parser.process(QApplication::arguments());

    const QStringList args = parser.positionalArguments();
    if(args.size() < 3) {
        cerr << "Error: Wrong number of arguments" << endl;
        QApplication::exit(1);
    }

    float stepLength = parser.value("steplength").toStdString() == "default"
                        ? 1.0 / 24.0
                        : parser.value("steplength").toFloat();

    return AppArgs{
        args[0],
        parser.isSet(visualize),
        args[1].toInt(),
        args[2].toInt(),
        stepLength
    };
}

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    AppArgs args = snowSimParseArgs();

    // Add --viz to command to run GUI.
    if (args.vizualize) {
        MainWindow w;
        w.show();
        return a.exec();
    } else {
        Mesh m;
        m.loadFromFile(args.infile.toStdString());
        MPM mpm = MPM(m, args.numParticles, args.numFrames, args.stepLength);
        mpm.runSimulation();
    }

    auto t0 = high_resolution_clock::now();
    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();

    ////////////////////////////////////////////////////////////////////////////////

    cout << "Total simulation takes: " << duration << " milliseconds." <<endl;

    a.exit();
}
