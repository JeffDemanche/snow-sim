#include <QCoreApplication>
#include <QCommandLineParser>
#include <QDir>

#include <iostream>
#include <chrono>

#include "mesh.h"
#include "mpm.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    ////////////////////////////////////////////////////////////////////////////////
    QCommandLineParser parser;
    parser.addHelpOption();
    parser.addPositionalArgument("snowmesh", "Mesh file to use as snow");
    parser.addPositionalArgument("numparticles", "Number of particles to create within snowmesh");

    parser.process(a);

    const QStringList args = parser.positionalArguments();
    if(args.size() < 1) {
        cerr << "Error: Wrong number of arguments" << endl;
        a.exit(1);
        return 1;
    }
    QString infile = args[0];
    int numParticles = args[1].toInt();

    ////////////////////////////////////////////////////////////////////////////////

    Mesh m;
    m.loadFromFile(infile.toStdString());

    MPM mpm = MPM(m, numParticles);

    auto t0 = high_resolution_clock::now();
    auto t1 = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(t1-t0).count();

    ////////////////////////////////////////////////////////////////////////////////

    cout << "Total simulation takes: " << duration << " milliseconds." <<endl;

    a.exit();
}
