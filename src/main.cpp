#include <QApplication>
#include "ArmSim.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ArmSim as;
    as.show();
    return a.exec();
}
