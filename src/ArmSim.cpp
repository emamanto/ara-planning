#include "ArmSim.h"

ArmSim::ArmSim() : QMainWindow()
{
    this->setWindowTitle(tr("Arm Motion Search Simulator"));
    this->setCentralWidget(new Visualizer(this));
}
