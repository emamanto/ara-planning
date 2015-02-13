#include <QApplication>
#include "MazeWidget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MazeWidget m;
    m.show();
    return a.exec();
}
