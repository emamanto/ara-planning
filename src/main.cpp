#include <QApplication>
#include "SearchWidget.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SearchWidget s;
    s.show();
    return a.exec();
}
