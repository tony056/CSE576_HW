#include <QtWidgets/QApplication>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    /***** Use the following 2 lines to scale up the application window *****/

    //qputenv("QT_AUTO_SCREEN_SCALE_FACTOR", "1");
    //qputenv("QT_SCALE_FACTOR", "0.75"); // use a suitable scaling factor

    const int RESTART_CODE = 1000;
    int return_code;
    do
    {
        QApplication a(argc, argv);
        MainWindow w;
        w.show();
        return_code = a.exec();
    }while (return_code == RESTART_CODE);

    return return_code;
}
