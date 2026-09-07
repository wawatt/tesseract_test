#include "main_window.h"

#include <QApplication>
#include <QFile>
#include <QTextStream>
#include <iostream>

int main(int argc, char* argv[]) {
    // 高 DPI 屏幕清晰适配
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    QCoreApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);

    QApplication app(argc, argv);
    app.setApplicationName("RobotSimulationWorkstation");
    app.setOrganizationName("TesseractRobotics");

    // 加载现代深色工业主题 QSS 样式表
    QFile qssFile(":/styles/dark_theme.qss");
    if (qssFile.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream ts(&qssFile);
        app.setStyleSheet(ts.readAll());
        qssFile.close();
    } else {
        std::cerr << "[Main] Warning: Could not open dark_theme.qss from resource" << std::endl;
    }

    sim_app::MainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}
