/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.8.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollBar>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionOpen;
    QWidget *centralWidget;
    QPushButton *openButton;
    QLabel *ImgDisplay;
    QPushButton *saveButton;
    QPushButton *resetButton;
    QPushButton *gaussianBlurButton;
    QPushButton *bwButton;
    QDoubleSpinBox *blurSpinBox;
    QPushButton *noiseButton;
    QSlider *zoomSlider;
    QLabel *label;
    QScrollBar *horizontalScrollBar;
    QScrollBar *verticalScrollBar;
    QPushButton *firstDerivxButton;
    QPushButton *secondDerivButton;
    QDoubleSpinBox *firstDerivSpinBox;
    QDoubleSpinBox *secondDerivSpinBox;
    QPushButton *meanButton;
    QPushButton *saveDisplayButton;
    QPushButton *sharpenButton;
    QDoubleSpinBox *sharpenSigmaSpinBox;
    QDoubleSpinBox *sharpenMagSpinBox;
    QPushButton *toggleButton;
    QPushButton *halfButton;
    QPushButton *sobelButton;
    QPushButton *bilateralButton;
    QDoubleSpinBox *bilateralSigmaSSpinBox;
    QDoubleSpinBox *bilateralSigmaISpinBox;
    QDoubleSpinBox *noiseSpinBox;
    QFrame *line;
    QCheckBox *separableCheckBox;
    QCheckBox *colorNoiseCheckBox;
    QPushButton *peaksButton;
    QPushButton *houghButton;
    QPushButton *medianButton;
    QSpinBox *medianBox;
    QSpinBox *meanBox;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QPushButton *rotateButton;
    QDoubleSpinBox *orientationSpinBox;
    QLabel *label_12;
    QDoubleSpinBox *peakThresholdSpinBox;
    QLabel *label_13;
    QSlider *brightnessSlider;
    QLabel *label_14;
    QPushButton *smartButton;
    QPushButton *randomButton;
    QPushButton *pixelButton;
    QPushButton *histogramButton;
    QLabel *label_15;
    QLabel *label_16;
    QSpinBox *clusterBox;
    QPushButton *firstDerivyButton;
    QStatusBar *statusBar;
    QToolBar *mainToolBar;
    QMenuBar *menuBar;
    QMenu *menuFile;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(996, 693);
        actionOpen = new QAction(MainWindow);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        openButton = new QPushButton(centralWidget);
        openButton->setObjectName(QStringLiteral("openButton"));
        openButton->setGeometry(QRect(10, 10, 71, 31));
        ImgDisplay = new QLabel(centralWidget);
        ImgDisplay->setObjectName(QStringLiteral("ImgDisplay"));
        ImgDisplay->setGeometry(QRect(330, 19, 640, 521));
        saveButton = new QPushButton(centralWidget);
        saveButton->setObjectName(QStringLiteral("saveButton"));
        saveButton->setGeometry(QRect(90, 10, 71, 31));
        resetButton = new QPushButton(centralWidget);
        resetButton->setObjectName(QStringLiteral("resetButton"));
        resetButton->setGeometry(QRect(10, 50, 75, 23));
        gaussianBlurButton = new QPushButton(centralWidget);
        gaussianBlurButton->setObjectName(QStringLiteral("gaussianBlurButton"));
        gaussianBlurButton->setGeometry(QRect(10, 170, 75, 23));
        bwButton = new QPushButton(centralWidget);
        bwButton->setObjectName(QStringLiteral("bwButton"));
        bwButton->setGeometry(QRect(10, 80, 75, 23));
        blurSpinBox = new QDoubleSpinBox(centralWidget);
        blurSpinBox->setObjectName(QStringLiteral("blurSpinBox"));
        blurSpinBox->setGeometry(QRect(130, 170, 62, 22));
        blurSpinBox->setSingleStep(0.5);
        noiseButton = new QPushButton(centralWidget);
        noiseButton->setObjectName(QStringLiteral("noiseButton"));
        noiseButton->setGeometry(QRect(10, 110, 75, 23));
        zoomSlider = new QSlider(centralWidget);
        zoomSlider->setObjectName(QStringLiteral("zoomSlider"));
        zoomSlider->setGeometry(QRect(40, 600, 291, 19));
        zoomSlider->setMaximum(999);
        zoomSlider->setOrientation(Qt::Horizontal);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 600, 31, 16));
        horizontalScrollBar = new QScrollBar(centralWidget);
        horizontalScrollBar->setObjectName(QStringLiteral("horizontalScrollBar"));
        horizontalScrollBar->setGeometry(QRect(330, 540, 641, 16));
        horizontalScrollBar->setMaximum(999);
        horizontalScrollBar->setOrientation(Qt::Horizontal);
        verticalScrollBar = new QScrollBar(centralWidget);
        verticalScrollBar->setObjectName(QStringLiteral("verticalScrollBar"));
        verticalScrollBar->setGeometry(QRect(970, 20, 20, 521));
        verticalScrollBar->setMaximum(999);
        verticalScrollBar->setOrientation(Qt::Vertical);
        firstDerivxButton = new QPushButton(centralWidget);
        firstDerivxButton->setObjectName(QStringLiteral("firstDerivxButton"));
        firstDerivxButton->setGeometry(QRect(10, 200, 75, 23));
        secondDerivButton = new QPushButton(centralWidget);
        secondDerivButton->setObjectName(QStringLiteral("secondDerivButton"));
        secondDerivButton->setGeometry(QRect(10, 240, 75, 23));
        firstDerivSpinBox = new QDoubleSpinBox(centralWidget);
        firstDerivSpinBox->setObjectName(QStringLiteral("firstDerivSpinBox"));
        firstDerivSpinBox->setGeometry(QRect(150, 210, 62, 22));
        firstDerivSpinBox->setSingleStep(0.5);
        secondDerivSpinBox = new QDoubleSpinBox(centralWidget);
        secondDerivSpinBox->setObjectName(QStringLiteral("secondDerivSpinBox"));
        secondDerivSpinBox->setGeometry(QRect(150, 240, 62, 22));
        secondDerivSpinBox->setSingleStep(0.5);
        meanButton = new QPushButton(centralWidget);
        meanButton->setObjectName(QStringLiteral("meanButton"));
        meanButton->setGeometry(QRect(10, 140, 75, 23));
        saveDisplayButton = new QPushButton(centralWidget);
        saveDisplayButton->setObjectName(QStringLiteral("saveDisplayButton"));
        saveDisplayButton->setGeometry(QRect(170, 10, 71, 31));
        sharpenButton = new QPushButton(centralWidget);
        sharpenButton->setObjectName(QStringLiteral("sharpenButton"));
        sharpenButton->setGeometry(QRect(10, 270, 75, 23));
        sharpenSigmaSpinBox = new QDoubleSpinBox(centralWidget);
        sharpenSigmaSpinBox->setObjectName(QStringLiteral("sharpenSigmaSpinBox"));
        sharpenSigmaSpinBox->setGeometry(QRect(130, 270, 62, 22));
        sharpenSigmaSpinBox->setSingleStep(0.5);
        sharpenMagSpinBox = new QDoubleSpinBox(centralWidget);
        sharpenMagSpinBox->setObjectName(QStringLiteral("sharpenMagSpinBox"));
        sharpenMagSpinBox->setGeometry(QRect(240, 270, 62, 22));
        sharpenMagSpinBox->setSingleStep(0.5);
        toggleButton = new QPushButton(centralWidget);
        toggleButton->setObjectName(QStringLiteral("toggleButton"));
        toggleButton->setGeometry(QRect(100, 50, 141, 51));
        toggleButton->setAcceptDrops(false);
        halfButton = new QPushButton(centralWidget);
        halfButton->setObjectName(QStringLiteral("halfButton"));
        halfButton->setGeometry(QRect(10, 390, 75, 23));
        sobelButton = new QPushButton(centralWidget);
        sobelButton->setObjectName(QStringLiteral("sobelButton"));
        sobelButton->setGeometry(QRect(10, 420, 75, 23));
        bilateralButton = new QPushButton(centralWidget);
        bilateralButton->setObjectName(QStringLiteral("bilateralButton"));
        bilateralButton->setGeometry(QRect(10, 300, 75, 23));
        bilateralSigmaSSpinBox = new QDoubleSpinBox(centralWidget);
        bilateralSigmaSSpinBox->setObjectName(QStringLiteral("bilateralSigmaSSpinBox"));
        bilateralSigmaSSpinBox->setGeometry(QRect(140, 300, 51, 22));
        bilateralSigmaSSpinBox->setSingleStep(0.5);
        bilateralSigmaISpinBox = new QDoubleSpinBox(centralWidget);
        bilateralSigmaISpinBox->setObjectName(QStringLiteral("bilateralSigmaISpinBox"));
        bilateralSigmaISpinBox->setGeometry(QRect(240, 300, 62, 22));
        bilateralSigmaISpinBox->setMaximum(255);
        bilateralSigmaISpinBox->setSingleStep(0.5);
        noiseSpinBox = new QDoubleSpinBox(centralWidget);
        noiseSpinBox->setObjectName(QStringLiteral("noiseSpinBox"));
        noiseSpinBox->setGeometry(QRect(130, 110, 62, 22));
        noiseSpinBox->setSingleStep(0.5);
        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(310, 10, 20, 551));
        line->setLineWidth(2);
        line->setFrameShape(QFrame::VLine);
        line->setFrameShadow(QFrame::Sunken);
        separableCheckBox = new QCheckBox(centralWidget);
        separableCheckBox->setObjectName(QStringLiteral("separableCheckBox"));
        separableCheckBox->setGeometry(QRect(200, 170, 70, 17));
        colorNoiseCheckBox = new QCheckBox(centralWidget);
        colorNoiseCheckBox->setObjectName(QStringLiteral("colorNoiseCheckBox"));
        colorNoiseCheckBox->setGeometry(QRect(200, 110, 70, 17));
        peaksButton = new QPushButton(centralWidget);
        peaksButton->setObjectName(QStringLiteral("peaksButton"));
        peaksButton->setGeometry(QRect(10, 480, 75, 23));
        houghButton = new QPushButton(centralWidget);
        houghButton->setObjectName(QStringLiteral("houghButton"));
        houghButton->setGeometry(QRect(10, 360, 75, 23));
        medianButton = new QPushButton(centralWidget);
        medianButton->setObjectName(QStringLiteral("medianButton"));
        medianButton->setGeometry(QRect(10, 330, 75, 23));
        medianBox = new QSpinBox(centralWidget);
        medianBox->setObjectName(QStringLiteral("medianBox"));
        medianBox->setGeometry(QRect(130, 330, 42, 22));
        meanBox = new QSpinBox(centralWidget);
        meanBox->setObjectName(QStringLiteral("meanBox"));
        meanBox->setGeometry(QRect(130, 140, 42, 22));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(100, 140, 31, 16));
        QFont font;
        font.setPointSize(7);
        label_2->setFont(font);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(100, 170, 31, 16));
        label_3->setFont(font);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(99, 214, 51, 16));
        label_4->setFont(font);
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(100, 245, 51, 16));
        label_5->setFont(font);
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(98, 274, 31, 16));
        label_6->setFont(font);
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(99, 304, 41, 16));
        label_7->setFont(font);
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(100, 334, 31, 16));
        label_8->setFont(font);
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QStringLiteral("label_9"));
        label_9->setGeometry(QRect(100, 110, 31, 16));
        label_9->setFont(font);
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QStringLiteral("label_10"));
        label_10->setGeometry(QRect(202, 304, 41, 16));
        label_10->setFont(font);
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QStringLiteral("label_11"));
        label_11->setGeometry(QRect(210, 276, 31, 16));
        label_11->setFont(font);
        rotateButton = new QPushButton(centralWidget);
        rotateButton->setObjectName(QStringLiteral("rotateButton"));
        rotateButton->setGeometry(QRect(10, 450, 75, 23));
        orientationSpinBox = new QDoubleSpinBox(centralWidget);
        orientationSpinBox->setObjectName(QStringLiteral("orientationSpinBox"));
        orientationSpinBox->setGeometry(QRect(130, 450, 62, 22));
        orientationSpinBox->setMaximum(360);
        orientationSpinBox->setSingleStep(0.5);
        label_12 = new QLabel(centralWidget);
        label_12->setObjectName(QStringLiteral("label_12"));
        label_12->setGeometry(QRect(100, 450, 31, 16));
        label_12->setFont(font);
        peakThresholdSpinBox = new QDoubleSpinBox(centralWidget);
        peakThresholdSpinBox->setObjectName(QStringLiteral("peakThresholdSpinBox"));
        peakThresholdSpinBox->setGeometry(QRect(130, 480, 62, 22));
        peakThresholdSpinBox->setMaximum(255);
        peakThresholdSpinBox->setSingleStep(0.5);
        label_13 = new QLabel(centralWidget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setGeometry(QRect(100, 480, 31, 16));
        label_13->setFont(font);
        brightnessSlider = new QSlider(centralWidget);
        brightnessSlider->setObjectName(QStringLiteral("brightnessSlider"));
        brightnessSlider->setGeometry(QRect(430, 600, 271, 19));
        brightnessSlider->setMaximum(999);
        brightnessSlider->setOrientation(Qt::Horizontal);
        label_14 = new QLabel(centralWidget);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(390, 600, 31, 16));
        smartButton = new QPushButton(centralWidget);
        smartButton->setObjectName(QStringLiteral("smartButton"));
        smartButton->setGeometry(QRect(10, 510, 91, 23));
        randomButton = new QPushButton(centralWidget);
        randomButton->setObjectName(QStringLiteral("randomButton"));
        randomButton->setGeometry(QRect(60, 540, 75, 23));
        pixelButton = new QPushButton(centralWidget);
        pixelButton->setObjectName(QStringLiteral("pixelButton"));
        pixelButton->setGeometry(QRect(140, 540, 75, 23));
        histogramButton = new QPushButton(centralWidget);
        histogramButton->setObjectName(QStringLiteral("histogramButton"));
        histogramButton->setGeometry(QRect(220, 540, 91, 23));
        label_15 = new QLabel(centralWidget);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(10, 540, 51, 21));
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(10, 570, 41, 21));
        label_16->setFont(font);
        clusterBox = new QSpinBox(centralWidget);
        clusterBox->setObjectName(QStringLiteral("clusterBox"));
        clusterBox->setGeometry(QRect(60, 570, 42, 22));
        firstDerivyButton = new QPushButton(centralWidget);
        firstDerivyButton->setObjectName(QStringLiteral("firstDerivyButton"));
        firstDerivyButton->setGeometry(QRect(10, 220, 75, 23));
        MainWindow->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 996, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        MainWindow->setMenuBar(menuBar);

        menuBar->addAction(menuFile->menuAction());
        menuFile->addAction(actionOpen);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        actionOpen->setText(QApplication::translate("MainWindow", "Open", Q_NULLPTR));
        openButton->setText(QApplication::translate("MainWindow", "Open Image", Q_NULLPTR));
        ImgDisplay->setText(QString());
        saveButton->setText(QApplication::translate("MainWindow", "Save Image", Q_NULLPTR));
        resetButton->setText(QApplication::translate("MainWindow", "Reset", Q_NULLPTR));
        gaussianBlurButton->setText(QApplication::translate("MainWindow", "Gaussian blur", Q_NULLPTR));
        bwButton->setText(QApplication::translate("MainWindow", "B/W", Q_NULLPTR));
        noiseButton->setText(QApplication::translate("MainWindow", "Add noise", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "Zoom", Q_NULLPTR));
        firstDerivxButton->setText(QApplication::translate("MainWindow", "1st Deriv x", Q_NULLPTR));
        secondDerivButton->setText(QApplication::translate("MainWindow", "2nd Deriv", Q_NULLPTR));
        meanButton->setText(QApplication::translate("MainWindow", "Mean", Q_NULLPTR));
        saveDisplayButton->setText(QApplication::translate("MainWindow", "Save Display", Q_NULLPTR));
        sharpenButton->setText(QApplication::translate("MainWindow", "Sharpen", Q_NULLPTR));
        toggleButton->setText(QApplication::translate("MainWindow", "Toggle", Q_NULLPTR));
        halfButton->setText(QApplication::translate("MainWindow", "Half size", Q_NULLPTR));
        sobelButton->setText(QApplication::translate("MainWindow", "Sobel", Q_NULLPTR));
        bilateralButton->setText(QApplication::translate("MainWindow", "Bilateral", Q_NULLPTR));
        separableCheckBox->setText(QApplication::translate("MainWindow", "Separable", Q_NULLPTR));
        colorNoiseCheckBox->setText(QApplication::translate("MainWindow", "Color", Q_NULLPTR));
        peaksButton->setText(QApplication::translate("MainWindow", "Find peaks", Q_NULLPTR));
        houghButton->setText(QApplication::translate("MainWindow", "Hough", Q_NULLPTR));
        medianButton->setText(QApplication::translate("MainWindow", "Median", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "Radius", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "Sigma", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "Sigma_1st", Q_NULLPTR));
        label_5->setText(QApplication::translate("MainWindow", "Sigma_2nd", Q_NULLPTR));
        label_6->setText(QApplication::translate("MainWindow", "Sigma", Q_NULLPTR));
        label_7->setText(QApplication::translate("MainWindow", "Sigma S", Q_NULLPTR));
        label_8->setText(QApplication::translate("MainWindow", "Radius", Q_NULLPTR));
        label_9->setText(QApplication::translate("MainWindow", "Mag", Q_NULLPTR));
        label_10->setText(QApplication::translate("MainWindow", "Sigma I", Q_NULLPTR));
        label_11->setText(QApplication::translate("MainWindow", "Alpha", Q_NULLPTR));
        rotateButton->setText(QApplication::translate("MainWindow", "Rotate", Q_NULLPTR));
        label_12->setText(QApplication::translate("MainWindow", "Orien", Q_NULLPTR));
        label_13->setText(QApplication::translate("MainWindow", "Thres", Q_NULLPTR));
        label_14->setText(QApplication::translate("MainWindow", "Gain", Q_NULLPTR));
        smartButton->setText(QApplication::translate("MainWindow", "Smart K-means", Q_NULLPTR));
        randomButton->setText(QApplication::translate("MainWindow", "Random seed", Q_NULLPTR));
        pixelButton->setText(QApplication::translate("MainWindow", "Pixel seed", Q_NULLPTR));
        histogramButton->setText(QApplication::translate("MainWindow", "Histogram seed", Q_NULLPTR));
        label_15->setText(QApplication::translate("MainWindow", "K-means", Q_NULLPTR));
        label_16->setText(QApplication::translate("MainWindow", "Clusters", Q_NULLPTR));
        firstDerivyButton->setText(QApplication::translate("MainWindow", "1st Deriv y", Q_NULLPTR));
        menuFile->setTitle(QApplication::translate("MainWindow", "File", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
