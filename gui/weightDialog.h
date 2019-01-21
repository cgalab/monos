/********************************************************************************
** Form generated from reading UI file 'weightDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef WEIGHTDIALOG_H
#define WEIGHTDIALOG_H

#include <QtCore/QVariant>
#include <QAction>
#include <QApplication>
#include <QButtonGroup>
#include <QGridLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QSpacerItem>
#include <QSpinBox>
#include <QStatusBar>
#include <QWidget>

QT_BEGIN_NAMESPACE

class Ui_WeightDialog
{
public:
    QAction *actionQuit;
    QAction *actionVisToggleInput;
    QAction *actionResize;
    QAction *actionToggleFullscreen;
    QAction *actionVisToggleInputLabels;
    QAction *actionVisToggleArcs;
    QAction *actionTimeForwardAfterChains;
    QAction *actionFinishComputation;
    QAction *actionDefineWeight;
    QAction *actionEventStep;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QLineEdit *weightInput;
    QLabel *label_2;
    QSpinBox *edgeSelect;
    QPushButton *pushButtonOK;
    QLabel *label;
    QSpacerItem *verticalSpacer;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *WeightDialog)
    {
        if (WeightDialog->objectName().isEmpty())
            WeightDialog->setObjectName(QString::fromUtf8("WeightDialog"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(WeightDialog->sizePolicy().hasHeightForWidth());
        WeightDialog->setSizePolicy(sizePolicy);
        actionQuit = new QAction(WeightDialog);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionQuit->setCheckable(false);
        actionQuit->setEnabled(true);
        actionVisToggleInput = new QAction(WeightDialog);
        actionVisToggleInput->setObjectName(QString::fromUtf8("actionVisToggleInput"));
        actionVisToggleInput->setCheckable(true);
        actionVisToggleInput->setChecked(true);
        actionResize = new QAction(WeightDialog);
        actionResize->setObjectName(QString::fromUtf8("actionResize"));
        actionToggleFullscreen = new QAction(WeightDialog);
        actionToggleFullscreen->setObjectName(QString::fromUtf8("actionToggleFullscreen"));
        actionVisToggleInputLabels = new QAction(WeightDialog);
        actionVisToggleInputLabels->setObjectName(QString::fromUtf8("actionVisToggleInputLabels"));
        actionVisToggleInputLabels->setCheckable(true);
        actionVisToggleInputLabels->setChecked(false);
        actionVisToggleArcs = new QAction(WeightDialog);
        actionVisToggleArcs->setObjectName(QString::fromUtf8("actionVisToggleArcs"));
        actionVisToggleArcs->setCheckable(true);
        actionVisToggleArcs->setChecked(true);
        actionTimeForwardAfterChains = new QAction(WeightDialog);
        actionTimeForwardAfterChains->setObjectName(QString::fromUtf8("actionTimeForwardAfterChains"));
        actionFinishComputation = new QAction(WeightDialog);
        actionFinishComputation->setObjectName(QString::fromUtf8("actionFinishComputation"));
        actionDefineWeight = new QAction(WeightDialog);
        actionDefineWeight->setObjectName(QString::fromUtf8("actionDefineWeight"));
        actionEventStep = new QAction(WeightDialog);
        actionEventStep->setObjectName(QString::fromUtf8("actionEventStep"));
        centralWidget = new QWidget(WeightDialog);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        sizePolicy.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy);
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(0);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        weightInput = new QLineEdit(centralWidget);
        weightInput->setObjectName(QString::fromUtf8("weightInput"));

        gridLayout->addWidget(weightInput, 1, 1, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 1, 0, 1, 1);

        edgeSelect = new QSpinBox(centralWidget);
        edgeSelect->setObjectName(QString::fromUtf8("edgeSelect"));

        gridLayout->addWidget(edgeSelect, 0, 1, 1, 1);

        pushButtonOK = new QPushButton(centralWidget);
        pushButtonOK->setObjectName(QString::fromUtf8("pushButtonOK"));

        gridLayout->addWidget(pushButtonOK, 3, 1, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 0, 0, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 2, 1, 1, 1);

        WeightDialog->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(WeightDialog);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        WeightDialog->setStatusBar(statusBar);

        retranslateUi(WeightDialog);

        QMetaObject::connectSlotsByName(WeightDialog);
    } // setupUi

    void retranslateUi(QMainWindow *WeightDialog)
    {
        WeightDialog->setWindowTitle(QApplication::translate("WeightDialog", "MainWindow", 0, 0));
        actionQuit->setText(QApplication::translate("WeightDialog", "Quit", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionQuit->setToolTip(QApplication::translate("WeightDialog", "Exit application (Q)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionQuit->setShortcut(QApplication::translate("WeightDialog", "Q", 0, 0));
        actionVisToggleInput->setText(QApplication::translate("WeightDialog", "I", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionVisToggleInput->setToolTip(QApplication::translate("WeightDialog", "Toggle Input (I)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionVisToggleInput->setShortcut(QApplication::translate("WeightDialog", "I", 0, 0));
        actionResize->setText(QApplication::translate("WeightDialog", "resize", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionResize->setToolTip(QApplication::translate("WeightDialog", "reset Zoom (R)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionResize->setShortcut(QApplication::translate("WeightDialog", "R", 0, 0));
        actionToggleFullscreen->setText(QApplication::translate("WeightDialog", "F", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionToggleFullscreen->setToolTip(QApplication::translate("WeightDialog", "toggle fullscreen (F)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionToggleFullscreen->setShortcut(QApplication::translate("WeightDialog", "F", 0, 0));
        actionVisToggleInputLabels->setText(QApplication::translate("WeightDialog", "L", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionVisToggleInputLabels->setToolTip(QApplication::translate("WeightDialog", "Toggle Input Label Visibility (L)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionVisToggleInputLabels->setShortcut(QApplication::translate("WeightDialog", "L", 0, 0));
        actionVisToggleArcs->setText(QApplication::translate("WeightDialog", "S", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionVisToggleArcs->setToolTip(QApplication::translate("WeightDialog", "Toggle Straight-Skeleton Arcs Visibility (S)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionVisToggleArcs->setShortcut(QApplication::translate("WeightDialog", "S", 0, 0));
        actionTimeForwardAfterChains->setText(QApplication::translate("WeightDialog", "C", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionTimeForwardAfterChains->setToolTip(QApplication::translate("WeightDialog", "finsh computing both chains (C)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionTimeForwardAfterChains->setShortcut(QApplication::translate("WeightDialog", "C", 0, 0));
        actionFinishComputation->setText(QApplication::translate("WeightDialog", "\342\217\216", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionFinishComputation->setToolTip(QApplication::translate("WeightDialog", "compute final skeleton (Return)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionFinishComputation->setShortcut(QApplication::translate("WeightDialog", "Return", 0, 0));
        actionDefineWeight->setText(QApplication::translate("WeightDialog", "W", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionDefineWeight->setToolTip(QApplication::translate("WeightDialog", "define a weight for one edge (W)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionDefineWeight->setShortcut(QApplication::translate("WeightDialog", "W", 0, 0));
        actionEventStep->setText(QApplication::translate("WeightDialog", "n", 0, 0));
#ifndef QT_NO_TOOLTIP
        actionEventStep->setToolTip(QApplication::translate("WeightDialog", "compute next event (N)", 0, 0));
#endif // QT_NO_TOOLTIP
        actionEventStep->setShortcut(QApplication::translate("WeightDialog", "N", 0, 0));
        label_2->setText(QApplication::translate("WeightDialog", "Weight: ", 0, 0));
        pushButtonOK->setText(QApplication::translate("WeightDialog", "OK", 0, 0));
        label->setText(QApplication::translate("WeightDialog", "Edge: ", 0, 0));
    } // retranslateUi

};

namespace Ui {
    class WeightDialog: public Ui_WeightDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // WEIGHTDIALOG_H
