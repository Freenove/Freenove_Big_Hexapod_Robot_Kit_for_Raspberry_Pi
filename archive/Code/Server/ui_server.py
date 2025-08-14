# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'C:\Users\Freenove\Desktop\树莓派四足狗\界面\Server\ui_server.ui'
#
# Created by: PyQt5 UI code generator 5.11.3
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_server(object):
    def setupUi(self, server):
        server.setObjectName("server")
        server.resize(400, 300)
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        server.setFont(font)
        server.setStyleSheet("QWidget{\n"
"background:#484848;\n"
"}\n"
"QAbstractButton{\n"
"border-style:none;\n"
"border-radius:0px;\n"
"padding:5px;\n"
"color:#DCDCDC;\n"
"background:qlineargradient(spread:pad,x1:0,y1:0,x2:0,y2:1,stop:0 #858585,stop:1 #383838);\n"
"}\n"
"QAbstractButton:hover{\n"
"color:#000000;\n"
"background-color:#008aff;\n"
"}\n"
"QAbstractButton:pressed{\n"
"color:#DCDCDC;\n"
"border-style:solid;\n"
"border-width:0px 0px 0px 4px;\n"
"padding:4px 4px 4px 2px;\n"
"border-color:#008aff;\n"
"background-color:#444444;\n"
"}\n"
"\n"
"QLabel{\n"
"color:#DCDCDC;\n"
"\n"
"\n"
"}\n"
"QLabel:focus{\n"
"border:1px solid #00BB9E;\n"
"\n"
"}\n"
"\n"
"\n"
"")
        self.states = QtWidgets.QLabel(server)
        self.states.setGeometry(QtCore.QRect(120, 80, 160, 90))
        font = QtGui.QFont()
        font.setFamily("微软雅黑")
        font.setPointSize(72)
        self.states.setFont(font)
        self.states.setAlignment(QtCore.Qt.AlignCenter)
        self.states.setObjectName("states")
        self.pushButton_On_And_Off = QtWidgets.QPushButton(server)
        self.pushButton_On_And_Off.setGeometry(QtCore.QRect(160, 220, 80, 30))
        self.pushButton_On_And_Off.setObjectName("pushButton_On_And_Off")

        self.retranslateUi(server)
        QtCore.QMetaObject.connectSlotsByName(server)

    def retranslateUi(self, server):
        _translate = QtCore.QCoreApplication.translate
        server.setWindowTitle(_translate("server", "Server"))
        self.states.setText(_translate("server", "Off"))
        self.pushButton_On_And_Off.setText(_translate("server", "On"))

