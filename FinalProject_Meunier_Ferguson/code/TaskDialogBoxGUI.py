# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'TaskDialogBox.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(398, 316)
        self.task_diag_buttonBox = QtWidgets.QDialogButtonBox(Dialog)
        self.task_diag_buttonBox.setGeometry(QtCore.QRect(40, 270, 341, 32))
        self.task_diag_buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.task_diag_buttonBox.setStandardButtons(QtWidgets.QDialogButtonBox.Cancel|QtWidgets.QDialogButtonBox.Ok)
        self.task_diag_buttonBox.setObjectName("task_diag_buttonBox")
        self.listView = QtWidgets.QListView(Dialog)
        self.listView.setGeometry(QtCore.QRect(30, 20, 341, 231))
        self.listView.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.listView.setFrameShadow(QtWidgets.QFrame.Raised)
        self.listView.setObjectName("listView")
        self.TaskSettingsLabel = QtWidgets.QTextEdit(Dialog)
        self.TaskSettingsLabel.setGeometry(QtCore.QRect(141, 30, 101, 31))
        self.TaskSettingsLabel.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.TaskSettingsLabel.setAutoFillBackground(False)
        self.TaskSettingsLabel.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.TaskSettingsLabel.setObjectName("TaskSettingsLabel")
        self.samples_doubleSpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.samples_doubleSpinBox.setGeometry(QtCore.QRect(270, 70, 66, 24))
        self.samples_doubleSpinBox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.samples_doubleSpinBox.setDecimals(0)
        self.samples_doubleSpinBox.setObjectName("samples_doubleSpinBox")
        self.primer_SpinBox = QtWidgets.QDoubleSpinBox(Dialog)
        self.primer_SpinBox.setGeometry(QtCore.QRect(270, 110, 66, 24))
        self.primer_SpinBox.setDecimals(0)
        self.primer_SpinBox.setObjectName("primer_SpinBox")
        self.samples_label = QtWidgets.QLineEdit(Dialog)
        self.samples_label.setGeometry(QtCore.QRect(122, 70, 131, 21))
        self.samples_label.setObjectName("samples_label")
        self.primers_label = QtWidgets.QLineEdit(Dialog)
        self.primers_label.setGeometry(QtCore.QRect(122, 110, 131, 21))
        self.primers_label.setObjectName("primers_label")

        self.retranslateUi(Dialog)
        self.task_diag_buttonBox.accepted.connect(Dialog.accept)
        self.task_diag_buttonBox.rejected.connect(Dialog.reject)
        QtCore.QMetaObject.connectSlotsByName(Dialog)

    def retranslateUi(self, Dialog):
        _translate = QtCore.QCoreApplication.translate
        Dialog.setWindowTitle(_translate("Dialog", "Task Dialog"))
        self.TaskSettingsLabel.setPlaceholderText(_translate("Dialog", "Task Settings:"))
        self.samples_label.setText(_translate("Dialog", "Number of Samples"))
        self.primers_label.setText(_translate("Dialog", "Number of Primers"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    Dialog = QtWidgets.QDialog()
    ui = Ui_Dialog()
    ui.setupUi(Dialog)
    Dialog.show()
    sys.exit(app.exec_())

