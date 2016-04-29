'''
Author: Jojo Meunier jmeunier@bu.edu 4/10/16

Class for Colony Task dialog windows

once parameters are set for colony task default json workspace config file will be used 
to generate the work space and display workspace in WorkSpaceTab on main GUI

'''


from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import json

class ColonyTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(ColonyTaskDialogWindow, self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('CloningTaskDialogBox.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        self.diag.buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def ok_button_clicked(self):
        #code to set standard json file for workspace of pcr
        #and use this to draw 
        pass


# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ColonyTaskDialogWindow()
    window.show()
    sys.exit(app.exec_())
