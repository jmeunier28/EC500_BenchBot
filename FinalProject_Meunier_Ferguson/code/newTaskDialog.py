'''

Author: Jojo Meunier jmeunier@bu.edu 4/19/16

Class that creates new menu item based off uploaded python and ui files

uploading the python and ui files will give the user freedom and flexibility to 
define a new task with unique parameters 

uploading a custom json file will render a new 3D model of the workspace

'''



from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import json


class Create(QDialog):

    def __init__(self, parent=None):
        super(Create,self).__init__()
 
     
    def loadDiag(self,pyfile, uifile, json_file):

     	self.pyfile = pyfile
     	self.uifile = uifile
     	self.json_file = json_file

     	Ui_Dialog, QtBaseClass = uic.loadUiType(uifile)
     	self.diag = Ui_Dialog()
     	self.diag.setupUi(self)


# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = Create()
    window.show()
    sys.exit(app.exec_())