'''

Author: Jojo Meunier jmeunier@bu.edu 4/10/16

Class that contains Create New Task Dialog window 
Allows user to upload python, ui and json files 

uploading the python and ui files will give the user freedom and flexibility to 
define a new task with unique parameters 

uploading a custom json file will generate a new workspace

'''



from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import re
import newTaskDialog
from newTaskDialog import Create


class NewTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(NewTaskDialogWindow,self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('CreateCustomTask.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        #connect dialog buttons

        self.diag.upload_python.clicked.connect(self.upload_python_clicked)
        self.diag.upload_ui.clicked.connect(self.upload_ui_clicked)
        self.diag.upload_json.clicked.connect(self.upload_json_clicked)

        self.diag.buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def upload_python_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open Python File', '', "Python Files (*.py)") 
        for line in re.split("/",str(file[0])):
            if ".py" in line:
                global pyfile
                pyfile = line

    def upload_ui_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open UI File', '', "Ui Files (*.ui)")
        global uifile
        uifile = file[0]

    def upload_json_clicked(self):
        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open File', '', "JSON Files (*.json)")
        global json_file
        json_file = file[0]

    def ok_button_clicked(self):
        # code to set the new files will go here
        print(uifile)
        self.newTask = Create()
        self.newTask.loadDiag(pyfile,uifile,json_file)

      

# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = NewTaskDialogWindow()
    window.show()
    sys.exit(app.exec_())

