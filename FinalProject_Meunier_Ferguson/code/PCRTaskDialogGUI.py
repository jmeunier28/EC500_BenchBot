'''
Author: JoJo Meunier jmeunier@bu.edu 4/10/16

Class for PCR Task dialog windows

once parameters are set for PCR task default json workspace config file will be used 
to generate the work space and display workspace in WorkSpaceT[ab on main GUI

'''


from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import sys
import drawCubes, get_json_data
from drawCubes import glWidget
from get_json_data import CollectData




class PCRTaskDialogWindow(QDialog):

    def __init__(self, parent=None):
        super(PCRTaskDialogWindow,self).__init__()
        Ui_Dialog, QtBaseClass = uic.loadUiType('TaskDialogBox.ui')
        self.diag = Ui_Dialog()
        self.diag.setupUi(self)

        self.samples = self.diag.samples_doubleSpinBox
        self.samples.setRange(0,100)
        self.samples.setSingleStep(1)
        self.samples.setValue(0)

        self.primer = self.diag.primer_SpinBox
        self.primer.setRange(0,100)
        self.primer.setSingleStep(1)
        self.primer.setValue(0)

        #when okay button is clicked workspace config pops up and data should be sent back to main
        self.diag.task_diag_buttonBox.button(QDialogButtonBox.Ok).clicked.connect(self.ok_button_clicked)

    def ok_button_clicked(self):
        '''self.widget = glWidget()
        self.widget.setWindowTitle('Work Space Window')
        self.widget.show()'''
        primer = self.primer.value()
        samples = self.samples.value()
        return primer, samples

        

# main function
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = PCRTaskDialogWindow()
    window.show()
    sys.exit(app.exec_())