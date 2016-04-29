'''

Author: JoJo Meunier jmeunier@bu.edu 4/10/16

python script using pyopengl to draw shapes 
this will be used to model the workspace in 3D on an openGL widget in pyqt


Also draws path way as it would be for robot... 
goes from tube rack -> tip box -> micro plate -> waste container

not a biologist so idk if thats right 

'''

from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PyQt5.QtOpenGL import *
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import get_json_data
from get_json_data import CollectData
import sys

edges = ( #in order to draw a cube we will always connect our edges in this way
		(0,1),
		(0,3),
		(0,4),
		(2,1),
		(2,3),
		(2,7),
		(6,3),
		(6,4),
		(6,7),
		(5,1),
		(5,4),
		(5,7)
		)

class glWidget(QGLWidget):
    
    def __init__(self, parent = None):
        super(glWidget, self).__init__(parent)

    def big_cube(self):
        scale = 30.0
        verticies = (
            (scale,-scale,-scale*3),
            (scale,scale,-scale*3),
            (-scale,scale,-scale*3),
            (-scale,-scale,-scale*3),
            (scale,-scale,scale*3),
            (scale,scale,scale*3),
            (-scale,-scale,scale*3),
            (-scale,scale,scale*3)
            )
        return verticies

    def robot_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()
        robot_dimensions, robot_location = self.get_data.get_robot_data()

        self.x = float(robot_dimensions["width"]) / 10
        self.y = float(robot_dimensions["height"]) / 10
        self.z = float(robot_dimensions["length"]) / 10

        self.locationX = robot_location["x"]
        self.locationY = robot_location["y"]
        self.locationZ = robot_location["z"]

        robot_location = [self.locationX,self.locationY,self.locationZ]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, robot_location

    def tipBox_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()
        tip_dimensions, tip_location = self.get_data.get_tipBox_data()

        self.x = float(tip_dimensions["width"]) / 10
        self.y = float(tip_dimensions["height"]) / 10
        self.z = float(tip_dimensions["length"]) / 10

        self.locationXtip = tip_location["x"]
        self.locationYtip = tip_location["y"]
        self.locationZtip = tip_location["z"]

        tip_location = [self.locationXtip,self.locationYtip,self.locationZtip]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, tip_location

    def tubeBox_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        tube_dimensions, tube_location = self.get_data.get_tubeRack_data()

        self.x = float(tube_dimensions["width"]) / 10
        self.y = float(tube_dimensions["height"]) / 10
        self.z = float(tube_dimensions["length"]) / 10

        self.locationXtube = tube_location["x"]
        self.locationYtube = tube_location["y"]
        self.locationZtube = tube_location["z"]

        tube_location = [self.locationXtube,self.locationYtube,self.locationZtube]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, tube_location
        
    def waste_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        waste_dimensions, waste_location = self.get_data.get_wasteContainer_data()

        self.x = float(waste_dimensions["width"]) / 10
        self.y = float(waste_dimensions["height"]) / 10
        self.z = float(waste_dimensions["length"]) / 10

        self.locationX = waste_location["x"]
        self.locationY = waste_location["y"]
        self.locationZ = waste_location["z"]

        waste_location = [self.locationX,self.locationY,self.locationZ]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, waste_location

    def micro_cube(self):

        self.get_data = CollectData()
        self.get_data.loadfile()

        micro_dimensions, micro_location = self.get_data.get_microPlate_data()

        self.x = float(micro_dimensions["width"]) / 10
        self.y = float(micro_dimensions["height"]) / 10
        self.z = float(micro_dimensions["length"]) / 10

        self.locationX = micro_location["x"]
        self.locationY = micro_location["y"]
        self.locationZ = micro_location["z"]

        micro_location = [self.locationX,self.locationY,self.locationZ]

        verticies = (
            (self.x,-self.y,-self.z),
            (self.x,self.y,-self.z),
            (-self.x,self.y,-self.z),
            (-self.x,-self.y,-self.z),
            (self.x,-self.y,self.z),
            (self.x,self.y,self.z),
            (-self.x,-self.y,self.z),
            (-self.x,self.y,self.z)
            )

        return verticies, micro_location

    def find_path(self):
        '''
        Path way graph definition:
        robot_vert[2] -> tube_vert[5] -> tipbox_vert[2] -> micro_vert[1] -> waste_vert[2]
        if robot clears these verticies it will not hit anything
        
        '''

        robot_vert,robot_loc = self.robot_cube()
        tipbox_vert,tipbox_loc = self.tipBox_cube()
        tube_vert,tube_loc = self.tubeBox_cube()
        micro_vert,micro_loc = self.micro_cube()
        waste_vert,waste_loc = self.waste_cube()

        #cast all to a list:

        robot_vert = list(robot_vert)
        robot_loc = list(robot_loc)
        tipbox_vert = list(tipbox_vert)
        tipbox_loc = list(tipbox_loc)
        tube_vert = list(tube_vert)
        tube_loc = list(tube_loc)
        micro_vert = list(micro_vert)
        micro_loc = list(micro_loc)
        waste_vert = list(waste_vert)
        waste_loc = list(waste_loc)

        #adjust for the location of the cubes:
        robot_vert[2] = (robot_vert[2][0]+robot_loc[0],robot_vert[2][1]+robot_loc[1],robot_vert[2][2]+robot_loc[2])
        tipbox_vert[2] = (tipbox_vert[2][0]+tipbox_loc[0],tipbox_vert[2][1]+tipbox_loc[1],tipbox_vert[2][2]+tipbox_loc[2])
        tube_vert[5] = (tube_vert[5][0]+tube_loc[0],tube_vert[5][1]+tube_loc[1],tube_vert[5][2]+tube_loc[2])
        micro_vert[1] = (micro_vert[1][0]+micro_loc[0],micro_vert[1][1]+micro_loc[1],micro_vert[1][2]+micro_loc[2])
        waste_vert[2] = (waste_vert[2][0]+waste_loc[0],waste_vert[2][1]+waste_loc[1],waste_vert[2][2]+waste_loc[2])

        '''itermediate_vert = []
        itermediate_vert.append((robot_vert[2][0]+robot_vert[7][0]) / 2 + robot_loc[0])
        itermediate_vert.append((robot_vert[2][1]+robot_vert[7][1]) / 2 + robot_loc[1])
        itermediate_vert.append((robot_vert[2][2]+robot_vert[7][2]) / 2 + robot_loc[2])''' #add intermediate step at some point

        lines = (robot_vert[2],tube_vert[5],tipbox_vert[2],micro_vert[1],waste_vert[2])
        path_way = ((0,1),(1,2),(2,3),(3,4))

        return lines, path_way

    '''def label_stuff(self, x,  y, z, font, text, r,  g , b , a):

        blending = False
        if glIsEnabled(GL_BLEND):
            blending = True

        glColor3f(1,1,1)
        glWindowPos2fv(x,y,z)
        for ch in text:
            glutBitmapCharacter(font , ctypes.c_int(ord(ch)))
        
        if not blending:
            glDisable(GL_BLEND)'''

    def paintGL(self):

    #define the verticies of the cube to be drawn:

        robot_verticies, robot_location = self.robot_cube()
        tipBox_verticies, tipBox_location = self.tipBox_cube()
        tubeBox_verticies, tubeBox_location = self.tubeBox_cube()
        waste_verticies, waste_location = self.waste_cube()
        micro_verticies, micro_location = self.micro_cube()

        vert_data = [robot_verticies, tipBox_verticies, tubeBox_verticies, waste_verticies, micro_verticies]
        local_data = [robot_location, tipBox_location,tubeBox_location,waste_location,micro_location]
        colorMatrix = ((1,1,1),(0,1,0),(1,0,0),(1,1,0),(1,0,1)) #draw the cubes all different colors

        glMatrixMode(GL_MODELVIEW)
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        for i in range(0, len(vert_data)):
            glLineWidth(1)
            glLoadIdentity()
            glTranslatef(local_data[i][0],local_data[i][1],local_data[i][2])
            glBegin(GL_LINES)
            for edge in edges:
                for vertex in edge:
                    glColor3f(colorMatrix[i][0],colorMatrix[i][1],colorMatrix[i][2])
                    glVertex3fv(vert_data[i][vertex])                    
            glEnd()
        #draw path to show user how robot will travel:
        lines, path_way = self.find_path()
        glLoadIdentity()
        glLineWidth(5)
        glBegin(GL_LINES)
        for path in path_way:
            for line in path:
                glColor3f(0,0,1) #draw the pathway blue
                glVertex3fv(lines[line])
        glEnd()

        #draw big cube to make look 3D:
        '''verticies = self.big_cube()
        glLoadIdentity()
        glLineWidth(2)
        #glTranslatef(0.0,0.0,-15.0)
        glBegin(GL_LINES)
        for edge in edges:
            for vertex in edge:
                glColor3f(1,1,1)
                glVertex3fv(verticies[vertex])
        glEnd()'''
        #self.label_stuff(1.0, 1.0,1.0, GLUT_BITMAP_9_BY_15 , "Hallo World" , 1.0 , 1.0 , 1.0 , 1.0)

        glutSwapBuffers()


    def initializeGL(self):
        glClearColor(0.0, 0.0, 0.0, 0.0)
        glClearDepth(1.0) 
        glLoadIdentity()
        glMatrixMode(GL_PROJECTION)
        gluPerspective(100.0,680/480,.1,50.0)
        glDepthFunc(GL_LESS)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)   
        glRotatef(0.0,0,0,-20)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = glWidget()
    window.show()
    sys.exit(app.exec_())