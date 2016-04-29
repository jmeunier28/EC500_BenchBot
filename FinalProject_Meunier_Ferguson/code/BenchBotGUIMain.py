"""
Open Source BenchBot GUI Application: BenchBot
Contains main function for entire application, GUI initilization and functions
Authors: 
1. Mike Ferguson www.mikeahferguson.com 3/26/2016
2. JoJo Meunier jmeunier@bu.edu 4/3/2016
License: MIT
Requires PyQt5 to be installed.
Anything Qt specific 
Uses Python 3.x
"""


import serial, time, struct
import sys, os, threading
from threading import Thread
from PyQt5.QtCore import pyqtSignal, QPoint, QSize, Qt
from PyQt5.QtWidgets import (QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QWidget,
                            QMessageBox, QTableWidget, QTableWidgetItem, QDialog, QHBoxLayout, QVBoxLayout,QOpenGLWidget, QSlider, QDialogButtonBox)
from PyQt5 import uic, QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QColor
import DobotInverseKinematics
import serial.tools.list_ports
import CreateCustomTaskGUI, ColonyTaskDialogGUI, PCRTaskDialogGUI, drawCubes, get_json_data
from CreateCustomTaskGUI import NewTaskDialogWindow
from PCRTaskDialogGUI import PCRTaskDialogWindow
from ColonyTaskDialogGUI import ColonyTaskDialogWindow
from drawCubes import glWidget
from get_json_data import CollectData
import time
import math
import itertools

class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):

        super(DobotGUIApp, self).__init__(parent)
        Ui_MainWindow, QtBaseClass = uic.loadUiType('BenchBotMain.ui')
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        #conenct menubar QAction File option to a QFileDialog 

        self.ui.new_file_action.triggered.connect(self.file_dialog_clicked)

        # connect WorkSpace Tab widgets 

        self.drawCubes = glWidget()
        self.ui.gridLayout.addWidget(self.drawCubes)
        self.setLayout(self.ui.gridLayout)
        self.ui.runButton.clicked.connect(self.run_path_way)
        self.ui.workspaceBox.addItem("PCR")
        self.ui.workspaceBox.addItem("Cloning")
        #self.ui.testButton.clicked.connect(self.set_path_boundaries)


        # connect to menubar QAction item options for Task bar Dialog Box

        self.new_task = NewTaskDialogWindow()
        self.task = PCRTaskDialogWindow()
        self.task2 = ColonyTaskDialogWindow()

        self.ui.new_cloning_task_action.triggered.connect(self.new_task.show) 
        hello = self.ui.new_pcr_cloning_task_action
        hello.triggered.connect(self.task.show)

        self.ui.new_colony_cloning_task_action.triggered.connect(self.task2.show)



        # connect serial ports list refresh button clicked event to the update serial port list function

        self.ui.pushButtonRefreshSerialPortsList.clicked.connect(self.update_serial_port_list)
        # connect move coordinates button clicked event to function to move to the coordinate specified
        self.ui.pushButtonMoveToCoordinate.clicked.connect(self.pushButtonMoveToCoordinate_clicked)
        # connect move to angles button clicked event to function to move to the angles specified
        self.ui.pushButtonMoveToAngles.clicked.connect(self.pushButtonMoveToAngles_clicked)

        # connect step buttons
        self.ui.pushButtonStepForward.clicked.connect(self.pushButtonStepForward_clicked)
        self.ui.pushButtonStepBackward.clicked.connect(self.pushButtonStepBackward_clicked)
        self.ui.pushButtonStepLeft.clicked.connect(self.pushButtonStepLeft_clicked)
        self.ui.pushButtonStepRight.clicked.connect(self.pushButtonStepRight_clicked)
        self.ui.pushButtonStepUp.clicked.connect(self.pushButtonStepUp_clicked)
        self.ui.pushButtonStepDown.clicked.connect(self.pushButtonStepDown_clicked)

        # connect serial port connection ("connect") button that's located in the configuration menu
        self.ui.pushButtonConnectToSerialPort.clicked.connect(self.pushButtonConnectToSerialPort_clicked)


        ###
        # Define application class variables.
        ###

        # create a variable for the arduino serial port object
        self.arduinoSerial = None

        # current position variables
        self.currentXPosition = 0
        self.currentYPosition = 0
        self.currentZPosition = 0

        # if there is a port available whose description starts with "Arduino", try to connect to it
        self.try_to_connect_to_an_arduino_port_on_application_start()
        # populate the serial ports list widget
        self.update_serial_port_list()


    def file_dialog_clicked(self):

        file_dialog = QFileDialog()
        file_dialog.setFileMode(QFileDialog.AnyFile)
        file = file_dialog.getOpenFileName(self, 'Open File')
        print ('Path to file is:\n'), (file)

    def pushButtonMoveToCoordinate_clicked(self):

        # get moveTo coordinate text values from lineedits
        moveToX = self.ui.lineEditMoveToX.text()
        moveToY = self.ui.lineEditMoveToY.text()
        moveToZ = self.ui.lineEditMoveToZ.text()

        # check that the values were not empty
        if (moveToX == '' or moveToY == '' or moveToZ == ''):
            self.show_a_warning_message_box('Missing a coordinate value.',
                                            'Check that you entered a value for each dimension.',
                                            'Invalid coordinate for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToXFloat = float(moveToX)
            moveToYFloat = float(moveToY)
            moveToZFloat = float(moveToZ)
        except Exception as e:
            self.show_a_warning_message_box('Check that your coordinate values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Coordinate value conversion to float error')
            return

        self.move_to_cartesian_coordinate(moveToXFloat, moveToYFloat, moveToZFloat)

    def define_threeDspace(self):

        '''
        Robot is free to move within this 3D space
        '''

        minimumX = -100
        minimumY = -100
        minimumZ = -50
        maximumX = 100
        maximumY = 100
        maximumZ = 250

        nodes = []

        for x in range(minimumX,maximumX):
            for y in range(minimumY,maximumY):
                for z in range(minimumZ,maximumZ):
                    nodes.append([x,y,z]) #add all nodes in space to list
        return nodes



    def set_path_boundaries(self):

        '''
        Implement checks to ensure robot does not hit things that we do not want it to hit
        Takes dimensions and coordinates from user and then transfroms these to verticies of
        2D box. The z coordinate of each item remains fixed and is what finally maps this
        boundary to 3D. Each time the user tries to move the robot the path boundary is checked
        ***** Assumed that user defined coordinates are the upper left hand corner of each box ****
        '''

        space = self.define_threeDspace()

        #get coordinate locations of objects in standard json file
        self.get_data = CollectData()
        self.get_data.loadfile()
        #get coordinate locations of each item
        bot_loc, tube_loc, tip_loc, micro_loc, waste_loc = self.get_data.get_real_coordinates()
        location_array = [bot_loc,tube_loc,tip_loc,micro_loc,waste_loc]

        all_nodes = []
        top_nodes =[]
        bottom_nodes=[]
        robot_dimensions, _ = self.get_data.get_robot_data()
        tip_dimensions, _ = self.get_data.get_tipBox_data()
        tube_dimensions, _ = self.get_data.get_tubeRack_data()
        waste_dimensions, _ = self.get_data.get_wasteContainer_data()
        micro_dimensions, _ = self.get_data.get_microPlate_data()

        dimension_array = [robot_dimensions,tube_dimensions,tip_dimensions,micro_dimensions,waste_dimensions]

        #define all the places where the BenchBot should not go past

        for i in range(0,len(location_array)):
            for j in range(0,4):
                coordinates = self.create_twoDbox(dimension_array[i],location_array[i])
                top_nodes.append([coordinates[j][0],coordinates[j][1], location_array[i]["z"]-1]) #add all new x,y coords of boxes in our path
                bottom_nodes.append([coordinates[j][0],coordinates[j][1],location_array[i]["z"]-1]) # draw bottom of cube

        for i in range(0,len(top_nodes)):
            if bottom_nodes[i][0] <= space[i][0] <= top_nodes[i][0]: # if x coord in range
                if bottom_nodes[i][1] <= space[i][1] <= top_nodes[i][1]: #if y coord in range
                    if bottom_nodes[i][2] <= space[i][2] <= top_nodes[i][2]: #if z coord in range
                        space.remove(top_nodes[i]) #remove these from access to robot
                        space.remove(bottom_nodes[i])

        return top_nodes,bottom_nodes #return the spaces where the robot shouldnt go

    def create_twoDbox(self, dimensions, location):

        '''
        Map length and width of object to make 2D square
        return the coordinate locations of the four corners
        of the box
        '''
        self.length = dimensions["length"]
        self.width = dimensions["width"]
        self.locationX = location["x"]
        self.locationY = location["y"]

        #create new coordinate locations
        initialCoord = [self.locationX, self.locationY]
        upper_right_vert = [self.locationX + self.width,self.locationY] #coordinate y wont change
        lower_right_vert = [self.locationX+self.width,self.locationY - self.length]
        lower_left_vert = [self.locationX ,self.locationY -self.length] #coordinate x wont change
        #print("box dimensions are:\n")
        #print(initialCoord,upper_right_vert,lower_left_vert,lower_right_vert)
        coordinate_array = [initialCoord,upper_right_vert,lower_right_vert,lower_left_vert]

        return coordinate_array



    def run_path_way(self):

        '''
        Path of robot for now will follow that which is set in the drawCubes python script
        and shown by the blue line in the WorkSpace tab 3D model
        Graph Def:
        robot_vert[2] -> tube_vert[5] -> tipbox_vert[2] -> micro_vert[1] -> waste_vert[2]
        if robot clears these verticies it will not hit anything
        '''

        self.get_data = CollectData()
        self.get_data.loadfile()
        #get coordinate locations of each item
        bot_loc, tube_loc, tip_loc, micro_loc, waste_loc = self.get_data.get_real_coordinates()
        location_array = [bot_loc,tube_loc,tip_loc,micro_loc,waste_loc]
        logic_array = [False,True]

        for i in range(0,len(location_array)-1):
            for j in range(0,len(logic_array)):
                self.defined_path(location_array[0],location_array[i+1],logic_array[j])
        #last call puts robot back at initial position
        self.defined_path(bot_loc,waste_loc,reverse=True)


    def defined_path(self,move_from,move_to,reverse):

        '''
        coordinates are defined at location of right hand corner of object
        pathways are defined by graph
        when reverse is set to True move_to position will move back to move_from position
        when reverse is set to False the robot will move from location one
        to location two with forward movements
        '''
        self.location_one= move_from #initial pos
        self.location_two = move_to #move to this pos
        self.reverse = reverse
        stop = False

        result = self.check_path(self.location_one,self.location_two)
        if result:
            stop = True
            print("BenchBot is redirecting path...")
            old_location, new_location = self.fix_path(self.location_two["x"],self.location_two["y"],self.location_two["z"])
            self.defined_path(old_location, new_location, reverse=False) #recursively call to move again once object has been avoided

        if not stop:
            if self.reverse is False:

                newXPosition = self.location_two["x"]
                newYPosition = self.location_two["y"]
                newZPosition = self.location_two["z"]
                print("sending robot to %f %f %f" % (newXPosition,newYPosition,newZPosition))
                self.move_to_cartesian_coordinate(newXPosition,newYPosition,newZPosition)

            elif self.reverse is True:
                print("Setting BenchBot now...")

                currentXPosition = self.location_one["x"]
                currentYPosition = self.location_one["y"]
                currentZPosition =self.location_one["z"]
                self.move_to_cartesian_coordinate(currentXPosition,currentYPosition,currentZPosition)

    def check_points(self, loc_one,loc_two,point):

        '''
        Method that checks if point in 3d space lies between the start location and end
        location. if it does the var is set to True. If it does not returns False
        '''

        result = self.cross_product(loc_one,loc_two,point)
        var = False
        if result == 0:
            result = self.dot_product(loc_one,loc_two,point)
            if result > 0:
                distance = self.distance(loc_one,loc_two,point)
                if result < math.pow(distance,2):
                    var = True
        return var

    def check_path(self,move_from,move_to):

        '''
        Check the path between two points if there exists obstacles in
        between Bench Bot must stop and be redirected
        '''

        self.x_orig = move_from["x"]
        self.y_orig = move_from["y"]
        self.z_orig = move_from["z"]

        self.x_new = move_to["x"]
        self.y_new = move_to["y"]
        self.z_new = move_to["z"]

        loc_one = [self.x_orig,self.y_orig,self.z_orig]
        loc_two = [self.x_new,self.y_new,self.z_new]
        avaliable_space = self.set_path_boundaries()
        for i in range(0,len(avaliable_space)):
            self.check_points(loc_one,loc_two,avaliable_space[i])


    def fix_path(self, x,y,z):

        '''
        Take the location user is trying to move to and factor in obstacles
        present... we will find the best way to move up and around the obstacle
        so as to not move through the obstacle
        '''
        self.coordinates = [x,y,z]
        top_node,bottom_node = self.set_path_boundaries()
        for i in range(0,len(top_node)):
            mindistance = 10000 #some very high number
            if self.distance(self.coordinates,top_node[i]) < mindistance:
                mindistance = self.distance(self.coordinates,top_node[i])
            else:
                # if the minimum distance from free space to blocked space is at minimum
                # then we can travel to that coordinate
                newXcoord = top_node[i][0]
                newYcoord = top_node[i][1]
                newZcoord = top_node[i][2]

        coordDict = {"x":newXcoord,"y":newYcoord,"z":newZcoord}
        oldCoordDict = {"x":x,"y":y,"z":z}

        self.defined_path(oldCoordDict,coordDict, reverse=False) #now move to this location to avoid obstacle
        return oldCoordDict, coordDict


    def distance(self,p0,p1):
        return math.sqrt(math.pow(p1[0]-p0[0],2)+math.pow(p1[1]-p0[1],2)+math.pow(p1[2]-p0[2],2))

    def cross_product(self,p0,p1,point):
        diff = [p0[0] - point[0],p0[1]-point[1],p0[2]-point[2]]
        diff2 = [point[0] - p1[0],point[1]-p1[1],point[2]-p1[2]]
        return list(itertools.product(diff,diff2))

    def dot_product(self,p0,p1,point):
        diff = [p0[0] - point[0],p0[1]-point[1],p0[2]-point[2]]
        diff2 = [point[0] - p1[0],point[1]-p1[1],point[2]-p1[2]]
        return sum(p*q for p,q in zip(diff, diff2))

    def move_to_cartesian_coordinate(self, moveToXFloat, moveToYFloat, moveToZFloat):

        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors (likely due to coordinates out of range being input) NEED TO ADDRESS THIS AT SOME POINT
        stop = False
        coord_location = [moveToXFloat,moveToYFloat,moveToZFloat]

        if stop is False:
            try:
                moveToAngles = DobotInverseKinematics.convert_cartesian_coordinate_to_arm_angles(moveToXFloat,moveToYFloat,moveToZFloat,
                DobotInverseKinematics.lengthUpperArm, DobotInverseKinematics.lengthLowerArm, DobotInverseKinematics.heightFromBase)
            except Exception as e:
                self.show_a_warning_message_box('Unknown inverse kinematics error. Check that your coordinate values are within the robot\'s range. '
                                                + 'The error is shown below:',
                                                    repr(e),
                                                    'Inverse Kinematics Error')
                return


            # check that inverse kinematics did not run into a range error. If it does, it should return -999 for all angles, so check that.
            if(moveToAngles[0] == -999):
                self.show_a_warning_message_box('Desired coordinate is outside of the robot\'s range.',
                                                    'It is impossible for the robot arm to reach the coordinate you specified. Build longer arms if this range is desired.'
                                                    + 'You will probably need higher torque stepper motors as well.',
                                                    'Inverse Kinematics Range Error')
                return

            print('ik base angle')
            print(moveToAngles[0])
            print('ik upper angle')
            print(moveToAngles[1])
            print('ik lower angle')
            print(moveToAngles[2])


            moveToUpperArmAngleFloat = moveToAngles[1]
            moveToLowerArmAngleFloat = moveToAngles[2]

            transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
            #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
            # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
            # and the fact that the lower arm angle is calculated relative to the upper arm angle.
            transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90
            print('transformed upper angle:')
            print(transformedUpperArmAngle)
            print('transformed lower angle:')
            print(transformedLowerArmAngle)


            #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
            # need to pass in real world angles
            # real world base and upper arm angles are those returned by the ik function.
            # real world lower arm angle is -1 * transformedLowerArmAngle
            if(self.check_for_angle_limits_is_valid(moveToAngles[0], moveToAngles[1], -1 * transformedLowerArmAngle)):
                # continue on to execute the arduino code
                pass
            else:
                # exit, don't move. the check function takes care of the warning message
                return



            # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
            # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
            # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
            # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
            self.arduinoSerial.write( struct.pack('f', moveToAngles[0]))
            self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
            self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

            # if movement was successful, update the current position
            # note that float values are rounded to 3 decimal places for display and converted to strings
            self.ui.labelBaseAngleValue.setText(str(round(moveToAngles[0],3)))
            self.ui.labelUpperArmAngleValue.setText(str(round(moveToAngles[1],3)))
            self.ui.labelLowerArmAngleValue.setText(str(round(moveToAngles[2],3)))

            self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
            self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
            self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
            self.currentXPosition = moveToXFloat
            self.currentYPosition = moveToYFloat
            self.currentZPosition = moveToZFloat

            # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
            # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
            # this was quick and dirty and is prone to fatal errors (fatal for this program that is).

            for i in range(0,15):
                print ( self.arduinoSerial.readline() )




    def pushButtonMoveToAngles_clicked(self):
        # get moveTo angle text values from lineedits
        moveToBaseAngle = self.ui.lineEditMoveToBaseAngle.text()
        moveToUpperArmAngle = self.ui.lineEditMoveToUpperArmAngle.text()
        moveToLowerArmAngle = self.ui.lineEditMoveToLowerArmAngle.text()

        # check that the values were not empty
        if (moveToBaseAngle == '' or moveToUpperArmAngle == '' or moveToLowerArmAngle == ''):
            self.show_a_warning_message_box('Missing a angle value.',
                                            'Check that you entered a value for each angle.',
                                            'Invalid angles for move to command')
            return

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            moveToBaseAngleFloat = float(moveToBaseAngle)
            moveToUpperArmAngleFloat = float(moveToUpperArmAngle)
            moveToLowerArmAngleFloat = float(moveToLowerArmAngle)
        except Exception as e:
            self.show_a_warning_message_box('Check that your angle values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Angle value conversion to float error')
            return


        #check that the final angles are mechanically valid. note that this check only considers final angles, and not angles while the arm is moving
        # need to pass in real world angles
        # real world base, upper, and lower arm angles are those entered in the text box.
        if(self.check_for_angle_limits_is_valid(moveToBaseAngleFloat, moveToUpperArmAngleFloat, moveToLowerArmAngleFloat)):
            # continue on to execute the arduino code
            pass
        else:
            # exit, don't move. the check function takes care of the warning message
            return


        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        transformedLowerArmAngle = moveToLowerArmAngleFloat*-1
        print('transformed upper angle:')
        print(transformedUpperArmAngle)
        print('transformed lower angle:')
        print(transformedLowerArmAngle)

        # INSERT CODE HERE TO SEND MOVEMENT COMMANDS TO ARDUINO
        # I'm simply writing three floats to the arduino. See the following two stack exchange posts for more details on this:
        # http://arduino.stackexchange.com/questions/5090/sending-a-floating-point-number-from-python-to-arduino
        # ttps://arduino.stackexchange.com/questions/3753/how-to-send-numbers-to-arduino-uno-via-python-3-and-the-module-serial
        self.arduinoSerial.write( struct.pack('f',moveToBaseAngleFloat) )
        self.arduinoSerial.write( struct.pack('f',transformedUpperArmAngle) )
        self.arduinoSerial.write( struct.pack('f',transformedLowerArmAngle) )

        # if movement was successful, update the current position
        # note that float values are rounded to 3 decimal places for display and converted to strings
        self.ui.labelBaseAngleValue.setText(str(round(moveToBaseAngleFloat,3)))
        self.ui.labelUpperArmAngleValue.setText(str(round(moveToUpperArmAngleFloat,3)))
        self.ui.labelLowerArmAngleValue.setText(str(round(moveToLowerArmAngleFloat,3)))

        """
        # need to implement forward kinematics
        self.ui.labelCurrentXValue.setText(str(round(moveToXFloat,3)))
        self.ui.labelCurrentYValue.setText(str(round(moveToYFloat,3)))
        self.ui.labelCurrentZValue.setText(str(round(moveToZFloat,3)))
        """

        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        #for i in range(0,15 ):
        #    print ( self.arduinoSerial.readline() )


    # angles passed as arguments here should be real world angles (horizontal = 0, below is negative, above is positive)
    # i.e. they should be set up the same way as the unit circle is
    def check_for_angle_limits_is_valid(self, baseAngle, upperArmAngle, lowerArmAngle):

        returnBool = True
        # implementing limit switches and IMUs will make this function more accurate and allow the user to calibrate the limits
        # necessary for this function.
        # Not currently checking the base angle

        # check the upperArmAngle
        # max empirically determined to be around 107 - 108 degrees. Using 105.
        # min empirically determined to be around -23/24 degrees. Using -20.
        if (-20 <= upperArmAngle <= 105):
            # do nothing, return value already initialized true
            pass
        else:
            self.show_a_warning_message_box('Upper arm angle out of range.',
                                            'Upper arm must have an angle between -20 and 105 degrees. It is mechanically constrained.',
                                            'Upper Arm Range Error')
            returnBool = False

        # check the lowerArmAngle
        # the valid Lower Arm angle is dependent on the upper arm angle. The real world angle of the lower arm (0 degrees = horizontal) needs to be evaluated.
        # min empirically determined to be around -105 degrees. Using -102.
        # max empirically determined to be around 21 degrees. Using 18.


        if (-102 <= lowerArmAngle <= 18):
            # do nothing, already initialized true
            pass
        else:
            self.show_a_warning_message_box('Lower arm angle out of range.',
                                            'Lower arm must have a real world angle between -102 and 18 degrees. It is mechanically constrained.',
                                            'Lower Arm Range Error')
            returnBool = False



        minAngleBetweenArms = ((180 - 81) + -79)
        maxAngleBetweenArms = ((180 - 51) + 21)
        angleBetweenArms = ((180 - upperArmAngle) + lowerArmAngle)

        if (minAngleBetweenArms <= angleBetweenArms <= maxAngleBetweenArms):
            # do nothing, already initialized true
            pass
        else:
            self.show_a_warning_message_box('Angle between arms out of range out of range.',
                                            'Angle between arms (the inner "elbow" angle) must be between ' +
                                            str(minAngleBetweenArms) + ' and ' + str(maxAngleBetweenArms) + '.' +
                                            ' It is mechanically constrained.',
                                            'Inner Elbow Angle Range Error')
            returnBool = False


        return returnBool


    def pushButtonStepForward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepForwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition + stepSizeFloat, self.currentYPosition, self.currentZPosition)

    def pushButtonStepBackward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepBackwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition - stepSizeFloat, self.currentYPosition, self.currentZPosition)

    def pushButtonStepLeft_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepLeftSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition - stepSizeFloat, self.currentZPosition)

    def pushButtonStepRight_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepRightSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition + stepSizeFloat, self.currentZPosition)

    def pushButtonStepUp_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepUpSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition, self.currentZPosition + stepSizeFloat)

    def pushButtonStepDown_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepDownSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.move_to_cartesian_coordinate(self.currentXPosition, self.currentYPosition, self.currentZPosition - stepSizeFloat)


    def convert_step_size_numerical_text_to_number_plus_check_is_valid(self, stepSizeText):
        # check that the values were not empty
        if (stepSizeText == ''):
            self.show_a_warning_message_box('No step size value was entered.',
                                            'Check that you entered a value for the size of the step you tried to take.',
                                            'No step size value entered.')
            return None

        # convert values from string to float and ensure that the values entered were actually numbers
        try:
            stepSizeFloat = float(stepSizeText)
        except Exception as e:
            self.show_a_warning_message_box('Check that your step size values are numbers and not letters. The code '
                                            + 'error is shown below:',
                                                repr(e),
                                                'Step size value conversion to float error')
            return None

        if (stepSizeFloat < 0):
            self.show_a_warning_message_box('Step sizes can only be positive.',
                                            'You entered a negative step size. Please enter a positive one. The button determines the direction.',
                                            'Invalid Step Size')
            return None

        return stepSizeFloat



    def update_serial_port_list(self):
        """
        Updates the serial ports qtablewidget in the configuration tab with qlistwidgetitems containing text values
        that are the names of the serial port. Calls function to get a list of ports. This function works for
        Windows, Mac, and Linux
        """
        # must clear the listwidget of any previous items
        self.ui.tableWidgetSerialPorts.clear()
        # get a list of all serial ports (all, not just open ones)
        listOfSerialPorts = list(serial.tools.list_ports.comports())

        # initialize table widget with number of rows, columns. add column titles
        self.ui.tableWidgetSerialPorts.setRowCount(len(listOfSerialPorts))
        self.ui.tableWidgetSerialPorts.setColumnCount(2)
        self.ui.tableWidgetSerialPorts.setHorizontalHeaderLabels(['Serial Port', 'Description'])
        # add each serial port name (string) to the list as a qlistwidgetitem with the string value
        for i,port in enumerate(listOfSerialPorts):
            self.ui.tableWidgetSerialPorts.setItem(i, 0, QTableWidgetItem(port[0]))
            self.ui.tableWidgetSerialPorts.setItem(i, 1, QTableWidgetItem(port[1]))
        # sort the list of ports by description so the "Arduino" described port is likely to show up first
        self.ui.tableWidgetSerialPorts.sortItems(1, QtCore.Qt.AscendingOrder)



    def connect_to_serial_port(self, portName, baudRate):

        connectionStatus = True
        previousArduinoSerial = self.arduinoSerial

        # try to connect to the arduino serial port
        try:
            # update the variable for arduino serial. first argument is port name
            # the second argument is the baud rate, or how fast the serial connection is. needs to be the same in firmware
            # This will reset the Arduino, and make the LED flash once.
            self.arduinoSerial = serial.Serial(portName, baudRate)
            # Must give Arduino time to reset
            #  "Any time less than this does not seem to work..." quoted from someone else on a blog
            time.sleep(1.5)
            # I think this clears the input stream?
            # I think I may not need this, but I'm including it anyways, against the advice of someone on stackexchange.
            # I don't think it does anything bad, just one more line of unnecessary code.
            self.arduinoSerial.flushInput()
        except Exception as e:
            self.show_a_warning_message_box('Unknown error connecting to the arduino serial port named: ' + portName +
                                            '. Perhaps the port is busy (being used by another application)?' +
                                            ' Code error shown below:',
                                            repr(e),
                                            'Arduino Serial Port Connection Error')
            self.arduinoSerial = previousArduinoSerial
            connectionStatus =  False

        if(connectionStatus):
            self.update_serial_port_connection_status_info(connectionStatus)
            self.ui.pushButtonConnectToSerialPort.setText('Disconnect')

        # returns whether or not connection was successful
        return connectionStatus

    # need to add some checks here and gui updates, essential for good usability
    def disconnect_from_serial_port(self):
        # this close command technically shouldn't be needed since serial object should be destroyed in the text line when set to none
        # and the serial object's destructor closes the port. Including it to be safe
        self.arduinoSerial.close()
        self.arduinoSerial = None
        self.update_serial_port_connection_status_info(False)
        self.ui.pushButtonConnectToSerialPort.setText('Connect')

    def update_serial_port_connection_status_info(self, connectionStatus):
        connected = connectionStatus #redundant, but makes code nicer to read
        # update gui connection info depending on the connection status
        if(connected):
            connectedLabelsStyleSheet = (
                    """
                        color: green
                    """
                )
            self.ui.labelSerialPortConnectionStatus.setText('Connected')
            self.ui.labelSerialPortConnectionStatus.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusPortName.setText(self.arduinoSerial.port)
            self.ui.labelSerialPortConnectionStatusPortName.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusBaudRate.setText(str(self.arduinoSerial.baudrate))
            self.ui.labelSerialPortConnectionStatusBaudRate.setStyleSheet(connectedLabelsStyleSheet)
        else:
            connectedLabelsStyleSheet = (
                    """
                        color: red
                    """
                )
            self.ui.labelSerialPortConnectionStatus.setText('Not Connected')
            self.ui.labelSerialPortConnectionStatus.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusPortName.setText('No Connection')
            self.ui.labelSerialPortConnectionStatusPortName.setStyleSheet(connectedLabelsStyleSheet)
            self.ui.labelSerialPortConnectionStatusBaudRate.setText('No Connection')
            self.ui.labelSerialPortConnectionStatusBaudRate.setStyleSheet(connectedLabelsStyleSheet)



    def pushButtonConnectToSerialPort_clicked(self):


        pushButtonText = self.ui.pushButtonConnectToSerialPort.text()

        # if button is in disconnect state (meaning port already connected), execute disconnect functions
        # otherwise, execute connect functions
        if (pushButtonText == 'Disconnect'):
            self.disconnect_from_serial_port()
        else:
            # the treewidget.selectedItems() function returns a list of selected items of type QTreeWidgetItem.
            selectedSerialPorts = self.ui.tableWidgetSerialPorts.selectedItems()
            # if there is a selected port, try to connect to it
            if(len(selectedSerialPorts)):

                try:
                    baudRate = int(self.ui.lineEditBaudRate.text())
                except Exception as e:
                    self.show_a_warning_message_box('Check that your baud rate value is a number and does not contain letters.' +
                                                    'The code error is shown below:',
                                                    repr(e),
                                                    'Baud rate value conversion to int error')
                    return

                selectedSerialPortName = selectedSerialPorts[0].text()
                self.connect_to_serial_port(selectedSerialPortName, baudRate)

            else:
                self.show_a_warning_message_box('No Port Selected.','Select a port.', 'No Selected Port')


    def try_to_connect_to_an_arduino_port_on_application_start(self):

        baudRateValid = True
        connected = False
        possibleArduinoPortName = ''

        # get a list of all serial ports (all, not just open ones)
        listOfSerialPorts = list(serial.tools.list_ports.comports())

        # get the name of a potential arduino connected port based on the description
        for port in listOfSerialPorts:
            # if the description starts contains the word  'arduino', try to connect to it using a default baud rate as defined in the ui
            if ('arduino' in  port[1].lower()):
                possibleArduinoPortName = port[0]

        try:
            baudRate = int(self.ui.lineEditBaudRate.text())
        except Exception as e:
            self.show_a_warning_message_box('Check that your baud rate value is a number and does not contain letters.' +
                                        'The code error is shown below:',
                                        repr(e),
                                        'Baud rate value conversion to int error')
            baudRateValid = False

        if (baudRateValid and possibleArduinoPortName != ''):
            if (self.connect_to_serial_port(possibleArduinoPortName, baudRate)):
                connected = True



        if (connected == False):
            self.show_a_warning_message_box('Warning, the arduino is not connected to the computer.',
                                            'Try to connect to a port in the configuration settings. Ensure that the' +
                                            ' arduino is connected to the computer and not being used by another application.',
                                            'No Arduino Found on Application Start')


    def initialize_gui_upon_new_serial_port_connection(self):
        # update current position variables
        self.currentXPosition = 0
        self.currentYPosition = 0
        self.currentZPosition = 0

        # will need to update gui text

        # also note that the firmware will reset to think its at home
        # need to address this through limit switches and/or IMUs




    def show_a_warning_message_box(self, text, infoText, windowTitle):
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Warning)
            msg.setText(text)
            msg.setInformativeText(infoText)
            msg.setWindowTitle(windowTitle)
            msg.exec()

# main function
if __name__ == '__main__':
    # These first three lines initialize the Qt application/GUI.
    app = QApplication(sys.argv)
    window = DobotGUIApp()
    # displays the GUI
    window.show()

    # write whatever set up or logic code you want to here.


    # Says to exit the whole code when the Qt application is closed. app.exec returns some value when qt app quits
    sys.exit(app.exec_())

