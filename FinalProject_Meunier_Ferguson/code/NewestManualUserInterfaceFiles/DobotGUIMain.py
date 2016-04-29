"""
Open Source Dobot GUI Application: open-dobot-gui
Contains main function for entire application, GUI initilization and functions
First Author: Mike Ferguson www.mikeahferguson.com 3/26/2016
Additional Authors (Add your name below):
1.
License: MIT

Requires PyQt5 to be installed.

Anything Qt specific (functions, classes, etc.. starts with the letter Q)

The GUI template is created by a program called QtDesigner, which spits out a .ui file, which is basically just an XML
file that describes the GUI elements. In the designer, the elements are given object names. The first step of the code
below is to load the .ui file and get a reference to it.


     //\\
    //  \\  <-- lower arm
   //    \\
  // <-- upper arm
 ||
 || <-- base
The Dobot has 3 stepper motors. One for the base, one for the upper arm, and one for the lower arm. See the diagram above
for which arm is which. Any variables mentioning some variation of base, upper, and lower are reffering to the
base, upper arm, and lower arm respectively.


Just reminding myself here that there are some issues I need to address below.

Issue 1
###
        ###
        ###
        ###
        #I think that not addressing this might result in some small systematic error that might build up over a long time.
        #I really need to address this, but I'm out of time and the current algorithm gives a close enough approximation for testing.
        #NEED TO ADDRESS THIS!!!!!!
        ###
        ###
        ###
Issue 2
        #DO NOT CALL THE MOVE ANGLES FUNCTION (DO NOT CLICK THE MOVE ANGLES BUTTON) AS OF NOW,
    #THIS IS OLD CODE THAT DID WORK. I DRASTICALLY ALTERED THE STRUCTURE OF THE SOFTWARE, SO THIS ALMOST CERTAINLY DOESN'T WORK NOW.
    #UNPREDICTABLE BEHAVIOR WILL RESULT. I NEED TO ADDRESS THIS.
Issue 3
    #these functions are messed up for small step sizes, like 1. possibly due to the unaddressed linear move algorithm. NEED TO ADDRESS THIS
"""


import serial, time, struct, math
import sys, os, threading
from threading import Thread
from PyQt5.QtWidgets import QApplication, QMainWindow, QListWidget, QListWidgetItem, QFileDialog, QMessageBox, QTableWidget, QTableWidgetItem
from PyQt5 import uic, QtCore, QtGui
import DobotInverseKinematics
import serial.tools.list_ports

# This loads the GUI from the .ui file that is created by QtDesigner. The .ui file should be in the same folder as this
# python file (or specify different path).
Ui_MainWindow, QtBaseClass = uic.loadUiType('DobotMainUi.ui')


# Here, a class is defined to represent the entire GUI. It is derived from a Qt class named QMainWindow, which
# corresponds to the GUI type specified in QtDesigner. All of the functional aspects (as opposed to design aspects) of
# the GUI are defined in this class. For example, what happens when a user presses a button.
class DobotGUIApp(QMainWindow):
    # class initialization function (initialize the GUI)
    def __init__(self, parent=None):
        # I'm a python noob, but I'm guessing this means initialize the parent class. I imagine all the super classes
        # have to be explicitly initialized.
        super(DobotGUIApp, self).__init__(parent)
        # This sets up the ui variable of this class to refer to the loaded .ui file.
        self.ui = Ui_MainWindow()
        # This call is required. Does whatever set up is required, probably gets references to the elements and so on.
        self.ui.setupUi(self)

        # Anything named after self.ui. (e.g. self.ui.x) means you are referring to an object name x that corresponds
        # to an element in the gui. Qt uses a signals and slots framework to define what happens in response to UI
        # events. You'll have to look that up if you're interested in it.


        ###
        # Connect gui elements in the .ui file to event handling functions.
        ###

        # connect serial ports list refresh button clicked event to the update serial port list function
        self.ui.pushButtonRefreshSerialPortsList.clicked.connect(self.update_serial_port_list)
        # connect move coordinates button clicked event to function to move to the coordinate specified
        #note that lambda allows us to connect to the function AND pass an argument
        #super useful. False means it was called form the moveToCoordinate push button
        self.ui.pushButtonMoveToCoordinate.clicked.connect(lambda: self.pushButtonMoveToCoordinate_clicked(False))
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

        #change to true to print debugging info
        #can change local debug variables only to print out only some
        self.masterDebug = False;

        ###
        # Define application class variables.
        ###

        # create a variable for the arduino serial port object
        self.arduinoSerial = None

        # current position variables
        startXYZ = DobotInverseKinematics.get_cartesian_coordinate_from_angles_using_forward_kinematics(0,90,0)
        self.currentXPosition = startXYZ[0]
        self.currentYPosition = startXYZ[1]
        self.currentZPosition = startXYZ[2]

        #incremental position variables (may change the code architecture and get rid fo these later)
        self.incrementalStepToX = 0
        self.incrementalStepToY = 0
        self.incrementalStepToZ = 0

        #a local debug variable. change it to False or True if want only the debugginng code in this function to run.
        debug = self.masterDebug
        if(debug):
         print(startXYZ)

        #stepperPositions holds the stepper motor step numbers at each point on a line. For example, a 200 step/rev
        #stepper motor without microstepping might have a position of 10, which is 10 steps away from its starting position
        #in some direction. -10 would be 10 steps in the opposite direction
        self.stepperPositionsOnALine = []

        #these variables keep track of each stepper motor's stepper position (see comments immediately above this one for
        # a description of stepper position)
        self.basePos = 0
        self.upperPos = 0
        self.lowerPos = 0

        #stepperSequenceToDrawALine holds the actual stepper sequence data for a line (specifies whether or not a stepper
        #should move). see the readme on github
        self.stepperSequenceToDrawALine = []



        #START STEPPER MOTOR SETTINGS
        #The NEMA 17 stepper motors that Dobot uses are 200 steps per revolution.
        stepperMotorStepsPerRevolution = 200
        #I'm using a ramps 1.4 board with all 3 jumpers connected, which gives me a microstepping mode of 1/16.
        #In other words, the motor is set up so it takes 16 steps to move 1 of the default steps.
        #microstepping jumper guide for the a4988 stepper driver: https://www.pololu.com/product/1182
        baseMicrosteppingMultiplier = 16
        upperArmMicrosteppingMultiplier = 16
        lowerArmMicrosteppingMultiplier = 16
        """
        //The NEMA 17 stepper motors Dobot uses are connected to a planetary gearbox, the black cylinders.
        //It basically just means that the stepper motor is rotating a smaller gear. That smaller gear is in turn rotating a larger one.
        //The gears are set up such that rotating the smaller gear by some number of degrees rotates the larger one by a tenth of that number of degrees (10:1 ratio)
        //The bigger gears are actually moving the arm, so the number of steps is increased by a factor of 10 (the gear ratio).
        """
        stepperPlanetaryGearBoxMultiplier = 10
        #This variable will hold the aqctual number of steps per revolution and is calculate by multiplying the three previous variables together.
        #calculate the actual number of steps it takes for each stepper motor to rotate 360 degrees
        self.baseActualStepsPerRevolution = stepperMotorStepsPerRevolution * baseMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        self.upperArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * upperArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        self.lowerArmActualStepsPerRevolution = stepperMotorStepsPerRevolution * lowerArmMicrosteppingMultiplier * stepperPlanetaryGearBoxMultiplier
        #END STEPPER MOTOR SETTINGS


        ###
        # General initialization code
        ###

        # if there is a port available whose description starts with "Arduino", try to connect to it
        self.try_to_connect_to_an_arduino_port_on_application_start()
        # populate the serial ports list widget
        self.update_serial_port_list()




        ###
        # Useful Settings to adjust
        ###

        #determines how many slices to slice up a linear movement path into. More slices means straighter and smoother
        #lines. may increase processing time and/or number of steps to take.
        self.linearLineResolution = 100

        #determines the maximum number of step sequence data (3-tuples) that one can send at a time. This determines
        #maximum packet size for sending data to the arduino. see the readme in the github repository for more on the
        #serial communication protocol. 1000 means can send 1000 3-tuples, or 3000 characters at a time. There is only
        #about 8000 characters worth of memory on the arduino!! It complains at storing around 5000 characters.
        #NOTE: If you change this, you MUST change the corresponding MAX_INPUT variable in the arduino firmware
        self.maximumStepSequenceTuplesToSendAtATime = 1000



    #called when the move button in the move to coordinate group on the manual control tab is clicked
    #also called by the incremental step buttons. I will probably break this function up at some point.
    def pushButtonMoveToCoordinate_clicked(self, calledFromIncrementalStepButton):

        #initalize move to float coordinate values with dummy numbers easily detected if something goes wrong
        moveToXFloat = -992
        moveToYFloat = -992
        moveToZFloat = -992


        debugParameterConnectionCall = True

        #called from move to coordinate button
        if(calledFromIncrementalStepButton == False):
            if(debugParameterConnectionCall):
                print('called from move to coordinate button')
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
        #called from an incremental step button
        else:
            if(debugParameterConnectionCall):
                print('called from incremental step button')
            moveToXFloat = self.incrementalStepToX
            moveToYFloat = self.incrementalStepToY
            moveToZFloat = self.incrementalStepToZ






        # divide a line between starting and end points into many equally spaced points and move to each of those points,
        # so the arm moves in a straight line
        startingPointX = self.currentXPosition
        startingPointY = self.currentYPosition
        startingPointZ = self.currentZPosition
        #vector directions for vector equation of line
        directionX = moveToXFloat - startingPointX
        directionY = moveToYFloat - startingPointY
        directionZ = moveToZFloat - startingPointZ
        #determines how many slices to slice up the line in
        linearMovementResolution = self.linearLineResolution

        #clear the stepper sequence for the line and the stepper positions on the line
        self.stepperPositionsOnALine[:] = []
        self.stepperSequenceToDrawALine[:] = []

        #need to uncomment the debugging prints. commented to save time in the for loop
        debugLineSlicing = self.masterDebug

        #iterate through the line slices (points on the line path for the arm to move along)
        #simulate moving to each sub point, using inverse kinematics in the move_to_cartesian_coordinate function
        #to return the corresponding angles to move to. within the move_to_cartesian function, the angles are sent
        #to another function to convert them to stepper positions. these are stored in the corresponding stepper position
        #variable referenced in the constructor of this class
        for i in range(1, linearMovementResolution+1):
            nextPointX = startingPointX + (directionX * (i/linearMovementResolution))
            nextPointY = startingPointY + (directionY * (i/linearMovementResolution))
            nextPointZ = startingPointZ + (directionZ * (i/linearMovementResolution))
            #get the stepper position of each sub point on the line
            self.move_to_cartesian_coordinate(nextPointX, nextPointY, nextPointZ)

            """
            if(debugLineSlicing):
                print('i:')
                print(i)
                print(nextPointX)
                print(nextPointY)
                print(nextPointZ)
            """

        #from the list of stepper positions to move to, generate an actual step sequence. an algorithm is used
        #to take as evenly spaced steps as possible so that the movement is more linear
        self.generate_line_step_sequence()




        #####
        ###
        #all of the remaining code in this function is for the serial communicaton protocol (handshaking and packets)
        #see the readme on github for a somewhat detailed explanation
        ###
        #####
        #debugs the serial communication code below
        debug = self.masterDebug#note that you need to uncomment "testlist.append[]" and "print(arduinoBuffer)" if you want to debug (each occurs twice)




        #note, this is NOT the MAX_INPUT setting on the arduino. It should be 1/3 of that value
        maxStepDataTuplesCanSend = self.maximumStepSequenceTuplesToSendAtATime
        maxStepDataCommands = maxStepDataTuplesCanSend * 3 #max number of characters to send
        numLineHeaderData = 5 #num of header data characters
        numLineFooterData = 1 #num of footer data characters
        numHeaderFooterData = numLineHeaderData + numLineFooterData #num of the aforementioned deader and footer characters combined

        #calculate number of step data tuples. subtract the header/footer data count and divide the # of step commands by 3
        numStepDataTuples = (len(self.stepperSequenceToDrawALine)-numHeaderFooterData)/3

        #number of dataSets is the number of packets that will be sent to the arduino
        numDataSets = math.ceil(numStepDataTuples/maxStepDataTuplesCanSend)


        #send header data to arduino
        for h in range(0,numLineHeaderData):
            self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[h]) )



        if(debug):
            print(self.stepperSequenceToDrawALine)
            print('numStepDataTuples: ' + str(numStepDataTuples))
            print('numDataSets BEFORE rounding: ' + str(numStepDataTuples/maxStepDataTuplesCanSend))
            print('numDataSets: ' + str(numDataSets))

            print('start counting')

        i=0
        arduinoReady = False
        arduinoBuffer = ''
        testlist = []
        #iterate through guaranteed full sets of step data (max length packets of data)
        for i in range(0,numDataSets-1):
            dataPacketStartPos = int(numLineHeaderData + (i * maxStepDataCommands))
            dataPacketEndPos = int(maxStepDataCommands) + dataPacketStartPos
            #write a data packet to the arduino here
            for j in range(dataPacketStartPos,dataPacketEndPos):
                self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[j]) )
                #print(j)
               # testlist.append(j)
            if(debug): #note that you need to uncomment testlist.append[] if debugging
                print(testlist)
                print(len(testlist))
                testlist = []
            #having written a data packet, wait for the arduino to send a message backs saying its ready to receive another
            while(arduinoReady == False):
                if(self.arduinoSerial.in_waiting > 1):
                    #in_waiting is how many bytes are in the arduino's serial buffers. decode converts the data returned
                    #from the rad function (of type byte object) to a string variable. Strip removes non-letter/number characters
                    #like newline returns
                    arduinoBuffer = self.arduinoSerial.read(self.arduinoSerial.in_waiting).decode().strip()
                    if('d' in arduinoBuffer):#check for the 'd' in the 'done' message
                        #print(arduinoBuffer)
                        arduinoReady = True
            arduinoReady = False


        # i is 1 less than the end value of the range when exiting the forloop. I want to be the same value
        # if i is zero, than means it didn't enter the for loop, so don't change it
        if(i > 0):
            i+=1


        #iterate through last set (last packet of data to send. may not be full
        dataPacketStartPos = int(numLineHeaderData + (i * maxStepDataCommands))
        dataPacketEndPos = len(self.stepperSequenceToDrawALine)
        for j in range(dataPacketStartPos,dataPacketEndPos):
            self.arduinoSerial.write( str.encode(self.stepperSequenceToDrawALine[j]) )
            #print(j)
            #print(self.stepperSequenceToDrawALine[j])
            #testlist.append(j)
        if(debug): #note that you need to uncomment testlist.append[]
            print(testlist)
            print(len(testlist))
        while(arduinoReady == False):
                arduinoBuffer = self.arduinoSerial.read(self.arduinoSerial.in_waiting ).decode().strip()
                if('d' in arduinoBuffer):
                    print(arduinoBuffer)
                    arduinoReady = True
        arduinoReady = False




    def move_to_cartesian_coordinate(self, moveToXFloat, moveToYFloat, moveToZFloat):
        # call inverse kinematics function to convert from cartesian coordinates to angles for Dobot arm
        # moveToAngles is a list of angles (type float) with the following order: [base angle, upper arm angle, lower arm angle]
        # catch any errors (likely due to coordinates out of range being input) NEED TO ADDRESS THIS AT SOME POINT
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



        #these next 4 lines of code transform angles returned from the inverse kinematics code to the correct axes that
        #I defined for the dobot.
        moveToUpperArmAngleFloat = moveToAngles[1]
        moveToLowerArmAngleFloat = moveToAngles[2]

        transformedUpperArmAngle = (90 - moveToUpperArmAngleFloat)
        #-90 different from c++ code, accounts for fact that arm starts at the c++ simulation's 90
        # note that this line is different from the similar line in the move angles function. Has to do with the inverse kinematics function
        # and the fact that the lower arm angle is calculated relative to the upper arm angle.
        transformedLowerArmAngle = 360 + (transformedUpperArmAngle - moveToLowerArmAngleFloat) - 90


        debug = False
        if(debug):
            print('ik base angle')
            print(moveToAngles[0])
            print('ik upper angle')
            print(moveToAngles[1])
            print('ik lower angle')
            print(moveToAngles[2])
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

        #converts the angles to stepper positions and updates the corresponding self variable
        self.convert_angles_to_stepper_positions(moveToAngles[0],transformedUpperArmAngle,transformedLowerArmAngle)


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



    def convert_angles_to_stepper_positions(self, baseAngle,upperArmAngle,lowerArmAngle):

        debug = False

        print(baseAngle)
        baseStepNumber = int(( (abs(baseAngle)/360) * self.baseActualStepsPerRevolution ) + 0.5)
        print(baseStepNumber)
        #need this because of the abs value function, which is needed for proper rounding
        if (baseAngle < 0):
            baseStepNumber *= -1



        upperArmStepNumber = int(( (abs(upperArmAngle)/360) * self.upperArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (upperArmAngle < 0):
            upperArmStepNumber *= -1


        lowerArmStepNumber = int(( (abs(lowerArmAngle)/360) * self.lowerArmActualStepsPerRevolution ) + 0.5)
        #need this because of the abs value function, which is needed for proper rounding
        if (lowerArmAngle < 0):
            lowerArmStepNumber *= -1

        #necessary to reverse the direction in which the steppers move, so angles match my defined angles
        baseStepNumber *= -1
        upperArmStepNumber *= -1
        lowerArmStepNumber *= 1

        self.stepperPositionsOnALine.append(baseStepNumber)
        self.stepperPositionsOnALine.append(upperArmStepNumber)
        self.stepperPositionsOnALine.append(lowerArmStepNumber)


        if(debug):
            print("Base Angle")
            print(baseAngle)
            print("Base Step Number")
            print(baseStepNumber)
            print("Upper Arm Angle")
            print(upperArmAngle)
            print("Upper Arm Step Number")
            print(upperArmStepNumber)
            print("Lower Arm Angle")
            print(lowerArmAngle)
            print("Lower Arm Step Number")
            print(lowerArmStepNumber)


    #creates the line data as specified in the readme in the github repository
    def generate_line_step_sequence(self):

        debug = self.masterDebug

        self.stepperSequenceToDrawALine.append('l')
        print(self.stepperPositionsOnALine)
        #is it true that moving along a line means that the steppers will always be moving in the same direction? I'm assuming it does, but I need to verify this.
        #set the direction of the steppers. only need to check the first position to move to, to get the direction, since moving in a straight line
        if((self.stepperPositionsOnALine[0] - self.basePos) > 0):
            #digitalWrite(E_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(E_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')
        if((self.stepperPositionsOnALine[1] - self.upperPos) > 0):
            #digitalWrite(X_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(X_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')
        if((self.stepperPositionsOnALine[2] - self.lowerPos) > 0):
            #digitalWrite(Y_DIR_PIN, HIGH);
            self.stepperSequenceToDrawALine.append('H')
        else:
            #digitalWrite(Y_DIR_PIN, LOW);
            self.stepperSequenceToDrawALine.append('L')


        #generating the step sequences of a line from the step positions
        for j in range(0,len(self.stepperPositionsOnALine),3):
            self.generate_step_sequence_from_position_to_position(abs(self.stepperPositionsOnALine[j] - self.basePos),
                                                                  abs(self.stepperPositionsOnALine[j+1] - self.upperPos),
                                                                  abs(self.stepperPositionsOnALine[j+2] - self.lowerPos))
            #updating each stepper's step pos. probably could update it after the for loop, but leaving it for now
            self.basePos = self.stepperPositionsOnALine[j]
            self.upperPos = self.stepperPositionsOnALine[j+1]
            self.lowerPos = self.stepperPositionsOnALine[j+2]

        self.stepperSequenceToDrawALine.insert(4,'s')
        self.stepperSequenceToDrawALine.append('e')

        if(debug):
            print('Stepper Sequence')
            print(self.stepperSequenceToDrawALine)




    # the algorithm here is fairly complicated. Just know that it tries to generate as linear a path between two points
    # as possible by dividing the steps as evenly as possible among each stepper motor.
    def generate_step_sequence_from_position_to_position(self,numBaseSteps,numUpperArmSteps,numLowerArmSteps):


        #of the 3 stepper motors determine which one requires the most steps
        max_steps = float(max(max(numBaseSteps, numUpperArmSteps), numLowerArmSteps))#needs to be float so the rounding in the divisions work

        if(max_steps == 0):
            return

        baseStepSpace = max_steps+1
        upperStepSpace = max_steps+1
        lowerStepSpace = max_steps+1

        baseRemainder = 0
        upperRemainder = 0
        lowerRemainder = 0


        if(numBaseSteps != 0):
          baseStepSpace = int((max_steps / numBaseSteps) +.5)#round to nearest int
          baseRemainder = int(max_steps/baseStepSpace)#round to lowest int
          baseRemainder = numBaseSteps - baseRemainder#finish the calculation

        if(numUpperArmSteps != 0):
          upperStepSpace = int((max_steps / numUpperArmSteps) + 0.5)#round to nearest int
          upperRemainder = int(max_steps/upperStepSpace)#round to lowest int
          upperRemainder = numUpperArmSteps - upperRemainder#finish the calculation

        if(numLowerArmSteps != 0):
          lowerStepSpace = int((max_steps / numLowerArmSteps) + .5)#round to nearest int
          lowerRemainder = int(max_steps/lowerStepSpace)#round to lowest int
          lowerRemainder = numLowerArmSteps - lowerRemainder#finish the calculation

        """
        print(baseStepSpace)
        print(upperStepSpace)
        print(lowerStepSpace)
        print(baseRemainder)
        print(upperRemainder)
        print(lowerRemainder)
        """

        baseStepSpacei = 0
        upperStepSpacei = 0
        lowerStepSpacei = 0

        baseTaken = 0
        upperTaken = 0
        lowerTaken = 0

        baseflag = False
        upperflag = False
        lowerflag = False

        #step the motors at the same time by moving them 1 step at essentially the same time
        #Must alternate between HIGH and LOW signals to step the motors. I don't know the physics of why though. Just look up one of the million tutorials on stepper motors if you're curious why.

        for i in range(0,int(max_steps)):
            if ((baseStepSpacei == 0) and (baseTaken < numBaseSteps)):
                #digitalWrite(baseStepPin, HIGH);//only step the motor if it has more steps remaining to take
                baseflag = True
            if ((upperStepSpacei == 0) and (upperTaken < numUpperArmSteps)):
                #digitalWrite(upperArmStepPin, HIGH);
                upperflag = True
            if ((lowerStepSpacei == 0) and (lowerTaken < numLowerArmSteps)):
                #digitalWrite(lowerArmStepPin, HIGH);
                lowerflag = True

                #delay(1);

            if (baseflag):
                #digitalWrite(baseStepPin, LOW);
                baseStepSpacei = baseStepSpace
                baseTaken += 1
                baseflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            if (upperflag):
                #digitalWrite(upperArmStepPin, LOW);
                upperStepSpacei = upperStepSpace
                upperTaken += 1
                upperflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            if (lowerflag):
                #digitalWrite(lowerArmStepPin, LOW);
                lowerStepSpacei = lowerStepSpace
                lowerTaken += 1
                lowerflag = False
                self.stepperSequenceToDrawALine.append('1')
            else:
                self.stepperSequenceToDrawALine.append('0')
            baseStepSpacei -= 1
            upperStepSpacei -= 1
            lowerStepSpacei -= 1

        ###
        ###
        ###
        ###
        #I think that not adressing this might result in some small systematic error that might build up over a long time.
        #I really need to address this, but I'm out of time and the current algorithm gives a close enough approximation for testing.
        #NEED TO ADDRESS THIS!!!!!!
        ###
        ###
        ###
        """
        #take any remaining steps
        for i in range(0,baseRemainder):
            #digitalWrite(baseStepPin, HIGH);
            #delay(1);
            #digitalWrite(baseStepPin, LOW);
            baseTaken += 1


        for i in range(0,upperRemainder):
            #digitalWrite(upperArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(upperArmStepPin, LOW);
            upperTaken += 1

        for i in range(0,lowerRemainder):
            #digitalWrite(lowerArmStepPin, HIGH);
            #delay(1);
            #digitalWrite(lowerArmStepPin, LOW);
            lowerTaken += 1


        print(baseTaken)
        print(upperTaken)
        print(lowerTaken)
        """

        """
        a,b,c = -1

        for i in range(0,len(self.stepperSequenceToDrawALine),3):
            a = self.stepperSequenceToDrawALine[i]
            b = self.stepperSequenceToDrawALine[i+1]
            c = self.stepperSequenceToDrawALine[i+2]
            print(a)
            print(b)
            print(c)
        """








    #DO NOT CALL THE MOVE ANGLES FUNCTION (DO NOT CLICK THE MOVE ANGLES BUTTON) AS OF NOW,
    #THIS IS OLD CODE THAT DID WORK. I DRASTICALLY ALTERED THE STRUCTURE OF THE SOFTWARE, SO THIS ALMOST CERTAINLY DOESN'T WORK NOW.
    #UNPREDICTABLE BEHAVIOR WILL RESULT. I NEED TO ADDRESS THIS.

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

        """
        # code for debugging purposes. the firmware I am using (at time of writing this) is set up to print the 3 angles it read to the serial
        # this reads the 3 angles that the arduino printed from the serial. There is certainly a better way to do this.
        # this was quick and dirty and is prone to fatal errors (fatal for this program that is).
        for i in range(0,15 ):
            print ( self.arduinoSerial.readline() )
        """


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








    #functions for the incremental step buttons
    #these functions are messed up for small step sizes, like 1. possibly due to the unaddressed linear move algorithm. NEED TO ADDRESS THIS
    def pushButtonStepForward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepForwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition + stepSizeFloat
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepBackward_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepBackwardSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition - stepSizeFloat
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepLeft_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepLeftSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition - stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepRight_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepRightSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition + stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepUp_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepUpSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition + stepSizeFloat
            self.pushButtonMoveToCoordinate_clicked(True)

    def pushButtonStepDown_clicked(self):
        # get the step size from the appropriate line edit. convert it from string to float and do some basic checks
        stepSizeFloat = self.convert_step_size_numerical_text_to_number_plus_check_is_valid(self.ui.lineEditStepDownSize.text())
        # check that there were no errors in converting text to float. if there were, don't move. else move to coordinate
        if (stepSizeFloat == None):
            return
        else:
            self.incrementalStepToX = self.currentXPosition
            self.incrementalStepToY = self.currentYPosition
            self.incrementalStepToZ = self.currentZPosition - stepSizeFloat
            self.incrementalStepToZ = self.currentZPosition - stepSizeFloat
            self.pushButtonMoveToCoordinate_clicked(True)

    #error checking for grabbing the step size from the incremental step boxes
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




    ###
    ###
    #Start serial port connection management functions. will move these to a separate .py file at some point
    #Below are several serial port connection related functions. They are fairly boring, but are critical.
    #Note that they are not necessarily optimal. there are improvements that could be made like automatically
    #checking for change in port connections somehow.
    ###
    ###


    #loads a list of serial ports to the tablewidget in the configuration tab
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

    #looks for a port whose name as the words arduino in it on start up. only known to work on windows
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



    ###
    ###
    #End serial port connection management functions. will move these to a separate .py file at some point
    #Below are several serial port connection related functions. They are fairly boring, but are critical.
    #Note that they are not necessarily optimal. there are improvements that could be made like automatically
    #checking for change in port connections somehow.
    ###
    ###



    #A couple self explanatory functions is all that remains! Phew, jesus, this was A LOT of work for something so
    #simple that even still needs to be improved. Countless dozens of hours. Well over 100 hours. Lots of time spent
    #debugging and revising.

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



#pretty standard qt main function. sets up the whole gui
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




