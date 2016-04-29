'''

Author: JoJo Meunier jmeunier@bu.edu 4/11/16

python script to get data from standard workspace config json file

grabs the dimensions of each object and draws them in separate window 
using pyopengl to show the work space config set up

'''


import json
import sys
import os

class CollectData:

	filepath = 'task.json'
	data_array = []

	def __init__(self):
		#print("\ngrabbing data from task.json")
		pass

	def loadfile(self):
		with open(CollectData.filepath) as data_file:
			self.data = json.load(data_file)
		#print ("loaded json file to edit...")

	def get_tipBox_data(self):
		for objects in self.data:
			if objects["collection"]== "TipBox":
				return objects["parameters"][0]["Dimensions"], objects["parameters"][0]["3DLocation"]


	def get_tubeRack_data(self):
		for objects in self.data:
			if objects["collection"] == "TubeRack":
				return objects["parameters"][0]["Dimensions"], objects["parameters"][0]["3DLocation"]


	def get_wasteContainer_data(self):
		for objects in self.data:
			if objects["collection"] == "WasteContainer":
				return objects["parameters"][0]["Dimensions"], objects["parameters"][0]["3DLocation"]

	def get_microPlate_data(self):
		for objects in self.data:
			if objects["collection"] == "MicroPlate":
				return objects["parameters"][0]["Dimensions"], objects["parameters"][0]["3DLocation"]
	
	def get_robot_data(self):
		for objects in self.data:
			if objects["collection"] == "Robot":
				return objects["parameters"][0]["Dimensions"], objects["parameters"][0]["3DLocation"]

	def get_real_coordinates(self):
		for objects in self.data:
			if objects["collection"]== "TipBox":
				tip_loc = objects["parameters"][0]["Location"]
			if objects["collection"] == "TubeRack":
				tube_loc = objects["parameters"][0]["Location"]
			if objects["collection"] == "WasteContainer":
				waste_loc = objects["parameters"][0]["Location"]
			if objects["collection"] == "MicroPlate":
				micro_loc = objects["parameters"][0]["Location"]
			if objects["collection"] == "Robot":
				bot_loc = objects["parameters"][0]["Location"]

		return bot_loc, tube_loc, tip_loc, micro_loc, waste_loc








