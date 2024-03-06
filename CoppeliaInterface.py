# Program to make the Coppeliasim interface
# Before use it you need to install the zmq in the Anaconda Powershell with the commands
# pip install coppeliasim-zmqremoteapi-client
# More instructions can be read in the link
# https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python

import cv2 as cv
import numpy as np
from coppeliasim_zmqremoteapi_client import *
import csv
import config
from config import parse_settings_file
import os.path


class CoppeliaInterface:
    """
        Class used to make the interface between the CoppeliaSim and the program on Python.

        Attributes
        __________
            quadcopter_handle: Handle to an object used to control the quadcopter.
            The name of the object is in the config.cfg file.
            quadcopter_pos: Array with the 3D position of the green ball used to control the quadcopter.
            vision_handle: Handle to an object used to control the vision sensor and get the images.
            The name of the object is in the config.cfg file.
            xy_joint_handle: Handle to an object used to control the joint that moves the calibration target.
            The name of the object is in the config.cfg file.
            xy_joint_handle: Handle to an object used to control the joint that moves the calibration target.
            The name of the object is in the config.cfg file.
            client: The client used com communicate with the ZMQ interface.
            sim: The object used to communicate with the CoppeliaSim scene.

        Methods
        _______
            __init__(): Initialize the class variables.
            quadcopter_control(self, pos: list): move the quadcopter to the position pos
            get_image(self, sequence: list): Get the image form vision sensor on CoppeliaSim and saves the image on
            hard drive.
            init_control(self, quadcopter_name: str, quadcopter_vision_name: str = None): Initialize the variables used
            in quadcopter control
            __del__(): Class destructor
    """

    def __init__(self):
        """
            Initialize the class variables.
        """
        self.settings = parse_settings_file('config.yaml')
        self.quadcopter_handle = None
        self.quadcopter_pos = None
        self.vision_handle = None
        self.xy_joint_handle = None
        self.zy_joint_handle = None
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        self.client.setStepping(True)
        self.sim.startSimulation()
        self.image_list = []
        self.quadcopter_orientation = []
        self.handles = {}

    def quadcopter_control(self, pos: list, orientation: list, quad_target_handle: int, quad_base_handle: int):
        """
        This method is used to move the quadcopter in the CoppeliaSim scene to the position pos.
        :param quad_base_handle: The handle to get the quadcopter current position
        :param quad_target_handle:  The handle to the target of the quadcopter. This handle is used to position give the
        position that the quadcopter must be after control.
        :param orientation: The object orientation
        :param pos: The new position of the quadcopter is moved to.
        :return: A boolean indicating if the quadcopter reach the target position.
        """
        self.sim.setObjectPosition(quad_target_handle, self.sim.handle_world, pos)
        self.sim.setObjectOrientation(quad_target_handle, self.sim.handle_world,
                                      orientation)
        t_stab = self.sim.getSimulationTime() + self.settings['time to stabilize']
        tmp_diff_pos = 50
        tmp_diff_ori = 100
        stabilized = False
        while self.sim.getSimulationTime() < t_stab:
            diff_pos = np.subtract(pos,
                                   self.sim.getObjectPosition(quad_base_handle, self.sim.handle_world))
            diff_ori = np.subtract(orientation,
                                   self.sim.getObjectOrientation(quad_base_handle, self.sim.handle_world))
            norm_diff_pos = np.linalg.norm(diff_pos)
            norm_diff_ori = np.linalg.norm(diff_ori)
            if abs(norm_diff_pos - tmp_diff_pos) < 0.0001 and abs(norm_diff_ori - tmp_diff_ori) < 0.001:
                stabilized = True
                break
            tmp_diff_pos = norm_diff_pos
            tmp_diff_ori = norm_diff_ori
            self.client.step()
        if not stabilized:
            print('Time to stabilize the quadcopter position and orientation with given precision was short.')

    def get_image(self, sequence: list, file_name: str, vision_handle: int):
        """
        Method used to get the image from vision sensor on coppeliaSim and save the image in a file.
        The vision handle must be previously loaded.
        :param vision_handle: Vison sensor handle to CoppeliaSim vision sensor.
        :param file_name: File name to saved image
        :param sequence: Parameter not used yet
        :return: Nothing
        """
        img, resolution = self.sim.getVisionSensorImg(vision_handle)
        img = np.frombuffer(img, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
        img = cv.rotate(img, cv.ROTATE_180)
        for c in sequence:
            filename = file_name + str(c) + '.' + self.settings['extension']
            cv.imwrite(self.settings['path'] + filename, img)

    def init_control(self, handle_names: list):
        """
        Method used to get the objects used in the quadcopter control.
        :param handle_names: List of the objects to get handle
        :return: Nothing
        """
        for handle_name in handle_names:
            if handle_name not in self.handles:
                self.handles[handle_name] = self.sim.getObject(handle_name)

    def save_reconstruction_images(self,
                                   reconstruction_file_name: str,
                                   position_file_name: str,
                                   vision_sensor_name: str):
        """
        Method used to save reconstruction images
        :param reconstruction_file_name:
        :param position_file_name:
        :return:
        """
        self.init_control(
            [self.settings['quadcopter name'], self.settings['vision sensor name'], self.settings['quadcopter base']])
        count_image = 0
        position = []
        orientation = []
        if os.path.isfile(position_file_name):
            with open(position_file_name, 'r') as file:
                csvreader = csv.reader(file)
                cont_row = 0
                for row in csvreader:
                    if cont_row >= 1:
                        float_row = [float(c) for c in row]
                        position.append(float_row[:3])
                        orientation.append(float_row[3:])
                    else:
                        axes_names = row[:2]
                        orientation_angles = row[3:]
                    cont_row += 1
        else:
            print('Position csv file not found. Adjust the config.yaml file.')
            quit(-1)
        t = self.sim.getSimulationTime()
        for pos, orient in zip(position, orientation):
            for i in range(1, 5):
                self.client.step()

            self.quadcopter_control(pos,
                                    orient,
                                    self.handles[self.settings['quadcopter name']],
                                    self.handles[self.settings['quadcopter base']])
            self.get_image([count_image], reconstruction_file_name, self.handles[self.settings['vision sensor name']])
            count_image += 1
        print("Adjust the sequence to the maximum of {} images".format(str(count_image)))

    def save_reconstruct_images(self, file_name: str, position_file_name: str):
        """
        Method used to save reconstruction images
        :param file_name:
        :param position_file_name:
        :return:
        """
        self.init_control(
            [self.settings['quadcopter name'], self.settings['vision sensor name'], self.settings['quadcopter base']])
        count_image = 0
        position = []
        orientation = []
        if os.path.isfile(position_file_name):
            with open(position_file_name, 'r') as file:
                csvreader = csv.reader(file)
                cont_row = 0
                for row in csvreader:
                    if cont_row >= 1:
                        float_row = [float(c) for c in row]
                        position.append(float_row[:3])
                        orientation.append(float_row[3:])
                    else:
                        axes_names = row[:2]
                        orientation_angles = row[3:]
                    cont_row += 1
        else:
            print('Position csv file not found. Adjust the config.yaml file.')
            quit(-1)
        t = self.sim.getSimulationTime()
        for pos, orient in zip(position, orientation):
            for i in range(1, 5):
                self.client.step()

            self.quadcopter_control(pos,
                                    orient,
                                    self.handles[self.settings['quadcopter name']],
                                    self.handles[self.settings['quadcopter base']])
            self.get_image([count_image], file_name, self.handles[self.settings['vision sensor name']])
            count_image += 1
        print("Adjust the sequence to the maximum of {} images".format(str(count_image)))

    def save_calibration_images(self,
                                file_name: str,
                                position_file_name: str,
                                joint_positons_file: str):
        """
        Method used to save reconstruction images
        :param file_name:
        :param position_file_name:
        :return:
        """
        self.init_control(
            [self.settings['quadcopter name'],
             self.settings['vision sensor name'],
             self.settings['quadcopter base'],
             self.settings['target joint xy'],
             self.settings['target joint zy']])
        count_image = 0
        position = []
        orientation = []
        positionxy = []
        positionzy = []
        if os.path.isfile(position_file_name):
            with open(position_file_name, 'r') as file:
                csvreader = csv.reader(file)
                cont_row = 0
                for row in csvreader:
                    if cont_row >= 1:
                        float_row = [float(c) for c in row]
                        position.append(float_row[:3])
                        orientation.append(float_row[3:])
                    else:
                        axes_names = row[:2]
                        orientation_angles = row[3:]
                    cont_row += 1
        else:
            print('Positions to quadcopter csv file not found. Adjust the config.yaml file.')
            quit(-1)
        if os.path.isfile(joint_positons_file):
            with open(joint_positons_file, 'r') as file:
                csvreader = csv.reader(file)
                cont_row = 0
                for row in csvreader:
                    if cont_row >= 1:
                        float_row = [float(c) for c in row]
                        positionxy.append(float_row[0])
                        positionzy.append(float_row[1])
                    else:
                        angle_name = row
                    cont_row += 1
        else:
            print('Position csv file not found. Adjust the config.yaml file.')
            quit(-1)
        count_joint_positions = 0
        for pos, orient in zip(position, orientation):
            for i in range(1, 5):
                self.client.step()
            if count_joint_positions < len(positionxy):
                self.sim.setJointTargetPosition(self.handles[self.settings['target joint xy']],
                                                positionxy[count_joint_positions])
                self.sim.setJointTargetPosition(self.handles[self.settings['target joint zy']],
                                                positionzy[count_joint_positions])
            count_joint_positions += 1
            self.quadcopter_control(pos,
                                    orient,
                                    self.handles[self.settings['quadcopter name']],
                                    self.handles[self.settings['quadcopter base']])
            self.get_image([count_image], file_name, self.handles[self.settings['vision sensor name']])
            count_image += 1
        print("Adjust the sequence to the maximum of {} images".format(str(count_image)))

    def main(self):
        """
        The method used initialize the control and get the images from vision sensor. The simulation time is configured on config.cfg.
        :return:
        """
        self.init_control([self.settings['quadcopter name'], self.settings['vision sensor names']])
        count_image = 0
        position = []
        orientation = []
        if os.path.isfile(self.settings['positions file name']):
            with open(self.settings['positions file name'], 'r') as file:
                csvreader = csv.reader(file)
                cont_row = 0
                for row in csvreader:
                    if cont_row >= 1:
                        float_row = [float(c) for c in row]
                        position.append(float_row[:3])
                        orientation.append(float_row[3:])
                    else:
                        axes_names = row[:2]
                        orientation_angles = row[3:]
                    cont_row += 1
        else:
            print('Position csv file not found. Adjust the config.yaml file.')
            quit(-1)
        t = self.sim.getSimulationTime()
        total_time = t
        for pos, orient in zip(position, orientation):
            for i in range(1, 5):
                self.client.step()
            self.get_image([count_image], 'reconstruct_', self.handles[self.settings['vision sensor names']])
            # self.sim.setJointTargetVelocity(self.xy_joint_handle, 0.03)
            # self.sim.setJointTargetVelocity(self.zy_joint_handle, 0.03)
            count_image = count_image + 1
            diff_pos = np.subtract(pos, self.quadcopter_pos)
            print(diff_pos)
            diff_pos = diff_pos
            diff_or = np.subtract(orient, self.quadcopter_orientation)
            diff_or = diff_or
            for i in range(0, 1):
                tmp_pos = self.quadcopter_pos + diff_pos
                print('Increment')
                tmp_ori = self.quadcopter_orientation + diff_or
                print(tmp_ori)
                self.quadcopter_control(tmp_pos, tmp_ori)
            self.get_image([count_image], 'reconstruct_')
        print("Adjust the sequence to the maximum of {} images".format(str(count_image)))
        t = self.sim.getSimulationTime()
        while True:
            while (not self.sim.getSimulationTime() > t + self.settings[
                'time to stabilize'] or  # config.time_to_stabilize or
                   self.sim.getSimulationTime() > t + self.settings[
                       'total simulation time']):  # config.total_simulation_time):
                self.client.step()
            if self.sim.getSimulationTime() > total_time + self.settings[
                'total simulation time']:  # config.total_simulation_time:
                print('Time to stabilize is short')
                break
            t = self.sim.getSimulationTime()

    def __del__(self):
        """
        Class destructor. Close all windows with opencv images and stops the CoppeliaSim simulation.
        :return: Nothing
        """
        self.sim.stopSimulation()
        cv.destroyAllWindows()


if __name__ == '__main__':
    copp = CoppeliaInterface()
    position_file_name = 'positions.csv' #input('Enter the name to the file with positions: ')
    images_file_name = 'reconstruct' #input('Enter the name to the file to save images: ')
    joint_positions_file = 'positions_joints.csv' #input('Enter the name of the joint positions file: ')
    copp.main()
    # copp.save_calibration_images(images_file_name,
    #                              position_file_name,
    #                              joint_positions_file)
    # copp.save_reconstruction_images(images_file_name, position_file_name)
