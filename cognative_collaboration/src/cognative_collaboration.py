#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Cognative Collaboration Study application

:Author: Peter Lightbody <plightbody@lincoln.ac.uk>
:Organization: University of Lincoln
:Date: 10 March 2017
:Version: 0.0.1
:Status: Development
:Copyright: MIT

"""

import random
# import sys
from time import sleep, gmtime, strftime
from math import sqrt
import actionlib
import baxter_interface
from mary_tts.msg import maryttsAction, maryttsGoal
# from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from tracking.msg import labelledPose_array
# import matplotlib.pyplot as plt
import numpy as np
import rospkg
import rospy
import yaml
# from PyQt4.QtGui import QApplication, QDialog, QGridLayout, QHBoxLayout, QListWidget, QPushButton, QVBoxLayout
# from PyQt4 import QtCore
# import rospy
# from std_msgs.msg import String
# import getopt


class Box(object):

    def __init__(self, parent=None):
        super(Box, self).__init__()
        self.box_id = None
        self.tracking_ids_back = -1
        self.tracking_ids_right = -1
        self.tracking_ids_left = -1
        self.line_length = -1
        self.line_direction = -1
        self.grid_position = (-1, -1)
        self.category = ''
        self.correct_category = ''
        self.selected = False
        self.on_grid = True
        self.correct_percent = 0
        self.on_category_A = not self.on_grid
        self.on_category_B = not self.on_grid

    def dist_to_segment(self, ax, ay, bx, by):
        cx = self.line_length
        cy = self.line_direction

        # avoid divide by zero error
        a = max(by - ay, 0.0000000000000000000000001)
        b = max(ax - bx, 0.0000000000000000000000001)
        # compute the perpendicular distance to the theoretical infinite line
        dl = abs(a * cx + b * cy - b * ay - a * ax) / sqrt(a**2 + b**2)
        # compute the intersection point
        x = ((a / b) * ax + ay + (b / a) * cx - cy) / ((b / a) + (a / b))
        y = -1 * (a / b) * (x - ax) + ay
        # decide if the intersection point falls on the line segment
        if (ax <= x <= bx or bx <= x <= ax) and (ay <= y <= by or by <= y <= ay):
            return dl
        else:
            # if it does not, then return the minimum distance to the segment endpoints
            return min(sqrt((ax - cx)**2 + (ay - cy)**2), sqrt((bx - cx)**2 + (by - cy)**2))


class Controller(object):

    WAITING_STATE = 1
    SUGGESTING_STATE = 2
    REPEATING_STATE = 3

    def __init__(self, log_file_name, helpful):
        super(Controller, self).__init__()
        self.log_file_name = log_file_name
        self.helpful = helpful
        self.on_grid = list()
        self.off_grid = list()
        self.dirty_scene = False
        self.log_dirty_env_change()
        self.log_start()

    def log_dirty_env_change(self):
        self.log("env_changed", str(self.dirty_scene))

    def log_box_leaves_grid(self, box, taken_to):
        self.log("leaving_grid", "%s, %s" % (str(box.box_id), str(taken_to)))

    def log_robot_suggestion(self, suggestion, box, x, y):
        self.log("suggestion", "%s, %s, %s, %s, %s" % (str(box.box_id), str(box.correct_category), str(suggestion), str(x), str(y)))

    def log_start(self):
        self.log("begin", "start")

    def log_end(self):
        self.log("finished", "We have finished")

    def log_aborted(self):
        self.log("aborted_finish", "We have finished")

    def log_box_reset_to_grid(self, box):
        self.log("reset_box", "%s From %s" % (str(box.box_id), str(box.correct_category)))

    def start_trial(self):
        self.say('Hello. Let\'s start classifying the objects.')

    def end_trial(self):
        if(len(self.on_grid)):
            self.log_aborted()
            percentage, numerator, denominator = self.calculate_correct()
            self.say('Finished for now, thank you. You got %s percent correct.' % (str(percentage)))
            self.log("end_trial_aborted", '%i%% (%i of %i)' % (percentage, numerator, denominator))
        else:
            self.log_end()
            percentage, numerator, denominator = self.calculate_correct()
            self.say('All done thank you. You got %s percent correct.' % (str(percentage)))
            self.log("end_trial", '%i%% (%i of %i)' % (percentage, numerator, denominator))

    def calculate_correct(self):
        denominator = len([i for i in self.off_grid if i.difficulty > 0.00001])
        numerator = len([i for i in self.off_grid if i.category == i.correct_category and i.difficulty > 0.00001])
        percentage = int((float(numerator) / float(denominator)) * 100.0)
        return percentage, numerator, denominator

    def say(self, what_to_say):
        self.talk.wait_for_server()
        self.talk.send_goal(maryttsGoal(text=str(what_to_say)))
        self.talk.wait_for_result()

    def log(self, log_type, to_log):
        print "LOGGING", log_type, to_log
        with open("cog_cal_logs/%s.txt" % (str(self.log_file_name)), "a") as logFile:
            r_time = rospy.get_rostime()
            current_time = strftime("%Y-%m-%d %H:%M:%S", gmtime())
            logFile.write("%s, %s, %s, %s\n" % (str(current_time), str(r_time), str(log_type), str(to_log)))

    def load_config(self):
        with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/config.yaml', 'r') as stream:
            params = yaml.load(stream)
            # ============== Grid Settings ======================
            self.left_side_width = params['grid']['sides']['left']
            self.right_side_width = params['grid']['sides']['right']
            self.grid_width = params['grid']['size']['width']
            self.grid_height = params['grid']['size']['height']
            self.grid_cell_width = params['grid']['cell_size']['width']
            self.grid_cell_height = params['grid']['cell_size']['height']
            self.grid_zero_zero_position = params['grid']['offset']
            # ============== Box Settings ======================

            for item in params['object_list']:
                for key, body in item.items():
                    b = Box()
                    b.box_id = key
                    b.tracking_ids_back, b.tracking_ids_back, b.tracking_ids_back = body['tracking_ids']
                    b.line_length = body['line_length']
                    b.line_direction = body['line_direction']
                    b.grid_position_x = body['grid_position'][0]
                    b.grid_position_y = body['grid_position'][1]
                    b.world_position_x = self.grid_zero_zero_position[0] + (self.grid_cell_height * b.grid_position_x)
                    b.world_position_y = self.grid_zero_zero_position[1] + (self.grid_cell_width * b.grid_position_y)
                    b.world_position_z = self.grid_zero_zero_position[2]
                    self.on_grid.append(b)
            # ============== Box calculations ======================
            m = np.mean([i.line_length for i in self.on_grid])
            m = 0.0198
            for i in self.on_grid:
                i.difficulty = i.dist_to_segment(m, -1000, m, 1000)
                # i.category = "A" if i.line_length < m else "B"
                i.correct_category = "A" if i.line_length < m else "B"

        with open(rospkg.RosPack().get_path('cognative_collaboration') + '/config/raw_positions.yaml', 'r') as stream:
            params = yaml.load(stream)
            self.grid_positions = params['grid_position']
            self.natural_position = params['natural']
            self.speech = params['speech']

    def robot_setup(self):
        baxter_interface.RobotEnable().enable()
        self.robot = baxter_interface.Limb('right')
        self.robot.set_joint_position_speed(0.5)
        self.talk = actionlib.SimpleActionClient('/speak', maryttsAction)

    def move_to(self, x, y):
        self.dirty_scene = True
        self.log_dirty_env_change()
        box = [i for i in self.on_grid if i.grid_position_x == x and i.grid_position_y == y][0]
        self.robot.move_to_joint_positions(self.natural_position)
        self.talk.wait_for_server()
        text = self.process_speech(box.correct_category)
        self.log_robot_suggestion(text, box, x, y)
        self.talk.send_goal(maryttsGoal(text=text))
        self.robot.move_to_joint_positions(self.grid_positions[x][y])
        self.talk.wait_for_result()
        self.robot.move_to_joint_positions(self.natural_position)

    def process_speech(self, cat):
        return random.choice(self.speech).replace('{{category}}', 'AY' if cat == 'A' else 'BEE')

    def tracking(self, april_tag_aray):
        for box in self.on_grid:
            for tag in april_tag_aray.objects:
                if int(box.tracking_ids_back) == int(tag.label):
                    if tag.pose.position.y < -0.35:
                        print "removing %s" % (box.box_id), 'NOW ON CAT A'
                        box.on_category_A = True
                        box.category = 'A'
                        box.on_grid = False
                        try:
                            self.off_grid.append(self.on_grid.pop(self.on_grid.index(box)))
                            self.dirty_scene = False
                            self.log_dirty_env_change()
                            self.log_box_leaves_grid(box, 'A')
                        except:
                            pass
                    elif tag.pose.position.y > 0.45:
                        print "removing %s" % (box.box_id), 'NOW ON CAT B'
                        box.on_category_B = True
                        box.category = 'B'
                        box.on_grid = False
                        try:
                            self.off_grid.append(self.on_grid.pop(self.on_grid.index(box)))
                            self.dirty_scene = False
                            self.log_dirty_env_change()
                            self.log_box_leaves_grid(box, 'B')
                        except:
                            pass
        for box in self.off_grid:
            for tag in april_tag_aray.objects:
                if int(box.tracking_ids_back) == int(tag.label):
                    if tag.pose.position.y > -0.3 and tag.pose.position.y < 0.45:
                        print "resetting %s NOW ON %s %s" % (box.box_id, box.grid_position_x, box.grid_position_y)
                        try:
                            self.on_grid.append(self.off_grid.pop(self.off_grid.index(box)))
                            self.dirty_scene = False
                            self.log_dirty_env_change()
                            box.category = ""
                            self.log_box_reset_to_grid(box)
                        except:
                            print "RESET FAILED"
                            pass

    def state_machine(self):

        current_state = self.WAITING_STATE

        while self.on_grid and not rospy.is_shutdown():

            # public current state for web interface

            if current_state == self.WAITING_STATE:
                sleep(5)
                if not self.dirty_scene:
                    current_state = self.SUGGESTING_STATE

            elif current_state == self.SUGGESTING_STATE:
                box = [i for i in sorted(self.on_grid, key=lambda x: x.difficulty, reverse=not self.helpful) if i.difficulty > 0.00001]
                if not len(box):
                    if not self.dirty_scene:
                        self.dirty_scene = True
                        self.talk.wait_for_server()
                        self.talk.send_goal(maryttsGoal(text="Please catagorise the final object"))
                        self.talk.wait_for_result()
                else:
                    box = box[0]
                    print box.box_id, box.difficulty
                    self.move_to(box.grid_position_x, box.grid_position_y)
                current_state = self.WAITING_STATE
            elif current_state == self.REPEATING_STATE:
                current_state = self.SUGGESTING_STATE

        # else:
            # self.end_trial()

        print 'FINISHED'


def set_args():
    log_confirmed = order_confirmed = False

    while not log_confirmed:
        person = raw_input("Please name this logfile: ")
        if len(person) < 1:
            continue

        if not person.isalnum():
            print "Sorry, the file name can only contain letters and numbers. Please Try again."
            continue

        check = raw_input("You want to name the logfile \"" + person + "\". Is this correct?\n")
        if(check.lower() in ['y', 'yes']):
            log_confirmed = True

    while not order_confirmed:
        check = raw_input("""Would you like the robot to be:
            \t1) Helpful? (cloest to the catagory boundry)
            \t2) Not Helpful (furthest from the catagory boundary)
            """)
        if(check in ['1', '2']):
            if check == '1':
                helpful = True
            else:
                helpful = False
            order_confirmed = True
        else:
            print "That does not appear to be an option. Please try again."

    return str(person), bool(helpful)


def main():
    rospy.init_node('cognative_collaboration', anonymous=False, log_level=rospy.DEBUG)
    logName, helpful = set_args()
    c = Controller(logName, helpful)
    rospy.on_shutdown(c.end_trial)
    c.load_config()
    c.robot_setup()
    rospy.Subscriber('/tracking/results_array', labelledPose_array, c.tracking)
    c.start_trial()
    c.state_machine()

if __name__ == '__main__':
    main()
