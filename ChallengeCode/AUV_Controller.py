#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jul  7 12:05:08 2021

@author: BWSI AUV Challenge Instructional Staff
"""
import sys
import numpy as np
import math

class AUVController():
    def __init__(self):
        
        # initialize state information
        self.status = None
        self.__heading = None
        self.__speed = None
        self.__position = None
        self.stored_buoy_red = None
        self.stored_buoy_green = None
        self.stored_position = None
        self.stored_heading = None
        
        # assume we want to be going the direction we're going for now
        self.__desired_heading = None
        
        #reference data from realworld AUV testing for turning rate
        self.reference_turning_rate = 15.64
        self.reference_rudder = 25.0
        self.reference_speed = 5.0
        self.hard_rudder_degrees = 25.0
        self.full_rudder_degrees = 15.0

    def initialize(self, auv_state):
        self.__heading = auv_state['heading']
        self.__position = auv_state['position']
        
        # assume we want to be going the direction we're going for now
        self.__desired_heading = auv_state['heading']

    ### Public member functions
    def process_desired_rudder(self, desired_rudder, direction):
        if desired_rudder > (self.full_rudder_degrees + self.hard_rudder_degrees)/2:
            return('HARD ' + direction + 'RUDDER')
        elif desired_rudder > self.full_rudder_degrees:
            return(direction + 'FULL RUDDER')
        else:
            return(direction + str(int(desired_rudder)) + ' DEGREES RUDDER')

    def right_or_left_turn(self, heading, desired_heading):
        if (heading - desired_heading) > 180:
            direction = 'RIGHT '
        elif (desired_heading - heading) > 180:
            direction = 'LEFT '
        else:
            if desired_heading > heading:
                direction = 'RIGHT '
            else:
                direction = 'LEFT '
        return(direction)

    def get_intersect(self, a1, a2, b1, b2):
        s = np.vstack([a1,a2,b1,b2])        # s for stacked
        h = np.hstack((s, np.ones((4, 1)))) # h for homogeneous
        l1 = np.cross(h[0], h[1])           # get first line
        l2 = np.cross(h[2], h[3])           # get second line
        x, y, z = np.cross(l1, l2)          # point of intersection
        if z == 0:                          # lines are parallel
            return('Parallel')
        return(x/z, y/z)

    def store(self):
        self.stored_heading = self.__heading
        self.stored_position = self.__position
        self.stored_buoy_red = self.__green_buoys
        self.stored_buoy_green = self.__red_buoys

    def angle_difference(self, ref, target):
        if abs(target - ref) > 180:
            return(abs(abs(target - ref) - 360))
        else:
            return(abs(target - ref))

    def turning_point(self, intersect, target, position):
        if np.linalg.norm(intersect - position) < np.linalg.norm(intersect - target):
            return(position)
        else:
            return(np.array([(intersect[0] + ((position[0] - intersect[0])*(np.linalg.norm(intersect - target)/np.linalg.norm(intersect - position)))), (intersect[1] + ((position[1] - intersect[1])*(np.linalg.norm(intersect - target)/np.linalg.norm(intersect - position))))]))

    def buoy_coord(self, buoy, stored_buoy, position, stored_position, heading, stored_heading):
        positions_angle = math.degrees(np.mod(math.atan2(position[0] - stored_position[0], position[1] - stored_position[1]), 2*math.pi))
        relative_buoy = self.angle_difference(positions_angle, (heading + buoy))
        relative_stored_buoy = self.angle_difference(positions_angle, (stored_heading + stored_buoy))
        third_angle = abs(relative_buoy) - abs(relative_stored_buoy)
        position_distance = np.linalg.norm(position - stored_position)
        buoy_distance = math.sin(math.radians(relative_stored_buoy))*position_distance/math.sin(math.radians(third_angle))
        buoy_position = np.array([position[0] + buoy_distance*math.sin(math.radians(heading+buoy)), position[1] + buoy_distance*math.cos(math.radians(heading+buoy))])
        return(buoy_position)

    def obtuse_check(self, intersect, target, position):
        return(np.linalg.norm(target - position) > max(np.linalg.norm(target - intersect), np.linalg.norm(position - intersect)))

    def circle_heading(self, speed, start_point, end_point, heading, desired_heading):
        diameter = (np.linalg.norm(start_point - end_point))/(math.sin((desired_heading*math.pi/180)-(heading*math.pi/180)))
        required_turning_rate = 360/(math.pi*diameter/(speed*0.514444))
        rudder_position = float(np.mod(required_turning_rate*self.reference_rudder*self.reference_speed*0.514444/(self.reference_turning_rate*speed*0.514444), 360.0))
        if rudder_position > 180:
            rudder_position = abs(rudder_position - 360.0)
        return(rudder_position)

    def perpendicular_points(self, green, red):
        x1 = green[0]; y1 = green[1]; x2 = red[0]; y2 = red[1]
        #find the center
        cx = (x1+x2)/2
        cy = (y1+y2)/2
        #move the line to center on the origin
        x1-=cx; y1-=cy
        x2-=cx; y2-=cy
        #rotate both points
        xtemp = x1; ytemp = y1
        x1=-ytemp; y1=xtemp; 
        xtemp = x2; ytemp = y2
        x2=-ytemp; y2=xtemp; 
        #move the center point back to where it was
        x1+=cx; y1+=cy
        x2+=cx; y2+=cy
        return([np.array([x1, y1]), np.array([x2, y2])])

    def find_target_and_desired_heading(self, green_buoys, red_buoys, position):
        if (not green_buoys == None) and (not red_buoys == None):
            self.status = 'GATE FOUND'
            self.green_buoy_coord = self.buoy_coord(green_buoys, self.stored_buoy_green, position, self.stored_position, self.__heading, self.stored_heading)
            self.red_buoy_coord = self.buoy_coord(red_buoys, self.stored_buoy_red, position, self.stored_position, self.__heading, self.stored_heading)
            self.target = np.array([(self.green_buoy_coord[0] + self.red_buoy_coord[0])/2, (self.green_buoy_coord[1] + self.red_buoy_coord[1])/2]) #set target to midpoint
            self.__desired_heading = math.degrees(np.mod(math.atan2(self.target[0] - position[0], self.target[1] - position[1]), 2*math.pi))

        elif (not green_buoys == None) or (not red_buoys == None):
            self.status = 'SINGLE BUOY'
            self.target = green_buoys #set target to the one Buoy we see
            stored_target = self.stored_buoy_green
            if green_buoys == None:
                self.target = red_buoys
                stored_target = self.stored_buoy_red
            self.target = self.buoy_coord(self.target, stored_target, position, self.stored_position, self.__heading, self.stored_heading)
            self.__desired_heading = math.degrees(np.mod(math.atan2(self.target[0] - position[0], self.target[1] - position[1]), 2*math.pi))

        else:
            self.status = 'SEARCHING FOR BUOYS'
            self.target = None
            self.__desired_heading = self.__heading
    
    def decide(self, auv_state, green_buoys, red_buoys, sensor_type='POSITION'):
        self.__heading = auv_state['heading']
        self.__position = np.array(auv_state['position'])
        self.__green_buoys = np.array(green_buoys)
        self.__red_buoys = np.array(red_buoys)

        if self.__speed == None or self.stored_position == None or self.stored_buoy_green == None or self.stored_buoy_red == None or self.stored_heading == None:
            cmd = 'ENGINE HALF AHEAD'
            self.__speed = 2.5
        else:
            self.find_target_and_desired_heading(self.__green_buoys, self.__red_buoys, self.__position)
            
            if self.status == 'SEARCHING FOR BUOYS':
                cmd = 'RUDDER AMIDSHIPS'

            elif self.status == 'SINGLE BUOY':
                self.desired_rudder = self.circle_heading(self.__speed, self.__position, self.target, self.__heading, self.__desired_heading)
                self.direction = self.right_or_left_turn(self.__heading, self.__desired_heading)
                cmd = self.process_desired_rudder(self.desired_rudder, self.direction)
            
            elif self.status == 'GATE FOUND':
                perpendicular_points = self.perpendicular_points(self.green_buoy_coord, self.red_buoy_coord)
                intersect = self.get_intersect(self.__position, self.stored_position, perpendicular_points[0], perpendicular_points[1])
                if intersect == 'Parallel': #doesn't intersect
                    #turn to midpoint from furthest buoy to AUV
                    temp_target = self.green_buoy_coord
                    if abs(self.__green_buoys) < abs(self.__red_buoys):
                        temp_target = self.__red_buoys
                    temp_midpoint = np.array([(temp_target[0] + self.__position[0])/2, (temp_target[1] + self.__position[1])/2])
                    temp_desired_heading = math.degrees(np.mod(math.atan2(temp_midpoint[0] - self.__position[0], temp_midpoint[1] - self.__position[1]), 2*math.pi))
                    temp_direction = self.right_or_left_turn(self.__heading, temp_desired_heading)
                    temp_desired_rudder= self.circle_heading(self.__speed, self.__position, temp_midpoint, self.__heading, temp_desired_heading)
                    cmd = self.process_desired_rudder(temp_desired_rudder, temp_direction)
                elif self.obtuse_check(intersect, self.target, self.__position): #is valid intersection?
                    temp_point_list = self.turning_point(intersect, self.target, self.__position)
                    self.desired_rudder = self.circle_heading(self.__speed, self.__position, self.target, self.__heading, self.__desired_heading)
                    if self.desired_rudder > self.hard_rudder_degrees: #is the turning degrees unreasonable
                        #turn to midpoint from CLOSEST buoy to AUV
                        temp_target = self.green_buoy_coord
                        if abs(self.__green_buoys) > abs(self.__red_buoys):
                            temp_target = self.__red_buoys
                        temp_midpoint = np.array([(temp_target[0] + self.__position[0])/2, (temp_target[1] + self.__position[1])/2])
                        temp_desired_heading = math.degrees(np.mod(math.atan2(temp_midpoint[0] - self.__position[0], temp_midpoint[1] - self.__position[1]), 2*math.pi))
                        temp_direction = self.right_or_left_turn(self.__heading, temp_desired_heading)
                        temp_desired_rudder= self.circle_heading(self.__speed, self.__position, temp_midpoint, self.__heading, temp_desired_heading)
                        cmd = self.process_desired_rudder(temp_desired_rudder, temp_direction)
                    elif np.linalg.norm(temp_point_list - self.__position) < 1.0: #time to turn to perpendicular if near turning point
                        self.direction = self.right_or_left_turn(self.__heading, self.__desired_heading)
                        cmd = self.process_desired_rudder(self.desired_rudder, self.direction)
                    else: #go forward to reach the turning point
                        cmd = 'RUDDER AMIDSHIPS'
                else: #turn to midpoint from furthest buoy to AUV
                    temp_target = self.green_buoy_coord
                    if abs(self.__green_buoys) < abs(self.__red_buoys):
                        temp_target = self.__red_buoys
                    temp_midpoint = np.array([(temp_target[0] + self.__position[0])/2, (temp_target[1] + self.__position[1])/2])
                    temp_desired_heading = math.degrees(np.mod(math.atan2(temp_midpoint[0] - self.__position[0], temp_midpoint[1] - self.__position[1]), 2*math.pi))
                    temp_direction = self.right_or_left_turn(self.__heading, temp_desired_heading)
                    temp_desired_rudder= self.circle_heading(self.__speed, self.__position, temp_midpoint, self.__heading, temp_desired_heading)
                    cmd = self.process_desired_rudder(temp_desired_rudder, temp_direction)

                #now check if we go straight will it go through the gates and be roughly perpendicular
                #use shifted gates for better clearance
                self.__gshift = np.array([(self.green_buoy_coord[0] + ((self.red_buoy_coord[0] - self.green_buoy_coord[0])*(2/np.linalg.norm(self.green_buoy_coord - self.red_buoy_coord)))), (self.green_buoy_coord[1] + ((self.red_buoy_coord[1] - self.green_buoy_coord[1])*(2/np.linalg.norm(self.green_buoy_coord - self.red_buoy_coord))))])
                self.__rshift = np.array([(self.red_buoy_coord[0] + ((self.green_buoy_coord[0] - self.red_buoy_coord[0])*(2/np.linalg.norm(self.red_buoy_coord - self.green_buoy_coord)))), (self.red_buoy_coord[1] + ((self.green_buoy_coord[1] - self.red_buoy_coord[1])*(2/np.linalg.norm(self.red_buoy_coord - self.green_buoy_coord))))])
                self.__gshift_angle = math.degrees(np.mod(math.atan2(self.__gshift[0] - self.__position[0], self.__gshift[1] - self.__position[1]), 2*math.pi))
                self.__rshift_angle = math.degrees(np.mod(math.atan2(self.__rshift[0] - self.__position[0], self.__rshift[1] - self.__position[1]), 2*math.pi))

                if ((np.mod(self.__rshift_angle - self.__heading, 360) > 270 or np.mod(self.__gshift_angle - self.__heading, 360) > 270) and (np.mod(self.__rshift_angle - self.__heading, 360) < 90 or np.mod(self.__gshift_angle - self.__heading, 360) < 90)): #straight check
                    #calculate perpendicular heading
                    perpendicular_heading = math.degrees(np.mod(math.atan2(perpendicular_points[0][0] - perpendicular_points[1][0], perpendicular_points[0][1] - perpendicular_points[1][1]), 2*math.pi))
                    if abs(perpendicular_heading - self.__heading) < 1.0:
                        cmd = 'RUDDER AMIDSHIPS'
        self.store()
        return(cmd)