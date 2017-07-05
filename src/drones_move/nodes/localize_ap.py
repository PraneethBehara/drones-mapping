#!/usr/bin/env python

'''
This node localizes the APs
Subscriptions : None
Output: list of AP coordinates
'''

import rospy
import numpy

class Subscribers:

    def __init__(selfself, subscriptions=[]):
        for s in subscriptions:
            rospy.Subscriber(s[0], s[1], s[2])

class Localize():

    def __init__(self, n=3, A=35, threshold = None):
        self.n = n
        self.A = A
        self.threshold = threshold

    def locate_ap(self, list_ap, cl):
        while mod_dx > self.threshold:
            dist = self.get_list_dist(x, )

    def get_list_dist(self, list, cl):
        for l in range(len(list)):
            result = numpy.linalg.norm(cc[1] - cc[0])
            return result




