#!/usr/bin/env python

import argparse
import struct
import sys
import copy
import numpy as np

import rospy
import rospkg

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from action_primitive_variation.srv import *
from agent.srv import *


class Type(object):
    def __init__(self, parent, children):
        self.parentType = parent
        self.childrenTypes = children

    def getChildrenTypes(self):
        return self.childrenTypes

    def __str__(self):
        s = ''
        for t in self.childrenTypes:
            s = s + t + ' '

        s = s + '- ' + self.parentType
        return s

# class Predicate(object):
class TemplatedPredicate(object):
    def __init__(self, _operator, _vars):
        self.operator = _operator
        self.vars = _vars

    def __str__(self):
        args = ''
        for v in self.vars:
            args = args + ' ' + str(v)
        return "(" + self.operator + args + ")"

class StaticPredicate(object):
    def __init__(self, _operator, _args):
        self.operator = _operator
        self.vars = _args

    def __str__(self):
        args = ''
        for var in self.vars:
            args = args + str(var) + ' '
        return "(" + self.operator + ' ' + args + ")"

class Variable():
    def __init__(self, _var, _type):
        self.variable = _var
        self.type = _type

    def getName(self):
        return self.variable

    def getType(self):
        return self.type

    def __str__(self):
        return self.variable + ' - ' + self.type

class Action(object):
    def __init__(self, actionName, _args, _preConds, _effects, _srvFile, _gripper=None, _params=None, _locs=[]):
        self.name = actionName 
        self.args = _args
        self.preconditions = _preConds # list of predicates (recursive)
        self.effects = _effects
        self.srv = actionName + '_srv'
        self.srvFile = _srvFile
        self.gripper = _gripper
        self.executionParams = _params
        self.PDDLlocs = _locs

    def addVar(self, var):
        self.args.append(var) # check to make sure this actually sets it 

    def addPreCond(self, predicate):
        self.preconditions.append(predicate)

    def getArgs(self):
        return copy.deepcopy(self.args)

    def getGripper(self):
        return self.gripper

    def addEffect(self, predicate):
        self.effects.append(predicate)

    def getName(self):
        return self.name 

    def getSrv(self):
        return self.srv

    def getSrvFile(self):
        return self.srvFile

    def getEffects(self):
        return copy.deepcopy(self.effects)

    def getLocs(self):
        return self.PDDLlocs

    def getExecutionParams(self):
        return self.executionParams

    def getNonLocationVars(self):
        args = []
        for v in self.args:
            t = v.getType()
            if (t != 'waypoint') and (t != 'location'):
                args.append(t)
        return args

    def __str__(self):
        s = '(:action ' + self.name + '\n'
        s = s + '    :parameters ('
        for p in self.args:
            s = s + str(p) + ' '
        s = s + ')\n'

        if len(self.preconditions) > 1:    
            s = s + '    :precondition (and'
            for pcs in self.preconditions:
                s = s + '\n        ' + str(pcs)
            s = s + ')\n'
        elif len(self.preconditions) == 1:
            s = s + '    :precondition ' + str(self.preconditions[0]) + '\n'
        else:
            # s = s
            s = s + '    :precondition (and)\n'

        if len(self.effects) > 1:  
            s = s + '    :effect (and'
            for e in self.effects:
                s = s + '\n        ' + str(e)
            s = s + ')\n)'
        elif len(self.effects) == 1:  
            s = s + '    :effect ' + str(self.effects[0]) + '\n)'
        else: 
            # s = s 
            s = s + '    :effect (and)\n)' 
        return s