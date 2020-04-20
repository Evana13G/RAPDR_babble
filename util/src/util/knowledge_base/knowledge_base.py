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
from pddl.msg import * 
from agent.srv import *
from util.data_conversion import * 

from kb_subclasses import *
from action import Action
from type import Type
from predicate import TemplatedPredicate, StaticPredicate
from variable import Variable
from parameter import Parameter

class KnowledgeBase(object):
    def __init__(self):
        _domain = 'rapdr'
        _reqs = ['strips', 'typing', 'fluents', 'disjunctive-preconditions']
        _types = []
        _preds = []
        _actions = []
        _pddllocs = []

        ## types of atoms available
        _types.append(Type('entity', ['obj', 'gripper']))
        _types.append(Type('location', ['cartesian']))

        ## Types of predicates available, templated
        _preds.append(TemplatedPredicate('at', [Variable('?e', 'entity'), Variable('?loc', 'cartesian')]))
        _preds.append(TemplatedPredicate('touching', [Variable('?e', 'entity'), Variable('?e', 'entity')]))
        
        # _preds.append(TemplatedPredicate('touching', [Variable('?o', 'obj'), Variable('?o', 'obj')]))

        _preds.append(TemplatedPredicate('is_visible', [Variable('?o', 'obj')]))
        _preds.append(TemplatedPredicate('grasped', [Variable('?o', 'obj')]))
    
        #actionName, _args, _preConds, _effects, _params, _param_defaults, _srvFile

        ## PUSH 
        push = Action('push', [], [], [], [], PushSrv) # All default, add after 
        push.addArg(Variable('?g', 'gripper'))
        push.addArg(Variable('?loc0', 'cartesian'))
        push.addArg(Variable('?o', 'obj'))
        push.addArg(Variable('?loc1', 'cartesian'))
        push.addPreCond(StaticPredicate('at', ['?o', '?loc0']))
        push.addEffect(StaticPredicate('at', ['?o', '?loc1']))
        push.addEffect(StaticPredicate('not', [StaticPredicate('at', ['?o', '?loc0'])]))
        push_p1 = Parameter('rate' , None, 100, 50, 1000)
        push.addParam(push_p1)

        ## GRASP
        grasp = Action('grasp', [], [], [], [], GraspSrv) # All default, add after 
        grasp.addArg(Variable('?g', 'gripper'))
        grasp.addArg(Variable('?o', 'obj'))
        grasp.addPreCond(StaticPredicate('not', [StaticPredicate('grasped', ['?o'])]))
        grasp.addEffect(StaticPredicate('grasped', ['?o']))

        ## SHAKE
        shake = Action('shake', [], [], [], [], ShakeSrv) # All default, add after 
        shake.addArg(Variable('?g', 'gripper'))
        shake.addArg(Variable('?o', 'obj'))
        shake_p1 = Parameter('twistRange' , None, 1, 0, 5)
        shake_p2 = Parameter('speed', None, 0.3, 1.5, 0.05)
        push.addParam(shake_p1)
        push.addParam(shake_p2)

        ## PRESS
        press = Action('press', [], [], [], [], PressSrv) # All default, add after 
        press.addArg(Variable('?g', 'gripper'))
        press.addArg(Variable('?o', 'obj'))
        press_p1 = Parameter('hoverDistance' , None, 0.1, 0.03, 0.3)
        press_p2 = Parameter('pressAmount' , None, 0.01, 0.03, 0.3)
        press_p3 = Parameter('rate' , None, 100, 50, 1000)
        push.addParam(press_p1)
        push.addParam(press_p2)
        push.addParam(press_p3)

        ## DROP
        drop = Action('drop', [], [], [], [], DropSrv) # All default, add after 
        drop.addArg(Variable('?g', 'gripper'))
        drop.addArg(Variable('?o', 'obj'))
        drop_p1 = Parameter('dropHeight' , None, 0.1, 0.01, 0.3)
        push.addParam(drop_p1)

        _actions.append(push)
        _actions.append(grasp)
        _actions.append(shake)
        _actions.append(press)
        _actions.append(drop)


        self.domain = _domain
        self.requirements = _reqs
        self.types = _types
        self.predicates = _preds
        self.actions = _actions
        self.pddlLocs = _pddllocs
    

    def addLocs(self, newLocs):
        _newLocs = copy.deepcopy(self.pddlLocs)
        for loc in newLocs:
            _newLocs.append(str(loc))
        _newLocs = list(set(_newLocs))
        self.pddlLocs = _newLocs

    def typeChecker(self, elementName):
        for t in self.types:
            for c in t.getChildrenTypes():
                if c in str(elementName):
                    return c

    def getDomainData(self):
        
        data = {}
        _reqs = []
        _types = []
        _preds = []
        _acts = []
        _locs = []
        
        for r in self.requirements:
            _reqs.append(':' + r)
        for t in self.types:
            _types.append(str(t))
        for p in self.predicates:
            _preds.append(str(p))
        for a in self.actions:
            _acts.append(str(a))
        for l in self.pddlLocs:
            _locs.append(str(l))
            
        data['domain'] = self.domain
        data['requirements'] = _reqs
        data['types'] = _types
        data['predicates'] = _preds
        data['actions'] = _acts
        data['pddlLocs'] = _locs

        return data

    def getService(self, actionName):
        for action in self.actions:
            if action.getName() == actionName:
                return action.getSrv()

    def getServiceFile(self, actionName):
        for action in self.actions:
            if action.getName() == actionName:
                return action.getSrvFile()

    def getAction(self, name):
        for action in self.actions:
            if action.getName() == name:
                return action

    def getActions(self):
        return copy.deepcopy(self.actions)

    def getActionsLocs(self):
        locBindings = []
        # for action in self.actions:
        #     locBindings.append(LocationBinding(action.getName(), action.getExecutionParams()))
        return LocationBindingList(locBindings)

# (self, actionName, _args, _preConds, _effects, _params, _srvFile)
    def createAction(self, name, origAction, args, preconds, effects, params, srvFile, mode):
        theOGaction = self.getAction(origAction)
        newActionName = name
        newActionVars, newActionPreconds, newActionEffects, pddlLocs = pddlActionKBFormat(theOGaction.getArgs(), args, preconds, effects, mode)
        newActionSrvFile = srvFile
        newActionParams = params
        newAction = Action(newActionName, newActionVars, newActionPreconds, newActionEffects, newActionParams, newActionSrvFile)
        return newAction

    def addAction(self, newAction):
        self.actions.append(newAction)
        locs = copy.deepcopy(newAction.getLocs())
        for loc in newAction.getExecutionParams():
            locs.append(poseStampedToString(loc))
        locs = list(set(locs))
        self.addLocs(locs)