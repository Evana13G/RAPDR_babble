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
from kb_subclasses import *
from pddl.msg import * 
from agent.srv import *
from util.data_conversion import * 

class KnowledgeBase(object):
    def __init__(self):
        _domain = 'rapdr'
        _reqs = ['strips', 'typing', 'fluents', 'disjunctive-preconditions']
        _types = []
        _preds = []
        _actions = []
        _pddllocs = []

        _types.append(Type('object', ['location', 'obj', 'gripper', 'button']))
        _types.append(Type('location', ['waypoint']))

        _preds.append(TemplatedPredicate('gripper_at', [Variable('?g', 'gripper'), Variable('?wp', 'waypoint')]))
        _preds.append(TemplatedPredicate('obj_at', [Variable('?o', 'obj'), Variable('?wp', 'waypoint')]))
        _preds.append(TemplatedPredicate('button_at', [Variable('?b', 'button'), Variable('?wp', 'waypoint')]))
        _preds.append(TemplatedPredicate('pressed', [Variable('?b', 'button')]))
        _preds.append(TemplatedPredicate('is_visible', [Variable('?o', 'obj')]))
        _preds.append(TemplatedPredicate('obtained', [Variable('?o', 'obj')]))
        

# def push(req):
# def grasp(req):
# def shake(req):
# def press(req):
# def drop(req):

        # _a1 = Action('obtain_object', [], [], [], ObtainObjectSrv)
        # _a1.addVar(Variable('?g', 'gripper'))
        # _a1.addVar(Variable('?loc0', 'waypoint'))
        # _a1.addVar(Variable('?o', 'obj'))
        # _a1.addVar(Variable('?loc1', 'waypoint'))
        # #_a1.addPreCond(StaticPredicate('gripper_at', ['?g', '?loc0']))
        # _a1.addPreCond(StaticPredicate('obj_at', ['?o', '?loc1']))
        # _a1.addEffect(StaticPredicate('obj_at', ['?o', '?loc0']))
        # _a1.addEffect(StaticPredicate('obtained', ['?o']))

        # #_a1.addEffect(StaticPredicate('not', [StaticPredicate('obj_at', ['?o', '?loc1'])]))

        # _a2 = Action('press_button', [], [], [], PressButtonSrv)
        # _a2.addVar(Variable('?g', 'gripper'))
        # _a2.addVar(Variable('?loc0', 'waypoint'))
        # _a2.addVar(Variable('?b', 'button'))
        # _a2.addVar(Variable('?loc1', 'waypoint'))
        # _a2.addPreCond(StaticPredicate('gripper_at', ['?g', '?loc0']))
        # _a2.addPreCond(StaticPredicate('button_at', ['?b', '?loc1']))
        # #_a2.addEffect(StaticPredicate('gripper_at', ['?g', '?loc0']))
        # _a2.addEffect(StaticPredicate('pressed', ['?b']))

        # _actions.append(_a1)
        # _actions.append(_a2)

        self.domain = _domain
        self.requirements = _reqs
        self.types = _types
        self.predicates = _preds
        self.actions = _actions
        self.pddlLocs = _pddllocs
    
    def test(self, req):
        return 1

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
        for action in self.actions:
            locBindings.append(LocationBinding(action.getName(), action.getExecutionParams()))
        return LocationBindingList(locBindings)

    def createAction(self, name, origAction, args, preconds, effects, srvFile, gripper, params, mode):
        theOGaction = self.getAction(origAction)
        newActionName = name
        newActionVars, newActionPreconds, newActionEffects, pddlLocs = pddlActionKBFormat(theOGaction.getArgs(), args, preconds, effects, mode)
        newActionSrvFile = srvFile
        newActionParams = params
        newActionGripper = gripper
        newAction = Action(newActionName, newActionVars, newActionPreconds, newActionEffects, newActionSrvFile, newActionGripper, newActionParams, pddlLocs)
        return newAction

    def addAction(self, newAction):
        self.actions.append(newAction)
        locs = copy.deepcopy(newAction.getLocs())
        for loc in newAction.getExecutionParams():
            locs.append(poseStampedToString(loc))
        locs = list(set(locs))
        self.addLocs(locs)