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

from action import Action
from type import Type
from predicate import TemplatedPredicate, StaticPredicate
from variable import Variable
from parameter import Parameter

class KnowledgeBase(object):
    def __init__(self):
        _domain = 'rapdr'
        _reqs = ['strips', 'typing']
        _types = []
        _preds = []
        _actions = []
        _pddllocs = []

        ## types of atoms available
        _types.append(Type('object', ['entity', 'location']))
        _types.append(Type('entity', ['obj', 'gripper']))
        _types.append(Type('location', ['cartesian']))

        ## Types of predicates available, templated
        _preds.append(TemplatedPredicate('at', [Variable('?e', 'entity'), Variable('?loc', 'cartesian')]))
        _preds.append(TemplatedPredicate('touching', [Variable('?e', 'entity'), Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('is_visible', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('grasped', [Variable('?o', 'obj')]))
        _preds.append(TemplatedPredicate('pressed', [Variable('?o', 'obj')]))
        

        ## PUSH 
        push = Action('push', [], [], [], []) # All default, add after 
        push.addArg(Variable('?g', 'gripper'))
        push.addArg(Variable('?loc0', 'cartesian'))
        push.addArg(Variable('?o', 'obj'))
        push.addArg(Variable('?loc1', 'cartesian'))
        push.addPreCond(StaticPredicate('at', ['?o', '?loc0']))
        push.addEffect(StaticPredicate('at', ['?o', '?loc1']))
        push.addEffect(StaticPredicate('not', [StaticPredicate('at', ['?o', '?loc0'])]))
        push_p1 = Parameter('rate', 7.0, 1.0, 150.0)
        push_p2 = Parameter('movementMagnitude', 0.4, 0.1, 0.6)
        push_p3 = Parameter('orientation', 'left', None, None, ['left', 'right', 'top', 'front', 'back'])
        push.addParam(push_p1)
        push.addParam(push_p2)
        push.addParam(push_p3)
        push.setExecutionArgNames(['gripper', 'objectName'])

        # ## GRASP
        grasp = Action('grasp', [], [], [], []) # All default, add after 
        grasp.addArg(Variable('?g', 'gripper'))
        grasp.addArg(Variable('?o', 'obj'))
        grasp_p1 = Parameter('orientation', 'left', None, None, ['left', 'right', 'top', 'front', 'back'])
        grasp.addParam(grasp_p1)
        grasp.addEffect(StaticPredicate('grasped', ['?o']))
        grasp.setExecutionArgNames(['gripper', 'objectName'])

        ## SHAKE
        shake = Action('shake', [], [], [], []) # All default, add after 
        shake.addArg(Variable('?g', 'gripper'))
        shake.addArg(Variable('?o', 'obj'))
        shake_p1 = Parameter('rate', 5.0, 1.0, 20.0)
        shake_p2 = Parameter('movementMagnitude', 1.0, 0.0, 5.0)
        shake_p3 = Parameter('orientation', 'left', None, None, ['left', 'right', 'top', 'front', 'back'])
        shake.addParam(shake_p1)
        shake.addParam(shake_p2)
        shake.addParam(shake_p3)
        shake.setExecutionArgNames(['gripper', 'objectName'])

        ## PRESS
        press = Action('press', [], [], [], []) # All default, add after 
        press.addArg(Variable('?g', 'gripper'))
        press.addArg(Variable('?o', 'obj'))
        press_p1 = Parameter('rate', 100.0, 50.0, 1000.0)
        press_p2 = Parameter('movementMagnitude', 0.1, 0.03, 0.3)
        press_p3 = Parameter('orientation', 'left', None, None, ['left', 'right', 'top', 'front', 'back'])
        push.addPreCond(StaticPredicate('is_visible', ['?o']))
        push.addEffect(StaticPredicate('pressed', ['?o']))
        press.addParam(press_p1)
        press.addParam(press_p2)
        press.addParam(press_p3)
        press.setExecutionArgNames(['gripper', 'objectName'])

        # ## DROP
        # drop = Action('drop', [], [], [], []) # All default, add after 
        # drop.addArg(Variable('?g', 'gripper'))
        # drop.addArg(Variable('?o', 'obj'))
        # drop_p1 = Parameter('dropHeight', 0.1, 0.01, 0.3)
        # drop.addParam(drop_p1)
        # drop.setExecutionArgNames(['gripper', 'objectName'])

        _actions.append(push)
        # _actions.append(grasp)
        _actions.append(shake)
        _actions.append(press)
        # _actions.append(drop)


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

    def getDomainData(self, action_exclusions=[]):
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
            if a.getName() not in action_exclusions:
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

    def createAction(self, name, origAction, args, preconds, effects, params, mode):
        theOGaction = self.getAction(origAction)
        newActionName = name
        newActionVars, newActionPreconds, newActionEffects, pddlLocs = pddlActionKBFormat(theOGaction.getArgs(), args, preconds, effects, mode)
        newActionParams = params
        newAction = Action(newActionName, newActionVars, newActionPreconds, newActionEffects, newActionParams)
        return newAction

    def addAction(self, newAction):
        existing_action_names = [x.getName() for x in self.actions]
        name = newAction.getName()
        if name in existing_action_names:
            try:
                iteration = str(int(name.split('V')[1]) + 1)
            except:
                iteration = '2'
            new_name = name + '_V' + iteration
            newAction.setName(new_name)
        self.actions.append(newAction)

    def __str__(self):
        s = ''
        for a in self.actions:
            s = s + str(a) + '\n'
        return s