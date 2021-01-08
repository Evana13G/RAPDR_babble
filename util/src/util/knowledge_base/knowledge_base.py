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
        _reqs = ['strips', 'typing', 'negative-preconditions']
        _types = []
        _preds = []
        _actions = []
        _pddllocs = []

        ## types of atoms available
        _types.append(Type('object', ['entity', 'location']))
        _types.append(Type('entity', ['obj', 'gripper', 'burner']))
        _types.append(Type('location', ['cartesian']))

        ## Types of predicates available, templated
        _preds.append(TemplatedPredicate('at', [Variable('?e', 'entity'), Variable('?loc', 'cartesian')]))
        _preds.append(TemplatedPredicate('touching', [Variable('?e', 'entity'), Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('is_visible', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('covered', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('on_burner', [Variable('?e', 'entity'), Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('cooking', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('food_cooked', []))
        _preds.append(TemplatedPredicate('powered_on', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('pressed', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('on', [Variable('?e', 'entity'), Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('prepped', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('shaken', [Variable('?e', 'entity')]))
        _preds.append(TemplatedPredicate('ingredients_added', [Variable('?e', 'entity')]))

        ## PUSH 
        push = Action('push', [], [], [], []) # All default, add after 
        push.addArg(Variable('?g', 'gripper'))
        push.addArg(Variable('?loc0', 'cartesian'))
        push.addArg(Variable('?o', 'obj'))
        push.addPreCond(StaticPredicate('at', ['?o', '?loc0']))
        push.addEffect(StaticPredicate('not', [StaticPredicate('at', ['?o', '?loc0'])]))
        push_p1 = Parameter('rate', 7.0, 3.0, 150.0) # Discover Strike
        # push_p1 = Parameter('rate', 7.0, 3.0, 50.0) # Cook
        push_p2 = Parameter('movementmagnitude', 0.4, 0.1, 0.6)
        push_p3 = Parameter('orientation', 'left', None, None, ['left', 'right', 'top'])
        push.addParam(push_p1)
        push.addParam(push_p2)
        push.addParam(push_p3)
        push.setExecutionArgNames(['gripper', 'objectName'])

        ## SHAKE
        shake = Action('shake', [], [], [], []) # All default, add after 
        shake.addArg(Variable('?g', 'gripper'))
        shake.addArg(Variable('?o', 'obj'))
        shake.addPreCond(StaticPredicate('not', [StaticPredicate('covered', ['?o'])]))
        shake.addEffect(StaticPredicate('shaken', ['?o']))
        shake_p1 = Parameter('rate', 15.0, 1.0, 20.0)
        shake_p2 = Parameter('movementmagnitude', 1.0, 0.01, 5.0)
        shake_p3 = Parameter('orientation', 'top', None, None, ['left', 'right', 'top'])
        shake.addParam(shake_p1)
        shake.addParam(shake_p2)
        shake.addParam(shake_p3)
        shake.setExecutionArgNames(['gripper', 'objectName'])

        #####################################################################################################
        #####################################################################################################
        ###
        ###
        ### It seems like it cant handle plans which require 
        ### actions that have both 2 type obj arguments, and 
        ### 1 obj type argument
        ###
        ### So cover_obj(?g, ?o1, ?o2) works alone just for covering
        ### an obj, but using that rep with the full 
        ### [cover_obj, place_on_burner, cook, check_food] doesnt work
        ### unless cover_obj takes just 1 obj type argument
        ###
        ###
        ### It also appears that it has to be perfect chaining. 
        ### For example: 
        ###
        ###   ---------------------------------------------------
        ###       ACTION:: PRECONDS -> EFFECTS
        ###   ---------------------------------------------------
        ###       cover_obj:: None -> covered(?o)
        ###       place_on_burner:: covered(?o) -> on_burner(?o, ?b)
        ###       cook:: on_burner(?o, ?b) -> cooking(?o)
        ###
        ###    ^ This sequence WORKS, BUT...
        ###
        ###   ---------------------------------------------------
        ###       ACTION:: PRECONDS -> EFFECTS
        ###   ---------------------------------------------------
        ###       cover_obj:: None -> covered(?o)
        ###       place_on_burner:: None -> on_burner(?o, ?b)
        ###       cook:: on_burner(?o, ?b), covered(?o) -> cooking(?o)
        ###
        ###    ^ This sequence DOES NOT WORK
        ###
        ###

        # UNCOVER OBJECT
        uncover_obj = Action('uncover_obj', [], [], [], []) # All default, add after 
        uncover_obj.addArg(Variable('?g', 'gripper'))
        uncover_obj.addArg(Variable('?o1', 'obj'))
        uncover_obj.addArg(Variable('?loc1', 'cartesian'))
        uncover_obj.addArg(Variable('?o2', 'obj'))
        uncover_obj.addPreCond(StaticPredicate('covered', ['?o2']))
        # uncover_obj.addPreCond(StaticPredicate('touching', ['?o1', '?o2']))
        uncover_obj.addPreCond(StaticPredicate('at', ['?o2', '?loc1']))
        uncover_obj.addPreCond(StaticPredicate('not', [StaticPredicate('at', ['?o1', '?loc1'])]))
        uncover_obj.addEffect(StaticPredicate('not', [StaticPredicate('covered', ['?o2'])]))
        uncover_obj.setExecutionArgNames(['gripper', 'objectName1', 'objectName2'])

        # COVER OBJECT
        cover_obj = Action('cover_obj', [], [], [], []) # All default, add after 
        cover_obj.addArg(Variable('?g', 'gripper'))
        cover_obj.addArg(Variable('?o1', 'obj'))
        cover_obj.addArg(Variable('?loc1', 'cartesian'))
        cover_obj.addArg(Variable('?o2', 'obj'))
        cover_obj.addPreCond(StaticPredicate('not', [StaticPredicate('covered', ['?o2'])]))
        cover_obj.addPreCond(StaticPredicate('at', ['?o2', '?loc1']))
        cover_obj.addPreCond(StaticPredicate('not', [StaticPredicate('at', ['?o1', '?loc1'])]))
        cover_obj.addEffect(StaticPredicate('covered', ['?o2']))
        cover_obj.setExecutionArgNames(['gripper', 'objectName1', 'objectName2'])

        # PLACE ON BURNER
        place_on_burner = Action('place_on_burner', [], [], [], []) # All default, add after 
        place_on_burner.addArg(Variable('?g', 'gripper'))
        place_on_burner.addArg(Variable('?o', 'obj'))
        place_on_burner.addArg(Variable('?b', 'burner'))
        # place_on_burner.addPreCond(StaticPredicate('covered', ['?o']))
        place_on_burner.addPreCond(StaticPredicate('not', [StaticPredicate('covered', ['?o'])]))
        place_on_burner.addPreCond(StaticPredicate('prepped', ['?o']))
        place_on_burner.addEffect(StaticPredicate('on_burner', ['?o', '?b']))
        place_on_burner.setExecutionArgNames(['gripper', 'objectName1', 'objectName2'])

        # COOK
        cook = Action('cook', [], [], [], []) # All default, add after 
        cook.addArg(Variable('?g', 'gripper'))
        cook.addArg(Variable('?o', 'obj'))
        cook.addArg(Variable('?b', 'burner'))
        cook.addPreCond(StaticPredicate('on_burner', ['?o', '?b']))
        cook.addPreCond(StaticPredicate('covered', ['?o']))
        # cook.addPreCond(StaticPredicate('prepped', ['?o']))
        # cook.addPreCond(StaticPredicate('powered_on', ['?b']))
        cook.addEffect(StaticPredicate('cooking', ['?o']))
        cook.setExecutionArgNames(['gripper', 'objectName1', 'objectName2'])

        # CHECK FOOD
        check_food = Action('check_food', [], [], [], []) # All default, add after 
        check_food.addArg(Variable('?g', 'gripper'))
        check_food.addArg(Variable('?o', 'obj'))
        check_food.addPreCond(StaticPredicate('cooking', ['?o']))
        check_food.addEffect(StaticPredicate('food_cooked', []))
        check_food.setExecutionArgNames(['gripper', 'objectName'])

        # FOOD PREPPED
        prep_food = Action('prep_food', [], [], [], []) # All default, add after 
        prep_food.addArg(Variable('?g', 'gripper'))
        prep_food.addArg(Variable('?o', 'obj'))
        prep_food.addPreCond(StaticPredicate('not', [StaticPredicate('covered', ['?o'])]))
        # prep_food.addPreCond(StaticPredicate('covered', ['?o']))

        prep_food.addPreCond(StaticPredicate('shaken', ['?o']))
        # prep_food.addPreCond(StaticPredicate('ingredients_added', ['?o']))
        prep_food.addEffect(StaticPredicate('prepped', ['?o']))
        prep_food.setExecutionArgNames(['gripper', 'objectName'])

        # SORT INGREDIENTS
        add_ingredients = Action('add_ingredients', [], [], [], []) # All default, add after 
        add_ingredients.addArg(Variable('?g', 'gripper'))
        add_ingredients.addArg(Variable('?o', 'obj'))
        add_ingredients.addPreCond(StaticPredicate('not', [StaticPredicate('covered', ['?o'])]))
        add_ingredients.addEffect(StaticPredicate('ingredients_added', ['?o']))
        add_ingredients.setExecutionArgNames(['gripper', 'objectName'])

        _actions.append(push)
        _actions.append(shake)

        _actions.append(uncover_obj)
        _actions.append(cover_obj)
        _actions.append(place_on_burner)
        _actions.append(cook)
        _actions.append(check_food)
        _actions.append(prep_food)
        _actions.append(add_ingredients)
        
###############################################################################################
#
# need a reason to use 'push' before the action that fails (cover)
# push shake cover
#
# PREP TABLE, (not (at cover orig_loc))... or (not (covered cup))
# Uncover action is to push one of the objs.. results in it not being covered 
# ... ^^ the thing I don't like about this is that its not called push. Maybe I should 
# make 'pushed' an end effect, like shake.... I dont think it would work the same though, 
# if I'm being completely honest...
#
# action1: pre: at(cup loc1), not( at(cover loc1)) 
#
# ['shake', 'cover_obj', 'prep_food'] -- > WORKS, Doesnt work, NULL
# ['cover_obj', 'place_on_burner', 'cook'] -- > Doesnt work, DOesnt work (same as first), Null
#
###############################################################################################

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
        acts = [a.getName() for a in self.actions]
        for action in self.actions:
            if action.getName() == name:
                return copy.deepcopy(action)

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

    # def removeAction(self, actionName):
    #     index_to_delete = 0
    #     while index_to_delete < len(self.actions):
    #         if self.actions[i].getName() == actionName:
    #             break
    #         index_to_delete += 1        
    #     del self.actions[index_to_delete]

    def reset(self):
        self.actions = [x for x in self.actions if ':' not in x.getName()]

    def __str__(self):
        s = ''
        for a in self.actions:
            s = s + str(a) + '\n'
        return s

######################################################################
######################################################################
######################################################################

