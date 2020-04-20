#!/usr/bin/env python

import sys
import cv2
import math
import time
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
    PoseArray,
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
)

from std_msgs.msg import (
    Header,
    Empty,
)

# from knowledge_base.kb_subclasses import *
from knowledge_base.action import Action

def getPredicateLocation(predList, _oprtr, _obj):
    for pred in predList:
        if ((pred.operator == _oprtr) and (_obj in pred.objects)):
            return pred.locationInformation
    return None

# Given predlist, returns names of objects that are "visible"
def getVisibleObjects(predList):
    obj_list = []
    for pred in predList:
        if (pred.operator == "is_visible"):
            obj_list.append(pred.objects[0] + ' ')
    return obj_list

def is_touching(object1_loc, object2_loc, epsilon=0.135):
    if ((object1_loc is not None) and (object2_loc is not None)):
        obj1 = np.array((object1_loc.pose.position.x, 
                            object1_loc.pose.position.y,
                            object1_loc.pose.position.z)) 
        obj2 = np.array((object2_loc.pose.position.x, 
                            object2_loc.pose.position.y,
                            object2_loc.pose.position.z)) 
        if np.linalg.norm(obj1 - obj2) < epsilon:
            return True 
        return False

def is_obtained(object1_loc, object2_loc):
    if (object1_loc is not None) and (object2_loc is not None): 
        x1 = object1_loc.pose.position.x
        x2 = 0.7
        if x1 < x2:
            return True
    print('Unable to detect if object is obtained')
    return False

def pddlStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            stringList.append(str(pred.operator) + '(' + str(pred.objects[0]) + ', (' + poseStampedToString(pred.locationInformation) + '))')
        else:
            stringList.append(str(pred.operator) + '(' + str(' '.join(pred.objects)) + ')')
    return stringList

def pddlObjectsStringFormat(predicates_list):
    strData = []
    objData = pddlObjects(predicates_list, False)
    for objType in ['cartesian', 'gripper', 'obj']:
        s = ''
        for item in objData[objType]:
            s = s + item + ' '
        s = s + '- ' + objType
        strData.append(s)
    return strData

def pddlObjectsStringFormat_fromDict(dictObj):
    strData = []
    for objType in ['cartesian', 'gripper', 'obj']:
        s = ''
        for item in dictObj[objType]:
            s = s + item + ' '
        s = s + '- ' + objType
        strData.append(s)
    return strData

def pddlObjects(predicates_list, mod=True):
    cartesian = []
    buttons = []
    grippers = []
    objs = []
    for pred in predicates_list:
        if pred.operator == "at":
            if mod == True:
                loc = poseStampedToString(pred.locationInformation) + ' '
                cartesian.append(loc)
            else: 
                loc = poseStampedToString(pred.locationInformation)
                cartesian.append(loc)

        elif 'gripper' in str(pred.objects):
            grippers.append(str(pred.objects))
        else:
            objs.append(str(pred.objects))

    cartesian = list(set(cartesian))
    buttons = list(set(buttons))
    grippers = list(set(grippers))
    objs = list(set(objs))

    objects = {}
    objects['types'] = ['cartesian', 'gripper', 'obj']
    objects['cartesian'] = cartesian
    objects['gripper'] = grippers
    objects['obj'] = objs

    return objects

def pddlInitStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            loc = poseStampedToString(pred.locationInformation)
            stringList.append('(at ' + str(pred.objects[0]) + ' ' + loc + ')')
        else:
            stringList.append('(' + pred.operator + ' ' + str(" ".join(pred.objects)) + ')')
    return stringList

def pddlCondsKBFormat(_vars, args, predicates_list, mode):
    predList = []
    varSymbols = []
    coorespondingVarTypes = []
    for v in _vars:
        varSymbols.append(v.getName())
        coorespondingVarTypes.append(v.getType())
    for pred in predicates_list:
        if (pred.object in args):
            i_obj = args.index(pred.object)
            _symbol_obj = varSymbols[i_obj]

            if 'noLoc' in mode:
                if pred.operator != "at":
                    predList.append(StaticPredicate(pred.operator, [_symbol_obj]))
            else:
                if pred.operator == "at":
                    i_loc = args.index(poseStampedToString(pred.locationInformation))
                    _symbol_loc = varSymbols[i_loc]
                    _type_obj = coorespondingVarTypes[i_obj]
                    predList.append(StaticPredicate(_type_obj + '_at ', [_symbol_obj, _symbol_loc]))
                else:
                    predList.append(StaticPredicate(pred.operator, [_symbol_obj])) 

    return predList

def getPredicateDiffs(predList1, predList2):
    diffs = []
    p1 = pddlInitStringFormat(predList1)
    p2 = pddlInitStringFormat(predList2)
    for i in range(len(p1)):
        if p1[i] not in p2:
            diffs.append(predList1[i])
    for i in range(len(p2)):
        if p2[i] not in p1:
            diffs.append(predList2[i])
    return diffs

def removeLocPredicates(predList):
    newList = []
    for pred in predList:
        if pred.operator != 'at':
            newList.append(pred)
    return newList

def removeNonArgPredicates(predList, args):
    newList = []
    for pred in predList:
        if pred.object in args:
            newList.append(pred)
    return newList

def getPredicateCommonElements(predList1, predList2):
    common = []
    commonStr = []
    p1 = pddlInitStringFormat(predList1)
    p2 = pddlInitStringFormat(predList2)
    for i in range(len(p1)):
        if p1[i] in p2:
            if p1[i] not in commonStr:
                commonStr.append(p1[i])
                common.append(predList1[i])
    for i in range(len(p2)):
        if p2[i] in p1:
            if p2[i] not in commonStr:
                commonStr.append(p2[i])
                common.append(predList2[i])
    return common

def removePredicateList(listToRemoveFrom, listToRemove):
    newList = []
    p1 = pddlInitStringFormat(listToRemoveFrom)
    p2 = pddlInitStringFormat(listToRemove)
    for i in range(len(p1)):
        if p1[i] not in p2:
            newList.append(listToRemoveFrom[i])
    return list(set(newList))

def poseStampedToString(val):
    x = round(val.pose.position.x, 1)
    y = round(val.pose.position.y, 1)
    z = round(val.pose.position.z, 1)
    if x == -0.0:
        x = 0.0
    if y == -0.0:
        y = 0.0
    if z == -0.0:
        z = 0.0

    return (str(x) + ',' + 
            str(y) + ',' + 
            str(z))

def getElementDiffs(predList1, predList2, OGargs=[]):
    nonRepeatingDiffs =[]
    diffs = getPredicateDiffs(predList1, predList2)
    for o in diffs:
        if str(o.object) not in OGargs:
            nonRepeatingDiffs.append(o.object)
    return list(set(nonRepeatingDiffs))

def typeChecker(elementName, types=["obj", "gripper", "cartesian"]):
    for t in types:
        if t in str(elementName):
            return t
        return "obj"

def getBoundLocs(preds):
    locVars = []
    coorespondingArgs = []
    for pred in preds:
        if pred.operator  == 'at':
            coorespondingArgs.append(poseStampedToString(pred.locationInformation))

    coorespondingArgs = list(set(coorespondingArgs))

    for i in range(len(coorespondingArgs)):
        locVars.append(Variable('?loc'+str(i), 'cartesian'))

    return locVars, coorespondingArgs 

def removeNoneInstances(lst):
    newLst = []
    for elem in lst:
        if elem is not None:
            newLst.append(elem)
    return newLst

def pddlActionKBFormat(_vars, args, preCondsPredList, effectsPredList, mode=[]):
    #print('\nWelcome to the function from hell!')
    diffsObjs = []
    diffsVars = []
    templatedVars = []

    args = removeNoneInstances(args)
    for _v in copy.deepcopy(_vars):
        if _v.getType() != 'cartesian':
            templatedVars.append(_v)

    if 'noLoc' in mode:
        pre = removeLocPredicates(preCondsPredList.predicates)
        eff = removeLocPredicates(effectsPredList.predicates)
    else:
        pre = preCondsPredList.predicates
        eff = effectsPredList.predicates

    object_diffs = getElementDiffs(pre, eff, args) # consider these in appending more templated vars  

    _preconditions = removeNonArgPredicates(preCondsPredList.predicates, (args + object_diffs))
    _effects = removeNonArgPredicates(effectsPredList.predicates, (args + object_diffs))

    common_pred = getPredicateCommonElements(_preconditions, _effects)

    newPreconditions = _preconditions
    newEffects = removePredicateList(_effects, common_pred)

    for i in range(len(object_diffs)):
        templatedVars.append(Variable('?obj' +str(i), typeChecker(object_diffs[i])))

    if 'noLoc' in mode:
        templatedVars = templatedVars
        args = args + object_diffs  # merge the arguments
        locArgs = []
    else:
        locVars, locArgs = getBoundLocs(newPreconditions + newEffects)
        templatedVars = templatedVars + locVars
        args = args + object_diffs + locArgs # merge the arguments

    preconds = pddlCondsKBFormat(templatedVars, args, newPreconditions, mode)
    effects = pddlCondsKBFormat(templatedVars, args, newEffects, mode)

    return templatedVars, preconds, effects, locArgs

