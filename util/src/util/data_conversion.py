
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

from tf.transformations import *
from kb_subclasses import *



def getPredicateLocation(predList, _oprtr, _obj):
    for pred in predList:
        if ((pred.operator == _oprtr) and (pred.object == _obj)):
            return pred.locationInformation
    return None

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
        # x2 = object2_loc.pose.position.x
        x2 = 0.7
        if x1 < x2:
            return True
    print('Unable to detect if object is obtained')
    return False

def pddlStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ', (' + poseStampedToString(pred.locationInformation) + '))')
        else:
            stringList.append(str(pred.operator) + '(' + str(pred.object) + ')')
    return stringList

def pddlObjectsStringFormat(predicates_list):
    strData = []
    objData = pddlObjects(predicates_list, False)
    for objType in ['waypoint', 'button', 'gripper', 'obj']:
        s = ''
        for item in objData[objType]:
            s = s + item + ' '
        s = s + '- ' + objType
        strData.append(s)
    return strData

def pddlObjectsStringFormat_fromDict(dictObj):
    strData = []
    for objType in ['waypoint', 'button', 'gripper', 'obj']:
        s = ''
        for item in dictObj[objType]:
            s = s + item + ' '
        s = s + '- ' + objType
        strData.append(s)
    return strData

def pddlObjects(predicates_list, mod=True):
    waypoints = []
    buttons = []
    grippers = []
    objs = []
    for pred in predicates_list:
        if pred.operator == "at":
            if mod == True:
                loc = poseStampedToString(pred.locationInformation) + ' '
                waypoints.append(loc)
            else: 
                loc = poseStampedToString(pred.locationInformation)
                waypoints.append(loc)
        if 'button' in str(pred.object):
            buttons.append(str(pred.object))
        elif 'gripper' in str(pred.object):
            grippers.append(str(pred.object))
        else:
            objs.append(str(pred.object))

    waypoints = list(set(waypoints))
    buttons = list(set(buttons))
    grippers = list(set(grippers))
    objs = list(set(objs))

    objects = {}
    objects['types'] = ['waypoint', 'button', 'gripper', 'obj']
    objects['waypoint'] = waypoints
    objects['button'] = buttons
    objects['gripper'] = grippers
    objects['obj'] = objs

    return objects

def pddlInitStringFormat(predicates_list):
    stringList = []
    for pred in predicates_list:
        if pred.operator == "at":
            loc = poseStampedToString(pred.locationInformation)
            if 'button' in str(pred.object):
                stringList.append('(button_at ' + pred.object + ' ' + loc + ')')
            elif 'gripper' in str(pred.object):
                stringList.append('(gripper_at ' + pred.object + ' ' + loc + ')')
            else:
                stringList.append('(obj_at ' + pred.object + ' ' + loc + ')')
        else:
            stringList.append('(' + pred.operator + ' ' + pred.object + ')')
    return stringList

def pddlCondsKBFormat(_vars, args, predicates_list, mode):
    predList = []
    varSymbols = []
    coorespondingVarTypes = []
    for v in _vars:
        varSymbols.append(v.getName())
        coorespondingVarTypes.append(v.getType())

    # print(" **** Info on pddlKBFormat **** ")
    # print(" **** varSymbols **** ")
    # print(varSymbols)
    # print(" **** coorespondingVarTypes **** ")
    # print(coorespondingVarTypes)
    #print(" **** args **** ")
    #print(args)

    for pred in predicates_list:
        if (pred.object in args):
            # print('object = ' + pred.object)
            i_obj = args.index(pred.object)
            # print('i_obj = ' + str(i_obj))
            _symbol_obj = varSymbols[i_obj]
            # print('_symbol_obj = ' + str(_symbol_obj))

            if 'noLoc' in mode:
                if pred.operator != "at":
                    predList.append(StaticPredicate(pred.operator, [_symbol_obj]))
            else:
                if pred.operator == "at":
                    i_loc = args.index(poseStampedToString(pred.locationInformation))
                    # print('i_loc = ' + str(i_loc))
                    _symbol_loc = varSymbols[i_loc]
                    # print('_symbol_loc = ' + str(_symbol_loc))
                    _type_obj = coorespondingVarTypes[i_obj]
                    # print('_type_obj = ' + str(_type_obj))
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

def typeChecker(elementName, types=["obj", "gripper", "button", "waypoint"]):
    # print('element name in type checker ' + elementName)
    for t in types:
        if t in str(elementName):
            # print(t)
            return t
        # print('obj')
        return "obj"

def getBoundLocs(preds):
    locVars = []
    coorespondingArgs = []
    for pred in preds:
        if pred.operator  == 'at':
            coorespondingArgs.append(poseStampedToString(pred.locationInformation))

    coorespondingArgs = list(set(coorespondingArgs))

    for i in range(len(coorespondingArgs)):
        locVars.append(Variable('?loc'+str(i), 'waypoint'))

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

    # print '\n**** Initial Info ****'
    # print '\nOG vars'
    # for v in _vars:
    #     print(str(v))
    # print '\nPassed args'
    # for a in args:
    #     print(str(a))
    # print '\nPreconditions'
    # for p in preCondsPredList.predicates:
    #     print(str(p))
    # print '\nEffects'
    # for e in effectsPredList.predicates:
    #     print(str(e))

    # Strip out 'None's and loc info
    args = removeNoneInstances(args)
    for _v in copy.deepcopy(_vars):
        if _v.getType() != 'waypoint':
            templatedVars.append(_v)

    # print '\n**** Templated Variables, remove waypoints ****'
    # for v in templatedVars:
    #     print v

    if 'noLoc' in mode:
        pre = removeLocPredicates(preCondsPredList.predicates)
        eff = removeLocPredicates(effectsPredList.predicates)
    else:
        pre = preCondsPredList.predicates
        eff = effectsPredList.predicates

    object_diffs = getElementDiffs(pre, eff, args) # consider these in appending more templated vars  
    
    # print '\n**** Object Diffs ****'
    # for o in object_diffs:
    #     print(o)

    _preconditions = removeNonArgPredicates(preCondsPredList.predicates, (args + object_diffs))
    _effects = removeNonArgPredicates(effectsPredList.predicates, (args + object_diffs))


    common_pred = getPredicateCommonElements(_preconditions, _effects)

    # print '\n**** Predicate In Common ****'
    # for p in common_pred:
    #     print p

    newPreconditions = _preconditions
    newEffects = removePredicateList(_effects, common_pred)

    # print '\n**** New Preconditions ****'
    # for p in newPreconditions:
    #     print p
    # print '\n**** New Effects ****'
    # for p in newEffects:
    #     print p


    for i in range(len(object_diffs)):
        templatedVars.append(Variable('?obj' +str(i), typeChecker(object_diffs[i])))

    if 'noLoc' in mode:
        templatedVars = templatedVars
        args = args + object_diffs  # merge the arguments
        locArgs = []
    else:
        locVars, locArgs = getBoundLocs(newPreconditions + newEffects)
        # print '\nlocVars'
        # for i in locVars:
        #     print i.getName()
        # print '\nlocArgs'
        # print locArgs
        templatedVars = templatedVars + locVars
        args = args + object_diffs + locArgs # merge the arguments

    preconds = pddlCondsKBFormat(templatedVars, args, newPreconditions, mode)
    effects = pddlCondsKBFormat(templatedVars, args, newEffects, mode)

    return templatedVars, preconds, effects, locArgs

