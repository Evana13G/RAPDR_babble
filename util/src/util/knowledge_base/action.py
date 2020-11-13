#!/usr/bin/env python

import copy 

class Action(object):
    def __init__(self, actionName, _args, _preConds, _effects, _params, _srvFile, _pddlLocs = []):
        self.name = actionName 
        self.args = _args
        self.preconditions = _preConds # list of predicates (recursive)
        self.effects = _effects
        self.srv = actionName + '_srv'
        self.params = _params
        self.srvFile = _srvFile
        self.pddlLocs = _pddlLocs
        self.executionArgNames = []

    #### SETTERS
    def setExecutionArgNames(self, argNames):
        self.executionArgNames = argNames
        
    def addArg(self, arg):
        self.args.append(arg) # check to make sure this actually sets it 

    def addPreCond(self, predicate):
        self.preconditions.append(predicate)

    def addEffect(self, predicate):
        self.effects.append(predicate)

    def addParam(self, param):
        self.params.append(param)

    def setParamDefault(self, param, val):
        self.paramDefaults[param] = val

    #### GETTERS
    def getName(self):
        return self.name 

    def getArgs(self):
        return copy.deepcopy(self.args)

    def getPreconditions(self):
        return copy.deepcopy(self.preconditions)

    def getEffects(self):
        return copy.deepcopy(self.effects)

    def getSrv(self):
        return self.srv

    def getSrvFile(self):
        return self.srvFile

    def getParams(self):
        return self.params

    def getExecutionArgNames(self):
        return self.executionArgNames

    def getNonLocationVars(self):
        args = []
        for v in self.args:
            t = v.getType()
            if (t != 'waypoint') and (t != 'location'):
                args.append(t)
        return args

    def bind_args(self, args, locs):
        assert(len(args) == len(self.args) or len(args) + len(locs) == len(self.args))
        i_args = 0
        i_locs = 0
        bound_vars = {}

        for i in range(len(self.args)):
            pddl_arg = self.args[i].getName()
            if 'loc' in pddl_arg:
                bound_vars[pddl_arg] = locs[i_locs]
                i_locs += 1
            else:
                bound_vars[pddl_arg] = args[i_args]
                i_args += 1
        return bound_vars        
        
    def get_instatiated_preconditions(self, args, locs):
        bindings = self.bind_args(args, locs)
        instatiated = []
        for p in self.preconditions:
            instatiated.append(p.get_instatiated_str(bindings))
        return instatiated

    def get_instatiated_effects(self, args, locs):
        bindings = self.bind_args(args, locs)
        instatiated = []
        for p in self.effects:
            instatiated.append(p.get_instatiated_str(bindings))
        return instatiated

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

######################################################
# PUSH
# string objectName 
# float64 startOffset
# float64 endOffset
# int64 rate

# PRESS
# string objectName 
# float64 hoverDistance
# float64 pressAmount
# float64 rate

# SHAKE
# string objectName
# float64 twistRange
# float64 speed

# DROP
# string objectName
# int64 dropHeight

# GRASP
# string objectName

