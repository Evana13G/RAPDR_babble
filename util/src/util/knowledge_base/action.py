#!/usr/bin/env python

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

    #### SETTERS
    def addArg(self, arg):
        self.args.append(arg) # check to make sure this actually sets it 

    def addPreCond(self, predicate):
        self.preconditions.append(predicate)

    def addEffect(self, predicate):
        self.effects.append(predicate)

    def addParam(self, param):
        self.params.append(param)

    #### GETTERS
    def getName(self):
        return self.name 

    def getArgs(self):
        return copy.deepcopy(self.args)

    def getEffects(self):
        return copy.deepcopy(self.effects)

    def getSrv(self):
        return self.srv

    def getSrvFile(self):
        return self.srvFile

    def getParams(self):
        return self.params
        
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