#!/usr/bin/env python

class Parameter():
    def __init__(self, _name, _default, _min, _max, _possibleVals=None):
        self.name = _name
        self.default_val = str(_default)
        self.min = str(_min)
        self.max = str(_max)
        self.possible_vals = _possibleVals

    # SETTERS 
    def setVal(self, _val):
        self.default_val = _val

    # GETTERS
    def getName(self):
        return self.name

    def getDefaultVal(self):
        return self.default_val

    def getMin(self):
        return self.min

    def getMax(self):
        return self.max

    def getPossibleVals(self):
        return self.possible_vals

    # def __str__(self):
    #     return self.variable + ' - ' + self.type

