#!/usr/bin/env python

class Parameter():
    def __init__(self, _name, _default, _min, _max):
        self.name = _name
        self.default_val = _default
        self.min = _min
        self.max = _max

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



    # def __str__(self):
    #     return self.variable + ' - ' + self.type

