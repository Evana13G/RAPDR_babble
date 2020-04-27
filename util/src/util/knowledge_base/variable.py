#!/usr/bin/env python

## Aka arg
class Variable():
    def __init__(self, _var, _type):
        self.variable = _var
        self.type = _type

    def getName(self):
        return self.variable

    def getType(self):
        return self.type
    	
    def __str__(self):
        return self.variable + ' - ' + self.type
