#!/usr/bin/env python

class TemplatedPredicate(object):
    def __init__(self, _operator, _vars):
        self.operator = _operator
        self.vars = _vars

    def get_instatiated_str(self, mapping):
        args = ''
        for var in self.vars:
            args = args + ' ' + str(mapping[var])
        return "(" + self.operator + args + ")"
        
    def __str__(self):
        args = ''
        for v in self.vars:
            args = args + ' ' + str(v)
        return "(" + self.operator + args + ")"


class StaticPredicate(object):
    def __init__(self, _operator, _args):
        self.operator = _operator
        self.vars = _args

    def get_instatiated_str(self, mapping):
        args = ''
        for var in self.vars:
            args = args + ' ' + str(mapping[var])
        return "(" + self.operator + args + ")"

    def __str__(self):
        args = ''
        for var in self.vars:
            args = args + ' ' + str(v)
        return "(" + self.operator + args + ")"