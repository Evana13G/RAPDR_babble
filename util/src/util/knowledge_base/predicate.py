#!/usr/bin/env python

class TemplatedPredicate(object):
    def __init__(self, _operator, _vars):
        self.operator = _operator
        self.vars = _vars

    def get_instatiated_str(self, mapping):
        args = ''
        for var in self.vars:
            if type(var) is TemplatedPredicate:
                val = var.get_instatiated_str(mapping)
            else:
                val = str(mapping[var])
            args = args + ' ' + val
        return "(" + self.operator + args + ")"
        
    def __str__(self):
        args = ''
        for v in self.vars:
            # if type(v) is TemplatedPredicate:
                
            args = args + ' ' + str(v)
        return "(" + self.operator + args + ")"


class StaticPredicate(object):
    def __init__(self, _operator, _args):
        self.operator = _operator
        self.args = _args

    def get_instatiated_str(self, mapping):
        args = ''
        for arg in self.args:
            if type(arg) is StaticPredicate:
                val = arg.get_instatiated_str(mapping)
            else:
                val = str(mapping[arg])
            args = args + ' ' + val
        return "(" + self.operator + args + ")"

    def __str__(self):
        args = ''
        for arg in self.args:
            args = args + ' ' + str(arg)
        return "(" + self.operator + args + ")"