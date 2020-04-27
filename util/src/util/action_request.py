#!/usr/bin/env python

class ActionRequest:
    def __init__(self, _actionName, args, argVals, params, paramVals):
        self.actionName = _actionName
        self.initVals(args, argVals, params, paramVals)

    def initVals(self, args, argVals, params, paramVals):
        for i in range(0, len(args)):
            setattr(self, args[i], argVals[i])
        for i in range(0, len(params)):
            setattr(self, params[i], paramVals[i])

    def defaults_setter(self):
        return

    def __str__(self):
        attrs = vars(self)
        s = 'Action Request:\n'
        s = s + '\n'.join("---%s: %s" % item for item in attrs.items())
        return s

