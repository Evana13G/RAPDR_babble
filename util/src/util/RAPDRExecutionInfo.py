#!/usr/bin/env python

class RAPDRExecutionInfo:
    def __init__(self, task_name):
        self.task = task_name

    def init(self):
        return

    def __str__(self):
        attrs = vars(self)
        s = 'Execution Info:\n'
        s = s + '\n'.join("---%s: %s" % item for item in attrs.items())
        return s

