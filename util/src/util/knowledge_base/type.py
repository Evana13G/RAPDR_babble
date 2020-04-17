#!/usr/bin/env python


class Type(object):
    def __init__(self, parent, children):
        self.parentType = parent
        self.childrenTypes = children

    def getChildrenTypes(self):
        return self.childrenTypes

    def __str__(self):
        s = ''
        for t in self.childrenTypes:
            s = s + t + ' '

        s = s + '- ' + self.parentType
        return s
