'''
Created on Feb 21, 2018

@author: gaignier
'''
import spade
import traceback


class MessageListener:

    def __init__(self, template=None):
        self.template = template

    def haveTemplate(self):
        return self.template is not None

    def getTemplate(self):
        return self.template


class Behaviour(spade.Behaviour.Behaviour, MessageListener):
    def __init__(self, name, template=None):
        MessageListener.__init__(self, template)
        spade.Behaviour.Behaviour.__init__(self)
        self.name = name

    def getName(self):
        return self.name

    def process(self):
        pass

    def _process(self):
        try:
            self.process()
        except:
            traceback.print_exc()


class PeriodicBehaviour(spade.Behaviour.PeriodicBehaviour, MessageListener):
    def __init__(self, name, period, template=None):
        spade.Behaviour.PeriodicBehaviour.__init__(self, period)
        MessageListener.__init__(self, template)
        self.name = name

    def getOn(self):
        return self.template

    def getName(self):
        return self.name

    def onTick(self):
        pass

    def _onTick(self):
        try:
            self.onTick()
        except:
            traceback.print_exc()


class OneShotBehaviour(spade.Behaviour.OneShotBehaviour, MessageListener):
    def __init__(self, name, template=None):
        spade.Behaviour.OneShotBehaviour.__init__(self)
        MessageListener.__init__(self, template)
        self.name = name

    def getName(self):
        return self.name

    def process(self):
        pass

    def _process(self):
        try:
            self.process()
        except:
            traceback.print_exc()


class EventBehaviour(spade.Behaviour.EventBehaviour, MessageListener):
    def __init__(self, name, template=None):
        spade.Behaviour.EventBehaviour.__init__(self)
        MessageListener.__init__(self, template)
        self.name = name

    def getName(self):
        return self.name

    def process(self):
        pass

    def _process(self):
        try:
            self.process()
        except:
            traceback.print_exc()


class FSMBehaviour(spade.Behaviour.FSMBehaviour, MessageListener):
    def __init__(self, name, template=None):
        spade.Behaviour.FSMBehaviour.__init__(self)
        MessageListener.__init__(self, template)
        self.name = name

    def getName(self):
        return self.name

    def process(self):
        pass

    def _process(self):
        try:
            self.process()
        except:
            traceback.print_exc()



