from Queue import Queue
from Queue import Empty
from spadeutils.behaviours.spadeBehaviours import Behaviour


class TaskExecutor(Behaviour):
    def __init__(self):
        super(TaskExecutor, self).__init__("TaskExecutor")
        self.waitingTasks = Queue()

    def onStart(self):
        print "TaskExecutor started"

    def onEnd(self):
        print "TaskExecutor ended"

    def addTask(self, task):
        self.waitingTasks.put(task)

    def process(self):
        try:
            task = self.waitingTasks.get(True, 2)
            task.run()
        except Empty:
            pass
