import rospy


SUCCESS = 'success'
FAILURE = 'failure'
RUNNING = 'running'


class BehaviorTree:
    def __init__(self, root, context=None):
        self.root = root
        if context is None:
            context = {}
        self.context = context

    def run(self):
        rospy.loginfo('Running behavior tree')
        return self.root.run(self.context)
    
    def run_forever(self, rate):
        rate = rospy.Rate(rate)
        while not rospy.is_shutdown():
            self.run()
            rate.sleep()


class Leaf:
    def run(self, context):
        raise NotImplementedError


class Selector:
    def __init__(self, children):
        self.children = children

    def run(self, context):
        for child in self.children:
            result = child.run(context)
            if result != FAILURE:
                return result
        return FAILURE


class Sequence:
    def __init__(self, children):
        self.children = children

    def run(self, context):
        for child in self.children:
            result = child.run(context)
            if result != SUCCESS:
                return result
        return SUCCESS


class Inverter:
    def __init__(self, child):
        self.child = child

    def run(self, context):
        result = self.child.run(context)
        if result == SUCCESS:
            return FAILURE
        elif result == FAILURE:
            return SUCCESS
        return RUNNING
