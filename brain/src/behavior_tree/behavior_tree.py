import rospy


SUCCESS = 'success'
FAILURE = 'failure'
RUNNING = 'running'


class BehaviorTree:
    def __init__(self, root, rate, context={}):
        self.root = root
        self.rate = rate  # [Hz]
        self.context = context

    def run(self):
        return self.root.run(**self.context)
    
    def run_forever(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.run(**self.context)
            rate.sleep()


class Leaf:
    def run(self, **context):
        raise NotImplementedError


class Selector:
    def __init__(self, children):
        self.children = children

    def run(self, **context):
        for child in self.children:
            result = child.run(**context)
            if result != FAILURE:
                return result
        return FAILURE


class Sequence:
    def __init__(self, children):
        self.children = children

    def run(self, **context):
        for child in self.children:
            result = child.run(**context)
            if result != SUCCESS:
                return result
        return SUCCESS


class Inverter:
    def __init__(self, child):
        self.child = child

    def run(self, **context):
        result = self.child.run(**context)
        if result == SUCCESS:
            return FAILURE
        elif result == FAILURE:
            return SUCCESS
        return RUNNING
