SUCCESS = 'success'
FAILURE = 'failure'
RUNNING = 'running'


class BehaviorTree:
    def __init__(self, root, context=None):
        self.root = root
        self.context = context
        self.root.context = context

    def run(self):
        return self.root.run()


class Leaf:
    def __init__(self):
        self._context = None

    def run(self):
        raise NotImplementedError
    
    @property
    def context(self):
        return self._context
    
    @context.setter
    def context(self, context):
        self._context = context


class Selector:
    def __init__(self, children):
        self._context = None
        self.children = children

    def run(self):
        for child in self.children:
            result = child.run()
            if result != FAILURE:
                return result
        return FAILURE
    
    @property
    def context(self):
        return self._context
    
    @context.setter
    def context(self, context):
        self._context = context
        for child in self.children:
            child.context = context


class Sequence:
    def __init__(self, children):
        self._context = None
        self.children = children

    def run(self):
        for child in self.children:
            result = child.run()
            if result != SUCCESS:
                return result
        return SUCCESS
    
    @property
    def context(self):
        return self._context
    
    @context.setter
    def context(self, context):
        self._context = context
        for child in self.children:
            child.context = context


class Inverter:
    def __init__(self, child):
        self._context = None
        self.child = child

    def run(self):
        result = self.child.run()
        if result == SUCCESS:
            return FAILURE
        elif result == FAILURE:
            return SUCCESS
        return RUNNING
    
    @property
    def context(self):
        return self._context
    
    @context.setter
    def context(self, context):
        self._context = context
        self.child.context = context
