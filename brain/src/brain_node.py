#!/usr/bin/env python3

import rospy
from rospy import Subscriber
from std_msgs.msg import String
from behavior_tree.behavior_tree import BehaviorTree, Selector, Sequence, Inverter, Leaf, SUCCESS, FAILURE, RUNNING
No = Not = Inverter


class BrainNode:
    def __init__(self):
        rospy.init_node('brain_node')
        self.behavior_tree = self._create_behavior_tree()
        self.context_subscriber = Subscriber('brain/context', String, self._context_callback, queue_size=1)
    
    def _context_callback(self, message):
        context_str = message.data
        key_value_pairs = context_str.split(',')
        context = {}
        for key_value_pair in key_value_pairs:
            key, value = key_value_pair.split(':')
            key = key.strip()
            value = value.strip()
            context[key] = value
        rospy.loginfo('----------------------------------------')
        rospy.loginfo(f'Updating Context Values: {context}')
        rospy.loginfo('----------------------------------------')
        self.behavior_tree.context = {**self.behavior_tree.context, **context}
        result = self.behavior_tree.run()
        rospy.loginfo('----------------------------------------')
        rospy.loginfo(f'Behavior Tree Result: {result}')
        rospy.loginfo('----------------------------------------')
        return True

    def _create_behavior_tree(self):
        localize = Selector([IsLocalized(), Localize()])
        explore = Selector([IsExplored(), Explore()])
        pick_up = Selector([No(ObjectsRemaining()),
                            IsHoldingObject(),
                            Sequence([Selector([CanPickUp(),
                                                GoToPickUp()]),
                                      PickUp()])])
        drop_off = Selector([Not(IsHoldingObject()),
                             Sequence([Selector([CanDropOff(),
                                                 GoToDropOff()]),
                                       DropOff()])])
        return_to_anchor = ReturnToAnchor()
        root = Sequence([localize, explore, pick_up, drop_off, return_to_anchor])
        behavior_tree = BehaviorTree(root, rate=10, context=self._create_context())
        return behavior_tree
    
    def _create_context(self):
        context = {'IsLocalized': SUCCESS,
                   'Localize': RUNNING,
                   'IsExplored': SUCCESS,
                   'Explore': RUNNING,
                   'ObjectsRemaining': SUCCESS,
                   'IsHoldingObject': FAILURE,
                   'CanPickUp': FAILURE,
                   'GoToPickUp': RUNNING,
                   'PickUp': RUNNING,
                   'CanDropOff': FAILURE,
                   'GoToDropOff': RUNNING,
                   'DropOff': RUNNING,
                   'ReturnToAnchor': RUNNING}
        return context 

    def run(self):
        self.behavior_tree.run()
        rospy.spin()


class IsLocalized(Leaf):
    def run(self, **context):
        result = context['IsLocalized']
        rospy.loginfo(f'IsLocalized: {result}')
        return result
    

class Localize(Leaf):
    def run(self, **context):
        result = context['Localize']
        rospy.loginfo(f'Localize: {result}')
        return result
    

class IsExplored(Leaf):
    def run(self, **context):
        result = context['IsExplored']
        rospy.loginfo(f'IsExplored: {result}')
        return result
    

class Explore(Leaf):
    def run(self, **context):
        result = context['Explore']
        rospy.loginfo(f'Explore: {result}')
        return result


class ObjectsRemaining(Leaf):
    def run(self, **context):
        result = context['ObjectsRemaining']
        rospy.loginfo(f'ObjectsRemaining: {result}')
        return result
    

class IsHoldingObject(Leaf):
    def run(self, **context):
        result = context['IsHoldingObject']
        rospy.loginfo(f'IsHoldingObject: {result}')
        return result
    

class CanPickUp(Leaf):
    def run(self, **context):
        result = context['CanPickUp']
        rospy.loginfo(f'CanPickUp: {result}')
        return result
    

class GoToPickUp(Leaf):
    def run(self, **context):
        result = context['GoToPickUp']
        rospy.loginfo(f'GoToPickUp: {result}')
        return result
    

class PickUp(Leaf):
    def run(self, **context):
        result = context['PickUp']
        rospy.loginfo(f'PickUp: {result}')
        return result
    

class CanDropOff(Leaf):
    def run(self, **context):
        result = context['CanDropOff']
        rospy.loginfo(f'CanDropOff: {result}')
        return result
    

class GoToDropOff(Leaf):
    def run(self, **context):
        result = context['GoToDropOff']
        rospy.loginfo(f'GoToDropOff: {result}')
        return result
    

class DropOff(Leaf):
    def run(self, **context):
        result = context['DropOff']
        rospy.loginfo(f'DropOff: {result}')
        return result
    

class ReturnToAnchor(Leaf):
    def run(self, **context):
        result = context['ReturnToAnchor']
        rospy.loginfo(f'ReturnToAnchor: {result}')
        return result
        

if __name__ == '__main__':
    brain_node = BrainNode()
    brain_node.run()
