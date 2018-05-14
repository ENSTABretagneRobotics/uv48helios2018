import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
// define state Idle
class Idle(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['mission'])
    self.counter = 0

  def execute(self, userdata):
    rospy.loginfo('Executing state Idle')
    if self.counter < 3:
      self.counter += 1
      return 'mission'
    else:
      return 'idle'

//definit etat mission
class Mission(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset', 'sleep'])

  def execute(self, userdata):
    rospy.loginfo('Executing state Mission')
    if self.counter < 3:
      self.counter += 1
      return 'mission'
    elif self.counteur == 2:
      return 'sleep'
    else:
      return 'idle'

//definit etat sleep
class Sleep(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['reset', 'mission', 'sleep'])

  def execute(self, userdata):
    rospy.loginfo('Executing state Sleep')
    if self.counter < 3:
      self.counter += 1
      return 'mission'
    elif self.counteur == 2:
      return 'reset'
    else:
      return 'sleep'

//definit etat bar
class Reset(smach.State):
  def __init__(self):
    smach.State.__init__(self, outcomes=['idle'])

  def execute(self, userdata):
    rospy.loginfo('Executing state Resest')
    if self.counter < 3:
      self.counter += 1
      return 'idle'


def main():
  rospy.init_node('smach_example_state_machine')

  // Create a SMACH state machine
  sm = smach.StateMachine(outcomes=['Idle'])

  // Open the container
  with sm:
  // Add states to the container
    smach.StateMachine.add('Idle', Idle(), transitions={'reset':'Reset', 'sleep':'Sleep'})
    smach.StateMachine.add('Mission', Mission(), transitions={'reset':'Reset', 'sleep':'Sleep'})
    smach.StateMachine.add('Sleep', Sleep(), transitions={'reset':'Reset', 'mission':'Mission', 'sleep':'Sleep'})
    smach.StateMachine.add('Reset', Reset(), transitions={'idle':'Idle'})

  // Execute SMACH plan
  outcome = sm.execute()


if __name__ == '__main__':
      main()
