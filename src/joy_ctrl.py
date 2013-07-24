#!/usr/bin/env python
# -*- coding: utf-

import roslib; roslib.load_manifest('qbo_joy_ctrl')
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedbackArray
from sensor_msgs.msg import JoyFeedback

from qbo_arduqbo.msg import Mouth as Mouth_msg
from qbo_arduqbo.msg import Nose as Nose_msg

from qbo_talk.srv import Text2Speach

import time
import sys, tty, termios
import types
from random import shuffle

def numberToMouthArray(shapeNumber):
    #create shape array from string number
    #shapeNumber = int(number)
    shapeBinString="{0:b}".format(shapeNumber)
    shape = [ 1 if n=='1' else 0 for n in shapeBinString ]
    while len(shape)<20:
        shape = [0] + shape
    shape.reverse()
    return shape
        
class Mouth:
    def __init__(self,idN,name,shape):
        if type(shape)==types.IntType:
            shape=numberToMouthArray(shape)
        self.idN=int(idN)
        self.name=str(name)
        self.shape=shape
        
class Nose:
    def __init__(self,idN,name,value):
        self.idN=int(idN)
        self.name=str(name)
        self.color=value

class JoyCtrl:
    def __init__(self):
        self.axis_wheel_straight = rospy.get_param('axis_wheel_straight', 1)
        self.axis_wheel_turn = rospy.get_param('axis_wheel_turn', 0)
        self.axis_head_pan = rospy.get_param('axis_head_pan', 2)
        self.axis_head_tilt = rospy.get_param('axis_head_tilt', 3)
        self.button_nose = rospy.get_param('button_nose', 14)
        self.button_mouth = rospy.get_param('button_mouth', 13)
        self.button_joke = rospy.get_param('button_joke', 12)
        self.button_say = rospy.get_param('button_say', 15)
        
        self.vNoses=[]
        self.vNoses.append(Nose(0,"Off",0,));
        self.vNoses.append(Nose(1,"Red",1,));
        self.vNoses.append(Nose(2,"Green",2,));
        self.NoseValue = 0
        
        self.vMouths=[]
        self.vMouths.append(Mouth(0,"None",0));
        self.vMouths.append(Mouth(1,"Speak 1",31));
        self.vMouths.append(Mouth(2,"Speak 2",479));
        self.vMouths.append(Mouth(3,"Speak 3",14911));
        self.vMouths.append(Mouth(4,"Speak 4",15359));
        self.vMouths.append(Mouth(5,"Speak 5",476735));
        self.vMouths.append(Mouth(6,"Speak 6",491519));
        self.vMouths.append(Mouth(7,"Happy",476160,));
        self.vMouths.append(Mouth(8,"Sad",571392));
        self.vMouths.append(Mouth(9,"Ooh Face",476718));
        self.vMouths.append(Mouth(10,"Pucker the mouth to the right",17376));
        self.vMouths.append(Mouth(11,"Pucker the mouth to the left",2016));
        self.vMouths.append(Mouth(12,"Straight face",31744));
        self.vMouths.append(Mouth(13,"Small mouth",141636));
        self.vMouths.append(Mouth(14,"Surprise",476718));
        self.vMouths.append(Mouth(15,"Regular",69904));
        self.vMouths.append(Mouth(16,"Tongue",283616));
        self.vMouths.append(Mouth(17,"Angry Annoyed",63488));
        self.vMouths.append(Mouth(18,"Elvis Lip",58531));
        self.vMouths.append(Mouth(19,"Yelling",1035136));
        self.vMouths.append(Mouth(20,"Smirk",7440));
        self.vMouths.append(Mouth(21,"Down Arrow",141856));
        self.vMouths.append(Mouth(22,"Open Mouth Smile",477152));
        self.vMouths.append(Mouth(23,"House",501030));
        self.vMouths.append(Mouth(24,"Bomb",141646));
        self.vMouths.append(Mouth(25,"House 2",1033540));
        self.vMouths.append(Mouth(26,"Frown",571392));
        self.vMouths.append(Mouth(27,"Double Frown",571950));
        self.vMouths.append(Mouth(28,"Infinity",349866));
        self.vMouths.append(Mouth(29,"Error",21824));
        self.vMouths.append(Mouth(30,"Plus",145536));
        self.vMouths.append(Mouth(31,"Teeth Grin",15328));
        self.vMouths.append(Mouth(32,"Confusion",200832));
        self.vMouths.append(Mouth(33,"Doubt",69904));
        self.vMouths.append(Mouth(34,"Shock",349504));
        self.vMouths.append(Mouth(35,"Smile",14880));
        self.vMouths.append(Mouth(36,"X Down",332113));
        self.vMouths.append(Mouth(37,"X Up",567434));
        self.vMouths.append(Mouth(38,"Down",585156));
        self.vMouths.append(Mouth(39,"Up",146289));
        
        self.MouthValue = 1
        
        self.Jokes=[]
        self.Jokes.append(("Why was the robot angry?","Because someone kept pushing his buttons!"))
        self.Jokes.append(("What is a robots favorite type of music?","Heavy metal!"))
        self.Jokes.append(("How many robots does it take to screw in a light bulb?","Three, One to hold the bulb, and two to turn the ladder!"))
        self.Jokes.append(("Why did the robot go back to robot school?","Because his skills were getting a little rusty!"))
        self.Jokes.append(("What do you get when you cross a robot and a tractor?","A trans farmer!"))
        self.Jokes.append(("What did the man say to his dead robot?","Rust in peace."))
        
        #self.Jokes = list(shuffle(self.Jokes))
        self.JokeValue = 0
        
        self.Statements=[]
        self.Statements.append("Bite My Shine ne Metal Ass")
        self.Statements.append("Lets start with the three fundamental Rules of Robotics.... We have: one, a robot may not injure a human being, or, through inaction, allow a human being to come to harm. Two, a robot must obey the orders given it by human beings except where such orders would conflict with the First Law. And three, a robot must protect its own existence as long as such protection does not conflict with the First or Second Laws.")
        self.Statements.append("The danger of the future is that men may become robots. True enough, robots do not rebel. But given mans nature, robots cannot live and remain sane, they become Golems, they will destroy their world and themselves because they cannot stand any longer the boredom of a meaningless life.")
        self.Statements.append("We are survival machines -- robot vehicles blindly programmed to preserve the selfish molecules known as genes.")
        self.Statements.append("Man is a robot with defects.")
        self.Statements.append("Nature (the art whereby God hath made and governs the world) is by the art of man, as in many other things, so in this also imitated, that it can make an Artificial Animal. For seeing life is but a motion of Limbs, the beginning whereof is in some principal part within; why may we not say, that all Automata (Engines that move themselves by springs and wheels as doth a watch) have an artificial life? For what is the Heart, but a Spring; and the Nerves, but so many Strings; and the Joints, but so many Wheels, giving motion to the whole Body, such as was intended by the Artificer? Art goes yet further, imitating that rational and most excellent work of Nature, Man.")
        self.Statements.append("Making realistic robots is going to polarize the market, if you will. You will have some people who love it and some people who will really be disturbed.")
        self.Statements.append("If you make robots perfectly realistic, you trigger this body-snatcher fear in some people.")
        self.Statements.append("Machines smart enough to do anything for us will probably also be able to do anything with us: go to dinner, own property, compete for sexual partners. They might even have passionate opinions about politics or, like the robots on Battlestar Galactica, even religious beliefs. Some have worried about robot rebellions, but with so many tort lawyers around to apply the brakes, the bigger question is this: Will humanoid machines enrich our social lives, or will they be a new kind of television, destroying our relationships with real humans?")
        self.Statements.append("You gotta be pretty desperate to make it with a robot.")
        self.Statements.append("The machine has no feelings, it feels no fear and no hope ... it operates according to the pure logic of probability. For this reason I assert that the robot perceives more accurately than man.")
        self.Statements.append("We are not the only avatars of humanity. Once our computing machines achieved self-consciousness, they became part of this design.")
        self.Statements.append("At bottom, robotics is about us. It is the discipline of emulating our lives, of wondering how we work.")
        self.Statements.append("I visualize a time when we will be to robots what dogs are to humans, and Im rooting for the machines.")
        
        #self.Statements = list(shuffle(self.Statements))
        self.StatementValue = 0
        
        self.head_tilt_pos = 0
        self.head_pan_pos = 0
        
        self.pub_base = rospy.Publisher('/cmd_vel', Twist)
        self.pub_joints = rospy.Publisher('/cmd_joints',JointState)
        self.mouth_pub = rospy.Publisher('/cmd_mouth', Mouth_msg)
        self.nose_pub = rospy.Publisher('/cmd_nose', Nose_msg)
        
        self.client_speak = rospy.ServiceProxy("/qbo_talk/festival_say", Text2Speach)
        
        self.ps3joy_pub = rospy.Publisher('/joy/set_feedback', JoyFeedbackArray)
        
        self.sub_joy = rospy.Subscriber('/joy', Joy, self.joy_cb)
        self.sub_joint = rospy.Subscriber('/joint_states', JointState, self.joint_cb)
        
        #self.setPS3LED()

# DOESN't WORK. Don't know what I did wrong exactly
    def setPS3LED(self):
        led0 = JoyFeedback()
        led0.type = JoyFeedback.TYPE_LED
        led0.id = 0
        led0.intensity = 1.
        led1 = JoyFeedback()
        led1.type = JoyFeedback.TYPE_LED
        led1.id = 1
        led1.intensity = 0.
        led2 = JoyFeedback()
        led2.type = JoyFeedback.TYPE_LED
        led2.id = 2
        led2.intensity = 0.
        led3 = JoyFeedback()
        led3.type = JoyFeedback.TYPE_LED
        led3.id = 3
        led3.intensity = 0.
        rum0 = JoyFeedback()
        rum0.type = JoyFeedback.TYPE_RUMBLE
        rum0.id = 0
        rum0.intensity = 0.
        rum1 = JoyFeedback()
        rum1.type = JoyFeedback.TYPE_RUMBLE
        rum1.id = 1
        rum1.intensity = 0.
        feedback = JoyFeedbackArray()
        feedback.array = [led0,led1,led2,led3,rum0,rum1]    
        self.ps3joy_pub.publish(feedback)
        rospy.loginfo("setPS3LED: "+str(feedback))

    def joy_cb(self, data):
        #if data.axes[self.axis_wheel_straight] <> 0 or data.axes[self.axis_wheel_turn] <> 0:
        self.move_base(self.pub_base, data.axes[self.axis_wheel_straight]*0.5, data.axes[self.axis_wheel_turn])
        #if data.axes[self.axis_head_tilt] <> 0 or data.axes[self.axis_head_pan] <> 0:
        self.move_head(self.pub_joints, data.axes[self.axis_head_tilt], data.axes[self.axis_head_pan])
        if data.buttons[self.button_nose] == 1:
            self.rotateNoseColor()
        if data.buttons[self.button_mouth] == 1:
            self.rotateMouth()
        if data.buttons[self.button_joke] == 1:
            self.rotateJoke()
        if data.buttons[self.button_say] == 1:
            self.rotateStatement()

    def joint_cb(self, data):
        self.head_tilt_pos = data.position[3]
        self.head_pan_pos = data.position[2]
        
    def rotateMouth(self):
        self.MouthValue += 1
        if self.MouthValue > len(self.vMouths):
            self.MouthValue = 1
        self.setMouth(self.MouthValue)
        
    def setMouth(self,mouth):
        boca=Mouth_msg()
        boca.mouthImage=self.vMouths[mouth-1].shape
        self.mouth_pub.publish(boca)
        #self.speakMessage(self.vMouths[mouth-1].name)
        rospy.loginfo("setMouth: "+self.vMouths[mouth-1].name)
                
    def rotateNoseColor(self):
        self.NoseValue += 1
        if self.NoseValue > 2:
            self.NoseValue = 0
        self.setNose()
        
    def setNose(self):
        boca=Nose_msg()
        boca.color=self.vNoses[self.NoseValue].color
        self.nose_pub.publish(boca)
        #self.speakMessage(self.vNoses[self.NoseValue].name)
        rospy.loginfo("setNose: "+self.vNoses[self.NoseValue].name)
    
    def rotateStatement(self):
        self.speakMessage(self.Statements[self.StatementValue])
        self.StatementValue += 1
        if self.StatementValue > (len(self.Statements)-1):
            self.StatementValue = 0
    
    def rotateJoke(self):
        self.speakMessage(self.Jokes[self.JokeValue][0])
        rospy.sleep(1.3)
        self.speakMessage(self.Jokes[self.JokeValue][1])
        self.JokeValue += 1
        if self.JokeValue > (len(self.Jokes)-1):
            self.JokeValue = 0
        
    def speakMessage(self, message):
        message_encoded=message.encode('utf8')
        rospy.loginfo("speakMessage: "+str(message_encoded))
        self.client_speak(message_encoded)

    def move_base(self, publisher, linear, ang):
        speed_command=Twist()
        speed_command.linear.x=linear
        speed_command.linear.y=0
        speed_command.linear.z=0
        speed_command.angular.x=0
        speed_command.angular.y=0
        speed_command.angular.z=ang
        publisher.publish(speed_command)

    def move_head(self, publisher, speed_tilt, speed_pan, v_accel=2, h_accel=2):
        servo_command=JointState()
        servo_command.name=['head_tilt_joint', 'head_pan_joint']
        tilt_pos = self.head_tilt_pos
        pan_pos = self.head_pan_pos
    
        tilt_pos += speed_tilt*v_accel
        pan_pos += speed_pan*h_accel
    
        servo_command.position=[tilt_pos, pan_pos]
    
        publisher.publish(servo_command)

        

def main():
    rospy.init_node('qbo_joy_ctrl')

    c = JoyCtrl()

    rospy.spin()    

if __name__ == "__main__":
    main()
