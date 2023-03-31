#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from gtts import gTTS
import os

class Speaker():
    def __init__(self):
        """ Put the node name here, and description of the node"""
        rospy.init_node('speaker')

        # Subscribers 
        self.sub_topic = rospy.Subscriber("/speaker/speech", String, self.speaker_callback)

     
        
    def speaker_callback(self, msg): 
        
        text = msg.data
        # Language in which you want to convert
        language = 'sv'
        
        myobj = gTTS(text, lang=language, slow=False)
        
        # Saving the converted audio in a mp3 file named
        path = "/home/robot/dd2419_ws/src/speak/src/recordings/msg.mp3"
        myobj.save(path)
        
        play = "mpg123 " + path
        # Playing the converted file
        os.system(play)


if __name__ == "__main__":

    speaker=Speaker()
    rospy.spin()