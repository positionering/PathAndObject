#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import cv2
from darkflow.net.build import TFNet
import numpy as np
import pyrealsense2 as rs
import datetime

def reduce(txt):
    res=""
    while(len(txt) > 4):
        #Find TL y
        txt = txt[txt.find("'y': ")+len("'y': "):len(txt)]
        y = txt[0:txt.find(',')]
        
        #Find TL x
        txt = txt[txt.find("'x': ")+len("'x': "):len(txt)]
        res += txt[0:txt.find('}')] + ',' + y

        #Find BR y
        txt = txt[txt.find("'y': ")+len("'y': "):len(txt)]
        y = txt[0:txt.find(',')]
        
        #Find BR x
        txt = txt[txt.find("'x': ")+len("'x': "):len(txt)]
        res += ',' + txt[0:txt.find('}')] + ',' + y

        #Find label
        txt = txt[txt.find("'label': '")+len("'label': '"):len(txt)]
        res += ',' + txt[0:txt.find("'")] + ';'
        txt = txt[txt.find("'"):len(txt)]
    return res[0:len(res)-1]




	
def getImage():
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    return np.asanyarray(color_frame.get_data())
    
def paintSquares(frame, results):
    for color, result in zip(colors, results):
        tl = (result['topleft']['x'], result['topleft']['y'])
        br = (result['bottomright']['x'], result['bottomright']['y'])
        label = result['label']
        confidence = result['confidence']
        text = '{}: {:.0f}%'.format(label, confidence * 100)
        frame = cv2.rectangle(frame, tl, br, color, 5)
        frame = cv2.putText(
        frame, text, tl, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
    return frame
        
def publisher():	
    tfnet = TFNet(options)
    
    while not rospy.is_shutdown():
        frame = getImage()
        results = tfnet.return_predict(frame)
        cv2.imwrite("./test1.png", frame)       
#        cv2.imshow("YOLO-CAM", cv2.resize(paintSquares(frame, results), (640, 480)))
        res = reduce(str(results))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break 
        if len(results) > 0:    
            msg_to_publish.data = res
            pub.publish(msg_to_publish)
            rate.sleep()
            #cv2.imwrite("./test1.png",frame)
        else:
            msg_to_publish.data = "empty"
            pub.publish(msg_to_publish)
            rate.sleep()
        print(msg_to_publish.data)        

options = {
    'model': 'cfg/yolo.cfg',
    'load': 'bin/yolov2.weights',
    'threshold': 0.5,
    'gpu': 0.7,
}

pipeline = rs.pipeline()
config = rs.config()
#config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)
pipeline.start(config)
colors = [tuple(255*np.random.rand(3)) for _ in range(10)]
         

if __name__ == "__main__":
    print('In main')
    rospy.init_node("yolo_2_7_node")
    pub = rospy.Publisher('Yolo_data', String, queue_size=10)
    rate = rospy.Rate(100000000)
    msg_to_publish = String()
    print('Created node balle')
    publisher()
