#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from object_detection.msg import ObjectAttr, BoundingBoxes, BoundingBox
from rospy.numpy_msg import numpy_msg
from timeit import default_timer as timer
import yolo
import number_classifier
import copy as cp
import thread
from threading import Lock
from time import sleep
import glob

number_path = '/home/anonymous/NumberDatasets/Numbers/'
ss_path     = '/home/anonymous/NumberDatasets/SS/'

def NumbersfromDataset():
    bBox = BoundingBox()
    bBoxes = BoundingBoxes()
    # rate = rospy.Rate(10)
    rgb = np.array([0,0,0])
    colors = [(255,255,0), (255,0,0), (0,255,0), (0,0,255), (0, 192, 255), (0, 192, 255), (0, 192, 255), (0, 192, 255), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

    yl = yolo.YOLO()
    # numberClassifier = number_classifier.Classifier(model_path='model_data/number_classifier_weights.h5')
    # sevenSegmentClassifier = number_classifier.Classifier(model_path='model_data/seven_segment_classifier_weights.h5')
    const_x = 1
    const_y = 1
    class_names = yl.class_names
    nul = BoundingBox()
    nul.xmin = nul.xmax = nul.ymin = nul.ymax = -1
    count_number_collected = 175
    count_ss_collected = 2012
    path = "/home/anonymous/Rasyid/Rasyid Ganteng Nitip/BarunastraITS-2019/yolov3_keras/dataset/angka/*.jpg"
    filenames = [img for img in glob.glob(path)]
    print(filenames)
    for img in filenames :
        image = cv2.imread(img)
        image_copy = np.array(image, dtype=np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        out_boxes, out_scores, out_classes = yl.detect_image(image)
        for i, c in reversed(list(enumerate(out_classes))):
            if c==4:
                score = int(out_scores[i]*100)
                top, left, bottom, right = out_boxes[i]
                bBox.ymin = top = int(max(0, np.floor(top + 0.5)) * const_y)
                bBox.xmin = left = int(max(0, np.floor(left + 0.5)) * const_x)
                bBox.ymax = bottom = int(min(image.shape[1], np.floor(bottom + 0.5)) * const_y)
                bBox.xmax = right = int(min(image.shape[0], np.floor(right + 0.5)) * const_x)
                crop_img = saveNumberDetected(image_copy, bBox, count_number_collected, score, c)
                count_number_collected+=1
            elif c==5:
                score = int(out_scores[i]*100)
                top, left, bottom, right = out_boxes[i]
                bBox.ymin = top = int(max(0, np.floor(top + 0.5)) * const_y)
                bBox.xmin = left = int(max(0, np.floor(left + 0.5)) * const_x)
                bBox.ymax = bottom = int(min(image.shape[1], np.floor(bottom + 0.5)) * const_y)
                bBox.xmax = right = int(min(image.shape[0], np.floor(right + 0.5)) * const_x)
                crop_img = saveNumberDetected(image_copy, bBox, count_ss_collected, score, c)
                count_ss_collected+=1
        # cv2.waitKey(1)
    yl.close_session()


def saveNumberDetected(image,box, count, score, cl) :
    crop_img = image[box.ymin:box.ymax, box.xmin:box.xmax]
    dim = (128, 128)
    crop_img = cv2.resize(crop_img, dim, interpolation=cv2.INTER_AREA)
    if cl==4:
        dirpath = number_path
    else:
        dirpath = ss_path
    filename = dirpath + '{} - {}%'.format(count, score) + '.jpg'
    cv2.imwrite(filename, crop_img)
    return crop_img

# rospy.init_node("Camera", anonymous=False)
# bridge = CvBridge()
# NumbersfromDataset()
# exit()

flag = True
flushing = True
active_cam = 0  
caps = [None, None]
cam_exist = [0, 0]
caps[1] = cv2.VideoCapture(2)
if not caps[1].isOpened():
    print("Couldn't open webcam on drone")
else:
    cam_exist[1]=1
caps[0] = cv2.VideoCapture(1)
if not caps[0].isOpened():
    caps[0] = cv2.VideoCapture(0)
if not caps[0].isOpened():
    raise IOError("Couldn't open webcam on boat")
cam_exist[0] = 1

def switchCam(data):
    mtx = Lock()
    global active_cam
    active_cam = data.data
    print("Camera switched to " + str(active_cam))

def listenCamSwitching():
    cam_sub = rospy.Subscriber("cam_switch", Int8, switchCam)
    while not rospy.is_shutdown():
        rospy.spin()

def nothing(x):
    pass

def grab_frame():
    global flag, caps, flushing, active_cam
    mtx = Lock()
    while flag:
        if(cam_exist[active_cam]==0):continue
        if flushing:
            caps[active_cam].grab()
            sleep(0.001)
        sleep(0.001)
    flag = True

rospy.init_node("Camera", anonymous=False)
bridge = CvBridge()
obj_pub = rospy.Publisher("object_attr", numpy_msg(ObjectAttr), queue_size=1)
# NumbersfromDataset()
try:
    thread.start_new_thread (grab_frame, ())
except:
    print("Error: unable to start thread")

try:
    thread.start_new_thread (listenCamSwitching, ())
except:
    print("Error: unable to start thread for subscribing")

print("Vision of Nala Destroyer")
objMsg = ObjectAttr()
bBox = BoundingBox()
bBoxes = BoundingBoxes()
rate = rospy.Rate(15)
rgb = np.array([0,0,0])
colors = [(255,255,0), (255,0,0), (0,255,0), (0,0,255), (0, 192, 255), (0, 192, 255), (0, 192, 255), (0, 192, 255), (0, 0, 0), (0, 0, 0), (0, 0, 0), (0, 0, 0)]

yolo = yolo.YOLO()
classifier = number_classifier.Classifier(model_path_number='model_data/number_classifier_weights_test.h5', model_path_seven_segment='model_data/seven_segment_classifier_weights.h5')
const_x = 640.0/416.0
const_y = 480.0/416.0
class_names = yolo.class_names
nul = BoundingBox()
nul.xmin = nul.xmax = nul.ymin = nul.ymax = -1
fps = "FPS: "
font = cv2.FONT_HERSHEY_SIMPLEX
print("Vision started")
# count_number_collected = 1450
while not rospy.is_shutdown():
    try:
        if(cam_exist[active_cam]==0):continue
        start = timer()
        flushing = False
        ret, image = caps[active_cam].read()
        if ret is False: continue
        if image.shape[0]<1: continue
        image_copy = np.array(image, dtype=np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        out_boxes, out_scores, out_classes = yolo.detect_image(cv2.resize(image, (416, 416)))
        cv2.putText(image, fps, (15, 30), font, 0.65, (255, 0, 0), 2, cv2.LINE_AA)
        box = [[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul],[nul]]
        seven_segment_max_score = -1
        ss_top=0; ss_bot=0; ss_left=0; ss_right=0
        number_max_score = -1
        n_top=0; n_bottom=0; n_left=0; n_right=0
        cc = 0
        for i, c in reversed(list(enumerate(out_classes))):
            predicted_class = class_names[c]
            score = int(out_scores[i]*100)
            if(score<50 and (c==2 or c==1)): continue
            top, left, bottom, right = out_boxes[i]
            bBox.ymin = top = int(max(0, np.floor(top + 0.5)) * const_y)
            bBox.xmin = left = int(max(0, np.floor(left + 0.5)) * const_x)
            bBox.ymax = bottom = int(min(image.shape[1], np.floor(bottom + 0.5)) * const_y)
            bBox.xmax = right = int(min(image.shape[0], np.floor(right + 0.5)) * const_x)
            # print('predicted class : ' + str(predicted_class))
            if c>99:
                if c==4 and score>number_max_score:
                    number_max_score=score
                    n_top=top; n_bottom=bottom; n_left=left; n_right=right
                elif c==5 and score>seven_segment_max_score:
                    seven_segment_max_score=score
                    ss_top=top; ss_bottom=bottom; ss_left=left; ss_right=right
            else:
                if c==0:
                    rgb[0] = np.mean(image[top:bottom, left:right, 0])
                    rgb[1] = np.mean(image[top:bottom, left:right, 1])
                    rgb[2] = np.mean(image[top:bottom, left:right, 2])
                    if rgb[0]>rgb[1] and rgb[0]>rgb[2]: bBox.colorIdx = 0
                    elif rgb[1]>rgb[0] and rgb[1]>rgb[2]: bBox.colorIdx = 1
                    else: bBox.colorIdx = 2
                else:
                    bBox.colorIdx = -1
                if c==4:
                    c = classifier.predict_image_number(image[top:bottom, left:right, :]) + 4
                    cc = c-3
                elif c==5:
                    c = classifier.predict_image_seven_segment(image[top:bottom, left:right, :]) + 8
                    cc = c-7
                    # print(cc)
                # midx = float( (left + right) * 0.5 )
                # bBox.sudut = (midx - 320.5) / 320 * 45;     # derajat
                bBox.sudut = 0
                if box[c][0].xmin == -1:
                    box[c][0] = cp.deepcopy(bBox)
                else:
                    box[c].append(cp.deepcopy(bBox))
                cv2.rectangle(image, (left,top), (right,bottom), colors[c], 2)
                hmm = 15 if top<15 else -10
                if c<4: cv2.putText(image, "{} {}%".format(predicted_class, score), (left+3, top+hmm), font, 0.5, colors[1], 1, cv2.LINE_AA)
                else: cv2.putText(image, "{}:{} {}%".format(predicted_class, str(cc), score), (left+3, top+hmm), font, 0.5, colors[1], 1, cv2.LINE_AA)
        # if seven_segment_max_score>-1:
        #     bBox.ymin=n_top;bBox.xmin=n_left;bBox.ymax=n_bottom;bBox.xmax=n_right
        #     bBox.sudut=0
        #     bBox.colorIdx=0
        #     some preprocessing
        #     c = sevenSegmentClassifier.predict_image(image[n_top:n_bottom, n_left:n_right, :])
        #     box[c].append(bBox)
        #     cv2.rectangle(image, (n_left,n_top), (n_right,n_bottom), colors[c], 2)
        #     hmm = 15 if top<15 else -10
        #     cv2.putText(image, "{} {}%".format("Seven Segment", number_max_score), (left+3, top+hmm), font, 0.5, colors[1], 1, cv2.LINE_AA)
        for it in range(12):
            # print(box[it])
            # print("======={}".format(it))
            objMsg.detected_object[it].boxes = box[it]
        try:
            objMsg.img = bridge.cv2_to_imgmsg(image, "bgr8")
            obj_pub.publish(objMsg)
        except CvBridgeError as err:
            print(err)
        flushing = True
    except KeyboardInterrupt:
        flag = False
        break
    rate.sleep()
    end = timer()
    fps = "FPS: " + str(round(1/(end-start), 2))
#        print(fps)
    # cv2.imshow("aaa", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    # if cv2.waitKey(1) == 10 :
    #     for i, c in reversed(list(enumerate(out_classes))):
    #         if c==4:
    #             score = int(out_scores[i]*100)
    #             top, left, bottom, right = out_boxes[i]
    #             bBox.ymin = top = int(max(0, np.floor(top + 0.5)) * const_y)
    #             bBox.xmin = left = int(max(0, np.floor(left + 0.5)) * const_x)
    #             bBox.ymax = bottom = int(min(image.shape[1], np.floor(bottom + 0.5)) * const_y)
    #             bBox.xmax = right = int(min(image.shape[0], np.floor(right + 0.5)) * const_x)
    #             crop_img = saveNumberDetected(image_copy, bBox, count_number_collected, score)
    #             count_number_collected+=1
         
yolo.close_session()
classifier.close_session()
cv2.destroyAllWindows()
while flag is False:
    continue
caps[0].release()
caps[1].release()
print("\nexitted")
