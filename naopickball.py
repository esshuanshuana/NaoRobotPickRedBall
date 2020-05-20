# encoding=utf-8
from __future__ import division
from naoqi import ALProxy
from PIL import Image
import motion
import cv2
import math
import vision_definitions
import numpy as np
import almath
import time

IP = "11.1.8.202"  # 设置ROBOT IP
PORT = 9559  # 设置端口
motionProxy = ALProxy("ALMotion", IP, PORT)#申请motion的代理
names = list()
times = list()
keys = list()
x=0
y=0
flag = None

#取得头部偏角
def getHeadAngle(IP, PORT):
    # motionProxy = ALProxy("ALMotion",IP,PORT)
    actuator = ["HeadYaw", "HeadPitch"] #执行机构 头部偏航 最高点 怎么赋值给HeadYaw和HeadPitch
    useSensor = False #传感器 什么时候会变成ture
    headAngle = motionProxy.getAngles(actuator, useSensor)
    return headAngle

#Pitch关节角
def getHeadPitchAngle(IP, PORT):
    # motionProxy = ALProxy("ALMotion",IP,PORT)
    actuator = "HeadPitch"
    useSensor = False
    headAngle = motionProxy.getAngles(actuator, useSensor)
    return headAngle

def setHeadAngle(alpha, beta):
    # motionProxy = ALProxy("ALMotion", IP, PORT)
    motionProxy.setStiffnesses("Head", 1.0)#设置头部钢化为1.0 0的时候不会移动
    maxSpeedFraction = 0.3#最大速度分数为0.3
    names = ["HeadYaw", "HeadPitch"]
    angles = [alpha, beta]
    #传入部位名称、角度值列表、速度
    motionProxy.angleInterpolationWithSpeed(names, angles, maxSpeedFraction)

    motionProxy.setStiffnesses("Head", 0.0)


# 获取图像
def getImage(IP, PORT, cameraID):#cameraID=1
    camProxy = ALProxy("ALVideoDevice", IP, PORT)#申请video的代理
    # vision_definitions.kCameraSelectID = vision_definitions.kBottomCamera ，以前的API写法，使用底部摄像头

    if (cameraID == 0):  # Bottom Camera
        camProxy.setCameraParameter("test", 18, 0)#const std::string& Handle, const int& Id, const int& NewValue
    elif (cameraID == 1):  # Top Camera
        camProxy.setCameraParameter("test", 18, 1)#test和18是什么意思

    resolution = vision_definitions.kVGA  # 是常量??定义resolution分辨率
    colorSpace = vision_definitions.kRGBColorSpace  # 定义色域
    fps = 15 #每秒传输帧数

    nameId = camProxy.subscribe("test", resolution, colorSpace, fps)  # 使用Borker订阅模块
    naoImage = camProxy.getImageRemote(nameId)  # 获取当前图片
    #处理返回的图像，并将其保存为PNG，获取图像大小和像素数组
    imageWidth = naoImage[0]
    imageHeight = naoImage[1]
    array = naoImage[6]
    im = Image.frombytes("RGB", (imageWidth, imageHeight), array)

    im.save("camImage.png", "PNG")  # 临时图片路径 
    camProxy.unsubscribe(nameId)
   
def Binarization(image, pattern="red"):
    """
    Binarization()此方法是采用HSV色域进行二值化图像处理
    :param image:  传入的图像对象
    :param pattern: 需要识别的颜色，默认黄色
    :return:
    """
    # Setting the pattern
    lower = []
    upper = []
    if pattern == "red":
        lower = np.array([0, 120, 120])#HSV0-127 0-10 156-180
        upper = np.array([10, 255, 255])
    elif pattern == "yellow":#26-34
        lower = np.array([20, 100, 100])
        upper = np.array([34, 255, 255])
    elif pattern == "blue":
        lower = np.array([110, 70, 70])
        upper = np.array([124, 255, 255])
    
    
    # BGR to HSV 将一副图像从rgb颜色空间转换到hsv颜色空间，颜色去除白色背景部分
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) #把输入图像灰度化
    # Binarization 设阈值，去除背景部分
    mask = cv2.inRange(hsv, lower, upper)#低于lower的值,高于upper值,图像值变为0

    # Opened the image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)) #形态学基本structure
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) #开运算

    cv2.imshow("Binarization", opened)
    return opened


def calcTheLocate(img):
    """
    计算出图中目标中心的的相对坐标
    :param img: 传图图像对象
    :return:
    """
    col = np.ones(640)  # 采用Numpy创建全为1，长度为640的矩阵
    row = np.ones(480)  # 采用Numpy创建全为1，长度为480的矩阵
    colsum = []  # 存储每列的容器
    rowsum = []  # 存储每行的容器
    x = 0
    xw = 0 # w:west
    xe = 0 # e:est
    y = 0
    yn = 0 #n:north
    ys = 0 #s:south
    for i in range(0, 480):  # 遍历每行
        product = np.dot(col, img[i][:])  # 点乘
        colsum.append(product)
    for i in range(0, 480):  # 计算出坐标
        if (colsum[i] == max(colsum)):
            y = i
            val = max(colsum) / 255
            yn = i - val
            ys = i + val
            break
    for i in range(0, 640):
        product = np.dot(row, img[:, i])
        rowsum.append(product)
    for i in range(0, 640):
        if (rowsum[i] == max(rowsum)):
            x = i
            val = max(colsum) / 255
            xw = i - val
            xe = i + val
            break
    print("locate  x: ", x, xw,  xe, "........ locate y :", y, yn, ys)

    x = int (x)
    y= int (y)
    xw = int (xw)
    xe = int (xe)
    yn = int (yn)
    ys = int (ys)

    #画出轮廓
    cv2.circle(img, (x, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (xw, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (xe, y), 5, (55, 255, 155), -1)
    cv2.circle(img, (x, yn), 5, (55, 255, 155), -1)
    cv2.circle(img, (x, ys), 5, (55, 255, 155), -1)
    cv2.putText(img, "center", (x - 20, y - 20),
    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (55, 255, 155), 2)

    cv2.imshow("two", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return x,y


def getDistanse(x, y, cameraID):
    """
    计算出摄像头在空间中距离目标点的直线距离
    :param x: x坐标
    :param y: y坐标
    :param cameraID:
    :return:
    """
    x = x - 320#图片中心点的像素坐标
    y = y - 240
    alpha = ((-x / 640) * 60.97) * math.pi / 180  # rads
    beta = ((y / 480) * 47.64) * math.pi / 180  # rads
    headAngle = getHeadAngle(IP, PORT)
    alpha = alpha + headAngle[0]#矫正角度
    beta = beta + headAngle[1]

    print("alpha", alpha, "beta", beta)
    print("alpha", alpha / math.pi * 180, "beta", beta / math.pi * 180)

    setHeadAngle(alpha, beta)
    motionProxy.setStiffnesses("Head", 0.0)
    #  摄像头高度
    H = 495
    cameraAngle = 1.2 * math.pi / 180
    if cameraID == 0:
        H = 495
        cameraAngle = 1.2 * math.pi / 180
    elif cameraID == 1:
        H = 477.33
        cameraAngle = 39.7 * math.pi / 180

  #  h = H - 210 - 105 / 2  ################## the height and the diam
    h = H - 155 -  40 / 2
    headPitchAngle = getHeadPitchAngle(IP, PORT)
    # s = (h-100)/math.tan(cameraAngle + headPitchAngle[0])
    s = h / math.tan(cameraAngle + headPitchAngle[0])
    # s = h/math.tan(cameraAngle +beta)
    x = s * math.cos(alpha) / 1000
    y = s * math.sin(alpha) / 1000
    return x, y, alpha


def getDistanceBottom(x, y):
    x = x - 320
    y = y - 240
    alpha = ((-x / 640) * 60.97) * math.pi / 180  # rads
    beta = ((y / 480) * 47.64) * math.pi / 180  # rads
    headAngle = getHeadAngle(IP, PORT)
    alpha = alpha + headAngle[0]
    beta = beta + headAngle[1]
    setHeadAngle(alpha, beta)
    motionProxy.setStiffnesses("Head", 0.0)

    print("Use bottom camera : ")
    print("alpha", alpha, "beta", beta)
    print("alpha", alpha / math.pi * 180, "beta", beta / math.pi * 180)

    H = 477.33  # Not sure
    cameraAngle = 39.7 * math.pi / 180
    h = H - 210 - 105 / 2
    s = h / math.tan(cameraAngle + beta)
    x = s * math.cos(alpha) / 1000
    y = s * math.sin(alpha) / 1000
    z = 30 / 2
    return x, y, z


#设定头部位姿
def head(motionProxy):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([1.96, 3.92, 5.96])
    keys.append([0.261799, 0.274544, 0.269942])

    names.append("HeadYaw")
    times.append([1.96, 3.92, 5.96])
    keys.append([0.387463, -0.659662, -0.0123138])

    motionProxy.setMoveArmsEnabled(False, False)
    motionProxy.angleInterpolation(names, keys, times, True)


#搜索landmark
def searchLandmark(motionProxy):
    motionProxy.setMoveArmsEnabled(False, False)
    setHeadAngle(0, 0)
    motionProxy.moveTo(-0.1, 0.1, math.pi / 8)
    head(motionProxy)
    print("search.........")
    markId = getMarkId()
    return markId


def landmark():
    # Set here your robto's ip.
    #ip = "11.1.25.57"
    # Set here the size of the landmark in meters.
    landmarkTheoreticalSize = 0.06  # in meters
    # Set here the current camera ("CameraTop" or "CameraBottom").
    currentCamera = "CameraTop"

    memoryProxy = ALProxy("ALMemory", IP, 9559)
    landmarkProxy = ALProxy("ALLandMarkDetection", IP, 9559)

    motionProxy.setMoveArmsEnabled(False, False)

    # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
    landmarkProxy.subscribe("landmarkTest")

    # Wait for a mark to be detected.
    try:
        markData = memoryProxy.getData("LandmarkDetected")
    except:
        print("landmark error!!!!!!!!!")
    while (markData is None or len(markData) == 0):
        markData = memoryProxy.getData("LandmarkDetected")

    # Retrieve landmark center position in radians.

    #####################
    markInfoArray = markData[1]
    for markInfo in markInfoArray:
        markShapeInfo = markInfo[0]
        markExtraInfo = markInfo[1]
        alpha = markShapeInfo[1]
        beta = markShapeInfo[2]
        print "mark  ID: %d" % (markExtraInfo[0])
        print "  alpha %.3f - beta %.3f" % (markShapeInfo[1], markShapeInfo[2])
        print "  width %.3f - height %.3f" % (markShapeInfo[3], markShapeInfo[4])

    ############
    wzCamera = markData[1][0][0][1]
    wyCamera = markData[1][0][0][2]

    # Retrieve landmark angular size in radians.
    angularSize = markData[1][0][0][3]

    # Compute distance to landmark.
    distanceFromCameraToLandmark = landmarkTheoreticalSize / (2 * math.tan(angularSize / 2))

    # Get current camera position in NAO space.
    transform = motionProxy.getTransform(currentCamera, 2, True)
    transformList = almath.vectorFloat(transform)
    robotToCamera = almath.Transform(transformList)

    # Compute the rotation to point towards the landmark.
    cameraToLandmarkRotationTransform = almath.Transform_from3DRotation(0, wyCamera, wzCamera)

    # Compute the translation to reach the landmark.
    cameraToLandmarkTranslationTransform = almath.Transform(distanceFromCameraToLandmark, 0, 0)

    # Combine all transformations to get the landmark position in NAO space.
    robotToLandmark = robotToCamera * cameraToLandmarkRotationTransform * cameraToLandmarkTranslationTransform

    print "x " + str(robotToLandmark.r1_c4) + " (in meters)"
    print "y " + str(robotToLandmark.r2_c4) + " (in meters)"
    print "z " + str(robotToLandmark.r3_c4) + " (in meters)"

    x = float(robotToLandmark.r1_c4)
    y = float(robotToLandmark.r2_c4)
    z = float(robotToLandmark.r3_c4)
    print(x, y)
    # print("xxxxxxxxxxxxxxxxxxxxx")
    # print math.sqrt(1 - (math.cos(alpha))**2 + (math.cos(beta))**2)
    # theta = math.acos(math.sqrt(1 - (math.cos(alpha))**2 - (math.cos(beta))**2))
    theta = math.atan(y / x)

    motionProxy.moveTo(x, y, theta)

    landmarkProxy.unsubscribe("landmarkTest")


def getMarkId():
    # Set here your robto's ip.
    #ip = "11.1.8.195"
    # Set here the size of the landmark in meters.
    # landmarkTheoreticalSize = 0.06  # in meters
    # Set here the current camera ("CameraTop" or "CameraBottom").
    # currentCamera = "CameraTop"

    memoryProxy = ALProxy("ALMemory", IP, 9559)
    landmarkProxy = ALProxy("ALLandMarkDetection", IP, 9559)

    # Subscribe to LandmarkDetected event from ALLandMarkDetection proxy.
    landmarkProxy.subscribe("landmarkTest")
    markId = None
    # Wait for a mark to be detected.
    try:
        markData = memoryProxy.getData("LandmarkDetected")
    # while (markData is None or len(markData) == 0):
        markData = memoryProxy.getData("LandmarkDetected")
    except:
        print("getMarkId landmark error!!!!!!!!!")
    
    if (markData is None or len(markData) == 0):
        markId = None
    else:
        markInfoArray = markData[1]
        for markInfo in markInfoArray:
            markShapeInfo = markInfo[0]
            markExtraInfo = markInfo[1]
            alpha = markShapeInfo[1]
            beta = markShapeInfo[2]
            print "mark  ID: %d" % (markExtraInfo[0])
            markId = markExtraInfo[0]

    return markId


# ('lwx : ', -1.627792477607727, 'lwy : ', 0.47067877650260925, 'lwz : ', -0.4202498197555542)
# ('rwx : ', 1.8139231204986572, 'rwy : ', 0.49902573227882385, 'rwz : ', 0.371066689491272)


def grabball(IP,PORT):
    names = list()
    times = list()
    keys = list()

    names.append("HeadPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.078192, [3, -0.866667, 0], [3, 0.44, 0]], [0.0199001, [3, -0.44, 0], [3, 0.826667, 0]], [0.0229681, [3, -0.826667, -0.00306797], [3, 2.02667, 0.00752147]], [0.0628521, [3, -2.02667, 0], [3, 0.586667, 0]], [0.0628521, [3, -0.586667, 0], [3, 0.32, 0]], [0.0413761, [3, -0.32, 0], [3, 0.413333, 0]], [0.078192, [3, -0.413333, 0], [3, 0.746667, 0]], [0.078192, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("HeadYaw")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.023052, [3, -0.866667, 0], [3, 0.44, 0]], [0.108872, [3, -0.44, -0.045471], [3, 0.826667, 0.0854304]], [0.369652, [3, -0.826667, 0], [3, 2.02667, 0]], [0.0137641, [3, -2.02667, 0], [3, 0.586667, 0]], [0.0137641, [3, -0.586667, 0], [3, 0.32, 0]], [-0.0153821, [3, -0.32, 0.0023752], [3, 0.413333, -0.00306797]], [-0.01845, [3, -0.413333, 0], [3, 0.746667, 0]], [-0.01845, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LAnklePitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-1.22111, [3, -0.866667, 0], [3, 0.44, 0]], [-1.18508, [3, -0.44, 0], [3, 0.826667, 0]], [-1.18582, [3, -0.826667, 0.000505661], [3, 2.02667, -0.00123969]], [-1.19031, [3, -2.02667, 0.00449062], [3, 0.586667, -0.00129992]], [-1.22111, [3, -0.586667, 0], [3, 0.32, 0]], [-1.19031, [3, -0.32, 0], [3, 0.413333, 0]], [-1.22417, [3, -0.413333, 0], [3, 0.746667, 0]], [0.0820305, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LAnkleRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.0767419, [3, -0.866667, 0], [3, 0.44, 0]], [0.0785398, [3, -0.44, 0], [3, 0.826667, 0]], [0.0782759, [3, -0.826667, 0], [3, 2.02667, 0]], [0.0802851, [3, -2.02667, 0], [3, 0.586667, 0]], [0.0798099, [3, -0.586667, 0.000475241], [3, 0.32, -0.000259222]], [0.0715585, [3, -0.32, 0], [3, 0.413333, 0]], [0.0798099, [3, -0.413333, 0], [3, 0.746667, 0]], [-0.118682, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LElbowRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-1.01393, [3, -0.866667, 0], [3, 0.44, 0]], [-1.05535, [3, -0.44, 0], [3, 0.826667, 0]], [-1.05075, [3, -0.826667, -0.00460194], [3, 2.02667, 0.0112822]], [-0.66148, [3, -2.02667, 0], [3, 0.586667, 0]], [-0.872804, [3, -0.586667, 0.0918522], [3, 0.32, -0.0501012]], [-1.08734, [3, -0.32, 0], [3, 0.413333, 0]], [-0.479966, [3, -0.413333, -0.00966164], [3, 0.746667, 0.0174533]], [-0.462512, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LElbowYaw")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.825334, [3, -0.866667, 0], [3, 0.44, 0]], [-0.820732, [3, -0.44, 0], [3, 0.826667, 0]], [-0.83147, [3, -0.826667, 0.0107381], [3, 2.02667, -0.0263257]], [-1.05767, [3, -2.02667, 0], [3, 0.586667, 0]], [-1.05697, [3, -0.586667, -0.000701277], [3, 0.32, 0.000382515]], [-0.937242, [3, -0.32, -0.0255385], [3, 0.413333, 0.0329872]], [-0.881391, [3, -0.413333, 0], [3, 0.746667, 0]], [-1.19206, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LHand")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.02, [3, -0.866667, 0], [3, 0.44, 0]], [0.0268, [3, -0.44, 0], [3, 0.826667, 0]], [0.0176001, [3, -0.826667, 0], [3, 2.02667, 0]], [0.02, [3, -2.02667, 0], [3, 0.586667, 0]], [0.0176001, [3, -0.586667, 0], [3, 0.32, 0]], [0.03, [3, -0.32, 0], [3, 0.413333, 0]], [0.02, [3, -0.413333, 0], [3, 0.746667, 0]], [0.28, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LHipPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.705598, [3, -0.866667, 0], [3, 0.44, 0]], [-0.703368, [3, -0.44, 0], [3, 0.826667, 0]], [-0.704064, [3, -0.826667, 0.000696417], [3, 2.02667, -0.00170734]], [-0.842994, [3, -2.02667, 0.0571024], [3, 0.586667, -0.0165296]], [-0.92496, [3, -0.586667, 0], [3, 0.32, 0]], [-0.790634, [3, -0.32, 0], [3, 0.413333, 0]], [-0.903484, [3, -0.413333, 0], [3, 0.746667, 0]], [0.1309, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LHipRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.082794, [3, -0.866667, 0], [3, 0.44, 0]], [-0.0802851, [3, -0.44, -0.000297619], [3, 0.826667, 0.000559164]], [-0.079726, [3, -0.826667, -0.000168552], [3, 2.02667, 0.000413224]], [-0.0785398, [3, -2.02667, 0], [3, 0.586667, 0]], [-0.0919981, [3, -0.586667, 0], [3, 0.32, 0]], [-0.0820305, [3, -0.32, 0], [3, 0.413333, 0]], [-0.0950661, [3, -0.413333, 0], [3, 0.746667, 0]], [0.113446, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LHipYawPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.248466, [3, -0.866667, 0], [3, 0.44, 0]], [-0.244346, [3, -0.44, -0.000256565], [3, 0.826667, 0.000482032]], [-0.243864, [3, -0.826667, 0], [3, 2.02667, 0]], [-0.251327, [3, -2.02667, 0.000713633], [3, 0.586667, -0.000206578]], [-0.251534, [3, -0.586667, 0.000206578], [3, 0.32, -0.000112679]], [-0.254818, [3, -0.32, 0], [3, 0.413333, 0]], [-0.223922, [3, -0.413333, -0.00829199], [3, 0.746667, 0.0149791]], [-0.185005, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LKneePitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[2.14909, [3, -0.866667, 0], [3, 0.44, 0]], [2.11185, [3, -0.44, 0], [3, 0.826667, 0]], [2.11688, [3, -0.826667, 0], [3, 2.02667, 0]], [2.11185, [3, -2.02667, 0], [3, 0.586667, 0]], [2.15523, [3, -0.586667, 0], [3, 0.32, 0]], [2.11185, [3, -0.32, 0], [3, 0.413333, 0]], [2.15369, [3, -0.413333, 0], [3, 0.746667, 0]], [-0.0925025, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LShoulderPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[1.41124, [3, -0.866667, 0], [3, 0.44, 0]], [1.3913, [3, -0.44, 0.00163295], [3, 0.826667, -0.00306796]], [1.38823, [3, -0.826667, 0.00306796], [3, 2.02667, -0.00752145]], [1.09956, [3, -2.02667, 0], [3, 0.586667, 0]], [1.12591, [3, -0.586667, 0], [3, 0.32, 0]], [1.09258, [3, -0.32, 0.0210966], [3, 0.413333, -0.0272498]], [0.980875, [3, -0.413333, 0], [3, 0.746667, 0]], [1.494, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LShoulderRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.18097, [3, -0.866667, 0], [3, 0.44, 0]], [0.18864, [3, -0.44, 0], [3, 0.826667, 0]], [0.18864, [3, -0.826667, 0], [3, 2.02667, 0]], [-0.10821, [3, -2.02667, 0], [3, 0.586667, 0]], [0.00762796, [3, -0.586667, -0.063619], [3, 0.32, 0.0347013]], [0.18675, [3, -0.32, -0.069973], [3, 0.413333, 0.0903818]], [0.488692, [3, -0.413333, 0], [3, 0.746667, 0]], [0.191986, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("LWristYaw")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.115008, [3, -0.866667, 0], [3, 0.44, 0]], [0.125746, [3, -0.44, 0], [3, 0.826667, 0]], [0.125746, [3, -0.826667, 0], [3, 2.02667, 0]], [0.178024, [3, -2.02667, 0], [3, 0.586667, 0]], [0.052114, [3, -0.586667, 0], [3, 0.32, 0]], [0.141372, [3, -0.32, -0.0629946], [3, 0.413333, 0.081368]], [0.485202, [3, -0.413333, 0], [3, 0.746667, 0]], [0.129154, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RAnklePitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-1.21642, [3, -0.866667, 0], [3, 0.44, 0]], [-1.1796, [3, -0.44, -0.00244942], [3, 0.826667, 0.00460194]], [-1.175, [3, -0.826667, 0], [3, 2.02667, 0]], [-1.21642, [3, -2.02667, 0], [3, 0.586667, 0]], [-1.21642, [3, -0.586667, 0], [3, 0.32, 0]], [-1.18267, [3, -0.32, 0], [3, 0.413333, 0]], [-1.21949, [3, -0.413333, 0], [3, 0.746667, 0]], [0.0698132, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RAnkleRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.075124, [3, -0.866667, 0], [3, 0.44, 0]], [-0.0858622, [3, -0.44, 0], [3, 0.826667, 0]], [-0.076658, [3, -0.826667, 0], [3, 2.02667, 0]], [-0.082794, [3, -2.02667, 0], [3, 0.586667, 0]], [-0.082794, [3, -0.586667, 0], [3, 0.32, 0]], [-0.079726, [3, -0.32, -0.00245438], [3, 0.413333, 0.00317024]], [-0.0659201, [3, -0.413333, -0.0138059], [3, 0.746667, 0.0249396]], [0.115192, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RElbowRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.39968, [3, -0.866667, 0], [3, 0.44, 0]], [0.703368, [3, -0.44, 0], [3, 0.826667, 0]], [0.656244, [3, -0.826667, 0.0384419], [3, 2.02667, -0.0942447]], [0.305308, [3, -2.02667, 0], [3, 0.586667, 0]], [0.502655, [3, -0.586667, -0.117101], [3, 0.32, 0.0638732]], [0.84823, [3, -0.32, -0.0741289], [3, 0.413333, 0.0957498]], [1.01229, [3, -0.413333, 0], [3, 0.746667, 0]], [0.479966, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RElbowYaw")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[1.20602, [3, -0.866667, 0], [3, 0.44, 0]], [0.694641, [3, -0.44, 0], [3, 0.826667, 0]], [0.993092, [3, -0.826667, -0.049501], [3, 2.02667, 0.121357]], [1.20722, [3, -2.02667, 0], [3, 0.586667, 0]], [0.820305, [3, -0.586667, 0], [3, 0.32, 0]], [0.966912, [3, -0.32, -0.0266021], [3, 0.413333, 0.034361]], [1.00319, [3, -0.413333, -0.0362816], [3, 0.746667, 0.065541]], [1.29154, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RHand")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.32, [3, -0.866667, 0], [3, 0.44, 0]], [0.02, [3, -0.44, 0], [3, 0.826667, 0]], [0.14, [3, -0.826667, -0.0946418], [3, 2.02667, 0.232025]], [1, [3, -2.02667, 0], [3, 0.586667, 0]], [0, [3, -0.586667, 0], [3, 0.32, 0]], [0.03, [3, -0.32, -0.03], [3, 0.413333, 0.03875]], [0.4, [3, -0.413333, 0], [3, 0.746667, 0]], [0, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RHipPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.704148, [3, -0.866667, 0], [3, 0.44, 0]], [-0.698012, [3, -0.44, -0.00106574], [3, 0.826667, 0.00200231]], [-0.694944, [3, -0.826667, 0], [3, 2.02667, 0]], [-0.702614, [3, -2.02667, 0.00767017], [3, 0.586667, -0.00222031]], [-0.91584, [3, -0.586667, 0], [3, 0.32, 0]], [-0.694944, [3, -0.32, 0], [3, 0.413333, 0]], [-0.898966, [3, -0.413333, 0], [3, 0.746667, 0]], [0.143117, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RHipRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.0859461, [3, -0.866667, 0], [3, 0.44, 0]], [0.0798099, [3, -0.44, 0], [3, 0.826667, 0]], [0.0813439, [3, -0.826667, 0], [3, 2.02667, 0]], [0.0798099, [3, -2.02667, 0], [3, 0.586667, 0]], [0.090548, [3, -0.586667, 0], [3, 0.32, 0]], [0.0813439, [3, -0.32, 0.00178504], [3, 0.413333, -0.00230567]], [0.0782759, [3, -0.413333, 0.00306797], [3, 0.746667, -0.00554214]], [-0.116937, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RHipYawPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.248466, [3, -0.866667, 0], [3, 0.44, 0]], [-0.244346, [3, -0.44, -0.000256565], [3, 0.826667, 0.000482032]], [-0.243864, [3, -0.826667, 0], [3, 2.02667, 0]], [-0.251327, [3, -2.02667, 0.000713633], [3, 0.586667, -0.000206578]], [-0.251534, [3, -0.586667, 0.000206578], [3, 0.32, -0.000112679]], [-0.254818, [3, -0.32, 0], [3, 0.413333, 0]], [-0.223922, [3, -0.413333, -0.00829199], [3, 0.746667, 0.0149791]], [-0.185005, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RKneePitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[2.15224, [3, -0.866667, 0], [3, 0.44, 0]], [2.11389, [3, -0.44, 0], [3, 0.826667, 0]], [2.12156, [3, -0.826667, -0.00399988], [3, 2.02667, 0.00980617]], [2.15531, [3, -2.02667, 0], [3, 0.586667, 0]], [2.15531, [3, -0.586667, 0], [3, 0.32, 0]], [2.11083, [3, -0.32, 0], [3, 0.413333, 0]], [2.14918, [3, -0.413333, 0], [3, 0.746667, 0]], [-0.0925025, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RShoulderPitch")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[1.51495, [3, -0.866667, 0], [3, 0.44, 0]], [1.34739, [3, -0.44, 0.00271577], [3, 0.826667, -0.00510235]], [1.34229, [3, -0.826667, 0.00510235], [3, 2.02667, -0.012509]], [0.817664, [3, -2.02667, 0], [3, 0.586667, 0]], [0.879646, [3, -0.586667, 0], [3, 0.32, 0]], [0.694641, [3, -0.32, 0], [3, 0.413333, 0]], [1.68773, [3, -0.413333, 0], [3, 0.746667, 0]], [1.49749, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RShoulderRoll")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[-0.167552, [3, -0.866667, 0], [3, 0.44, 0]], [-0.684169, [3, -0.44, 0.0929618], [3, 0.826667, -0.174655]], [-0.970403, [3, -0.826667, 0], [3, 2.02667, 0]], [0.279253, [3, -2.02667, -0.120586], [3, 0.586667, 0.0349066]], [0.314159, [3, -0.586667, 0], [3, 0.32, 0]], [-0.434587, [3, -0.32, 0.150543], [3, 0.413333, -0.194451]], [-0.720821, [3, -0.413333, 0], [3, 0.746667, 0]], [-0.153589, [3, -0.746667, 0], [3, 0, 0]]])

    names.append("RWristYaw")
    times.append([2.56, 3.88, 6.36, 12.44, 14.2, 15.16, 16.4, 18.64])
    keys.append([[0.1309, [3, -0.866667, 0], [3, 0.44, 0]], [-0.127409, [3, -0.44, 0], [3, 0.826667, 0]], [0.139626, [3, -0.826667, -0.0348903], [3, 2.02667, 0.0855374]], [0.233874, [3, -2.02667, 0], [3, 0.586667, 0]], [-0.0488692, [3, -0.586667, 0.0741594], [3, 0.32, -0.0404506]], [-0.109956, [3, -0.32, 0.0250282], [3, 0.413333, -0.0323281]], [-0.220938, [3, -0.413333, 0], [3, 0.746667, 0]], [0.120428, [3, -0.746667, 0], [3, 0, 0]]])

    try:
      # uncomment the following line and modify the IP if you use this script outside Choregraphe.
      motion = ALProxy("ALMotion", IP, 9559)
      motion.angleInterpolationBezier(names, times, keys)
    except BaseException, err:
      print err








def putball(IP, strength=8.0):
    names = list()
    times = list()
    keys = list()

    if strength < 0.0:
        strength = -4.0
    elif strength > 20.0:
        strength = 16.0
    else:
        strength = strength * 1.0 - 4.0
    
    names.append("RElbowRoll")
    times.append([0.40000, 1.20000, 2.50000])
    keys.append([[0.98640, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [0.35081, [3, -0.26667, 0.00000], [3, 0.20000, 0.00000]],
                 [0.57072, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RElbowYaw")
    times.append([0.40000])
    keys.append([[1.38669, [3, -0.13333, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RHand")
    times.append([0.40000, 1.20000, 2.50000])
    keys.append([[0.00450, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [0.00489, [3, -0.26667, 0.00000], [3, 0.20000, 0.00000]],
                 [0.00454, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RShoulderPitch")
    times.append([0.40000, 0.80000, 1.20000, 2.50000])
    keys.append([[1.41439, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [0.87790, [3, -0.13333, 0.20461], [3, 0.13333, -0.20461]],
                 [0.18675, [3, -0.13333, 0.00000], [3, 0.20000, 0.00000]],
                 [0.33161, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RShoulderRoll")
    times.append([0.40000, 0.80000, 1.20000, 3.00000])
    keys.append([[-0.05236, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [-0.96866, [3, -0.13333, 0.00000], [3, 0.13333, 0.00000]],
                 [-0.68591, [3, -0.13333, -0.12566], [3, 0.09333, 0.08796]],
                 [strength, [3, strength / 2, 0.00000], [3, 0.00000, 0.00000]]])

    names.append("RWristYaw")
    times.append([0.40000, 1.20000, 3.00000])
    keys.append([[-0.00925, [3, -0.13333, 0.00000], [3, 0.26667, 0.00000]],
                 [-0.69639, [3, -0.26667, 0.21931], [3, 0.20000, -0.16449]],
                 [-1.16064, [3, -0.20000, 0.00000], [3, 0.00000, 0.00000]]])

    motion = ALProxy("ALMotion", IP, 9559)
    postureProxy = ALProxy("ALRobotPosture", IP, 9559)
    postureProxy.goToPosture("StandInit", 0.5)

    motion.angleInterpolationBezier(names, times, keys);
    motion.openHand("RHand")
    time.sleep(1.0)
    postureProxy.goToPosture("StandInit", 0.5)



if __name__ == '__main__':

    setHeadAngle(0, 0.25)
    motionProxy.setStiffnesses("Head", 0.0)#使机器人静止
    
    getImage(IP, PORT, 0)#使用顶部摄像机获取图片
    img = cv2.imread("camImage.png")#保存图像
    af = Binarization(img, "red")#二值化处理
    x, y = calcTheLocate(af)#返回中心坐标
    print("find cinter: ", x, y)#打印坐标
    af = Binarization(img, "red")
    x, y = calcTheLocate(af)
    x, y, theta = getDistanse(x, y, 0)
    print("walk 0:", x, y, theta)
    
    moveConfig = [["MaxStepFrequency", 0]]
    motionProxy.moveTo(x - 0.16, y - (x - 0.16) * math.tan(math.radians(12)), theta, moveConfig)

    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.say("I have arrived!")

    setHeadAngle(0, 0)
    motionProxy.setStiffnesses("Head", 0.0)#使机器人静止
       
    getImage(IP, PORT, 1)
    img = cv2.imread("camImage.png")
    af = Binarization(img, "red")
    x, y = calcTheLocate(af)
    
    print("find cinter: ", x, y)#打印坐标
    x, y, theta = getDistanse(x, y, 1)
    print("walk 1:", x, y, theta)   
    #motionProxy.moveTo(x - 0.1, y - (x - 0.1) * math.tan(math.radians(12)), theta, moveConfig)
    motionProxy.walkTo(x - 0.04, y, theta)
    
    #捡球
    tts = ALProxy("ALTextToSpeech", IP, PORT)
    tts.say("I will grab")
    grabball(IP,PORT)

    markId = None
    #markId = getMarkId(motionProxy)
    while True:
        if markId == None:
            markId = searchLandmark(motionProxy)###
        else:
            break

    landmark()
    putball(IP,20)
    motion1 = ALProxy("ALMotion", IP, 9559)
    postureProxy1 = ALProxy("ALRobotPosture",IP,9559)
    postureProxy1.goToPosture("StandInit",0.5)
    motionProxy.rest()
