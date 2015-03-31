"""
    Functions to create Messages
"""

import imghdr

import cv2
import rospy

from cv_bridge import CvBridge, CvBridgeError

from config import *

"""
    ROS Messages
"""

def ImageFactory(img_path, img_type='bgr8', debug=True):
    """ Reads an image from file and create a ROS message """

    image = None

    bridge = CvBridge()

    if imghdr.what(img_path) in ['jpeg', 'png', 'bmp']:
        # If the file was an accepted image try to read it.
        try:
            current_img = cv2.imread(img_path, -1)
            image = bridge.cv2_to_imgmsg(current_img, img_type)
        except CvBridgeError, err:
            rospy.logerr(err)
    else:
        rospy.logerr(img_path + " is not an accepted file.")

    return image

def PoseFactory(data):
    """ Pose

    Point position
    Quaternion orientation

    """
    msg = Pose()

    msg.position = PointFactory(data)
    msg.orientation = QuaternionFactory(data)

    return msg

def PoseStampedFactory(data):
    """ PoseStamped

    Header header
    Pose pose

    """
    msg = PoseStamped()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.pose = PoseFactory(data)

    return msg

def QuaternionFactory(data):
    """ Quaternion

    float64 x
    float64 y
    float64 z
    float64 w

    """
    msg = Quaternion()

    msg.x = data.x
    msg.y = data.y
    msg.z = data.z
    msg.w = data.w

    return msg

def PointFactory(data):
    """ Point

    float64 x
    float64 y
    float64 z

    """
    msg = Point()

    msg.x = data.x
    msg.y = data.y
    msg.z = data.z

    return msg

"""
    Vision Messages
"""

#TODO Add lists in the MockCmd to implement VectorMsgs


def QRAlertsVectorMsgFactory(data):
    """ QRAlertsVectorMsg

    Header header
    QRAlertMsg[] qrAlerts

    """
    msg = QRAlertsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.qrAlerts = []
    msg.qrAlerts.append(QRAlertMsgFactory(data))

    return msg

def QRAlertMsgFactory(data):
    """ QRAlertMsg

    float32 yaw
    float32 pitch
    string QRcontent

    """
    msg = QRAlertMsg()

    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.QRcontent = data.QRcontent

    return msg

def LandoltcAlertMsgFactory(data):
    """ LandoltcAlertMsg

    float32 yaw
    float32 pitch
    float32[] angles
    float32 posterior

    """
    msg = LandoltcAlertMsg()

    msg.angles = data.angles
    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.posterior = data.posterior

    return msg

def LandoltcAlertsVectorMsgFactory(data):
    """ LandoltcAlertsVectorMsg

    Header header
    LandoltcAlertMsg[] landoltcAlerts

    """
    msg = LandoltcAlertsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.landoltcAlerts = []
    msg.landoltcAlerts.append(LandoltcAlertMsgFactory(data))

    return msg

def LandoltcPredatorMsgFactory(data):
    """ LandoltcPredatorMsg

    Header header
    int32 x
    int32 y
    int32 width
    int32 height
    float32 posterior
    Image img

    """
    msg = LandoltcAlertMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.x = data.x
    msg.y = data.y
    msg.width = data.width
    msg.height = data.height
    msg.posterior = data.posterior
    msg.img = ImageFactory(data.img_path)

    return msg

def HoleDirectionMsgFactory(data):
    """ HoleDirectionMsg

    float32 yaw
    float32 pitch
    float32 probability
    uint64  holeId

    """
    msg = HoleDirectionMsg()

    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.probability = data.probability
    msg.holeId = data.holeId

    return msg

def HolesDirectionsVectorMsgFactory(data):
    """ HolesDirectionsVectorMsg

    Header header
    HoleDirectionMsg[] holesDirections

    """
    if data.debug:
        rospy.loginfo('Creating HolesDirectionsVectorMsg.')
    msg = HolesDirectionsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.holesDirections = []
    msg.holesDirections.append(HoleDirectionMsgFactory(data))

    return msg

def HazmatAlertMsgFactory(data):
    """ HazmatAlertMsg

    float32 yaw
    float32 pitch
    uint8 patternType

    """
    msg = HazmatAlertMsg()

    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.patternType = data.patternType

    return msg

def HazmatAlertsVectorMsgFactory(data):
    """ HazmatAlertsVectorMsg

    Header header
    HazmatAlertMsg[] landoltcAlerts

    """
    msg = HazmatAlertsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.hazmatAlerts = []
    msg.hazmatAlerts.append(HazmatAlertMsgFactory(data))

    return msg

def EnhancedHoleMsgFactory(data):
    """ EnhancedHoleMsg

    Header header
    float32 keypointX
    float32 keypointY
    float32[] verticesX
    float32[] verticesY

    """
    msg = EnhancedHoleMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.keypointX = data.keypointX
    msg.keypointY = data.keypointY
    msg.verticesX = data.verticesX
    msg.verticesY = data.verticesY

    return msg

def EnhancedHolesVectorMsgFactory(data):
    """ LandoltcAlertsVectorMsg

    Header header
    LandoltcAlertMsg[] landoltcAlerts

    """
    msg = LandoltcAlertsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.landoltcAlerts = []
    msg.landoltcAlerts.append(LandoltcAlertMsgFactory(data))

    return msg

def DataMatrixAlertMsgFactory(data):
    """ DataMatrixAlertMsg

    float32 yaw
    float32 pitch
    string datamatrixContent

    """
    msg = DataMatrixAlertMsg()

    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.datamatrixContent = data.datamatrixContent

    return msg

def DataMatrixAlertsVectorMsgFactory(data):
    """ DataMatrixAlertsVectorMsg

    Header header
    DataMatrixAlertMsg[] dataMatrixAlerts

    """
    msg = DataMatrixAlertsVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.dataMatrixAlerts = []
    msg.dataMatrixAlerts.append(DataMatrixAlertMsgFactory(data))

    return msg

def CandidateHoleMsgFactory(data):
    """ CandidateHoleMsg

    Header header
    float32 keypointX
    float32 keypointY
    float32[] verticesX
    float32[] verticesY
    float32[] outlineX
    float32[] outlineY

    """
    msg = CandidateHoleMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.keypointX = data.keypointX
    msg.keypointY = data.keypointY
    msg.verticesX = data.verticesX
    msg.verticesY = data.verticesY
    msg.outlineX = data.outlineX
    msg.outlineY = data.outlineY

    return msg

def CandidateHolesVectorMsgFactory(data):
    """ CandidateHolesAlertsVectorMsg

    Header header
    CandidateHoleAlertMsg[] candidateHoles
    Image image

    """
    msg = CandidateHolesVectorMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.candidateHoles = []
    msg.candidateHoles.append(CandidateHoleMsgFactory(data))
    msg.image = ImageFactory(data.img_path)

    return msg

def HolePositionMsgFactory(data):
    """ HolePositionMsg

    Pose holePose
    float32 probability
    uint64 holeId

    """
    msg = HolePositionMsg()

    msg.holePose = PoseFactory(data)
    msg.probability = data.probability
    msg.holeId = data.holeId

    return msg

def HolesPositionsVectorMsgFactory(data):
    """ HolePositionsVectorMsg

    Header header
    HolePositionMsg[] holePositions

    """
    msg = HolePositionMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.holePositions = []
    msg.holePositions.append(HolePositionMsgFactory(data))

    return msg

def FaceDirectionMsgFactory(data):
    """ FaceDirectionMsg

    Header header
    float32 yaw
    float32 pitch
    float32 probability

    """
    msg = FaceDirectionMsg()

    msg.header = Header()
    msg.header.stamp = rospy.Time.now()

    msg.yaw = data.yaw
    msg.pitch = data.pitch
    msg.probability = data.probability

    return msg

"""
    Data Fusion Message
"""

def VictimInfoMsgFactory(data):
    """ VictimInfoMsg

    int32 id
    string victimFrameId
    PoseStamped victimPose
    float32 probability
    string[] sensors
    bool valid

    """
    msg = VictimInfoMsg()

    msg.id = data.id
    msg.victimFrameId = data.victimFrameId
    msg.victimPose = PoseStampedFactory(data)
    msg.probability = data.probability
    msg.sensors = data.sensors
    msg.valid = data.valid

    return msg

def WorldModelMsgFactory(data):
    """ WorldModelMsg

    VictimInfoMsg[] victims
    VictimInfoMsg[] visitedVictims

    """
    pass
