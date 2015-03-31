# Configuration message used by the Coordinator object
# to communicate with the Mock object.

# Specifies the task to be executed by the Mock.
string  cmd

# An absolute path on the filesystem. The Mock will use this
# directory
string  directory

# In this topic the Mock will publish its data.
string  input_topic

# A time limit to execute the task. The task should stop
# after the timeout.
int32   timeout

# If set to True enables log info.
bool    debug

# Defines the type of message or alert to be send by the Mock.
string  alert_type

# The main data of the message.
string  alert_payload

# Yaw position to be included in the alert or message.
float32 yaw

# Pitch position to be included in the alert or message.
float32 pitch

# Roll position to be included in the alert or message.
float32 roll

# How long the Mock will publish an alert.
int32   alert_duration

# Defines the flow of the alerts.
# e.g following a specific distribution.
string  alert_sender

# Introduces a delay between alerts.
float32   alert_delay

# How many times an alert will be sent.
int32   alert_repetitions

# x coordinate
int32   x

# y coordinate
int32   y

# z coordinate
int32   z

# w coordinate
int32   w

# Posterior probability
float32 posterior

# Probability
float32 probability

# Hole ID
uint64 holeId

# LandoltC angles
float32[] angles

# Height
int32 height

# Width
int32 width

# Image path
string img_path

# Pattern type for Hazmat
uint8 patternType

# Enhanced hole attributes
# Keypoint.pt.x
float32 keypointX

# Keypoint.pt.y
float32 keypointY

# The blob's bounding rectangle vertices X coordinate
float32[] verticesX

# The blob's bounding rectangle vertices Y coordinate
float32[] verticesY

# QR content
string QRcontent

# Datamatrix content
string datamatrixContent

# Blob's outline points X coordinate
float32[] outlineX

# Blob's outline points Y coordinate
float32[] outlineY
