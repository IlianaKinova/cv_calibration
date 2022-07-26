import enum
from typing import Tuple
import rospy as ros
from sensor_msgs.msg import CameraInfo
import roslib

try:
    roslib.load_manifest('cv_calibration')
    from kortex_driver.msg import *
    from kortex_driver.srv import *
except Exception as e:
    print(f'Failed to load ros modules:{e}')

"""
Extrinsic parameters are :
Rotation parameters matrix is : 
|  1.0  ;  0.0  ;  0.0  |
|  0.0  ;  1.0  ;  0.0  |
|  0.0  ;  0.0  ;  1.0  |
Translation parameters are : [ x = -0.027060000225901604 ; y = -0.009970000013709068 ; z = -0.004705999977886677 ]
---------------------------------

"""
class visionOption(enum.Enum):

    OPTION_UNSPECIFIED = 0
    OPTION_BACKLIGHT_COMPENSATION = 1
    OPTION_BRIGHTNESS = 2
    OPTION_CONTRAST = 3
    OPTION_EXPOSURE = 4
    OPTION_GAIN = 5
    OPTION_GAMMA = 6
    OPTION_HUE = 7
    OPTION_SATURATION = 8
    OPTION_SHARPNESS = 9
    OPTION_WHITE_BALANCE = 10
    OPTION_ENABLE_AUTO_EXPOSURE = 11
    OPTION_ENABLE_AUTO_WHITE_BALANCE = 12
    OPTION_VISUAL_PRESET = 13
    OPTION_LASER_POWER = 14
    OPTION_ACCURACY = 15
    OPTION_MOTION_RANGE = 16
    OPTION_FILTER_OPTION = 17
    OPTION_CONFIDENCE_THRESHOLD = 18
    OPTION_EMITTER_ENABLED = 19
    OPTION_FRAMES_QUEUE_SIZE = 20
    OPTION_TOTAL_FRAME_DROPS = 21
    OPTION_AUTO_EXPOSURE_MODE = 22
    OPTION_POWER_LINE_FREQUENCY = 23
    OPTION_ASIC_TEMPERATURE = 24
    OPTION_ERROR_POLLING_ENABLED = 25
    OPTION_PROJECTOR_TEMPERATURE = 26
    OPTION_OUTPUT_TRIGGER_ENABLED = 27
    OPTION_MOTION_MODULE_TEMPERATURE = 28
    OPTION_DEPTH_UNITS = 29
    OPTION_ENABLE_MOTION_CORRECTION = 30
    OPTION_AUTO_EXPOSURE_PRIORITY = 31
    OPTION_COLOR_SCHEME = 32
    OPTION_HISTOGRAM_EQUALIZATION_ENABLED = 33
    OPTION_MIN_DISTANCE = 34
    OPTION_MAX_DISTANCE = 35
    OPTION_TEXTURE_SOURCE = 36
    OPTION_FILTER_MAGNITUDE = 37
    OPTION_FILTER_SMOOTH_ALPHA = 38
    OPTION_FILTER_SMOOTH_DELTA = 39
    OPTION_HOLES_FILL = 40
    OPTION_AUTO_EXPOSURE_CONVERGE_STEP = 42
    OPTION_STEREO_BASELINE = 41

class RobotController:
    def __init__(self, name:str='my_gen3'):
        self.name = name
        
        self.colorInfo = None
        self.depthInfo = None

    def getExtrinsic(self):
        

        try:
            res = self.get_extrinsic_parameters()
        except ros.ServiceException:
            ros.logerr("Failed to call GetExtrinsicParameters")
            return False
        else:
            # Print the result
            # The message description can be seen at msg/generated/vision_config/ExtrinsicParameters.msg
            extrinsic_parameters = res.output
            self.extrinsic_parameters = extrinsic_parameters
            out = "\n----------------------------------\n"
            out += "Extrinsic parameters are :\n"
            out += "Rotation parameters matrix is : " + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row1.column1) + "  ;  " + str(extrinsic_parameters.rotation.row1.column2) + "  ;  " + str(extrinsic_parameters.rotation.row1.column3) + "  |" + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row2.column1) + "  ;  " + str(extrinsic_parameters.rotation.row2.column2) + "  ;  " + str(extrinsic_parameters.rotation.row2.column3) + "  |" + "\n"
            out += "|  " + str(extrinsic_parameters.rotation.row3.column1) + "  ;  " + str(extrinsic_parameters.rotation.row3.column2) + "  ;  " + str(extrinsic_parameters.rotation.row3.column3) + "  |" + "\n"

            out += "Translation parameters are : "
            out += "[ x = " + str(extrinsic_parameters.translation.t_x)
            out += " ; y = " + str(extrinsic_parameters.translation.t_y)
            out += " ; z = " + str(extrinsic_parameters.translation.t_z) + " ]"
            out += "\n" + "---------------------------------" + "\n"



            ros.loginfo(out)
            print(out)
            return True
    
    def init(self):
        try:
            get_extrinsic_parameters_full_name = '/' + self.name + '/vision_config/get_extrinsic_parameters'
            ros.wait_for_service(get_extrinsic_parameters_full_name, 0.5)
            self.get_extrinsic_parameters = ros.ServiceProxy(get_extrinsic_parameters_full_name, GetExtrinsicParameters)

            set_extrinsic_parameters_full_name = '/' + self.name + '/vision_config/set_extrinsic_parameters'
            ros.wait_for_service(set_extrinsic_parameters_full_name, 0.5)
            self.set_extrinsic_parameters = ros.ServiceProxy(set_extrinsic_parameters_full_name, SetExtrinsicParameters)

            set_extrinsic_parameters_full_name = '/' + self.name + '/vision_config/set_extrinsic_parameters'
            ros.wait_for_service(set_extrinsic_parameters_full_name, 0.5)
            self.set_extrinsic_parameters = ros.ServiceProxy(set_extrinsic_parameters_full_name, SetExtrinsicParameters)

            get_option_value_full_name = '/' + self.name + '/vision_config/get_option_value'
            ros.wait_for_service(get_option_value_full_name, 0.5)
            self.get_option_value = ros.ServiceProxy(get_option_value_full_name, GetOptionValue)
        except ros.ROSException as e:
            ros.logerr(f'Timed out:{e}')

    def updateCameraInfo(self):
        try:
            self.colorInfo = ros.wait_for_message('/camera/color/camera_info', CameraInfo)
            self.depthInfo = ros.wait_for_message('/camera/depth/camera_info', CameraInfo)
        except ros.ROSException as e:
            ros.logerr(f'Update camrea info timed out: {e}')
        print('Done updating')

    def setTranslation(self, pos:Tuple[float,float]):
        if not self.getExtrinsic():
            return False
        self.extrinsic_parameters.translation.t_x = pos[0]
        self.extrinsic_parameters.translation.t_y = pos[1]
        try:
            req = SetExtrinsicParameters()
            ros.loginfo(f'Req:{self.set_extrinsic_parameters.__dict__}')
            req.input = self.extrinsic_parameters
            ros.loginfo(f'Req:{req.__dict__}')
            self.set_extrinsic_parameters(req)
        except ros.ROSException as e:
            print(f'Timed out:{e}')
            return False
        else:
            return True

    def exploreVisionConfig(self):
        try:
            for sensor in [0,1,2]:
                req = GetOptionValue()
                req.input = 1
                ros.loginfo(f'Sensor {sensor}')
                for i in range(0, 43):
                    try:
                        req = GetOptionValueRequest()
                        req.input.sensor = sensor
                        req.input.option = i
                        res = self.get_option_value(req)
                    except ros.ServiceException:
                        ros.logerr(f"Failed to call GetOptionValue. Option:{i}")
                    else:
                        option_value = res.output
                        out = f"Option:{i}\tValue:{option_value.value}"
                        ros.loginfo(out)

        except ros.ROSException as e:
            print(f'Timed out:{e}')
            return False
        else:
            return True