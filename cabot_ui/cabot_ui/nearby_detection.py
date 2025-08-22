import rclpy
from rclpy.node import Node
from ios_msgs.msg import NINearbyObjects
from cabot_ui.event import NavigationEvent
from std_msgs.msg import String 
from people_msgs.msg import People
import rclpy.logging
from cabot_msgs.msg import PoseLog
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from datetime import datetime

class NearbyDetection:
    def __init__(self, node: Node):
        self.node = node
        self.node.create_subscription(NINearbyObjects, '/iphone/nearby_objects', self.nearby_objects_callback, 10)
        self.node.create_subscription(People, '/people', self.people_callback, 10)
        self.node.create_subscription(PoseLog, '/cabot/pose_log', self.cabot_pose_callback, 10)
        self.cabot_pose = Pose()
        self.tour_pub = self.node.create_publisher(String, '/group_tour', 10)
        self.__logger = rclpy.logging.get_logger('cabot_ui_manager.nearby_detection')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        self.nearby_objects = []
        self.nearby_people = None
        self.last_update_people = datetime.now()
        self.last_update_guide = datetime.now()

    def nearby_objects_callback(self, msg):
        
        # objects:
        #     - distance: 0.06805288791656494
        #       direction:
        #         x: 0.0
        #         y: 0.0
        #         z: 0.0
        #       horizontal_angle: 0.0
        #       vertical_direction_estimate: 0
        #       identifier: ''
        
        if (len(msg.objects) == 0):
            return

        self.__logger.info(f"Received nearby objects: {msg}")

        self.last_update_guide = datetime.now()
        self.nearby_objects = msg.objects
        distance = msg.objects[0].distance

        ## send notification if the guide is far away
        if (distance > 5):
            guide_event = String()
            guide_event.data = "Away"
            self.tour_pub.publish(guide_event)

    def people_callback(self, msg):
        
        # header:
        #   stamp:
        #     sec: 1755160665
        #     nanosec: 41458949
        #   frame_id: map
        # people:
        # - name: '21'
        #   position:
        #     x: -11.640559988860902
        #     y: -10.888054171577481
        #     z: 0.0
        #   velocity:
        #     x: -0.5231126048226792
        #     y: 1.0738667565515994
        #     z: 0.0
        #   reliability: 1.0
        #   tagnames: []
        #   tags: []

        if (len(msg.people) > 0):
            self.nearby_people = msg.people
            # self.__logger.info(f"Received people data: {msg}")
            self.last_update_people = datetime.now()
        

    def get_distance(self, point1, point2):
        return ((point1.position.x - point2.position.x)**2 + (point1.position.y - point2.position.y)**2)**0.5 


    def cabot_pose_callback(self, msg):
        # self.__logger.info(f"Received cabot pose data: {msg}")
        self.cabot_pose = msg.pose

        # stamp=builtin_interfaces.msg.Time(sec=1755498097, nanosec=127273477), frame_id='map_global'), 
        # pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=55.225614784089856, y=19.553391045153475, z=19.988103947949433), 
        # orientation=geometry_msgs.msg.Quaternion(x=0.02383309289202716, y=0.025674937416823317, z=0.47712539067725873, w=0.8781367449565673)), 
        # lat=35.6191932469664, lng=139.7764709316823, floor=5
      

    def describe_guide(self):
        time_diff_guide = (datetime.now() - self.last_update_guide).total_seconds()
        
        # time diff in seconds
        if (time_diff_guide > 3 or len(self.nearby_objects) == 0): 
            self.__logger.info(f"{time_diff_guide}, No tour guide is nearby or the data is outdated.")
            return "周囲にガイドはいません。" #No guide is detected.

        guide_description = ""

        #### Describe the guide
        distance = self.nearby_objects[0].distance
        direction = self.nearby_objects[0].direction
        
        # not enbale "camera assistance" on the ios app as it's not for moving objects
        orientation_description= "" #"not facing you."
        if (direction.z != 0):
            # the tour guide is facing the suitcase 
            # x: 1--> 90 deg; 
            clock = 0
            if direction.x < -0.83:
                clock = 9
            elif direction.x < -0.5: 
                clock = 10
            elif direction.x < -0.17:
                clock = 11
            elif direction.x < 0.17:
                clock = 12 
            elif direction.x < 0.5:
                clock = 1
            elif direction.x < 0.83:
                clock = 2
            else:
                clock = 3
            orientation_description = f"ガイドは{clock}時の方向" #f"{clock}時の方向でこちらを向いており" #f"facing you at {clock} o'clock."

        if orientation_description == "":
            guide_description = f"ガイドは{distance:.1f}メートル先。後ろを向いてます。"
        else:
            guide_description = f"{orientation_description}。{distance:.1f}メートル先こちらを向いてます。" # f"The tour guide is {orientation_description} and is {distance:.2f} meters away. "
        self.__logger.info(f"guide description: {guide_description}")
        return guide_description

    def describe_nearby_people(self):

        #### Describe the nearby people
        time_diff_people = (datetime.now() - self.last_update_people).total_seconds()
        if (time_diff_people > 3 or (self.nearby_people is None) or (len(self.nearby_people) == 0)):
            self.__logger.info(f"{time_diff_people}, No people are nearby or the data is outdated.")
            return "周囲に人はいません。" #"No people are nearby."
        
        people_description = "周囲に人はいません。"
        
        # TF conversion from "map_global" to "map" frame  
        try:
            transform = self.tf_buffer.lookup_transform('map', 'map_global', rclpy.time.Time())
            robot_pose = do_transform_pose(self.cabot_pose, transform)
        except Exception as e:
            self.__logger.error(f"Failed to transform cabot_pose: {e}")


        if len(self.nearby_people) == 1:
            person = self.nearby_people[0]
            distance = self.get_distance(person, robot_pose)
            people_description = f"人1人。距離{distance:.1f}メートル。" # f"One person is {distance:.2f} meters away from the AI suitcase."
            
        elif (len(self.nearby_people) > 1):
            people_description = f"人{len(self.nearby_people)}人。距離" # f"{len(self.nearby_people)} people are nearby. Their distances to the AI suitcase are "
            for person in self.nearby_people:
                distance = self.get_distance(person, robot_pose)
                people_description += f"{distance:.1f}。" # f"{distance:.2f} meters. "
            people_description += "メートル。"
        self.__logger.info(f"people description: {people_description}")

        return people_description
    
    
    def get_nearby_description(self):
        guide_desc = self.describe_guide()
        people_desc = self.describe_nearby_people()
        return guide_desc + people_desc
        