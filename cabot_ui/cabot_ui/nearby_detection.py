import rclpy
from rclpy.node import Node
from ios_msgs.msg import NINearbyObjects
from cabot_ui.event import NavigationEvent
from std_msgs.msg import String 
from people_msgs.msg import People
import rclpy.logging

class NearbyDetection:
    def __init__(self, node: Node):
        self.node = node
        self.node.create_subscription(NINearbyObjects, '/iphone/nearby_objects', self.nearby_objects_callback, 10)
        self.node.create_subscription(People, '/people', self.people_callback, 10)

        self.tour_pub = self.node.create_publisher(String, '/group_tour', 10)
        self.guide_description = ""
        self.people_description = ""
        self.__logger = rclpy.logging.get_logger('cabot_ui_manager.nearby_detection')

    def nearby_objects_callback(self, msg):
        self.__logger.info(f"Received nearby objects: {msg}")
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

        distance = msg.objects[0].distance
        direction = msg.objects[0].direction
        
        if (distance > 5):
            self.tour_pub.publish("Away")
        
        # not enbale "camera assistance" on the ios app as it's not for moving objects
        orientation_description= "and is not facing you in the front. "
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
            orientation_description = f"facing you at {clock} o'clock."

        self.guide_description = f"The tour guide is {distance:.2f} meters away, {orientation_description}."
        self.__logger.info(f"guide description: {self.guide_description}")

    def people_callback(self, msg):
        self.__logger.info(f"Received people data: {msg}")
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

        if len(msg.people) == 1:
            person = msg.people[0]
            distance = self.get_distance(person)
            self.people_description = f"One person is {distance:.2f} meters away from the AI suitcase."
            
        elif (len(msg.people) > 1):
            self.people_description = f"There are {len(msg.people)} people nearby. Their distances to the AI suitcase are "
            for person in msg.people:
                distance = self.get_distance(person)
                self.people_description += f"{distance:.2f} meters. "
        self.__logger.info(f"people description: {self.people_description}")

    def get_distance(self, point):
        return ((point.position.x)**2 + (point.position.y)**2)**0.5 * 0.01