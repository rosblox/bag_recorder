import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32

import importlib


import rosbag2_py
from datetime import datetime
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class Topic():
    def __init__(self, name: str, type_str: str, type):
        self.name = name
        self.type_str = type_str
        self.type = type


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        self.is_recording = False
        self.bag = None

        # self.srv = self.create_service(Trigger, '~/trigger', self.callback)

        self.get_logger().info('Bag recorder initialized')




        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.start()


    # def callback(self, request: Trigger.Request, response: Trigger.Response):
    #     if not self.is_recording:
    #         self.start()
    #         response.message = f"Recording to {self.bag}"
    #     else:
    #         self.stop()
    #         response.message = f"Finished recording to {self.bag}"
    #         self.get_logger().info('Stopped recording bag.')

    #     response.success=True
    #     return response


    def record_message(self, name, msg):
        if self.is_recording:
            self.writer.write(name, serialize_message(msg), self.get_clock().now().nanoseconds)


    def start(self):
        now = datetime.now()
        self.bag = f"bag_{now.strftime('%Y_%m_%d_%H_%M_%S')}"
        uri = f"/bags/{self.bag}"
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri=uri,storage_id='sqlite3',max_bagfile_size=300000000)
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)


        topic_list = self.get_topic_names_and_types()

        def create_topic_subscribers(topic):
            # self.get_logger().info(f"{topic[0]}")

            module, message = topic[1][0].replace("/",".").rsplit('.', 1)
            module = importlib.import_module(module)
            message_class = getattr(module, message)

            topic = Topic(topic[0],topic[1][0],message_class)

            included_topics = [
            "/zed2i/zed_node/right_raw/image_raw_color/compressed",
            "/zed2i/zed_node/left_raw/image_raw_color/compressed",
            "/zed2i/zed_node/odom",
            
            "/vortex/set_propeller_setpoint",
            "/vortex/present_tether_length",
            "/pixhawk/setpoint",

            "/dynamixel/set_velocity",
            "/dynamixel/present_position",

            "/fmu/out/SensorCombined",
            "/fmu/out/VehicleOdometry",
            "/fmu/in/ActuatorMotors",

            "/livox/lidar",

            ]


            if  topic.name in included_topics:

                self.get_logger().info(f"{topic.name}, {topic.type_str}, {topic.name}")
                topic_callback = lambda msg : self.record_message(name=topic.name, msg=msg) 
                topic_subscription = self.create_subscription(topic.type, topic.name, topic_callback,qos_profile=self.qos_profile)

                topic_info = rosbag2_py._storage.TopicMetadata(name=topic.name, type=topic.type_str, serialization_format='cdr')
                self.writer.create_topic(topic_info)

                return topic_subscription
            else:
                
                return

        self.topic_subscribers = list(map(create_topic_subscribers, topic_list))

        self.is_recording = True


    def stop(self):
        del self.writer
        del self.topic_subscribers

        self.is_recording = False


def main(args=None):
    rclpy.init(args=args)
    bag_recorder = BagRecorder()
    rclpy.spin(bag_recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()