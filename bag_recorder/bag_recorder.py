import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_srvs.srv import Trigger
from std_msgs.msg import String, Int32

import rosbag2_py
from datetime import datetime
import time

class Topic():
    def __init__(self, name: str, type_str: str, type):
        self.name = name
        self.type_str = type_str
        self.type = type


class BagRecorder(Node):
    def __init__(self):
        super().__init__('bag_recorder')

        self.topics = []
        self.topics.append(Topic("test","std_msgs/msg/String", String))
        self.topics.append(Topic("test2","std_msgs/msg/Int32", Int32))

        self.is_recording = False
        self.bag = None

        for topic in self.topics:
            topic_callback = lambda msg : self.record_message(name=topic.name, msg=msg) 
            self.subscription = self.create_subscription(topic.type, topic.name, topic_callback, 10)

        self.srv = self.create_service(Trigger, '~/trigger', self.callback)

        self.get_logger().info('My log message 1')


    def callback(self, request: Trigger.Request, response: Trigger.Response):
        if not self.is_recording:
            self.start()
            response.message = f"Recording to {self.bag}"
        else:
            self.stop()
            response.message = f"Finished recording to {self.bag}"
        
        response.success=True
        return response


    def record_message(self, name, msg):
        if self.is_recording:
            self.writer.write(name, serialize_message(msg), self.get_clock().now().nanoseconds)


    def start(self):
        now = datetime.now()
        self.bag = f"bag_{now.strftime('%Y_%m_%d_%H_%M_%S')}"
        uri = f"/bags/{self.bag}"
        self.writer = rosbag2_py.SequentialWriter()
        storage_options = rosbag2_py._storage.StorageOptions(uri=uri,storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        for topic in self.topics:
            topic_info = rosbag2_py._storage.TopicMetadata(name=topic.name, type=topic.type_str, serialization_format='cdr')
            self.writer.create_topic(topic_info)

        self.is_recording = True


    def stop(self):
        del self.writer
        self.is_recording = False


def main(args=None):
    rclpy.init(args=args)
    bag_recorder = BagRecorder()
    rclpy.spin(bag_recorder)
    rclpy.shutdown()


if __name__ == '__main__':
    main()