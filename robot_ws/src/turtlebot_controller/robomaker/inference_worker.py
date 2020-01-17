import sys

import numpy as np
import tensorflow as tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

import boto3

IMAGE_SIZE = 360                # Use Lidar scanner which look around 360 degree
LIDAR_SCAN_MAX_DISTANCE = 5.0   # Max distance Lidar scanner can measure
CRASH_DISTANCE = 0.13           # Min distance to obstacle

class InferenceWorker:
    def __init__(self, model_path):
        self.model_path = model_path
        self.aws_region = rospy.get_param('ROS_AWS_REGION')
        self.bucket_name = rospy.get_param("MODEL_S3_BUCKET")
        self.model_key_prefix = rospy.get_param("MODEL_S3_PREFIX")
        self.model_file = rospy.get_param("MODEL_FILE")
        
    def run(self):
        self.graph = self.load_graph()
        self.session = tf.Session(graph=self.graph, config=tf.ConfigProto(allow_soft_placement=True, log_device_placement=True))

        print('INFO: Creating publisher on /cmd_vel')
        self.ack_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=100)

        print('INFO: Creating subscriber on /scan')
        rospy.Subscriber('/scan', LaserScan, self.callback_scan)

        print('INFO: Finished initialization')

    def load_graph(self):
        print('Loading graph...')
        
        print("bucker name:%s" % self.bucket_name)
        print("Key name:%s" % self.model_key_prefix)
        print("Filename: %s" % self.model_path)
        
        model_key = "{}/{}".format(self.model_key_prefix,self.model_file)
        
        s3 = boto3.resource('s3', region_name=self.aws_region)
        s3.meta.client.download_file(Bucket=self.bucket_name, Key=model_key, Filename=self.model_path)
        
        with tf.gfile.GFile(self.model_path, "rb") as f:
            graph_def = tf.GraphDef()
            graph_def.ParseFromString(f.read())

        with tf.Graph().as_default() as graph:
            tf.import_graph_def(graph_def, name="turtlebot")

        print('INFO: Finished loading graph')

        return graph

    def callback_scan(self, data):
    
        if not data:
            return
        
        self.ranges = data.ranges

        size = len(self.ranges)
        x = np.linspace(0, size-1, IMAGE_SIZE)
        xp = np.arange(size)
        state = np.clip(np.interp(x, xp, self.ranges), 0, LIDAR_SCAN_MAX_DISTANCE)
        state[np.isnan(state)] = LIDAR_SCAN_MAX_DISTANCE

        state = np.expand_dims(state, axis=0)
        x = self.graph.get_tensor_by_name('turtlebot/main_level/agent/main/online/network_0/observation/observation:0')
        y = self.graph.get_tensor_by_name('turtlebot/main_level/agent/main/online/network_1/ppo_head_0/policy:0')
        inferenceOutput = np.argmax(self.session.run(y, feed_dict={
            x: state
        }))

        self.takeAction(inferenceOutput)

    def takeAction(self, action):
        # Convert discrete to continuous
        if action == 0:  # turn left
            steering = 1.159
            throttle = 0.08
        elif action == 1:  # turn right
            steering = -1.159
            throttle = 0.08
        elif action == 2:  # straight
            steering = 0
            throttle = 0.2
        elif action == 3:  # steer to the left
            steering = 0.6
            throttle = 0.09
        elif action == 4:  # steer to the right
            steering = -0.6
            throttle = 0.09
        else:  # should not be here
            raise ValueError("Invalid action")
            
        speed = Twist()
        speed.linear.x = throttle
        speed.angular.z = steering
        self.ack_publisher.publish(speed)

if __name__ == '__main__':
    model_path = sys.argv[1]
    print('Starting Inference Worker, Specified Model Directory: ', model_path)

    rospy.init_node('rl_coach', anonymous=True)
    inference_worker = InferenceWorker(model_path)
    inference_worker.run()
    rospy.spin()

