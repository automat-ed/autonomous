import rospy
import tf


class Tf_broadcaster():

    def __init__(self, device_name):
        rospy.init_node('tf_broadcaster_{}'.format(device_name))
        self.rate = rospy.Rate(100)
        self.broadcaster = tf.TransformBroadcaster()

    def broadcast(self):
        while not rospy.is_shutdown():
            self.broadcaster.sendTransform(tf.StampedTransform(tf.Transform(tf.Quaternion(0, 1, 0, 0),
                                                                            tf.Vector3(0, 0.2, 0),
                                                                            rospy.get_rostime(), 
                                                                            'base_link',
                                                                            'lidar_frame')))
            self.rate.sleep()





def main():
    tf_broadcaster_lidar = Tf_broadcaster('lidar')
    tf_broadcaster_lidar.broadcast()





if __name__ == '__main__':
    main()




