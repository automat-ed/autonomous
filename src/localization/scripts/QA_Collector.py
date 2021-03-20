import rospy
from nav_msgs.msg import Odometry
import QA_Plotter


class Collector():

  def __init__(self):
    # Position data
    self.gps_gt         = []
    self.gps_ekf_local  = []
    self.gps_ekf_global = []

    # I/O
    self.gps_gt_file = 'GPS_gt.txt'
    self.gps_local_file = 'GPS_local.txt'
    self.gps_global_file = 'GPS_global.txt'

    rospy.init_node('QA_plotter')

    # Subscribers
    gps_gt_sub = rospy.Subscriber('/odometry/gps', Odometry, self.store_gps, 'gt')
    gps_ekf_local_sub = rospy.Subscriber('odometry/filtered/local', Odometry, self.store_gps, 'ekf_local')
    gps_ekf_local_sub = rospy.Subscriber('odometry/filtered/global', Odometry, self.store_gps, 'ekf_global')



  # Save data to file
  def store_gps(self, msg, data_type):
    if data_type == 'gt':
      x = msg.pose.pose.position.x
      y = msg.pose.pose.position.y
      self.gps_gt.append((x, y))
    elif (data_type == 'ekf_local' or data_type == 'ekf_global'):
      pass




  def write_to_file(self):
    # Ground Truth Odometry
    with open(self.gps_gt_file, 'w') as f:
      for (x, y) in self.gps_gt:
        f.write("{} {}\n".format(x, y))
      
    # # EKF local
    # with open(self.gps_local_file, 'w') as f:
    #   for (lat, lng) in self.gps_ekf_local:
    #     f.write(lat, lng)

    # # EKF global
    # with open(self.gps_global_file, 'w') as f:
    #   for (lat, lng) in self.gps_ekf_global:
    #     f.write(lat, lng)
    

  # def position_plot(self):
  #   # Ground Truth
  #   xs = [lng for (lat, lng) in self.gps_gt]
  #   ys = [lat for (lat, lng) in self.gps_gt]
  #   plt.plot(xs, ys, 'b')
    

    # plt.savefig('GPS_QA')









def main():
  collector = Collector()

  rospy.sleep(150)
  print("Data has been collected")
  collector.write_to_file()
  QA_Plotter.plot()

  




if __name__ == "__main__":
  main()