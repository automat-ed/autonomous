import matplotlib.pyplot as plt

# File names
gps_gt = "GPS_gt.txt"
gps_ekf_local = "GPS_local.txt"
gps_ekf_global = "GPS_global.txt"

def plot():
  # Position (x, y)
  # Ground truth
  xs = []
  ys = []

  with open(gps_gt, 'r') as f:
    for line in f.readlines():
      coordinate = line.split()
      xs.append(float(coordinate[0]))
      ys.append(float(coordinate[1]))

  
  plt.plot(xs, ys, 'g', label="ground truth")
  plt.xlabel("X (m)")
  plt.ylabel("Y (m)")
  plt.title("Position")
  plt.grid()
  plt.legend()


  # # EKF local
  # xs = []
  # ys = []

  # with open(gps_ekf_local, 'r') as f:
  #   for line in f.readlines():
  #     coordinate = line.split()
  #     xs.append(float(coordinate[0]))
  #     ys.append(float(coordinate[1]))

  # plt.plot(xs, ys, 'b')

  # # EKF global
  # xs = []
  # ys = []

  # with open(gps_ekf_global, 'r') as f:
  #   for line in f.readlines():
  #     coordinate = line.split()
  #     xs.append(float(coordinate[0]))
  #     ys.append(float(coordinate[1]))

  # plt.plot(xs, ys, 'r')


  plt.savefig("GPS_QA")


def main():
  plot()


if __name__ == '__main__':
  main()