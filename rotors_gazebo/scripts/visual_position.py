import rospy
import matplotlib.pyplot as plt 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import os 
import yaml

dataSave_dir = '/home/manh/exploration/src/hummingbird_simulator/rotors_gazebo/scripts/position.txt'

class SaveData():
    def __init__ (self):
        self.data_dir = dataSave_dir
        if os.path.exists(self.data_dir):
            os.remove(self.data_dir)
        
        self.gt_pose = rospy.Subscriber("/hummingbird/ground_truth/pose", Pose, self.gtPose_callback)
        
        self.ss_pose = rospy.Subscriber("/hummingbird/odometry_sensor1/pose", Pose, self.ssPose_callback)
        
        self.gtPose = Pose()
        self.ssPose = Pose()

        self.gtPose_x = []
        self.gtPose_y = []
        self.gtPose_z = []

        self.ssPose_x = []
        self.ssPose_y = []
        self.ssPose_z = []

        rospy.on_shutdown(self.plotting)

    def gtPose_callback(self, msg):
        self.gtPose = msg
    
    def ssPose_callback(self, msg):
        self.ssPose = msg
    
    def write_data (self):
        a = open(self.data_dir, 'a')

        a.write (
                 str(self.gtPose.position.x) + " " + str(self.gtPose.position.y) + " " + str(self.gtPose.position.z) + " " +
                 str(self.ssPose.position.x) + " " + str(self.ssPose.position.y) + " " + str(self.ssPose.position.z) + "\n")

        self.gtPose_x.append(self.gtPose.position.x)
        self.gtPose_y.append(self.gtPose.position.y)
        self.gtPose_z.append(self.gtPose.position.z)

        self.ssPose_x.append(self.ssPose.position.x)
        self.ssPose_y.append(self.ssPose.position.y)
        self.ssPose_z.append(self.ssPose.position.z)

        a.close()
    
    def plotting (self):
        start = 5

        plt.figure(1)
        plt.plot(self.gtPose_x[start:len(self.gtPose_x)], self.gtPose_y[start:len(self.gtPose_y)], color='black', label='gt', linewidth = 1.0)
        plt.plot(self.ssPose_x[start:len(self.ssPose_x)], self.ssPose_y[start:len(self.ssPose_y)], color='red', label='odom', linewidth = 1.0, linestyle = "--")
        plt.legend()
        plt.grid(True)
        plt.title("Plane xOy")
        plt.xlabel("x [m]")
        plt.ylabel('y [m]')
        plt.gca().set_aspect('equal', adjustable='box')

        plt.figure(2)
        plt.plot(self.gtPose_x[start:len(self.gtPose_x)], self.gtPose_z[start:len(self.gtPose_z)], color='black', label='gt', linewidth = 1.0)
        plt.plot(self.ssPose_x[start:len(self.ssPose_x)], self.ssPose_z[start:len(self.ssPose_z)], color='red', label='odom', linewidth = 1.0, linestyle = "--")
        plt.legend()
        plt.grid(True)
        plt.title("Plane xOz")
        plt.xlabel("x [m]")
        plt.ylabel('z [m]')
        plt.gca().set_aspect('equal', adjustable='box')

        plt.figure(3)
        plt.plot(self.gtPose_y[start:len(self.gtPose_y)], self.gtPose_z[start:len(self.gtPose_z)], color='black', label='gt', linewidth = 1.0)
        plt.plot(self.ssPose_y[start:len(self.ssPose_y)], self.ssPose_z[start:len(self.ssPose_z)], color='red', label='odom', linewidth = 1.0, linestyle = "--")
        plt.legend()
        plt.grid(True)
        plt.title("Plane yOz")
        plt.xlabel("y [m]")
        plt.ylabel('z [m]')
        plt.gca().set_aspect('equal', adjustable='box')

        plt.show()

if __name__ == "__main__":
    rospy.init_node('save_data',anonymous=False)
    rate = 10 
    r = rospy.Rate(rate)
    rospy.loginfo("\033[92mInit Node Saving and Ploting...\033[0m")

    m = SaveData()
    while not rospy.is_shutdown():
        m.write_data()
        r.sleep()
