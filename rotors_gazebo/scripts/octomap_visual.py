import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from octomap_msgs.msg import Octomap
import numpy as np
from io import BytesIO
import octomap
import struct

def octomap_callback(msg):
    # Chuyển đổi thông điệp Octomap thành OcTree
    print("Received Octomap data")

    # Đọc dữ liệu binary từ msg.data
    binary_data = msg.data
    octree = octomap.OcTree()
    
    # Giả sử rằng bạn có thể đọc dữ liệu binary từ msg.data. Điều này phụ thuộc vào cách dữ liệu được gửi và nhận
    # Bạn có thể cần xử lý msg.data để phù hợp với thư viện OctoMap của bạn

    # Khởi tạo matplotlib 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Duyệt qua các nút trong Octree và hiển thị các ô bị chiếm
    for node in octree:
        if octree.isNodeOccupied(node):
            x, y, z = node.getX(), node.getY(), node.getZ()
            ax.scatter(x, y, z, c='red', marker='s')

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    plt.show()

def main():
    rospy.init_node('octomap_visualization_node')
    rospy.Subscriber("/red/octomap_binary", Octomap, octomap_callback)
    rospy.spin()

if __name__ == "__main__":
    print("Starting ROS node")
    main()
