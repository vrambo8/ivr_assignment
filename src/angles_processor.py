from . import image1, image2

class Server:
    def __init__(self):
        self.angles1 = None
        self.angles2 = None

    def image1_callback(self, msg):
        self.angles1 = msg
        self.compute_average()

    def image2_callback(self, msg):
        self.angles2 = msg
        self.compute_average()

    def compute_average(self):
        if self.angles1 is not None and self.angles2 is not None:
            print("Angles1: ", self.angles1)
            print("Angles2: ", self.angles2)

if __name__ == "__main__":
    server = Server()

    rospy.init_node('angles')
    rospy.Subscriber("/joints_pos1", Float64MultiArray, server.image1_callback())
    rospy.Subscriber("/joints_pos2", Float64MultiArray, server.image2_callback())