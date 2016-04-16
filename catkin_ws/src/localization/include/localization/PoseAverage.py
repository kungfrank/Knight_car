from geometry_msgs.msg import Transform
import tf.transformations as tr
import math

class PoseAverage(object):


    def __init__(self):
        self.translation = [0.0, 0.0, 0.0]
        self.n = 0
        self.sum_sines = 0
        self.sum_cosines = 0

    def add_pose(self, transform_in):
        self.n += 1
        translation_in = transform_in.translation
        self.translation = [(self.translation[0]*(self.n - 1) + translation_in.x)/self.n,\
                            (self.translation[1]*(self.n - 1) + translation_in.y)/self.n,\
                            (self.translation[2]*(self.n - 1) + translation_in.z)/self.n]
        rotation_in = transform_in.rotation
        quaternion = (rotation_in.x, rotation_in.y, rotation_in.z, rotation_in.w)
        theta_in = tr.euler_from_quaternion(quaternion)[2]
        self.sum_sines += math.sin(theta_in)
        self.sum_cosines += math.cos(theta_in)


    def get_average(self):
        if self.n == 0:
            return None
        transform_out = Transform()
        theta_out = math.atan2(self.sum_sines, self.sum_cosines)
        trans = transform_out.translation
        rot = transform_out.rotation
        (trans.x,trans.y,trans.z)=self.translation
        (rot.x,rot.y,rot.z,rot.w)=tr.quaternion_from_euler(0,0,theta_out)

        return transform_out