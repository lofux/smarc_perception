#!/usr/bin/env python3
import rospy
import sys
from sensor_msgs.msg import PointCloud2

from nav_msgs.msg import Odometry
import numpy as np
import ros_numpy
import matplotlib.pyplot as plt
import cv2
from geometry_msgs.msg import Point
import tf2_ros
import tf



#write a class (use class attributes inside the callback)

class PtcloudToImg:

    """
    Class to turn the MBES pointcloud messages to images.
    """


    def __init__(self):

        #do some stuff
        #rospy.sleep(5)
        self.pt_array = None
        self.pt_array_y = None
        self.pt_array_x = None
        self.pt_array_intensity = None
        self.pose = None
        self.pose_x = None
        self.pose_y = None
        self.pub = None
        self.point_x = None
        self.point_y = None


        sub = rospy.Subscriber('/lolo/mbes/enu/bathy_cloud', PointCloud2, self.callback)
        sub2 = rospy.Subscriber('/lolo/dr/odom', Odometry, self.callback2)

        #self.pub = rospy.Publisher("lolo/mbes/enu/featurePos", PoseArray , queue_size=0)

        print('sub: ', sub)
        print('i subscribed')
        #self.pub = rospy.Publisher('/point_array', , queue_size=0)



    def callback(self, data):

        #do some callback stuff
        #print('hej')
        self.pt_array = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        x = self.pt_array['x']
        y = self.pt_array['y']
        z = self.pt_array['z']
        intensity = self.pt_array['intensity']

        self.pt_array_y = self.pt_array['y']
        self.pt_array_x = self.pt_array['x']
        self.pt_array_intensity = self.pt_array['intensity']


    def callback2(self, data):
        self.pose = data.pose.pose  # the x,y,z pose and quarternion orientation
        #print('the pose is: ', self.pose)
        #print('the x position is: ', self.pose.position.x) # works! :)
        self.pose_x = self.pose.position.x
        self.pose_y = self.pose.position.y



def main():
    rospy.init_node("mbes_pointcloud_2_img")
    rospy.loginfo("Starting mbes pointcloud to image node.")
    print('i subscribed')
    pt2im = PtcloudToImg()
    #y = self.pt_array['y']

    pub = rospy.Publisher("lolo/mbes/enu/featurePos", Odometry, queue_size=0)
    pub2 = rospy.Publisher("lolo/mbes/enu/featurePoints", Point, queue_size=0)

    tfBuffer = tf2_ros.Buffer()
    #trans = tfBuffer.lookup_transform('lolo/dr/odom', 'lolo/mbes/enu/bathy_cloud', rospy.Time())
    trans = tfBuffer.lookup_transform('map', 'lolo/enu/mbes_link' , rospy.Time()) #'lolo/enu/mbes_link',
    print('trans: ', trans)

    #print(pt2im.pt_array)
    iter = 0

    img = np.zeros([256, 256])
    xodomVec = np.zeros([256])
    yodomVec = np.zeros([256])
    intensity_vec_now = np.zeros([256])

    while not rospy.is_shutdown():
        intensity = pt2im.pt_array_intensity


        if not pt2im.pt_array_intensity =='None':


            if iter > 256:
                iter_num = np.remainder(iter, 256)

                #print(pt2im.pt_array_intensity)
                #print('iter_num: ', iter_num)

                # check if pt_array_intensity == to last iteration if so reset iter one step back
                if np.array_equal(intensity_vec_now, pt2im.pt_array_intensity):
                    #print('equal intensity ')
                    iter_num = iter_num -1
                    iter = iter -1

                img[iter_num, :] = pt2im.pt_array_intensity
                xodomVec[iter_num] = pt2im.pose_x
                yodomVec[iter_num] = pt2im.pose_y
                #print('pt2im.pose_x: ', pt2im.pose_x)
                intensity_vec_now = pt2im.pt_array_intensity







                #save image every 256th iteration
                if iter_num == 255:
                    print('saving new image now!')
                    #print('i: ',i)
                    #print('iter: ', iter)
                    #print('iter_num ', iter_num)

                    #np.save('img', img**3)
                    img_new = img/255
                    print('img_new: ', img_new)


                    fig = plt.figure(frameon = False)
                    ax = plt.Axes(fig, [0., 0., 1., 1.,])
                    ax.set_axis_off()
                    fig.add_axes(ax)
                    ax.imshow(img_new, aspect='auto')#, cmap='jet')#, cmap='inferno')
                    fig.savefig('intensityImageTest1.png')

                    #Then run feature detection on the new image , Print out just one feature to start with
                    #I_1 = cv2.imread('intensityImageTest1.png')
                    #Im_1 = cv2.cvtColor(I_1, cv2.COLOR_BGR2GRAY)

                    #Im_1 =  np.reshape(Im_1, [256, 256])
                    #corners = cv2.goodFeaturesToTrack(Im_1, 1, 0.01, 10)
                    img = img.astype(np.uint8)
                    corners = cv2.goodFeaturesToTrack(img, 1, 0.01, 10)

                    print('corners: ', corners)

                    for i in corners:
                        x_corner, y_corner = i.ravel()
                        #print('x corner: ', int(x_corner))
                        #print('y corner: ', int(y_corner))

                    #print('xodomVec: ', xodomVec)
                    xtrue =  xodomVec[int(x_corner)]
                    yimg =  pt2im.pt_array_y[int(y_corner)]
                    #print('xtrue: ', xtrue)
                    #print('yimg: ', yimg)
                    yodom = yodomVec[int(y_corner)]
                    ytrue =  yodom + yimg
                    #print('ytrue: ', ytrue)

                    #publish the xtrue&ytrue message over ROS
                    # Publish
                    PoseArray_msg = Odometry()

                    PoseArray_msg.pose.pose.position.x = xtrue
                    PoseArray_msg.pose.pose.position.y = ytrue

                    Point_msg = Point()

                    Point_msg.x = xtrue
                    Point_msg.y = ytrue

                    #PoseArray_msg.poses = pt

                    #pt = (xtrue, ytrue, 0)
                    #PoseArray_msg(position=pt)
                    #ytrue, 0]
                    pub.publish(PoseArray_msg)
                    print('i published message')

                    pub2.publish(Point_msg)

        iter = iter + 1


    rospy.spin()


if __name__ == "__main__":
    main()


#
