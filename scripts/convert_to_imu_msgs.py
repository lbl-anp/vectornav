import rospy;
from sensor_msgs.msg import Imu;
from nucleardrone.msg import imu;
import numpy as np;

pub=[];

def main():
    global pub;
    rospy.init_node('imu_converter',anonymous=True);
    rospy.Subscriber('/vn300/imu',imu,imu_converter);
    pub=rospy.Publisher('imu',Imu,queue_size=30);
    rospy.spin();
    return;

def imu_converter(n_imu):
    global pub;
    diag=np.zeros(9)
    diag[0]=1.0
    diag[4]=1.0
    diag[8]=1.0
    msg = Imu()
    msg.header=n_imu.header
    msg.orientation_covariance=-np.ones(9)
    msg.linear_acceleration=n_imu.Accel
    #msg.linear_acceleration.x*=1
    #msg.linear_acceleration.y*=1
    #msg.linear_acceleration.z*=-1

    msg.linear_acceleration_covariance=diag
    msg.angular_velocity=n_imu.AngularRate
    #msg.angular_velocity.x*=-1;
    #msg.angular_velocity.y*=-1;
    #msg.angular_velocity.z*=-1;

    msg.angular_velocity_covariance=diag
    msg.header.frame_id='imu_link'
    pub.publish(msg)

if __name__=="__main__":
    main()
