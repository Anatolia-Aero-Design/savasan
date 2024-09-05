from server_comm.srv import sendqr
from server_comm.msg import Qr
import rospy


if __name__ == '__main__':
    rospy.init_node("uav_waypoint_manager", anonymous=True)
    
    
    rospy.wait_for_service('/send_qr_message')
    send_lock_message = rospy.ServiceProxy('/send_qr_message', sendqr)
    response = send_lock_message(Qr(start_hour=0,
                                            start_min = 0,
                                            start_second=0,
                                            start_milisecond=0,
                                            stop_hour=0,
                                            stop_min=0,
                                            stop_second=0,
                                            stop_milisecond=0,
                                            qr_text=0))
    print(response)
