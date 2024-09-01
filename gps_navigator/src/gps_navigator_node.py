#!/usr/bin/env python

import rospy
from server_comm.msg import KonumBilgileri
from mavros_msgs.srv import CommandLong, CommandLongRequest
from std_srvs.srv import Empty, EmptyResponse
from mavros_msgs.srv import CommandInt

class GPSNavigator:
    def __init__(self):
        self.navigation_active = False
        self.gps_sub = None
        self.team_number = None
        self.command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

        # Services to start and stop navigation
        self.start_service = rospy.Service('start_navigation', Empty, self.start_navigation)
        self.stop_service = rospy.Service('stop_navigation', Empty, self.stop_navigation)
        
        # Service to set team number
        self.set_team_service = rospy.Service('set_team_number', Empty, self.set_team_number)

    def set_team_number(self, req):
        # Ask for team number input (you can also pass this through the service request if needed)
        try:
            self.team_number = int(input("Enter team number: "))
            rospy.loginfo(f"Team number set to {self.team_number}")
        except ValueError:
            rospy.logerr("Invalid team number. Please enter a valid integer.")
        return EmptyResponse()
    
    def start_navigation(self, req):
        if not self.navigation_active:
            self.gps_sub = rospy.Subscriber('/konum_bilgileri', KonumBilgileri, self.gps_callback)
            self.navigation_active = True
            print("Navigation started.")
        return EmptyResponse()

    def stop_navigation(self, req):
        if self.navigation_active:
            self.gps_sub.unregister()
            self.navigation_active = False
            print("Navigation stopped.")
        return EmptyResponse()

    def gps_callback(self, msg: KonumBilgileri):
        if not self.navigation_active or self.team_number is None:
            return

        try:
            # Search for the target with the matching team number
            target = next((konum for konum in msg.konumBilgileri if konum.takim_numarasi == self.team_number), None)

            if target is not None:
                latitude = target.IHA_enlem
                longitude = target.IHA_boylam
                altitude = target.IHA_irtifa
                self.send_goto_command(latitude, longitude, altitude)
            else:
                rospy.logwarn(f"Team number {self.target_id} not found in the received data.")

        except Exception as e:
            rospy.logerr(f"Failed to parse GPS data: {e}")

    def send_goto_command(self, lat, lon, alt):
        rospy.wait_for_service('/mavros/cmd/command')
        try:
            command_int = rospy.ServiceProxy('/mavros/cmd/command_int', CommandInt) # after service is on creating service obj to use functions of server
            response = command_int( # creating comman_int obj to send command to vehicle (above service)
                # we can add target_system and target_component to the beginning 
                frame = 3,  # the coordinate system of the component
                command = 192,   # command id
                current = 2,  #not used
                autocontinue = 0, #not used
                param1 = 30,  # param1 = ground speed for command 192
                param2 = 0,  # param2 = Bitmask of option flags.
                param3 = 0,  # param3 = Loiter radius for planes.
                param4 = 0,  #param4 = Yaw heading. 0 clockwise 1 counter clockwise
                x = int(lat * 1e7),  #param 5 = target latitude
                y = int(lon * 1e7), # param 6 = target longitude
                z = alt  # param 7 = target altitude
            )
            rospy.loginfo("Position command sent")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)

if __name__ == '__main__':
    rospy.init_node('gps_navigator_node', anonymous=True)
    gps_navigator = GPSNavigator()
    rospy.spin()
