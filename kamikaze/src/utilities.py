import math

def haversine_formula(latitude_1, longtitude_1, latitude_2, longtitude_2):
    R = 6371000 # Radius of the Earth in meters
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    
    longtitude_1_rad = math.radians(longtitude_1)
    longtitude_2_rad = math.radians(longtitude_2)
    
    lat_difference = abs(latitude_1_rad - latitude_2_rad)
    lon_difference = abs(longtitude_1_rad - longtitude_2_rad)
    
    a = (math.sin(lat_difference/2))**2 + (math.cos(latitude_1_rad) * math.cos(latitude_2_rad) * (math.sin(lon_difference/2))**2)
    c = (math.atan2(math.sqrt(a), math.sqrt(1 - a))) * 2
    distance = R * c
    return distance


def is_waypoint_reached(current_lat, current_lon, current_alt, 
                        target_latitude, target_longitude, target_altitude, tolerance=1.0, altitude_tolerance=1.0):

    distance = haversine_formula(latitude_1 = current_lat, latitude_2 = target_latitude, 
                                 longtitude_1 = current_lon, longtitude_2 = target_longitude)
    print(f"Distance is {distance}")
    
    altitude_dif = abs(current_alt - target_altitude)
    
    if distance <= tolerance and altitude_dif <= altitude_tolerance:
        return True
    else:
        return False 


def calculate_waypoints_with_angle(target_lat, target_lon, target_alt, home_alt,
                                   angle_degrees, distance1, distance2, distance3): # Calculate waypoints with the specified angle
    R = 6371000  # Radius of the Earth in meters

    # Convert latitude and longitude from degrees to radians
    target_lat_rad = math.radians(target_lat)
    target_lon_rad = math.radians(target_lon)

    # Calculate the waypoints
    wp2_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos((2 * distance1) / R) +
                            math.cos(target_lat_rad) * math.sin((2 * distance1) / R) * math.cos(math.radians(angle_degrees)))
    wp2_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin((2 * distance1) / R) * math.cos(target_lat_rad),
                            math.cos((2 * distance1) / R) - math.sin(target_lat_rad) * math.sin(wp2_lat_rad))

    wp3_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance2 / R) +
                            math.cos(target_lat_rad) * math.sin(distance2 / R) * math.cos(math.radians(angle_degrees)))
    wp3_lon_rad = target_lon_rad + math.atan2(math.sin(math.radians(angle_degrees)) * math.sin(distance2 / R) * math.cos(target_lat_rad),
                            math.cos(distance2 / R) - math.sin(target_lat_rad) * math.sin(wp3_lat_rad))

    wp5_lat_rad = math.asin(math.sin(target_lat_rad) * math.cos(distance3 / R) +
                            math.cos(target_lat_rad) * math.sin(distance3 / R) * math.cos(math.radians(angle_degrees + 180)))
    wp5_lon_rad = target_lon_rad + math.atan2(math.cos(math.radians(angle_degrees + 90)) * math.sin(distance3 / R) * math.cos(target_lat_rad),
                            math.cos(distance3 / R) - math.sin(target_lat_rad) * math.sin(wp5_lat_rad))

    return [
        (math.degrees(wp2_lat_rad), math.degrees(wp2_lon_rad), home_alt),
        (math.degrees(wp3_lat_rad), math.degrees(wp3_lon_rad), home_alt),
        (target_lat, target_lon, target_alt),
        (math.degrees(wp5_lat_rad), math.degrees(wp5_lon_rad), home_alt)
    ]