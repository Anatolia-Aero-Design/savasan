import math

EARTH_RADIUS = 6371000

def haversine_formula(latitude_1, longitude_1, latitude_2, longitude_2):
    """
    Calculate the great-circle distance between two points on the Earth's surface.
    """
    latitude_1_rad = math.radians(latitude_1)
    latitude_2_rad = math.radians(latitude_2)
    
    longitude_1_rad = math.radians(longitude_1)
    longitude_2_rad = math.radians(longitude_2)
    
    lat_difference = abs(latitude_1_rad - latitude_2_rad)
    lon_difference = abs(longitude_1_rad - longitude_2_rad)
    
    a = (math.sin(lat_difference / 2)) ** 2 + (math.cos(latitude_1_rad) * math.cos(latitude_2_rad) * (math.sin(lon_difference / 2)) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = EARTH_RADIUS * c
    return distance
