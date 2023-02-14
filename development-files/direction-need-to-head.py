import math
import numpy
target_coords = (52.212379, 0.099279)
current_coords = (51.43666011007926, -0.15768630520465418)

def angle_to_targetv1(current, target):
	dist_v = target[0] - current[0]
	dist_h = target[1] - current[1]
	return math.atan(dist_h / dist_v)*360/2/math.pi
def angle_to_target(cu, tar):
	long2 = cu[1]
	long1 = tar[1]
	lat1 = tar[0]
	lat2 = cu[0]
	dLon = (long2 - long1)
	x = math.cos(math.radians(lat2)) * math.sin(math.radians(dLon))
	y = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(math.radians(dLon))
	brng = numpy.arctan2(x,y)
	brng = numpy.degrees(brng)
	return brng
print(angle_to_target(target_coords, current_coords))
print(angle_to_target(target_coords, (50.193982, 0.099)))
print(angle_to_target(target_coords, (54.20092, 0.098799)))
