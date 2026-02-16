import math
import numpy as np
def arc_circle_crosses_circle(arc_center, arc_radius, angle_start, angle_end, other_center, other_radius):
    """
    Returns True if the arc (on arc_center, arc_radius, from angle_start to angle_end) crosses other_circle within the arc's sweep.
    """
    X1, Y1 = arc_center
    X2, Y2 = other_center
    R1 = arc_radius
    R2 = other_radius
    dx = X2 - X1
    dy = Y2 - Y1
    d = math.hypot(dx, dy)
    # No intersection if too far or one inside the other
    if d > R1 + R2 or d < abs(R1 - R2):
        return False
    # Find intersection points
    a = (R1**2 - R2**2 + d**2) / (2*d)
    h = math.sqrt(max(R1**2 - a**2, 0))
    xm = X1 + a * dx / d
    ym = Y1 + a * dy / d
    xs1 = xm + h * dy / d
    ys1 = ym - h * dx / d
    xs2 = xm - h * dy / d
    ys2 = ym + h * dx / d
    # Check if either intersection is within arc sweep
    for (xi, yi) in [(xs1, ys1), (xs2, ys2)]:
        angle = math.atan2(yi - Y1, xi - X1) % (2 * math.pi)
        # Normalize angles
        a_start = angle_start % (2 * math.pi)
        a_end = angle_end % (2 * math.pi)
        if a_start < a_end:
            in_arc = a_start <= angle <= a_end
        else:
            in_arc = angle >= a_start or angle <= a_end
        if in_arc:
            return True
    return False
import matplotlib.pyplot as plt
import numpy as np
import math
C1 = {'x' : 7, 'y': 7, 'radius': 4}
C2 = {'x' : 18, 'y': 18, 'radius': 5}
myMode = 'RSL'
myVP = [0,0,0,0]

def draw_dotted_circle(ax, circle, color='black', linestyle='dotted'):
    circle = plt.Circle([circle['x'],circle['y']], circle['radius'], color=color, fill=False, linestyle=linestyle)
    ax.add_patch(circle)

def random_point_on_circle(circle):
    """ 
    Generate a random point in a circle perimeter. 
    """
    xc, yc, radius = circle['x'],circle['y'], circle['radius']
    # Generate a random angle in radians
    theta = np.random.uniform(0, 2 * np.pi)

    # Convert polar coordinates to Cartesian coordinates
    x = radius * np.cos(theta) + xc
    y = radius * np.sin(theta) + yc

    return [x, y, theta-np.pi/2]

"Generate a tangent line between two circles"

def draw_tangent_line(circle_s = C1, circle_e = C2, mode = myMode, vehiclePars = myVP):

    X1, Y1, R1 = circle_s['x'], circle_s['y'], circle_s['radius']
    X2, Y2, R2 = circle_e['x'], circle_e['y'], circle_e['radius']
    [r_min_1, r_min_2, b_1, b_2] = vehiclePars

    if(R1):
        R1 = r_min_1
    if(R2):
        R2 = r_min_2
    dist = np.sqrt((X2-X1)**2 + (Y2-Y1)**2)
    
    #p1 : start point, p2 : end point
    if mode == "RSR":
        theta1 =   (np.arctan2((Y2-Y1),(X2-X1)))+(np.arccos((R1-R2)/dist))
        p1 = [X1 + R1*np.cos(theta1), Y1 + R1*np.sin(theta1)]
        p2 = [X2 + R2*np.cos(theta1), Y2 + R2*np.sin(theta1)]

    if mode == "LSL":   
        theta2 =   (np.arctan2((Y2-Y1),(X2-X1)))-(np.arccos((R1-R2)/dist))
        p1 = [X1 + R1*np.cos(theta2), Y1 + R1*np.sin(theta2)]
        p2 = [X2 + R2*np.cos(theta2), Y2 + R2*np.sin(theta2)]

    if mode == "LSR":
        theta3 =    (np.arctan2((Y2-Y1),(X2-X1))-np.arccos((R2+R1)/dist))
        p1 = [X1 + R1*np.cos(theta3), Y1 + R1*np.sin(theta3)]
        p2 = [X2 - R2*np.cos(theta3), Y2 - R2*np.sin(theta3)]

    if mode == "RSL":
        theta4 =    (np.arctan2((Y2-Y1),(X2-X1)) + np.arccos((R2+R1)/dist) )
        p1 = [X1 + R1*np.cos(theta4), Y1 + R1*np.sin(theta4)]
        p2 = [X2 - R2*np.cos(theta4), Y2 - R2*np.sin(theta4)]

    return p1, p2, b_1


def draw_arc_circle(circle, p1, p2, mode = myMode, vehiclePars=myVP):
    X1, Y1, R1 = circle['x'], circle['y'], circle['radius']
    [r_min_1, r_min_2, b_1, b_2] = vehiclePars

    # Only do this if the obstacle is not a point
    if(R1):
        R1 = r_min_1

    angle_start = np.arctan2(p1[1] - Y1, p1[0] - X1) % (2*np.pi)
    angle_end = np.arctan2(p2[1] - Y1, p2[0] - X1) % (2*np.pi)

    # Append arc circle to the path from start to p1
    if angle_start > angle_end:
        if ((mode == 'RSR') or (mode == 'RSL') or (mode == 'RT')):
            angle_end += 2 * np.pi
    if angle_end > angle_start:
        if ((mode == 'LSL') or (mode == 'LSR') or (mode == 'LT')):
            angle_start += 2 * np.pi      

    phi = np.linspace(angle_end, angle_start, 50)
    x_arc = X1 + R1 * np.cos(phi)
    y_arc = Y1 + R1 * np.sin(phi)
    
    return [x_arc, y_arc], [circle, angle_start, angle_end, R1, b_1]


# CHECK COLLISIONS

# GEOMETRY

# Check if the line segment from (x0, y0) to (x1, y1) intersects a circle centered at (c1, c2) with radius R
def segment_crosses_circle(p1, p2, circle):
    x1, y1 = p1
    x2, y2 = p2
    h, k, radius = circle['x'], circle['y'], circle['radius']
    
    if(y1==k and x1==h) or (y2==k and x2==h):
        return False
    dist = distance_point_to_line_segment(x1,y1,x2,y2,h,k)
    
    if (dist < radius-0.01):  # Subtract a small margin to avoid numerical errors
        return True
    else:
        return False

def distance_point_to_line_segment(x1, y1, x2, y2, h, k):
    def distance_point_to_line(x, y, a, b, c):
        return abs(a*x + b*y + c) / math.sqrt(a**2 + b**2)

    # Calculate the coefficients of the line equation (Ax + By + C = 0)
    a = y2 - y1
    b = x1 - x2
    c = x2*y1 - x1*y2

    # Calculate the distance from the point to the line
    distance = distance_point_to_line(h, k, a, b, c)

    # Check if the point is within the line segment
    dot_product1 = (h - x1)*(x2 - x1) + (k - y1)*(y2 - y1)
    dot_product2 = (h - x2)*(x1 - x2) + (k - y2)*(y1 - y2)

    if dot_product1 >= 0 and dot_product2 >= 0:
        return distance
    else:
        # The point is outside the line segment, so return the minimum distance
        return min(math.sqrt((h - x1)**2 + (k - y1)**2), math.sqrt((h - x2)**2 + (k - y2)**2))

#Check if the arc s intersects a circle
def circle_crosses_circle(X1,Y1,R1, X2,Y2,R2):
    
    dist = np.sqrt((X2-X1)**2 + (Y2-Y1)**2)
    if dist <= R1+R2:
        return True
    
    return False