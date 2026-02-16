import numpy as np
import math
from config import B_MAX, B_MIN

TRESHOLD = 0.95

class Circles():

    # Define a custom sorting key function

    
    def sort_key_2(self, circle, x_init):
        x, y = circle['x'], circle['y']
        return y+x

    def generate_non_overlapping_circles(self, num_circles, radius_range, map_size, sorting, margin =0.0):
        circles = []
        
        # Generating obstacles, init and goal
        for _ in range(num_circles + 2):
            overlap = True
            while(overlap):
                # Reminder that turns are made at max(R+b, r_min)
                radius = round(np.random.uniform(radius_range[0],radius_range[1]),3)
                x = int(np.random.uniform(radius + margin, map_size[1] - radius - margin))
                y = int(np.random.uniform(radius + margin, map_size[1] - radius - margin))

                new_circle = {'x': x, 'y': y, 'radius': radius}
                overlap = any(
                    np.sqrt((c['x'] - x)**2 + (c['y'] - y)**2) < c['radius']+ radius_range[1]
                    for c in circles
                )
                if not overlap:
                    circles.append(new_circle)

        if sorting == "Distance":       
            circles = self.sort_by_distance(circles)
        if sorting == "Random":        
            circles = self.sort_as_random(circles)
        if sorting == "Crescent" :       
            circles = self.sort_as_crescent(circles)

        return circles
    
    def sort_as_crescent(self, circles):
        "Sort obstacles by distance on the y=x axis. "
        def sort_key(circle):
            x, y = circle['x'], circle['y']
            return y+x
        
        circles = sorted(circles, key=sort_key)
        circles[0]['radius'] = 0
        circles[-1]['radius'] = 0
        return circles

    def sort_as_random(self, circles):
        "All random obstacle sorting"
        circles[0]['radius'] = 0
        circles[-1]['radius'] = 0
        return circles
    
    def sort_by_distance(self, circles):
        "Sort obstacles by distance to initial point"
        def sort_key(circle, init):
            x, y = circle['x'], circle['y']
            xi, yi = init['x'], init['y']
            return (x - xi) ** 2 + (y - yi) ** 2
        
        circles[0]['radius'] = 0
        init= circles[0]
        circles = sorted(circles, key=lambda circle: sort_key(circle, init))
        circles[-1]['radius'] = 0
        return circles

    def point_on_circle(self, circle, angle_degrees):
        # Convert angle from degrees to radians
        angle_radians = math.radians(angle_degrees)
        if (circle['radius']):
            radius = circle['radius'] + B_MIN
        else:
            radius = 0
        
        # Calculate coordinates of the point on the circle
        point_x = circle['x'] + radius * math.cos(angle_radians)
        point_y = circle['y'] + radius * math.sin(angle_radians)

        # The orientation should be tangent to the circle
        angle_radians = math.radians(angle_degrees-90)

        return point_x, point_y, angle_radians
    
    def open_map(self, file_path):
        with open(file_path, 'r') as file:
            content = file.read().replace('\n', '')
        content = content.replace(' INIT :','').replace(' GOAL :','').replace(' ','').replace('\n','')
        # Extract arrays using '[]' as delimiters
        array_strings = content.split(';')

        # Process each array
        arrays = []
        for array_string in array_strings:
            # Extract subarrays using '],[' as delimiters
            subarrays = array_string.strip('[').strip(']').split('],[')
            # Clean up and convert subarrays to lists of floats
            array = []
            for subarray_str in subarrays:
                subarray_values = [float(x) for x in subarray_str.split(',')]
                array.append(subarray_values)
            arrays.append(array)

        x_init = tuple(arrays[1][0] + [0])
        x_goal = tuple(arrays[2][0] + [0])

        circles = [ {'x': x_init[0], 'y':x_init[1], 'radius': 0} ]
        for array in arrays[0]:
            new_circle = {'x': array[0], 'y': array[1], 'radius': array[2]}
            circles.append(new_circle)
        circles.append({'x': x_goal[0], 'y':x_goal[1], 'radius': 0})

        return circles, x_init, x_goal

    def save_map(self, file_path, circles, x_init, x_goal):
        """Save the current obstacle map and init/goal points to a file."""
        # Extract obstacle data (skip the first and last which are init/goal with radius 0)
        obstacles = []
        for circle in circles[1:-1]:
            obstacles.append([circle['x'], circle['y'], circle['radius']])
        
        # Format the content
        content = str(obstacles).replace(' ', '')
        content += f" ; INIT : [{x_init[0]}, {x_init[1]}]"
        content += f" ; GOAL : [{x_goal[0]}, {x_goal[1]}]"
        
        # Write to file
        with open(file_path, 'w') as file:
            file.write(content)


def distance_point_to_line_segment(h, k, x1, y1, x2, y2):
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


def segment_intersects_circle(p1, p2, circle, b):
    x1, y1 = p1
    x2, y2 = p2
    h, k, radius = circle['x'], circle['y'], circle['radius'] + b
    dist = distance_point_to_line_segment(h,k,x1,y1,x2,y2)
    if (dist < radius):
        return True
    else:
        return False
    

def circles_intersect(circle1, s):
    arc, start_angle, end_angle, turn_radius, wingspan = s[0], s[1], s[2], s[3], s[4]
    X1, Y1, R1 = circle1['x'], circle1['y'], circle1['radius']
    X2, Y2, R2 = arc['x'], arc['y'], arc['radius'] 
    R2 = turn_radius + wingspan

    # Calculate the distance between the centers of the circles
    distance = math.sqrt((X2 - X1)**2 + (Y2 - Y1)**2)
    if (X1 == X2 and Y1==Y2):
        return False
    
    # Check if any point on the arc is inside the larger circle
    if distance > R1 + R2:
        return False
    
    angles = np.linspace(start_angle, end_angle, 10)
    for angle in angles:
        x = X2 + R2 * math.cos(angle)
        y = Y2 + R2 * math.sin(angle)
        if (x - X1)**2 + (y - Y1)**2 < R1**2:
            print("Collision detected between circle at ({}, {}) and arc centered at ({}, {})".format(X1, Y1, X2, Y2))
            print("Arc parameters: start_angle={}, end_angle={}, turn_radius={}, wingspan={}".format(start_angle, end_angle, turn_radius, wingspan))
            return True
        
    return False

