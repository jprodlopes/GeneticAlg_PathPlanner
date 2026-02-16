import tkinter as tk
B_MAX = 0.667
class DragDropcirclesApp:
    def __init__(self, root):

        #Dimensions
        self.height = 20
        self.windowSize = 400
        self.mapSize= 20
        self.amp = self.mapSize/self.windowSize
        self.root = root
        self.root.title("Drag and Drop circles")

        # Frame to hold the widgets
        frame = tk.Frame(root)
        frame.pack(side=tk.BOTTOM, fill=tk.X)

        # Canvas
        self.canvas = tk.Canvas(root, bg="white", bd=2, relief=tk.SUNKEN)
        self.canvas.pack(expand=tk.YES, fill=tk.BOTH)
        self.canvas.focus_set()

        # Distance Entry
        self.distance_entry = tk.Entry(frame, width=5)
        self.distance_entry.insert(0, "0.667")
        self.distance_entry.pack(side=tk.LEFT, padx=5)

        # Filename Entry
        self.filename_entry = tk.Entry(frame, width=10)
        self.filename_entry.insert(0, "exported_data")
        self.filename_entry.pack(side=tk.LEFT, padx=5)

        self.circles = []
        self.orange_points = []
        self.green_points = []
        self.distance = 0.667

        self.canvas.bind("<B1-Motion>", self.drag_circle)
        self.canvas.bind("<ButtonPress-1>", self.start_drag)
        self.canvas.bind("<ButtonRelease-1>", self.stop_drag)
        self.root.bind("<KeyPress-i>", self.add_orange_point)
        self.root.bind("<KeyPress-g>", self.add_green_point)
        self.canvas.bind("<Button-3>", self.right_click)  # Bind right-click event

        self.erase_button = tk.Button(frame, text="Erase All", command=self.erase_all)
        self.erase_button.pack(side=tk.LEFT, padx=5)

        self.distance_entry.bind("<Return>", self.update_distance)
        self.filename_entry.bind("<Return>", self.set_canvas_focus)

        # Draw graduated grid of light grey
        for i in range(0, self.windowSize, 50):
            self.canvas.create_line(i, 0, i, self.windowSize, fill="lightgrey", dash=(2, 2))
            self.canvas.create_line(0, i, self.windowSize, i, fill="lightgrey", dash=(2, 2))

        # Draw orange point at (20, 20)
        orange_point = self.canvas.create_oval(1/self.amp-5, 1/self.amp-5, 1/self.amp+5, 1/self.amp+5, 
                                               fill="green", tags="GOAL")
        self.orange_points.append(orange_point)
        # Draw green point at (350, 350)
        green_point = self.canvas.create_oval(19/self.amp-5, 19/self.amp-5, 19/self.amp+5, 19/self.amp+5,
                                               fill="orange", tags="INIT")
        self.green_points.append(green_point)

        # Bind the closing event to save the data
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def on_closing(self):
        self.export_data()
        self.root.destroy()

    def right_click(self, event):
        x = self.canvas.canvasx(event.x)
        y = self.canvas.canvasy(event.y)        

        for circle in self.circles:
            x1, y1, x2, y2 = self.canvas.coords(circle)
            center_x = (x1 + x2)/2
            center_y = (y1 + y2)/2
            radius = abs(x1-center_x)

            if radius > (x - center_x) ** 2 + (y - center_y)**2:
                print("CKECK")
                self.canvas.delete(circle)
                self.circles.remove(circle)

                
    def export_data(self):
        data_array1 =[]

        filename = self.filename_entry.get() + ".txt"

        # Append circle coordinates to the data array
        amp = self.mapSize/self.windowSize
        for circle in self.circles:
            x1, y1, x2, y2 = self.canvas.coords(circle)
            x = (x1 + x2) /2
            y = (y1 + y2) /2
            radius = abs(x2-x1)/2
            new_circle = [x*amp,y*amp,radius*amp]
            data_array1.append(new_circle)

        if(data_array1):
            with open(filename, "w") as file:
                file.write("[")
                for item in data_array1[:-1]:
                    file.write(f"{item},")
                file.write(f"{data_array1[-1]}")
                file.write("]\n")

                for point in self.green_points:
                    coords = [coord*amp for coord in self.canvas.coords(point)]
                    print(str(coords))
                    file.write(" ; INIT : " + str([coords[0], coords[1]]) + "\n")
                for point in self.orange_points:
                    coords = [coord*amp for coord in self.canvas.coords(point)]
                    print(str(coords))
                    file.write(" ; GOAL : " + str([coords[0], coords[1]]) + "\n")

    def start_drag(self, event):
        self.start_x = self.canvas.canvasx(event.x)
        self.start_y = self.canvas.canvasy(event.y)
        self.current_circle = None

    def stop_drag(self, event):
        if self.current_circle:
            center_x, center_y = self.start_x, self.start_y
            cur_x = self.canvas.canvasx(event.x)
            cur_y = self.canvas.canvasy(event.y)
            radius = ((cur_x - center_x) ** 2 + (cur_y - center_y) ** 2) ** 0.5

            # Check for overlap with existing circles
            overlap_circles = False
            for circle in self.circles:
                x1, y1, x2, y2 = self.canvas.coords(circle)
                other_center_x = (x1 + x2)/2
                other_center_y = (y1 + y2)/2
                other_radius = abs(x1-other_center_x)

                # Check if the distance between centers is smaller than the sum of radii
                if other_radius + radius >((other_center_x - center_x) ** 2 + (other_center_y - center_y) ** 2) ** 0.5:
                    overlap_circles = True

            # Check if any of the overlapping items are circles
            if not overlap_circles:
                # Only create the circle if there is no overlap
                if not self.check_boundary_exceed(center_x - radius, center_y - radius, center_x + radius, center_y+ radius):
                    circle = self.canvas.create_oval(
                        center_x - radius, center_y - radius,
                        center_x + radius, center_y + radius,
                        outline="black"
                    )
                    circle2 = self.canvas.create_oval(
                        center_x - radius - B_MAX/self.amp, center_y - radius - B_MAX/self.amp,
                        center_x + radius + B_MAX/self.amp , center_y + radius + B_MAX/self.amp,
                        outline="purple",
                        dash =(2,2)
                    )
                    self.circles.append(circle)
                else:
                    self.canvas.delete(self.current_circle)
            else:
                self.canvas.delete(self.current_circle)

    def drag_circle(self, event):
        if self.current_circle:
            self.canvas.delete(self.current_circle)

        center_x = self.start_x
        center_y = self.start_y
        cur_x = self.canvas.canvasx(event.x)
        cur_y = self.canvas.canvasy(event.y)

        radius = ((cur_x - center_x) ** 2 + (cur_y - center_y) ** 2) ** 0.5

        self.current_circle = self.canvas.create_oval(
            center_x - radius, center_y - radius,
            center_x + radius, center_y + radius,
            outline="black"
        )

    def add_orange_point(self, event):
        self.erase_orange_points()
        x, y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        if not self.check_point_inside_circles(x, y):
            orange_point = self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="orange", tags="INIT")
            self.orange_points.append(orange_point)

    def add_green_point(self, event):
        self.erase_green_points()
        x, y = self.canvas.canvasx(event.x), self.canvas.canvasy(event.y)
        if not self.check_point_inside_circles(x, y):
            green_point = self.canvas.create_oval(x-5, y-5, x+5, y+5, fill="green", tags="GOAL")
            self.green_points.append(green_point)

    def erase_all(self):
        for circle in self.circles:
            self.canvas.delete(circle)
        for point in self.orange_points:
            self.canvas.delete(point)
        for point in self.green_points:
            self.canvas.delete(point)
        self.circles = []
        self.orange_points = []
        self.green_points = []

    def erase_orange_points(self):
        for point in self.orange_points:
            self.canvas.delete(point)
        self.orange_points = []

    def erase_green_points(self):
        for point in self.green_points:
            self.canvas.delete(point)
        self.green_points = []

    def check_boundary_exceed(self, x1, y1, x2, y2 ):
        radius = abs(x1-x2)/2

        canvas_width = self.canvas.winfo_width()
        canvas_height = self.canvas.winfo_height()
        return x1 < 0 or y1  < 0 or x2 > canvas_width or y2 > canvas_height

    def check_point_inside_circles(self, x, y):
        for circle in self.circles:
            rx1, ry1, rx2, ry2 = self.canvas.coords(circle)
            if rx1 < x < rx2 and ry1 < y < ry2:
                return True
        return False

    def update_distance(self, event):
        new_distance = self.distance_entry.get()
        try:
            new_distance = int(new_distance)
            self.distance = new_distance
            self.canvas.focus_set()  # Set focus back to the canvas
        except ValueError:
            pass

    def set_canvas_focus(self, event):
        self.canvas.focus_set()


if __name__ == "__main__":
    root = tk.Tk()
    app = DragDropcirclesApp(root)
    root.geometry(str(app.windowSize)+"x"+str(int(app.windowSize*6/5)))
    root.mainloop()

