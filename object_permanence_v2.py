"""
User mouse input simulates object sightings (circles) of varying confidence (colour)
Displays a representation of miro's perception of the world in a tkinter window
"""

import time
import tkinter as tk

RES = (300, 300)
FPS = 60

root = tk.Tk()
root.title("Obj Permanence Map")

canvas = tk.Canvas(root, width=RES[0], height=RES[1], bg="black")
canvas.pack()

class TargetMarker:
    def __init__(self, pos):
        self.pos = pos
        self.size = 30
        self.item1 = None
        self.item2 = None
        
    def draw(self):
        x, y = self.pos
        half = self.size // 2
        # remove previous cross
        if self.item1: canvas.delete(self.item1)
        if self.item2: canvas.delete(self.item2)
        self.item1 = canvas.create_line(x - half, y - half, x + half, y + half, fill="blue", width=3)
        self.item2 = canvas.create_line(x - half, y + half, x + half, y - half, fill="blue", width=3)
        

class PotentialTargetObject:
    def __init__(self, pos, certainty):
        self.pos = [pos[0], pos[1]]
        self.certainty = certainty
        self.item = None

    def draw(self):
        x, y = world_pos_to_screen_pos(self.pos)
        r = (self.certainty / 100) * 20
        colour = "#%02x%02x00" % (int(255 - 255 * (self.certainty / 100)),
                                   int(255 * (self.certainty / 100)))
        if self.item:
            canvas.delete(self.item)
        self.item = canvas.create_oval(x - r, y - r, x + r, y + r, fill=colour, outline="")


def world_pos_to_screen_pos(pos):
    return int(RES[0] * (pos[0] + 1) / 2), int(RES[1] * (1 - (pos[1] + 1) / 2))

def screen_pos_to_world_pos(pos):
    return 2 * (pos[0] / RES[0]) - 1, 2 * (1 - pos[1] / RES[1]) - 1

def process_consumptions(objects, threshold_distance):
    consumed_indices = set()

    for i in range(len(objects)):
        if i in consumed_indices:
            continue
        for j in range(i + 1, len(objects)):
            if j in consumed_indices:
                continue

            o1 = objects[i]
            o2 = objects[j]

            dx = o1.pos[0] - o2.pos[0]
            dy = o1.pos[1] - o2.pos[1]
            if dx*dx + dy*dy < threshold_distance**2:
                if o1.certainty >= o2.certainty:
                    absorber, absorbed = o1, o2
                    absorbed_idx = j
                else:
                    absorber, absorbed = o2, o1
                    absorbed_idx = i

                ratio = absorbed.certainty / (absorber.certainty + absorbed.certainty)
                absorber.pos[0] += ratio * (absorbed.pos[0] - absorber.pos[0])
                absorber.pos[1] += ratio * (absorbed.pos[1] - absorber.pos[1])
                absorber.certainty = min(absorber.certainty + absorbed.certainty, 100)

                consumed_indices.add(absorbed_idx)

    objects[:] = [o for idx, o in enumerate(objects) if idx not in consumed_indices]


# ---------- main state ----------
potential_target_objects = []
click_time = None
forgetfullness = 2
threshold_distance = 0.2
last_time = time.perf_counter()

target_marker = TargetMarker((RES[0]//2, RES[1]//2))

# ---------- mouse events ----------
def on_mouse_down(event):
    global click_time
    click_time = time.perf_counter()

def on_mouse_up(event):
    global click_time
    if click_time is None:
        return
    click_duration = time.perf_counter() - click_time
    world_pos = screen_pos_to_world_pos((event.x, event.y))
    certainty = min(300 * click_duration, 100)
    potential_target_objects.append(PotentialTargetObject(world_pos, certainty))
    click_time = None

canvas.bind("<ButtonPress-1>", on_mouse_down)
canvas.bind("<ButtonRelease-1>", on_mouse_up)


# ---------- main loop ----------
def update():
    global last_time
    now = time.perf_counter()
    delta_time = now - last_time
    last_time = now

    # decay objects
    i = 0
    while i < len(potential_target_objects):
        potential_target_objects[i].certainty -= forgetfullness * delta_time
        if potential_target_objects[i].certainty <= 0:
            potential_target_objects.pop(i)
        else:
            i += 1

    process_consumptions(potential_target_objects, threshold_distance)

    # find biggest object
    biggest_object = None
    biggest_certainty = 0
    for o in potential_target_objects:
        if o.certainty > biggest_certainty:
            biggest_object = o
            biggest_certainty = o.certainty

    # draw all objects
    canvas.delete("all")  # clear everything
    for o in potential_target_objects:
        o.draw()

    # draw target marker
    if biggest_object:
        target_marker.pos = world_pos_to_screen_pos(biggest_object.pos)
    target_marker.draw()

    root.after(int(1000/FPS), update)


update()
root.mainloop()
