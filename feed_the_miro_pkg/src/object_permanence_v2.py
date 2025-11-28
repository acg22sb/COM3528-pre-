"""
User mouse input simulates object sightings (circles) of varying confidence (colour)
Displays a representation of miro's perception of the world in a tkinter window
"""

import time
# import tkinter as tk

class ObjectPermanenceManager:
    def __init__(self, with_gui=False, permanence_time=10):
        self.with_gui = with_gui

        if self.with_gui:

            self.RES = (300, 300)
            self.FPS = 60

            self.root = tk.Tk()
            self.root.title("Obj Permanence Map")
            self.canvas = tk.Canvas(root, width=RES[0], height=RES[1], bg="black")
            self.canvas.pack()

        self.potential_target_objects = []
        self.click_time = None
        self.permanence_time = permanence_time
        self.threshold_distance = 0.2
        self.last_time = time.perf_counter()

        if self.with_gui:
            self.target_marker = self.TargetMarker((RES[0]//2, RES[1]//2))
            self.canvas.bind("<ButtonPress-1>", on_mouse_down)
            self.canvas.bind("<ButtonRelease-1>", on_mouse_up)

    class TargetMarker:
        def __init__(self, pos):
            self.pos = pos
            self.size = 30
            self.item1 = None
            self.item2 = None

            
        def draw(self):
            if self.with_gui:
                x, y = self.pos
                half = self.size // 2
                if self.item1: self.canvas.delete(self.item1)
                if self.item2: self.canvas.delete(self.item2)
                self.item1 = self.canvas.create_line(x - half, y - half, x + half, y + half, fill="blue", width=3)
                self.item2 = self.canvas.create_line(x - half, y + half, x + half, y - half, fill="blue", width=3)
        

    class PotentialTargetObject:
        def __init__(self, pos, certainty):
            self.pos = [pos[0], pos[1]]
            self.certainty = certainty
            self.item = None

        def draw(self):
            if self.with_gui:
                x, y = self.world_pos_to_screen_pos(self.pos)
                r = (self.certainty / 100) * 20
                colour = "#%02x%02x00" % (int(255 - 255 * (self.certainty / 100)),
                                        int(255 * (self.certainty / 100)))
                if self.item:
                    self.canvas.delete(self.item)
                self.item = self.canvas.create_oval(x - r, y - r, x + r, y + r, fill=colour, outline="")


    def world_pos_to_screen_pos(self, pos):
        return int(self.RES[0] * (pos[0] + 1) / 2), int(self.RES[1] * (1 - (pos[1] + 1) / 2))

    def screen_pos_to_world_pos(self, pos):
        return 2 * (pos[0] / self.RES[0]) - 1, 2 * (1 - pos[1] / self.RES[1]) - 1

    def process_consumptions(self, objects, threshold_distance):
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
        return objects


    def add_observation(self, world_pos, certainty):
        self.potential_target_objects.append(self.PotentialTargetObject(world_pos, certainty))

    def get_target_pos(self):
        now = time.perf_counter()
        delta_time = now - self.last_time
        self.last_time = now

        # decay objects
        i = 0
        while i < len(self.potential_target_objects):
            self.potential_target_objects[i].certainty -= (1 / self.permanence_time) * delta_time
            if self.potential_target_objects[i].certainty <= 0:
                self.potential_target_objects.pop(i)
            else:
                i += 1

        self.potential_target_objects = self.process_consumptions(self.potential_target_objects, self.threshold_distance)

        # find biggest object
        biggest_object = None
        biggest_certainty = 0
        for o in self.potential_target_objects:
            if o.certainty > biggest_certainty:
                biggest_object = o
                biggest_certainty = o.certainty

        if self.with_gui:

            # draw all objects
            self.canvas.delete("all")  # clear everything
            for o in self.potential_target_objects:
                o.draw()

            # draw target marker
            if biggest_object:
                self.target_marker.pos = self.world_pos_to_screen_pos(biggest_object.pos)
            self.target_marker.draw()

            #self.root.after(int(1000/FPS), update)
            self.root.mainloop()
        
        if biggest_object:
            return biggest_object.pos
        else:
            return None


if __name__ == "__main__":
    state = 2
    start_time = time.perf_counter()

    import random

    opm = ObjectPermanenceManager()
    while True:
        if (time.perf_counter() - start_time) > state:
            opm.add_observation((random.uniform(-1, 1), random.uniform(-1, 1)), random.uniform(0, 1))
            state += 2
        print(opm.get_target_pos())
        time.sleep(1/60)
