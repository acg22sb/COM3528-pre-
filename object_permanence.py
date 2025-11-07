"""
User mouse input simulates object sightings (circles) of varying confidence (colour)
Note: To simulate object sightings, hold for click different times in different positions
Displays a representation of miro's perception of the world in a pygame window
Implements forgetfullness "I havent seen that object in a while must have been a false positive"
Implements consumption "I saw the object in a similar place before, it must be the same instance of a potential object, combine them"
Implements targeting, identifies the most probable possible instance of the target object as a target (marked with a blue cross)
Note: Currently implemented as an aside to the project, miro and ros incorporation is TODO
"""


import time, pygame

RES = (300, 300)
FPS = 60
screen = pygame.display.set_mode(RES)
pygame.display.set_caption("Obj Permanence Map")
background_colour = "black"
clock = pygame.time.Clock()

class TargetMarker:
    def __init__(self, pos):
        self.pos = pos
        self.w, self.h = 30, 30
        self.image = pygame.Surface((self.w, self.h), pygame.SRCALPHA)
        pygame.draw.line(self.image, "blue", (0, 0), (self.w, self.h), 8)
        pygame.draw.line(self.image, "blue", (0, self.h), (self.w, 0), 8)
        
    def draw(self):
        screen.blit(self.image, (self.pos[0] - self.w//2, self.pos[1] - self.h//2))
        

class PotentialTargetObject:
    def __init__(self, pos, certainty):
        self.pos = [pos[0], pos[1]]
        self.certainty = certainty

    def draw(self):
        colour = (int(255 - 255 * (self.certainty / 100)), int(255 * (self.certainty / 100)), 0)
        pygame.draw.circle(screen, colour, world_pos_to_screen_pos(self.pos), (self.certainty / 100) * 20)


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

potential_target_objects = []

click_time = None

forgetfullness = 2
threshold_distance = 0.2

last_time = time.perf_counter()

target_marker = TargetMarker((RES[0]//2, RES[1]//2))

while True:

    screen.fill(background_colour)

    delta_time = time.perf_counter() - last_time
    last_time = time.perf_counter()
    
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
            
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                click_time = time.perf_counter()
                
        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                click_duration = time.perf_counter() - click_time
                world_pos = screen_pos_to_world_pos(event.pos)
                certainty = min(300 * click_duration, 100)

                potential_target_objects.append(PotentialTargetObject(world_pos, certainty))


    i = 0
    while i < len(potential_target_objects):
        potential_target_objects[i].certainty -= forgetfullness * delta_time
        if potential_target_objects[i].certainty <= 0:
            potential_target_objects.remove(potential_target_objects[i])
        else:
            i += 1

    process_consumptions(potential_target_objects, threshold_distance)

    biggest_object = None
    biggest_certainty = 0
    for o in potential_target_objects:
        if o.certainty > biggest_certainty:
            biggest_object = o
            biggest_certainty = o.certainty

    for o in potential_target_objects:
        o.draw()

    if biggest_object != None:
        target_marker.pos = world_pos_to_screen_pos(biggest_object.pos)
    target_marker.draw()
    pygame.display.update()
    clock.tick(FPS)
    


