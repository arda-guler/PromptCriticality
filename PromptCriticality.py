import random
import numpy as np
import time
import tkinter as tk

_PI = np.pi
_2PI = 2 * _PI

## === SETUP ===
init_particle_count = 100
init_particle_min_spacing = 6
enrichment = 0.85
particle_drag_factor = 5e-2

enable_casing = True
casing_radius = 80
casing_reflectance = 0.8
casing_strength = 40

neutron_speed = 25
neutron_lifetime = 5
reaction_pressure_ratio = 3000
scatter_percent = 0.4
## === SETUP ===

neutrons = []
particles = []

cam = None

screen_x = 800
screen_y = 600

help_text = """
PromptCriticality
=================

Red particles are ready to undergo fission, blue particles are depleted.
Magenta particles represent neutrons.

Arrow keys to move around, Left Shift and Left Ctrl control zoom.

Click on a red particle to manually ignite fission, or wait until a
spontaneous emission triggers a chain reaction.
"""

class Neutron:
    def __init__(self, pos, vel, life):
        self.pos = pos
        self.vel = vel
        self.life = life

        self.passed_boundary = False

    def kinematics(self, dt):
        global neutrons, enable_casing, casing_radius, casing_reflectance
        
        self.pos = self.pos + self.vel * dt
        self.life -= dt

        coll = self.check_collision()

        if (not coll) and self.life < 0:
            neutrons.remove(self)
            del self

        elif enable_casing and not self.passed_boundary:
            pos_mag = np.linalg.norm(self.pos)

            if pos_mag > casing_radius:
                self.passed_boundary = True
                
                if random.uniform(0, 1) < casing_reflectance:
                    self.pos = self.pos / pos_mag * casing_radius
                    self.vel = -self.pos / pos_mag * np.linalg.norm(self.vel) * 0.9

                else:
                    pass
            

    def check_collision(self):
        global particles, neutrons
        
        for p in particles:
            if not p.active:
                continue
            
            if np.linalg.norm(p.pos - self.pos) < p.radius:
                if random.uniform(0, 1) < scatter_percent:
                    p.react(self)

                else:
                    neutrons.remove(self)
                    del self
                    
                return True

            # removal of neutron from the simulation is handled by Radioactive.react()

        return False

class Radioactive:
    def __init__(self, active, pos, vel=np.array([0, 0]), radius=5, spontaneity=2e-6,
                 neutron_chance=[3, 5], pressure=1):
        self.pos = pos
        self.vel = vel
        self.radius = radius
        self.active = active

        self.spontaneity = spontaneity
        self.neutron_chance = neutron_chance
        self.pressure = pressure

    def __eq__(self, other):
        if not isinstance(other, Radioactive):
            return False

        if np.linalg.norm(other.pos - self.pos) > 1e-5:
            return False

        return True

    def spontaneous_release(self, dt):
        if random.uniform(0, 1) * dt < self.spontaneity:
            self.react()

    def kinematics(self, dt):
        global particle_drag_factor, enable_casing, casing_radius
        self.pos = self.pos + self.vel * dt
        self.vel = self.vel - self.vel * particle_drag_factor * dt # slight drag applied

        # apply regular pressure to nearby particles
        for p in particles:
            if not p == self and np.linalg.norm(p.pos - self.pos) < 30:
                dist = np.linalg.norm(p.pos - self.pos)
                press_dir = (p.pos - self.pos) / dist
                press = self.pressure * press_dir / dist**2

                p.vel = p.vel + press

        if enable_casing:
            pos_mag = np.linalg.norm(self.pos)

            # apply regular casing pressure
            dist = casing_radius - pos_mag
            press_dir = -self.pos / pos_mag
            press = self.pressure * 10 * press_dir / dist**2
            self.vel = self.vel + press
            
            if pos_mag > casing_radius:
                self.pos = self.pos / pos_mag * casing_radius
                self.vel = -self.pos / pos_mag * np.linalg.norm(self.vel) * 0.3

    def react(self, absorbed_neutron=None):
        global neutron_speed, neutron_lifetime, neutrons, particles, reaction_pressure_ratio

        neutron_count = random.randint(self.neutron_chance[0], self.neutron_chance[1])

        # generate new neutrons
        for i in range(neutron_count):
            random_angle = random.uniform(0, _2PI)
            new_vel = np.array([neutron_speed * np.sin(random_angle), neutron_speed * np.cos(random_angle)])
            new_neutron = Neutron(self.pos, new_vel, neutron_lifetime)

            neutrons.append(new_neutron)

        # remove absorbed neutron
        if absorbed_neutron:
            neutrons.remove(absorbed_neutron)
            del absorbed_neutron

        # apply reaction pressure to nearby particles
        for p in particles:
            if not p == self:
                dist = np.linalg.norm(p.pos - self.pos)
                press_dir = (p.pos - self.pos) / dist
                press = self.pressure * reaction_pressure_ratio * press_dir / dist**2

                p.vel = p.vel + press
            
        self.active = False

def get_closest_particle_to_coords(x, y):
    global particles
    
    result = None
    for p in particles:
        if not result or np.linalg.norm(p.pos - np.array([x, y])) < np.linalg.norm(np.array([x, y]) - result.pos):
            result = p

    return result

class Camera:
    def __init__(self, pos, zoom):
        self.pos = pos
        self.zoom = zoom

    def set_pos(self, pos):
        self.pos = pos

    def set_zoom(self, zoom):
        self.zoom = zoom

    def move(self, movement):
        self.pos += movement

    def do_zoom(self, zoom):
        self.zoom *= zoom

    def get_state(self):
        return self.state

    def get_pos(self):
        return self.pos

    def get_zoom(self):
        return float(self.zoom)

def move_current_cam_left(event=None):
    global cam
    cam.move(np.array([-30.0 * cam.get_zoom(), 0]))

def move_current_cam_right(event=None):
    global cam
    cam.move(np.array([30.0 * cam.get_zoom(), 0]))

def move_current_cam_up(event=None):
    global cam
    cam.move(np.array([0, 30.0 * cam.get_zoom()]))

def move_current_cam_down(event=None):
    global cam
    cam.move(np.array([0, -30.0 * cam.get_zoom()]))

def zoom_current_cam_out(event=None):
    global cam
    cam.do_zoom(2)

def zoom_current_cam_in(event=None):
    global cam
    cam.do_zoom(0.5)

def space2canvas(xi, yi):
    global cam, screen_x, screen_y
    xo = ((xi - cam.pos[0]) * cam.zoom + screen_x * 0.5)
    yo = ((-yi + cam.pos[1]) * cam.zoom + screen_y * 0.5)

    return xo, yo

def canvas2space(xo, yo):
    global cam, screen_x, screen_y
    xi = ((xo - screen_x * 0.5) / cam.zoom) + cam.pos[0]
    yi = ((yo - screen_y * 0.5) / cam.zoom) - cam.pos[1]

    return xi, -yi

def manual_ignite(event):
    x, y = canvas2space(event.x, event.y)
    particle_to_ignite = get_closest_particle_to_coords(x, y)

    if particle_to_ignite.active:
        particle_to_ignite.react()

def main():
    global init_particle_count, enrichment, neutron_speed, neutron_lifetime, reaction_pressure_ratio, cam
    global enable_casing, casing_radius, casing_strength, help_text

    # generate particles of nuclear material
    prev_init_pos = np.array([1000.0, 1000.0])
    new_pos = np.array([1000.0, 1000.0])
    for i in range(init_particle_count):
        while np.linalg.norm(prev_init_pos - new_pos) < init_particle_min_spacing:
            if enable_casing:
                new_posR = random.uniform(1, min(init_particle_count * 2.5, casing_radius * 0.95))
            else:
                new_posR = random.uniform(1, init_particle_count * 2.5)
            new_posTheta = random.uniform(0, _2PI)
            new_pos = np.array([new_posR * np.sin(new_posTheta), new_posR * np.cos(new_posTheta)])

        if random.uniform(0, 1) < enrichment:
            new_active = True
        else:
            new_active = False
            
        new_particle = Radioactive(new_active, new_pos)
        particles.append(new_particle)

        prev_init_pos = new_pos

    # init graphics
    root = tk.Tk()
    root.geometry('820x620')
    root.title("Prompt Criticality")

    canvas = tk.Canvas(root, width=800, height=600)
    canvas.configure(bg="black")
    canvas.grid(row=0, column=0)

    # camera
    cam = Camera(np.array([0.0, 0.0]), 1.0)
    
    root.bind("<Up>", move_current_cam_up)
    root.bind("<Down>", move_current_cam_down)
    root.bind("<Left>", move_current_cam_left)
    root.bind("<Right>", move_current_cam_right)
    root.bind("<Shift_L>", zoom_current_cam_out)
    root.bind("<Control_L>", zoom_current_cam_in)

    canvas.bind('<Button-1>', manual_ignite)

    print(help_text)

    running = True
    dt = 0.01
    sim_time = 0
    while running:
        t_cycle_start = time.perf_counter()
        sim_time = sim_time + dt

        # PHYSICS
        for n in neutrons:
            n.kinematics(dt)

        for p in particles:
            if p.active:
                p.spontaneous_release(dt)

            p.kinematics(dt)

        if enable_casing and len(neutrons) > casing_strength:
            enable_casing = False

        # GRAPHICS
        canvas.delete("all")

        if enable_casing:
            cvs_pos = space2canvas(0, 0)
            canvas.create_oval(cvs_pos[0]-casing_radius * cam.zoom, cvs_pos[1]-casing_radius * cam.zoom,
                               cvs_pos[0]+casing_radius * cam.zoom, cvs_pos[1]+casing_radius * cam.zoom,
                               fill="gray")
        
        for p in particles:
            if p.active:
                fillcolor = "red"
            else:
                fillcolor = "blue"

            cvs_pos = space2canvas(p.pos[0], p.pos[1])
            canvas.create_oval(cvs_pos[0]-3, cvs_pos[1]-3,
                               cvs_pos[0]+3, cvs_pos[1]+3,
                               fill=fillcolor)

        for n in neutrons:
            cvs_pos = space2canvas(n.pos[0], n.pos[1])
            canvas.create_oval(cvs_pos[0]-2, cvs_pos[1]-2,
                               cvs_pos[0]+2, cvs_pos[1]+2,
                               fill="magenta")

        root.update()
        dt = time.perf_counter() - t_cycle_start

    root.destroy()
        
if __name__ == "__main__":
    main()
