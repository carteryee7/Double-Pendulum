import pygame

class Ball:
    def __init__(self, x, y, mass, radius, color, velo):
        self.x = x
        self.y = y
        self.mass = mass
        self.radius = radius
        self.color = color
        self.velo = velo

    def move(self):
        self.x += self.velo[0]
        self.y += self.velo[1]
    
    def scale_velo(self, vx, vy):
        self.velo[0] *= vx
        self.velo[1] *= vy
    
    def add_to_velo(self, vx, vy):
        self.velo[0] += vx
        self.velo[1] += vy
    
    def set_velo(self, vx, vy):
        self.velo[0] = vx
        self.velo[1] = vy
    
    def get_velo(self):
        return self.velo

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), self.radius)