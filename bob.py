import pygame

class Bob:
    def __init__(self, θ, x, y, mass, radius, color, velo: float):
        self.θ = θ
        self.x = x
        self.y = y
        self.mass = mass
        self.radius = radius
        self.color = color
        self.velo = velo
    
    def set_velo(self, v):
        self.velo = v
    
    def get_velo(self):
        return self.velo
    
    def set_x(self, x):
        self.x = x
    
    def set_y(self, y):
        self.y = y

    def draw(self, surface):
        pygame.draw.circle(surface, self.color, (int(self.x), int(self.y)), self.radius)