import pygame
import sys
from Orcabot_strategy.component import Robot

class manulDrive:
    
    def __init__(self,Test_ssl,team:str,Id:int):
        pygame.init()
        self.Test_ssl = Test_ssl
        self.robot = Robot(Test_ssl,team,Id)
        # Set up the display
        width, height = 110, 5
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Keyboard Input")

        # Set up the clock
        self.clock = pygame.time.Clock()

        # Variables to track if a key is currently pressed
        self.keys_pressed = set()

        self.moveSpeed = 2.0
        self.rotationalSpeed = 5.0

    def execute(self):
        vertical = 0.0
        horizontal = 0.0
        rotation = 0.0
        kick = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                # Check if the key is not already being tracked
                if event.key == pygame.K_ESCAPE:
                    pygame.quit()
                    sys.exit()
                elif event.key not in self.keys_pressed:
                    self.keys_pressed.add(event.key)
                if event.key == pygame.K_SPACE:
                    kick = True
            elif event.type == pygame.KEYUP:
                # Check if the released key was being tracked
                if event.key in self.keys_pressed:
                    self.keys_pressed.remove(event.key)
                    kick = False

        # Check if the 'W' key is currently pressed
        if pygame.K_w in self.keys_pressed:
            vertical = self.moveSpeed
        if pygame.K_a in self.keys_pressed:
            horizontal = self.moveSpeed
        if pygame.K_s in self.keys_pressed:
            vertical = -self.moveSpeed
        if pygame.K_d in self.keys_pressed:    
            horizontal = -self.moveSpeed
        if pygame.K_k in self.keys_pressed:   
            rotation = self.rotationalSpeed
        if pygame.K_l in self.keys_pressed:    
            rotation = -self.rotationalSpeed
        if len(self.keys_pressed) == 0:
            vertical = 0.0
            horizontal = 0.0
            rotation = 0.0
            
        self.robot.sendCommand(vertical,horizontal,rotation,kick)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        self.clock.tick(60)