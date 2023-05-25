import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
from math import *

# Initialize pygame and OpenGL
pygame.init()
pygame.display.set_mode((320, 240), DOUBLEBUF | OPENGL)
glMatrixMode(GL_PROJECTION)
gluPerspective(45, 640 / 480, 0.1, 50.0)
glMatrixMode(GL_MODELVIEW)
glLoadIdentity()
glTranslatef(0.0, 0.0, -5.0)

# Initialize joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Define font for text rendering
font = pygame.font.Font(None, 36)

# Main loop
while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()

    # Get joystick values
    # x = joystick.get_axis(0)
    # y = joystick.get_axis(1)
    # z = joystick.get_axis(2)
    yaw = joystick.get_axis(0)
    pitch = joystick.get_axis(1)
    roll = joystick.get_axis(2)

    # Convert joystick values to angles in radians
    pitch_angle = asin(pitch)
    roll_angle = asin(roll)
    yaw_angle = asin(yaw)

    # Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)


    # Draw the coordinate system
    glBegin(GL_LINES)
    # X axis
    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(0, 0.0, 0.0)
    glVertex3f(1.0, 0.0, 0.0)
    # Y axis
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(0.0, 0, 0.0)
    glVertex3f(0.0, 1.0, 0.0)
    # Z axis
    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, 0)
    glVertex3f(0.0, 0.0, 1.0)
    glEnd()




    # Draw the coordinate system
    # glPushMatrix()
    glRotatef(0 if abs(joystick.get_axis(1))<0.01 else joystick.get_axis(1), 1, 0, 0)
    glRotatef(0 if abs(joystick.get_axis(0))<0.01 else joystick.get_axis(0), 0, 1, 0)
    glTranslatef(0.01*joystick.get_hat(0)[0], 0                          , 0)
    glTranslatef(0                          , 0.01*joystick.get_hat(0)[1], 0)
    glTranslatef(0                          , 0                          , 0.01*joystick.get_button(2))
    glTranslatef(0                          , 0                          , -0.01*joystick.get_button(3))
    # glBegin(GL_LINES)
    # # X axis
    # glColor3f(1.0, 0.0, 0.0)
    # glVertex3f(0, 0.0, 0.0)
    # glVertex3f(1.0, 0.0, 0.0)
    # # Y axis
    # glColor3f(0.0, 1.0, 0.0)
    # glVertex3f(0.0, 0, 0.0)
    # glVertex3f(0.0, 1.0, 0.0)
    # # Z axis
    # glColor3f(0.0, 0.0, 1.0)
    # glVertex3f(0.0, 0.0, 0)
    # glVertex3f(0.0, 0.0, 1.0)
    # glEnd()
    # glPopMatrix()

    # Rotate the coordinate system based on joystick values
    
    
    # Draw a cube
    # glBegin(GL_QUADS)
    # glColor3f(1.0, 1.0, 1.0)
    # glVertex3f(-1.0, -1.0, 1.0)
    # glVertex3f(1.0, -1.0, 1.0)
    # glVertex3f(1.0, 1.0, 1.0)
    # glVertex3f(-1.0, 1.0, 1.0)
    # glEnd()

    # Draw text
    # text = font.render("Pitch: %.2f, Roll: %.2f, Yaw: %.2f" % (degrees(pitch_angle), degrees(roll_angle), degrees(yaw_angle)), 1, (255, 255, 255))
    # textpos = text.get_rect()
    # textpos.centerx = pygame.display.get_surface().get_rect().centerx
    # textpos.centery = pygame.display.get_surface().get_rect().centery + 100
    # pygame.display.get_surface().blit(text, textpos)

    # Update the display
    pygame.display.flip()
    pygame.time.wait(10)

    