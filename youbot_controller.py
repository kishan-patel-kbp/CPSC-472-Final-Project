"""youbot_controller controller."""

from types import MappingProxyType
from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs

class BerryMetadata():
    def __init__(self, x1, x2, color):
        self.x1 = x1
        self.x2 = x2
        self.color = color
        

import random
MAX_SPEED = 14.81
COLOR_DIST_THRESH = 300

BERRY_COLORS = {'r', 'o', 'y', 'p'}

BERRIES_PIXELS = {
    (212,140,95): ('orange', 0),
    (195,125,86): ('orange', 1),
    (151, 91, 63): ('orange', 2),
    (63,39,32): ('orange', 3),

    (224,68,48): ('red', 0),
    (207,62,47): ('red', 1),
    (165,44,34): ('red', 2),
    (70,19,18): ('red', 3),

    (224,214,32): ('yellow', 0),
    (219,208,34): ('yellow', 1),
    (192,180,32): ('yellow', 2),
    (160,149,22): ('yellow', 3),
    (71,70,14): ('yellow', 4),

    (209,137,181): ('pink', 0),
    (198,127,171): ('pink', 1),
    (151,91,131): ('pink', 2),
    (64,40,71): ('pink', 3),
}

FLOOR_PIXELS = [
    (216,183,171),
    (207,156,146),
    (220, 201, 197),
    (212, 170, 152),
    (221, 193, 180),
    (214, 178, 167),
    (221, 186, 170),
    (177,131,117),
    (209,166,148),
    (217, 194, 189),
    (184, 135, 117),
    (190, 139, 120),
    (202, 141, 132),
    (201, 151, 142),
    (202, 157, 147),
    (203, 151, 143),
    (202, 144, 132)
]

zombies_pixels = {
    'b': [
        (36,149,235),
        (36, 91, 170),
        (29, 76, 149),
        (14, 43, 102),
        (23, 65, 137),
        (30, 81, 159),
        (34, 87, 164),
        (28, 74, 146),
        (32, 97, 181),
        (28, 86, 167),
        (30, 101, 188),
        (14, 40, 94),
        (8, 33, 74),
        (10, 31, 66),
        (28, 93, 177),
        (13, 40, 102),
        (18, 58, 127),
        (30, 81, 159),
        (26, 87, 170),
        (15, 39, 89),
        (10, 39, 94),	
        (30, 141, 228),
        (24, 113, 202),
        (17, 75, 150),
        (10, 41, 99),
        (33, 161, 242),
        (23, 108, 195),
        (9, 35, 81),
        (30, 137, 225),	
        (25, 113, 203),			
    ],
    'g': [

    ],
    'p': [	
    ],
    'a': [
    ]
}

str_to_rgb = {
    'orange': [212,140,95],
    'red': [224,68,48],
    'yellow': [224,214,32],
    'pink': [209,137,181]
}


def base_forwards(wheels):
    # print("Called")
    for wheel in wheels:
        wheel.setPosition(float('inf'))
        wheel.setVelocity(SPEED)

def base_reset(wheels):
    print("resetting")
    for wheel in wheels:
        wheel.setVelocity(0.0)

def base_turn_right(wheels):
    speeds = [-.25*SPEED, -SPEED, -.25*SPEED, -SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def base_turn_left(wheels):
    speeds = [-SPEED, -.25*SPEED, -SPEED, -.25*SPEED]
    for i in range(4):
        wheels[i].setVelocity(speeds[i])
        
def dist(a, b):
    return sum([(a[i]-b[i])**2 for i in range(len(a))])

def approaching_wall(world_pixel_info):
    for i in range(len(world_pixel_info)):
        cnt = 0
        for j in range(len(world_pixel_info[i])):
            cnt += (world_pixel_info[i][j] == '0') # '0' means not a floor, zombie, or berry pixel
        print('cnt: ', cnt)
        if cnt >= 0.7 * len(world_pixel_info[i]): # 70% of the row is non floor/zombie/berry
            if i >= 0.6 * len(world_pixel_info): # this is past 60% of the rows
                return True
    return False

def get_berry_world_info(camera, world_pixel_info):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    
    berries_pixels_out = [] 
    
    for y in range(height):
        row = []
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            nxt_pixel = [0,0,0]
            for color in BERRIES_PIXELS:
                if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                    nxt_pixel = str_to_rgb[BERRIES_PIXELS[color][0]]
                    world_pixel_info[y][x] = BERRIES_PIXELS[color][0][0]
                    break
    return world_pixel_info

def get_floor_world_info(camera, world_pixel_info):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    floor_pixels_out = []
    
    for y in range(height):
        row = []
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            nxt_pixel = [0,0,0]
            for color in FLOOR_PIXELS:
                if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                    nxt_pixel = [255,255,255]
                    world_pixel_info[y][x] = 'f'
                    break
    return world_pixel_info
                
def get_zombie_world_info(camera, world_pixel_info):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    zombies_pixels_out = []
    
    for y in range(height):
        row = []
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            nxt_pixel = [0,0,0]
            for c in zombies_pixels:
                for color in zombies_pixels[c]:
                    if dist([r,g,b], color) <= COLOR_DIST_THRESH:
                        world_pixel_info[y][x] = c
                        nxt_pixel = zombies_pixels[c][0]
                        break
    return world_pixel_info

def get_berry_metadata(camera, world_pixel_info):
    
    width = camera.getWidth()
    height = camera.getHeight()
    
    all_berry_metadata = []
    first = False
    second = False
    
    x1 = -1
    x2 = -2
    for y in range(height):
        for x in range(width):
            if world_pixel_info[y][x] in BERRY_COLORS:
                if not first:
                    x1 = x
                    first = True
                else:
                    x2 = x
                    second = True
            elif first and second:
                berry_metadata = BerryMetadata(x1, x2, world_pixel_info[y][x])
                all_berry_metadata.append(berry_metadata)
                first = False
                second = False
                x1 = -1
                x2 = -1
                
    return all_berry_metadata

def get_closest_berry(all_berry_metadata):
    max_size = 0
    closest_berry_metadata = None
    
    for metadata in all_berry_metadata:
        size = metadata.x2 - metadata.x1
        if size > max_size:
            max_size = size
            closest_berry_metadata = metadata
            
    return closest_berry_metadata
        

def drive_to_berry(fr, fl, br, bl, camera, world_pixel_info):
    image_mid =  camera.getWidth() // 2
    all_berry_metadata = get_berry_metadata(camera, world_pixel_info)
    closest = get_closest_berry(all_berry_metadata)
    
    berry_center_position = -1
    if closest:
        berry_center_position = (closest.x2 + closest.x1) // 2
    
    error = abs(image_mid - berry_center_position)
    gain = error / image_mid
    
    print("image mid", image_mid)
    print("berry center", berry_center_position)
    
    fr.setVelocity(.5 * MAX_SPEED)
    fl.setVelocity(.5 * MAX_SPEED)
    br.setVelocity(.5 * MAX_SPEED)
    bl.setVelocity(.5 * MAX_SPEED)
    
    THRESHOLD = 2
    
    if berry_center_position == -1:
        print("berry not found")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif image_mid - THRESHOLD < berry_center_position < image_mid + THRESHOLD:
        print("berry aligned go straight")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif berry_center_position < image_mid:
        print("berry on the left")
        fr.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    else:
        print("berry on the right")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
                
    
#------------------CHANGE CODE ABOVE HERE ONLY--------------------------

def main():
    robot = Supervisor()

    # get the time step of the current world.
    timestep = int(robot.getBasicTimeStep())
    
    #health, energy, armour in that order 
    robot_info = [100,100,0]
    passive_wait(0.1, robot, timestep)
    pc = 0
    timer = 0
    
    robot_node = robot.getFromDef("Youbot")
    trans_field = robot_node.getField("translation")
    
    get_all_berry_pos(robot)
    
    robot_not_dead = 1
    
    #------------------CHANGE CODE BELOW HERE ONLY--------------------------
    
    #COMMENT OUT ALL SENSORS THAT ARE NOT USED. READ SPEC SHEET FOR MORE DETAILS
    accelerometer = robot.getDevice("accelerometer")
    accelerometer.enable(timestep)
    
    gps = robot.getDevice("gps")
    gps.enable(timestep)
    
    compass = robot.getDevice("compass")
    compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    camera2 = robot.getDevice("ForwardHighResSmallFov")
    camera2.enable(timestep)
    
    camera3 = robot.getDevice("ForwardHighRes")
    camera3.enable(timestep)
    
    camera4 = robot.getDevice("ForwardHighResSmall")
    camera4.enable(timestep)
    
    camera5 = robot.getDevice("BackLowRes")
    camera5.enable(timestep)
    
    camera6 = robot.getDevice("RightLowRes")
    camera6.enable(timestep)
    
    camera7 = robot.getDevice("LeftLowRes")
    camera7.enable(timestep)
    
    camera8 = robot.getDevice("BackHighRes")
    camera8.enable(timestep)
    
    gyro = robot.getDevice("gyro")
    gyro.enable(timestep)
    
    lightSensor = robot.getDevice("light sensor")
    lightSensor.enable(timestep)
    
    receiver = robot.getDevice("receiver")
    receiver.enable(timestep)
    
    rangeFinder = robot.getDevice("range-finder")
    rangeFinder.enable(timestep)
    
    lidar = robot.getDevice("lidar")
    lidar.enable(timestep)
    
    fr = robot.getDevice("wheel1")
    fl = robot.getDevice("wheel2")
    br = robot.getDevice("wheel3")
    bl = robot.getDevice("wheel4")
    
    fr.setPosition(float('inf'))
    fl.setPosition(float('inf'))
    br.setPosition(float('inf'))
    bl.setPosition(float('inf'))

    fr.setVelocity(0)
    fl.setVelocity(0)
    br.setVelocity(0)
    bl.setVelocity(0)   
    
    i=0
           
    tl_count = 0
    tr_count = 0
    
    #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
    
    
    while(robot_not_dead == 1):
        
        if(robot_info[0] < 0):
           
            robot_not_dead = 0
            print("ROBOT IS OUT OF HEALTH")
            #if(zombieTest):
            #    print("TEST PASSED")
            #else:
            #    print("TEST FAILED")
            #robot.simulationQuit(20)
            #exit()
            
        if(timer%2==0):
            trans = trans_field.getSFVec3f()
            robot_info = check_berry_collision(robot_info, trans[0], trans[2], robot)
            robot_info = check_zombie_collision(robot_info, trans[0], trans[2], robot)
            
        if(timer%16==0):
            robot_info = update_robot(robot_info)
            timer = 0
        
        if(robot.step(timestep)==-1):
            exit()
            
            
        timer += 1
        
     #------------------CHANGE CODE BELOW HERE ONLY--------------------------   

        world_pixel_info = get_berry_world_info(camera1, world_pixel_info)
        drive_to_berry(fr, fl, br, bl, camera1, world_pixel_info)
        
        if i==300:
            i = 0
        
        i+=1
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()