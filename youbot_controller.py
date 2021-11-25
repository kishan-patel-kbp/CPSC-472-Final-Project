"""youbot_controller controller."""

from typing import Mapping
from controller import Robot, Motor, Camera, Accelerometer, GPS, Gyro, LightSensor, Receiver, RangeFinder, Lidar
from controller import Supervisor

from youbot_zombie import *
   
#------------------CHANGE CODE BELOW HERE ONLY--------------------------
#define functions here for making decisions and using sensor inputs
    
MAX_SPEED = 14.81
TIME_STEP = 64


def getCameraRGBValues(camera):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    
    for y in range(height):
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            
            # print("x=" + str(x) + " y=" + str(y))
            print("red=" + str(r) + " green=" + str(g) + " blue=" + str(b))
    
def getStumpPosition(camera):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    image_mid = width // 2
    
    first = False
    second = False
    x1 = -1
    x2 = -1
    
    for y in range(height):
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            
            if r < 20 and g < 20 and b < 20:
                if not first:
                    x1 = x
                    first = True
                else:
                    x2 = x
                    second = True
            elif first and second:
                break
            else:
                continue
                   
    return (x1 + x2) // 2

def getRedBerryPosition(camera):
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    image_mid = width // 2
    
    first = False
    second = False
    x1 = -1
    x2 = -1
    
    for y in range(height):
        for x in range(width):
            r = Camera.imageGetRed(image, width, x, y)
            g = Camera.imageGetGreen(image, width, x, y)
            b = Camera.imageGetBlue(image, width, x, y)
            
            if r > 200 and b < 80:
                if not first:
                    x1 = x
                    first = True
                else:
                    x2 = x
                    second = True
            elif first and second:
                break
            else:
                continue
                   
    return (x1 + x2) // 2


def driveToStump(fr, fl, br, bl, camera):
    image_mid = camera.getWidth() // 2
    stump_position = getStumpPosition(camera)
    
    difference = abs(image_mid - stump_position)
    gain = difference / image_mid
    
    fr.setVelocity(.5 * MAX_SPEED)
    fl.setVelocity(.5 * MAX_SPEED)
    br.setVelocity(.5 * MAX_SPEED)
    bl.setVelocity(.5 * MAX_SPEED)
    
    THRESHOLD = 2
    
    if stump_position == -1:
        print("stump not found")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif image_mid - THRESHOLD < stump_position < image_mid + THRESHOLD:
        print("stump aligned go straight")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif stump_position < image_mid:
        print("stump on the left")
        fr.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    else:
        print("stump on the right")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED + gain * MAX_SPEED)
    

def driveToRedBerry(fr, fl, br, bl, camera):
    image_mid =  camera.getWidth() // 2
    berry_position = getRedBerryPosition(camera)
    
    difference = abs(image_mid - berry_position)
    gain = difference / image_mid
    
    fr.setVelocity(.5 * MAX_SPEED)
    fl.setVelocity(.5 * MAX_SPEED)
    br.setVelocity(.5 * MAX_SPEED)
    bl.setVelocity(.5 * MAX_SPEED)
    
    THRESHOLD = 2
    
    if berry_position == -1:
        print("berry not found")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif image_mid - THRESHOLD < berry_position < image_mid + THRESHOLD:
        print("berry aligned go straight")
        fr.setVelocity(.5 * MAX_SPEED)
        fl.setVelocity(.5 * MAX_SPEED)
        br.setVelocity(.5 * MAX_SPEED)
        bl.setVelocity(.5 * MAX_SPEED)
    elif berry_position < image_mid:
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
    
    # accelerometer = robot.getDevice("accelerometer")
    # accelerometer.enable(timestep)
    
    # gps = robot.getDevice("gps")
    # gps.enable(timestep)
    
    # compass = robot.getDevice("compass")
    # compass.enable(timestep)
    
    camera1 = robot.getDevice("ForwardLowResBigFov")
    camera1.enable(timestep)
    
    CAMERA_WIDTH = camera1.getWidth()
    CAMERA_HEIGHT = camera1.getHeight()
    
    # camera2 = robot.getDevice("ForwardHighResSmallFov")
    # camera2.enable(timestep)
    
    # camera3 = robot.getDevice("ForwardHighRes")
    # camera3.enable(timestep)
    
    # camera4 = robot.getDevice("ForwardHighResSmall")
    # camera4.enable(timestep)
    
    # camera5 = robot.getDevice("BackLowRes")
    # camera5.enable(timestep)
    
    # camera6 = robot.getDevice("RightLowRes")
    # camera6.enable(timestep)
    
    # camera7 = robot.getDevice("LeftLowRes")
    # camera7.enable(timestep)
    
    # camera8 = robot.getDevice("BackHighRes")
    # camera8.enable(timestep)
    
    # gyro = robot.getDevice("gyro")
    # gyro.enable(timestep)
    
    # lightSensor = robot.getDevice("light sensor")
    # lightSensor.enable(timestep)
    
    # receiver = robot.getDevice("receiver")
    # receiver.enable(timestep)
    
    # rangeFinder = robot.getDevice("range-finder")
    # rangeFinder.enable(timestep)
    
    # lidar = robot.getDevice("lidar")
    # lidar.enable(timestep)

    while robot.step(TIME_STEP) != -1:
        # driveToRedBerry(fr, fl, br, bl, camera1)
        driveToStump(fr, fl, br, bl, camera1)
        # getCameraRGBValues(camera1)

           

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
         #called every timestep
        
        
        #possible pseudocode for moving forward, then doing a 90 degree left turn
        #if i <100
            #base_forwards() -> can implement in Python with Webots C code (/Zombie world/libraries/youbot_control) as an example or make your own
        
        #if == 100 
            # base_reset() 
            # base_turn_left()  
            #it takes about 150 timesteps for the robot to complete the turn
                 
        #if i==300
            # i = 0
        
        #i+=1
        
        #make decisions using inputs if you choose to do so
         
        #------------------CHANGE CODE ABOVE HERE ONLY--------------------------
        
        
    return 0   


main()
