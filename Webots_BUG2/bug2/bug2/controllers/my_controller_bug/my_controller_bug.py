from controller import Robot
from controller import GPS
from controller import PositionSensor
import math

def give_alpha(x,y,gx,gy):
    alpha = ((math.atan2((gy-y),(gx-x))))*(180/3.14)
    return alpha

def get_dist_to_line(x,y,gx,gy,x_first,y_first):
    return abs((((gx-x_first)*(y_first-y))-((x_first-x)*(gy-y_first)))/(math.sqrt((gx-x_first)**2+(gy-y_first)**2)))

def run_robot(robot,gps,unit):
    # timestep = 64
    timestep = int(robot.getBasicTimeStep())
    gps.enable(1)
    unit.enable(1)
    
    max_speed = 6.28
    
    gx = 0.5
    # gy = 0.0
    gy = 0.5
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    left_sensor = robot.getDevice('left wheel sensor')
    right_sensor= robot.getDevice('right wheel sensor')
    
    left_sensor.enable(1)
    right_sensor.enable(1)
    
    prox_sensors = []
    for ind in range(8):
        sensor_name='ps'+str(ind)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[ind].enable(timestep)

    flag = 0
    start = 5
    
    x_second = 0.0
    y_second = 0.0
    
    pss = [0,0]
    prev_pss = [0,0]
    dists = [0,0]
    
    WHEEL_RADIUS = 0.0205
    WHEEL_CIR = 2*3.14*WHEEL_RADIUS
    WHEEL_UNIT = WHEEL_CIR/max_speed
    WHEELS_DIST= 0.2
    
    robopos = [0, 0, 0]

    while robot.step(timestep) != -1:
    
        x = gps.getValues()[0]
        # ry = gps.getValues()[1]
        y = gps.getValues()[2]  
    
        pss[0] = left_sensor.getValue()
        pss[1] = right_sensor.getValue()
        
        
        if(start==5):
            prev_pss[0] = pss[0]
            prev_pss[1] = pss[1]
            start = 1
            continue
        
        # print("--",pss[0]," ",pss[1])
        
        # print("--ps--",pss[0]," ",pss[1],"----")
        for ind in range(2):
            diff = pss[ind] - prev_pss[ind]
            if(diff < 0.001):
                diff = 0
                pss[ind] = prev_pss[ind]
            dists[ind] = diff * WHEEL_UNIT
            
        # print("--dists--",dists[0]," ",dists[1],"----",dists[0] - dists[1])
        
        v = (dists[0] + dists[1])/2.0
        w = (dists[0] - dists[1])/WHEELS_DIST
        
        
        dt = 1
        # print("w: ",w)
        # print("pos: ",robopos[2])
        robopos[2] += (w * dt)
        
        vx = v*math.cos(robopos[2])
        vy = v*math.sin(robopos[2])
        
        robopos[0] += (vx * dt)
        robopos[1] += (vy * dt)
        
        ang = robopos[2]*(180/3.14)
        # print(ang)
        
        for ind in range(2):
             prev_pss[ind] = pss[ind]

        # left_speed = max_speed
        # right_speed = -max_speed
        # left_motor.setVelocity(left_speed)
        # right_motor.setVelocity(right_speed) 
        # continue
    
        if(start==1):
            x_first = gps.getValues()[0]
            y_first = gps.getValues()[2]
            x_1st = x_first
            y_1st = y_first
            start=2
            continue
        elif(start==2):
            x_second = gps.getValues()[0]
            y_second = gps.getValues()[2]
            start=2
            orient = give_alpha(x_first,y_first,gx,gy)
            if(abs(orient-ang)<5):
                start = 0
            left_speed = max_speed
            right_speed = -max_speed
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed) 
            continue
            
        # angs = abs(unit.getRollPitchYaw()[2]*(180/math.pi))
        # print(angs)
        
        dist_to_line = get_dist_to_line(x,y,gx,gy,x_1st,y_1st)
        # print(dist_to_line)
           
        # for ind in range(8):
            # print("ind: {}, val: {}".format(ind,prox_sensors[ind].getValue()))
        
        left_wall = prox_sensors[5].getValue()>80
        left_corner = prox_sensors[6].getValue()>80
        front_wall = prox_sensors[7].getValue()>80
        
        left_wall = prox_sensors[5].getValue()>80
        left_corner = prox_sensors[6].getValue()>80
        front_wall = prox_sensors[7].getValue()>80
        
        left_speed = max_speed
        right_speed = max_speed
        
        
        if(flag==2):
            left_speed = max_speed
            right_speed = -max_speed
            left_motor.setVelocity(left_speed)
            right_motor.setVelocity(right_speed)
            orient = give_alpha(x,y,gx,gy)
            # print(orient," ",ang," ",abs(orient-ang))
            if(abs(orient-ang)<20):
                start=5
                flag = 0
                continue
            else:
                continue
        
        if(front_wall):
            flag = 1
        
        if(flag==1):
            if(left_wall and dist_to_line<0.03):
                left_speed = max_speed
                right_speed = -max_speed
                x_second = x
                y_second = y
                flag = 2
                continue
                        
            if front_wall:
                # print("turn right in place")
                left_speed = max_speed
                right_speed = -max_speed
            else:
                if left_wall:
                    # print("forward")
                    left_speed = max_speed
                    right_speed = max_speed
                else:
                     # print("turn left")
                     left_speed = max_speed/8
                     right_speed = max_speed
                     
                if left_corner:
                    # print("came too close, drive right")
                    left_speed = max_speed
                    right_speed = max_speed/8 
    
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    my_gps = my_robot.getDevice("gps")
    my_unit = my_robot.getDevice("inertial unit")
    run_robot(my_robot,my_gps,my_unit)
