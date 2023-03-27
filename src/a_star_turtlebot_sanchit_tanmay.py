#!/usr/bin/env python3

import pygame
import time
import numpy as np
import math
import heapq as hq
import rospy
from geometry_msgs.msg import Twist

def create_pygame_map(display_surface, clearance, radius):
    # Define the colors
    YELLOW = (255, 255, 0)
    RED = (255, 0, 0)
    WHITE = (255, 255, 255)

    offset = radius + clearance

    for x in range(display_surface.get_width()):
        for y in range(display_surface.get_height()):

         # Drawing Scaled Obstacles ie. with clearance

            # Equations for rectangle1 using half-plane method
            if (x >= 150-offset and x <= 165+offset) and (y >= 0-offset and y <= 125+offset):
                display_surface.set_at((x,y), YELLOW)

            # Equations for rectangle2 using half-plane method
            if (x >= 250-offset and x <= 265+offset) and (y >= 75-offset and y <= 200+offset):
                display_surface.set_at((x,y), YELLOW)

            # Equations for hexagon using half-plane method
            math.sqrt((x - 400)**2 + (y - 90)**2) <= 50+offset
            if math.sqrt((x - 400)**2 + (y - 90)**2) <= 50+offset :
                display_surface.set_at((x,y), YELLOW)

            # Equations for boundary using half-plane method
            if(x-offset) <= 0 or (x+offset) >= 600 or (y-offset) <= 0 or (y+offset) >= 200:
                display_surface.set_at((x,y), YELLOW)

         # Drawing Scaled Obstacles ie. with clearance

            # Equations for rectangle1 using half-plane method
            if (x >= 150-radius and x <= 165+radius) and (y >= 0-radius and y <= 125+radius):
                display_surface.set_at((x,y), RED)

            # Equations for rectangle2 using half-plane method
            if (x >= 250-radius and x <= 265+radius) and (y >= 75-radius and y <= 200+radius):
                display_surface.set_at((x,y), RED)

            # Equations for hexagon using half-plane method
            math.sqrt((x - 400)**2 + (y - 90)**2) <= 50+radius
            if math.sqrt((x - 400)**2 + (y - 90)**2) <= 50+radius :
                display_surface.set_at((x,y), RED)

            # Equations for boundary using half-plane method
            if(x-radius) <= 0 or (x+radius) >= 600 or (y-radius) <= 0 or (y+radius) >= 200:
                display_surface.set_at((x,y), RED)

         # Drawing Unscaled Obstacles ie. without clearance

            # Equations for rectangle1 using half-plane method
            if (x >= 150 and x <= 165) and (y >= 0 and y <= 125):
                display_surface.set_at((x,y), WHITE)

            # Equations for rectangle2 using half-plane method
            if (x >= 250 and x <= 265) and (y >= 75 and y <= 200):
                display_surface.set_at((x,y), WHITE)

            # Equations for hexagon using half-plane method
            math.sqrt((x - 400)**2 + (y - 90)**2) <= 50
            if math.sqrt((x - 400)**2 + (y - 90)**2) <= 50 :
                display_surface.set_at((x,y), WHITE)

    pygame.display.update()

def UserInput(obstacle_map):

    start = []
    goal = []
    velocity = []
    
    while True:
        start_x = int(input("\nEnter the x coordinate of the start point: "))
        while start_x < 0 or start_x > 600:
            print("\nInvalid input. Please enter a value between 0 and 600.")
            start_x = int(input("Enter the x coordinate of the start point: "))
        
        start_y = int(input("\nEnter the y coordinate of the start point: "))
        while start_y < 0 or start_y > 200:
            print("\nInvalid input. Please enter a value between 0 and 200.")
            start_y = int(input("Enter the y coordinate of the start point: "))

        start_theta = int(input("\nEnter Orientation of the robot at the start point (0 -> 360): "))

        if obstacle_map.get_at((start_x,pygame.Surface.get_height(obstacle_map)-1 - start_y))[0] == 1:
            break
        print("\nThe start point is inside an obstacle. Please enter a valid start point.")

    start.append(start_x)
    start.append(start_y)
    start.append(start_theta)
    
    while True:
        goal_x = int(input("\nEnter the x coordinate of the goal point: "))
        while (goal_x < 0 or goal_x > 600):
            print("\nInvalid input. Please enter a value between 0 and 600.")
            goal_x = int(input("Enter the x coordinate of the goal point: "))
        
        goal_y = int(input("\nEnter the y coordinate of the goal point: "))
        while goal_y < 0 or goal_y > 200:
            print("\nInvalid input. Please enter a value between 0 and 200.")
            goal_y = int(input("Enter the y coordinate of the goal point: "))

        if obstacle_map.get_at((goal_x,pygame.Surface.get_height(obstacle_map)-1 - goal_y))[0] == 1:
            break
        print("\nThe goal point is inside an obstacle. Please enter a valid goal point.")
    
    goal.append(goal_x)
    goal.append(goal_y)

    while True:
        rpm1= int(input("\nEnter the first RPM: "))
        if rpm1 >= 0:
            break
        print("\nInvalid input. Please enter a value greater than 0.")

    while True:
        rpm2= int(input("\nEnter the second RPM: "))
        if rpm2 >= 0:
            break
        print("\nInvalid input. Please enter a value greater than 0.")

    velocity1 = rpm1*2*math.pi*3.3/60
    velocity2 = rpm2*2*math.pi*3.3/60
    velocity.append(velocity1)
    velocity.append(velocity2)

    return start, goal, velocity

def nh_constraints(node, velocity, time_move, robot_wheel_radius, robot_wheel_distance, dt,Visited,obstacle_map):
    D = 0
    t = 0
    x_n = node[0]
    y_n = node[1]
    theta_n = np.deg2rad(node[2] % 360)
    while t<time_move:
        t = t + dt
        dx = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.cos(theta_n) * dt
        dy = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.sin(theta_n) * dt
        x_n += dx
        y_n += dy
        theta_n  += (robot_wheel_radius / robot_wheel_distance) * (velocity[1] - velocity[0]) * dt
        if (y_n < 0) or (y_n > 200) or (x_n < 0) or (x_n > 600) or (obstacle_map.get_at((int(x_n),pygame.Surface.get_height(obstacle_map)-1 - int(y_n)))[0] != 1):
            return None, None, False, None

        dx_t = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.cos(theta_n) * dt
        dy_t = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.sin(theta_n) * dt
        D += math.sqrt(dx_t**2 + dy_t**2)

    theta_n = np.rad2deg(theta_n) % 360
    new_node = []
    new_node.append(int((x_n)/0.5+0.5)* 0.5)
    new_node.append(int((y_n)/0.5+0.5)* 0.5)
    new_node.append(int(theta_n))

    if(Visited[int(new_node[0]*2)][int(new_node[1]*2)][int(new_node[2]/30)] == 1):
        return new_node, D, True, velocity
    else:
        Visited[int(new_node[0]*2)][int(new_node[1]*2)][int(new_node[2]/30)] = 1
        return new_node, D, False, velocity

def CheckGoal(node, goal, start, obstacle_map, ClosedList, start_time,vel_pub,twist,rate,robot_wheel_radius,robot_wheel_distance,velocity_action):
    if math.dist([node[0],node[1]],[goal[0],goal[1]]) < 5:
        print("\n\033[92;5m" + "*****  Goal Reached!  *****" + "\033[0m")
        end_time = time.time()
        time_taken = round(end_time - start_time, 2)
        print("\n\033[92m" + "Time taken: " + str(time_taken) + " seconds" + "\033[0m")
        Backtrack(start, node, ClosedList, obstacle_map,vel_pub,twist,rate,robot_wheel_radius,robot_wheel_distance,velocity_action)
        return True
    else:
        return False

def CheckNode(node_new, ClosedList, OpenList, current_node, goal, boolean, D,vel,velocity_action):
    node_new = tuple(node_new)
    if node_new not in ClosedList:
        if boolean:
            for node in OpenList:
                if node[2] == node_new:
                    idx = OpenList.index(node)
                    cost = current_node[3]+ D + math.dist([node_new[0],node_new[1]],[goal[0],goal[1]])
                    if node[0] > cost:
                        OpenList[idx][0] = cost
                        OpenList[idx][3] = current_node[3] + D 
                        OpenList[idx][1] = current_node[2]
                        velocity_action[node_new] = vel
                    break
        else:
            hq.heappush(OpenList, [current_node[3] + D + math.dist([node_new[0],node_new[1]],[goal[0],goal[1]]), current_node[2], node_new,current_node[3] + D,math.dist([node_new[0],node_new[1]],[goal[0],goal[1]])])
            velocity_action[node_new] = vel

def Backtrack(start, goal, ClosedList, obstacle_map,vel_pub,twist,rate,robot_wheel_radius,robot_wheel_distance,velocity_action):
    path = []
    path.append(goal)
    current_node = goal
    print("\n\033[92m" + "Goal Node is: " + str(goal) + "\033[0m\n")
    pygame.draw.circle(obstacle_map, (0,0,255), (int(goal[0]),int(200 - 1 - goal[1])), 2, 1)
    pygame.display.update()

    for key in list(ClosedList.keys()):
        if key == (start[0],start[1]):
            continue
        else:
            obstacle_map.set_at((int(key[0]),int(200 - 1 - key[1])),(255,255,255))
        pygame.display.update()
    while current_node != start:
        current_node = ClosedList[(current_node[0],current_node[1],current_node[2])]
        path.append(current_node)
    path.reverse()
    for i in range(len(path)):
        try:
            velocity = velocity_action[(path[i][0],path[i][1],path[i][2])]
            vel_l, vel_a = robot_velocity(velocity,robot_wheel_radius,robot_wheel_distance)
            velocity_publisher(vel_l,vel_a,vel_pub,twist,rate)
        except rospy.ROSInterruptException:
            pass
        if i != 0:
            pygame.draw.aaline(obstacle_map, (0,0,255), (int(path[i][0]),int(200 - 1 - path[i][1])), (int(path[i-1][0]),int(200 - 1 - path[i-1][1])), 1)
        pygame.display.update()

    velocity_publisher(0,0,vel_pub,twist,rate)

    pygame.time.wait(100)
    print("\n\033[92m" + "Path Length: " + str(len(path)) + "\033[0m\n")

def AStarPlanner(start, goal, obstacle_map, velocity,vel_pub,twist,rate):

    robot_wheel_radius = 3.3 #value in cm
    robot_wheel_distance= 16 #value in cm
    time_move = 1 #Time to move in seconds
    dt = 0.01 #Change in time
    velocity_arr = [[0,velocity[0]], [velocity[0],0], [velocity[0],velocity[0]], [0,velocity[1]], [velocity[1],0], [velocity[1],velocity[1]], [velocity[0],velocity[1]], [velocity[1],velocity[0]]]

    OpenList = []
    flag = False
    ClosedList = {}
    velocity_action = {}
    velocity_action[(start[0],start[1],start[2])] = [0,0]
    Visited = np.zeros((1200,400,360))
    cost_to_go = math.dist([start[0],start[1]],[goal[0],goal[1]])
    cost_to_come = 0
    total_cost = cost_to_go + cost_to_come
    node_start = [total_cost, start, start, cost_to_come, cost_to_go]
    hq.heappush(OpenList, node_start)
    hq.heapify(OpenList)
    start_time = time.time()
    
    while (len(OpenList) > 0):
        current_node = hq.heappop(OpenList)
        ClosedList[(current_node[2][0],current_node[2][1],current_node[2][2])] =  current_node[1]
        # obstacle_map.set_at((int(current_node[2][0]),int(200 - 1 - current_node[2][1])),(255,255,255))
        # pygame.display.update()
        if CheckGoal(current_node[2], goal, start, obstacle_map, ClosedList, start_time,vel_pub,twist,rate,robot_wheel_radius,robot_wheel_distance,velocity_action) == True:
            flag = True
            break

        for i in range(len(velocity_arr)):
            new_node = []
            new_node, D, boolean,vel = nh_constraints(current_node[2], velocity_arr[i], time_move, robot_wheel_radius, robot_wheel_distance, dt,Visited,obstacle_map)
            if new_node is not None:
                CheckNode(new_node, ClosedList, OpenList, current_node, goal, boolean, D,vel,velocity_action)

    if flag == False:
        print("\n\033[91m" + "No Valid Path Found!" + "\033[0m\n")
        end_time = time.time()
        time_taken = round(end_time - start_time, 2)
        print("\033[91m" + "Time taken: " + str(time_taken) + " seconds" + "\033[0m\n")

def robot_velocity(velocity,robot_wheel_radius,robot_wheel_distance):
    velocity_left = velocity[0]
    velocity_right = velocity[1]
    theta_dot = (velocity_right - velocity_left) * robot_wheel_radius / robot_wheel_distance
    x_dot = (velocity_right + velocity_left) * robot_wheel_radius / 2 * np.cos(theta_dot)
    y_dot = (velocity_right + velocity_left) * robot_wheel_radius / 2 * np.sin(theta_dot)
    linear_velocity = np.sqrt(x_dot**2 + y_dot**2)
    angular_velocity = theta_dot
    return linear_velocity, angular_velocity
    
def velocity_publisher(linear_velocity, angular_velocity,vel_pub,twist,rate):

    publish_till = rospy.Time.now() + rospy.Duration(1)
    while rospy.Time.now() <= publish_till:
        twist.linear.x = linear_velocity/100
        twist.angular.z = angular_velocity
        vel_pub.publish(twist)
        rate.sleep() 

def main():
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=100)
    rospy.init_node('turtlebot_publisher', anonymous=True)
    rate = rospy.Rate(100)
    twist = Twist()

    pygame.init()
    clock = pygame.time.Clock()
    clock.tick(60)
    obstacle_map = pygame.display.set_mode((600, 200))
    pygame.display.set_caption("A* Planner")
    pygame.display.update()
    obstacle_map.fill((1,1,1))

    font = pygame.font.Font('freesansbold.ttf', 32)
    text = font.render('Waiting for User Input in Terminal', True, (255,255,0),(1,1,1))
    textRect = text.get_rect()
    textRect.center = (600 // 2, 200 // 2)
    obstacle_map.blit(text, textRect)
    pygame.display.update()

    clearance= int(input("\nEnter the clearence for the obstacle: "))
    robot_radius = 10.5 #value in cm
    obstacle_map.fill((1,1,1))

    create_pygame_map(obstacle_map,clearance,robot_radius)
    start, goal, velocity = UserInput(obstacle_map)
    AStarPlanner(start, goal, obstacle_map, velocity,vel_pub,twist,rate)

if __name__ == '__main__':
    print("\n\033[1m" + "******************************************" + "\033[0m")
    print("\033[1m" + "     A* Algorithm : Turtle Bot Burger     " + "\033[0m")
    print("\033[1m" + "******************************************" + "\033[0m")
    main()