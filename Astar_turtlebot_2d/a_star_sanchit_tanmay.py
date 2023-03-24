import sys
import pygame
import vidmaker
import time
import numpy as np
import math
import heapq as hq
import argparse

def argument_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--save_video', action='store_true')
    args = parser.parse_args()
    return args

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
            if (x >= 100-offset and x <= 150+offset) and (y >= 0-offset and y <= 100+offset):
                display_surface.set_at((x,y), YELLOW)

            # Equations for rectangle2 using half-plane method
            if (x >= 100-offset and x <= 150+offset) and (y >= 150-offset and y <= 250+offset):
                display_surface.set_at((x,y), YELLOW)

            # Equations for hexagon using half-plane method
            if (y-(0.578*x)-(-offset*(0.578**2 + 1)**0.5 -123.21))>= 0 and (y-(-0.578*x)-(offset*((-0.578)**2 + 1)**0.5 + 373.21))<= 0 and (y-(0.578*x)-(offset*(0.578**2 + 1)**0.5 + 26.79))<= 0 and (y-(-0.578*x)-(-offset*((-0.578)**2 + 1)**0.5 + 223.21))>= 0 and (x-235+offset) >= 0 and (x-365-offset) <= 0:
                display_surface.set_at((x,y), YELLOW)

            # Equations for triangle using half-plane method
            if (y-(2*x)-(-offset*(2**2 + 1)**0.5 + (-895)))>= 0 and (y-(-2*x)-(offset*((-2)**2 + 1)**0.5 + (1145)))<= 0 and (x-460+offset) >= 0:
                display_surface.set_at((x,y), YELLOW)

            # Equations for boundary using half-plane method
            if(x-offset) <= 0 or (x+offset) >= 600 or (y-offset) <= 0 or (y+offset) >= 250:
                display_surface.set_at((x,y), YELLOW)

         # Drawing Scaled Obstacles ie. with clearance

            # Equations for rectangle1 using half-plane method
            if (x >= 100-clearance and x <= 150+clearance) and (y >= 0-clearance and y <= 100+clearance):
                display_surface.set_at((x,y), RED)

            # Equations for rectangle2 using half-plane method
            if (x >= 100-clearance and x <= 150+clearance) and (y >= 150-clearance and y <= 250+clearance):
                display_surface.set_at((x,y), RED)

            # Equations for hexagon using half-plane method
            if (y-(0.578*x)-(-clearance*(0.578**2 + 1)**0.5 -123.21))>= 0 and (y-(-0.578*x)-(clearance*((-0.578)**2 + 1)**0.5 + 373.21))<= 0 and (y-(0.578*x)-(clearance*(0.578**2 + 1)**0.5 + 26.79))<= 0 and (y-(-0.578*x)-(-clearance*((-0.578)**2 + 1)**0.5 + 223.21))>= 0 and (x-235+clearance) >= 0 and (x-365-clearance) <= 0:
                display_surface.set_at((x,y), RED)

            # Equations for triangle using half-plane method
            if (y-(2*x)-(-clearance*(2**2 + 1)**0.5 + (-895)))>= 0 and (y-(-2*x)-(clearance*((-2)**2 + 1)**0.5 + (1145)))<= 0 and (x-460+clearance) >= 0:
                display_surface.set_at((x,y), RED)

            # Equations for boundary using half-plane method
            if(x-clearance) <= 0 or (x+clearance) >= 600 or (y-clearance) <= 0 or (y+clearance) >= 250:
                display_surface.set_at((x,y), RED)

         # Drawing Unscaled Obstacles ie. without clearance

            # Equations for rectangle1 using half-plane method
            if (x >= 100 and x <= 150) and (y >= 0 and y <= 100):
                display_surface.set_at((x,y), WHITE)
            
            # Equations for rectangle2 using half-plane method
            if (x >= 100 and x <= 150) and (y >= 150 and y <= 250):
                display_surface.set_at((x,y), WHITE)

            # Equations for hexagon using half-plane method
            if (y-(0.578*x)-(-123.21))>= 0 and (y-(-0.578*x)-(373.21))<= 0 and (y-(0.578*x)-(26.79))<= 0 and (y-(-0.578*x)-(223.21))>= 0 and (x-235) >= 0 and (x-365) <= 0:
                display_surface.set_at((x,y), WHITE)

            # Equations for triangle using half-plane method
            if (y-(2*x)-(-895))>= 0 and (y-(-2*x)-(1145))<= 0 and (x-460) >= 0:
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
        while start_y < 0 or start_y > 250:
            print("\nInvalid input. Please enter a value between 0 and 250.")
            start_y = int(input("Enter the y coordinate of the start point: "))

        start_theta = int(input("\nEnter Orientation of the robot at the start point: "))

        if obstacle_map.get_at((start_x,pygame.Surface.get_height(obstacle_map)-1 - start_y))[0] == 1:
            break
        print("\nThe start point is inside an obstacle. Please enter a valid start point.")

    start.append(start_x)
    start.append(start_y)
    start.append(start_theta*30)
    
    while True:
        goal_x = int(input("\nEnter the x coordinate of the goal point: "))
        while (goal_x < 0 or goal_x > 600):
            print("\nInvalid input. Please enter a value between 0 and 600.")
            goal_x = int(input("Enter the x coordinate of the goal point: "))
        
        goal_y = int(input("\nEnter the y coordinate of the goal point: "))
        while goal_y < 0 or goal_y > 250:
            print("\nInvalid input. Please enter a value between 0 and 250.")
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
        if (y_n < 0) or (y_n > 250) or (x_n < 0) or (x_n > 600) or (obstacle_map.get_at((int(x_n),pygame.Surface.get_height(obstacle_map)-1 - int(y_n)))[0] != 1):
            return None, None, False

        theta_n  += (robot_wheel_radius / robot_wheel_distance) * (velocity[1] - velocity[0]) * dt
        dx_t = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.cos(theta_n) * dt
        dy_t = 0.5*robot_wheel_radius * (velocity[0] + velocity[1]) * math.sin(theta_n) * dt
        D += math.sqrt(dx_t**2 + dy_t**2)

    theta_n = np.rad2deg(theta_n) % 360

    new_node = []
    new_node.append(int((x_n)/0.5+0.5)* 0.5)
    new_node.append(int((y_n)/0.5+0.5)* 0.5)
    new_node.append(theta_n)

    if(Visited[int(new_node[0]*2)][int(new_node[1]*2)][int(new_node[2]/30)] == 1):
        return new_node, D, True
    else:
        Visited[int(new_node[0]*2)][int(new_node[1]*2)][int(new_node[2]/30)] = 1
        return new_node, D, False

def CheckGoal(node, goal, start, obstacle_map, ClosedList, start_time):
    if math.dist([node[0],node[1]],[goal[0],goal[1]]) < 1.5:
        print("\n\033[92;5m" + "*****  Goal Reached!  *****" + "\033[0m")
        end_time = time.time()
        time_taken = round(end_time - start_time, 2)
        print("\n\033[92m" + "Time taken: " + str(time_taken) + " seconds" + "\033[0m")
        Backtrack(start, node, ClosedList, obstacle_map)
        return True
    else:
        return False

def CheckNode(node_new, ClosedList, OpenList, current_node, goal, boolean, D):
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
                    break
        else:
            hq.heappush(OpenList, [current_node[3] + D + math.dist([node_new[0],node_new[1]],[goal[0],goal[1]]), current_node[2], node_new,current_node[3] + D,math.dist([node_new[0],node_new[1]],[goal[0],goal[1]])])

def Backtrack(start, goal, ClosedList, obstacle_map):
    args = argument_parser()
    video = vidmaker.Video("A*Planner_pygame.mp4", late_export=True)
    path = []
    path.append(goal)
    current_node = goal
    for key in list(ClosedList.keys()):
        if key == (start[0],start[1]):
            continue
        else:
            obstacle_map.set_at((int(key[0]),int(250 - 1 - key[1])),(255,255,255))
            if args.save_video:
                video.update(pygame.surfarray.pixels3d(obstacle_map).swapaxes(0, 1), inverted=False)
            pygame.display.update()
    while current_node != start:
        current_node = ClosedList[(current_node[0],current_node[1],current_node[2])]
        path.append(current_node)
    path.reverse()
    for i in range(len(path)):
        if i == 0:
            pygame.draw.aaline(obstacle_map, (0,0,255), (int(path[i][0]),int(250 - 1 - path[i][1])), (int(path[i][0]),int(250 - 1 - path[i][1])), 1)
        else:
            pygame.draw.aaline(obstacle_map, (0,0,255), (int(path[i][0]),int(250 - 1 - path[i][1])), (int(path[i-1][0]),int(250 - 1 - path[i-1][1])), 1)
        if args.save_video:
            video.update(pygame.surfarray.pixels3d(obstacle_map).swapaxes(0, 1), inverted=False)
        pygame.display.update()
  
    pygame.display.update()
    pygame.time.wait(100)
    print("\n\033[92m" + "Path Length: " + str(len(path)) + "\033[0m\n")

    if args.save_video:
        pygame.quit()
        video.export(verbose=True)
        video.compress(target_size=1024, new_file=False)

def AStarPlanner(start, goal, obstacle_map, velocity):

    robot_wheel_radius = 3.3 #value in cm
    robot_wheel_distance= 16 #value in cm
    time_move = 1 #Time to move in seconds
    dt = 0.1 #Change in time
    velocity_arr = [[0,velocity[0]], [velocity[0],0], [velocity[0],velocity[0]], [0,velocity[1]], [velocity[1],0], [velocity[1],velocity[1]], [velocity[0],velocity[1]], [velocity[1],velocity[0]]]

    OpenList = []
    flag = False
    ClosedList = {}
    Visited = np.zeros((1200,500,13))
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
        obstacle_map.set_at((int(current_node[2][0]),int(250 - 1 - current_node[2][1])),(255,255,255))
        pygame.display.update()
        if CheckGoal(current_node[2], goal, start, obstacle_map, ClosedList, start_time) == True:
            flag = True
            break

        for i in range(len(velocity_arr)):
            new_node = []
            new_node, D, boolean = nh_constraints(current_node[2], velocity_arr[i], time_move, robot_wheel_radius, robot_wheel_distance, dt,Visited,obstacle_map)
            if new_node is not None:
                CheckNode(new_node, ClosedList, OpenList, current_node, goal, boolean, D)

    if flag == False:
        print("\n\033[91m" + "No Valid Path Found!" + "\033[0m\n")
        end_time = time.time()
        time_taken = round(end_time - start_time, 2)
        print("\033[91m" + "Time taken: " + str(time_taken) + " seconds" + "\033[0m\n")

def main():
    args = argument_parser()
    pygame.init()
    clock = pygame.time.Clock()
    clock.tick(60)
    obstacle_map = pygame.display.set_mode((600, 250))
    pygame.display.set_caption("A* Planner")
    pygame.display.update()
    obstacle_map.fill((1,1,1))

    font = pygame.font.Font('freesansbold.ttf', 32)
    text = font.render('Waiting for User Input in Terminal', True, (255,255,0),(1,1,1))
    textRect = text.get_rect()
    textRect.center = (600 // 2, 250 // 2)
    obstacle_map.blit(text, textRect)
    pygame.display.update()

    clearance= int(input("\nEnter the clearence for the obstacle: "))
    robot_radius = 10.5 #value in cm
    obstacle_map.fill((1,1,1))

    create_pygame_map(obstacle_map,clearance,robot_radius)
    start, goal, velocity = UserInput(obstacle_map)
    AStarPlanner(start, goal, obstacle_map, velocity)
    print("\n\033[1m" + " Press q to exit " + "\033[0m")

    while True and args.save_video == False:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    pygame.quit()
                    sys.exit()
            elif event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

if __name__ == '__main__':
    print("\n\033[1m" + "******************************************" + "\033[0m")
    print("\033[1m" + "     A* Algorithm : Turtle Bot Burger     " + "\033[0m")
    print("\033[1m" + "******************************************" + "\033[0m")
    main()
