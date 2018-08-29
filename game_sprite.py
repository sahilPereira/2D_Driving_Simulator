import os
import pygame
from math import tan, radians, degrees, copysign, ceil
from pygame.math import Vector2
import stackelbergPlayer as SCP
from random import randrange
import random
import math
import numpy
import pandas as pd
import pickle
from argparse import ArgumentParser

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED =   (255, 0, 0)
GREY = (210, 210 ,210)
PURPLE = (255, 0, 255)

WIDTH = 1900
HEIGHT = 240
NUM_LANES = 3
LANE_WIDTH = int(HEIGHT/NUM_LANES)
ACTION_RESET_TIME = 0.25 # time till next action
NGSIM_RESET_TIME = 0.1

ppu = 32
car_lane_ratio = 3.7/1.8
CAR_HEIGHT = int((HEIGHT/3.0)/car_lane_ratio)
CAR_WIDTH = int(CAR_HEIGHT*2)

# lane center positions
LANE_1_C = (LANE_WIDTH * 1 - (LANE_WIDTH/2))/ppu
LANE_2_C = (LANE_WIDTH * 2 - (LANE_WIDTH/2))/ppu
LANE_3_C = (LANE_WIDTH * 3 - (LANE_WIDTH/2))/ppu

NEW_LANES = [LANE_1_C, LANE_2_C, LANE_3_C]


class Car(pygame.sprite.Sprite):
    def __init__(self, id, x, y, vel_x=0.0, vel_y=0.0, lane_id=1, color=RED, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):

        # init the sprite object
        super().__init__()

        self.id = id
        self.image = pygame.Surface([CAR_WIDTH, CAR_HEIGHT])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE)
 
        # Draw the car (a rectangle!)
        pygame.draw.rect(self.image, color, [0, 0, CAR_WIDTH, CAR_HEIGHT])
 
        # Fetch the rectangle object that has the dimensions of the image.
        self.rect = self.image.get_rect()

        self.position = Vector2(x, y)
        self.velocity = Vector2(vel_x, vel_y)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 20.0
        self.brake_deceleration = 10.0
        self.free_deceleration = 2.0

        self.acceleration = 0.0
        self.steering = 0.0

        self.angular_velocity = 0.0

        self.lane_id = lane_id

        self.left_mode, self.right_mode, self.do_accelerate, self.do_decelerate, self.do_maintain = False, False, False, False, False
        self.cruise_vel = 0.0

    def simCopy(self):
        sim_car = Car(self.id, self.position.x, self.position.y, self.velocity.x, self.velocity.y, self.lane_id)
        # dynamic controls
        sim_car.acceleration = self.acceleration
        sim_car.steering = self.steering
        # action controls
        sim_car.left_mode = self.left_mode
        sim_car.right_mode = self.right_mode
        sim_car.do_accelerate = self.do_accelerate
        sim_car.do_decelerate = self.do_decelerate
        sim_car.do_maintain = self.do_maintain
        sim_car.cruise_vel = self.cruise_vel

        return sim_car

    def updateNgsim(self, dt):

        if self.do_accelerate:
            self.accelerate(dt)
        elif self.do_decelerate:
            self.decelerate(dt)
        elif self.do_maintain:
            self.maintain(dt)

        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        # trigger movement
        if self.left_mode:
            self.steering += 30.0 * dt
        elif self.right_mode:
            self.steering -= 30.0 * dt

        if self.steering:
            turning_radius = self.length / tan(radians(self.steering))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        # self.angle += degrees(angular_velocity) * dt

    def update(self, dt, s_leader):
        
        if self.do_accelerate:
            self.accelerate(dt)
        elif self.do_decelerate:
            self.decelerate(dt)
        elif self.do_maintain:
            self.maintain(dt)

        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(0.0, min(self.velocity.x, self.max_velocity))

        # trigger movement
        new_lane_pos = (LANE_WIDTH * self.lane_id - (LANE_WIDTH/2))/ppu
        # print(new_lane_pos)
        if self.left_mode:
            self.moveLeft(dt, new_lane_pos)
        elif self.right_mode:
            self.moveRight(dt, new_lane_pos)

        if self.steering:
            turning_radius = self.length / tan(radians(self.steering))
            self.angular_velocity = self.velocity.x / turning_radius
        else:
            self.angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.position.y -= degrees(self.angular_velocity) * dt * dt

        if self.id == s_leader.id:
            self.position.x = 10
        else:
            self.position.x -= s_leader.velocity.x * dt

        # prevent the car from leaving the road
        if self.position.y < int((CAR_HEIGHT/2)/ppu):
            self.position.y = max(self.position.y, int((CAR_HEIGHT/2)/ppu))
        elif self.position.y > int((HEIGHT - int(CAR_HEIGHT/2))/ppu):
            self.position.y = min(self.position.y, int((HEIGHT - int((CAR_HEIGHT/2)/ppu))/ppu))

        # update rect for collision detection
        self.rect.x = self.position.x * ppu - self.rect.width / 2
        self.rect.y = self.position.y * ppu - self.rect.height / 2

    def setCruiseVel(self, cruise_vel):
        self.cruise_vel = cruise_vel

    def moveLeft(self, dt, new_lane_pos):
        self.steering += 30.0 * dt

        if self.position.y <= new_lane_pos:
            self.steering = 0
            self.left_mode = False

    def moveRight(self, dt, new_lane_pos):
        self.steering -= 30.0 * dt

        if self.position.y >= new_lane_pos:
            self.steering = 0
            self.right_mode = False

    def accelerate(self, dt):
        # the longitudinal velocity should never be less than 0
        if self.acceleration < 0.0:
            self.acceleration = 0.0
        else:
            self.acceleration += 1 * dt
        if self.acceleration == self.max_acceleration:
            self.do_accelerate = False

    def maintain(self, dt):
        vel_ceil = ceil(self.velocity.x)
        cruise_vel_ceil = ceil(self.cruise_vel)

        # check if car needs to speed up or slow down and accelerate accordingly
        is_speed = True if vel_ceil <= cruise_vel_ceil else False
        self.acceleration = self.max_acceleration if is_speed else -self.max_acceleration

        # speed up or slow down until the car reaches cruise velocity
        is_cruise_speed = is_speed and vel_ceil >= cruise_vel_ceil
        is_cruise_speed |= (not is_speed) and vel_ceil <= cruise_vel_ceil

        if is_cruise_speed:
            self.velocity.x = self.cruise_vel
            self.acceleration = 0.0
            self.do_maintain = False

    def decelerate(self, dt):
        if self.acceleration > 0.0:
            self.acceleration = -self.max_acceleration #0.0
        else:
            self.acceleration -= 1 * dt
        if self.velocity.x == 0.0:
            self.do_decelerate = False

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, id, x, y, vel_x=0.0, vel_y=0.0, lane_id=1, color=RED, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):

        # init the sprite object
        super().__init__()

        self.id = id
        self.image = pygame.Surface([CAR_WIDTH, CAR_HEIGHT])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE)
 
        # Draw the car (a rectangle!)
        pygame.draw.rect(self.image, color, [0, 0, CAR_WIDTH, CAR_HEIGHT])
 
        # Fetch the rectangle object that has the dimensions of the image.
        self.rect = self.image.get_rect()

        self.position = Vector2(x, y)
        self.velocity = Vector2(vel_x, vel_y)
        self.angle = angle
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_steering = max_steering
        self.max_velocity = 20
        self.brake_deceleration = 10
        # self.free_deceleration = 2
        self.lane_id = lane_id

        self.acceleration = 0.0
        self.steering = 0.0

    def simCopy(self):
        sim_car = Obstacle(self.id, self.position.x, self.position.y, self.velocity.x, self.velocity.y, self.lane_id, color=YELLOW)
        # dynamic controls
        sim_car.acceleration = self.acceleration
        sim_car.steering = self.steering

        return sim_car

    def update(self, dt, s_leader):
        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        if self.steering:
            turning_radius = self.length / tan(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        self.position += self.velocity.rotate(-self.angle) * dt
        self.position.x -= s_leader.velocity.x * dt
        self.angle += degrees(angular_velocity) * dt

        # update rect for collision detection
        self.rect.x = self.position.x * ppu - self.rect.width / 2
        self.rect.y = self.position.y * ppu - self.rect.height / 2

class Game:
    def __init__(self):
        pygame.init()
        pygame.display.set_caption("Car tutorial")
        width = WIDTH
        height = HEIGHT
        self.screen = pygame.display.set_mode((width, height))
        self.clock = pygame.time.Clock()
        self.ticks = 60
        self.exit = False

    def displayScore(self, steering, car_angle):
        font = pygame.font.SysFont(None, 25)
        text = font.render("Collision [obstacle]: "+str(steering), True, WHITE)
        text_angle = font.render("Collision [agent]: "+str(car_angle), True, WHITE)
        self.screen.blit(text, (0,0))
        self.screen.blit(text_angle, (0,25))

    def displayPos(self, position):
        font = pygame.font.SysFont(None, 25)
        # text = font.render("X: "+str(position.x)+", Y: "+str(position.y), True, WHITE)
        text = font.render("Velocity: "+str(position), True, WHITE)
        self.screen.blit(text, (0,50))

    def displayAction(self, action):
        font = pygame.font.SysFont(None, 25)
        text = font.render("Action: "+str(action), True, WHITE)
        self.screen.blit(text, (0,75))

    def updateSprites(self, vehicles):
        for auto in vehicles:
            rotated = pygame.transform.rotate(auto.image, auto.angle)
            rect = rotated.get_rect()
            self.screen.blit(rotated, auto.position * ppu - (rect.width / 2, rect.height / 2))

    def manualControl(self, car, all_obstacles):
        # User input
        pressed = pygame.key.get_pressed()

        if pressed[pygame.K_UP] and not car.do_accelerate:
            self.accelerate(car)
        elif pressed[pygame.K_DOWN] and not car.do_maintain:
            self.maintain(car, all_obstacles)
        elif pressed[pygame.K_SPACE] and not car.do_decelerate:
            self.decelerate(car)

        car.acceleration = max(-car.max_acceleration, min(car.acceleration, car.max_acceleration))

        if pressed[pygame.K_RIGHT] and not car.right_mode:
            self.turn_right(car)
        elif pressed[pygame.K_LEFT] and not car.left_mode:
            self.turn_left(car)

        car.steering = max(-car.max_steering, min(car.steering, car.max_steering))

    def stackelbergControl(self, controller, reference_car, all_agents, all_obstacles):

        # Step 1. select players to execute action at this instance
        players = controller.pickLeadersAndFollowers(all_agents, all_obstacles)

        # Step 2. iterate over the set of players and execute their actions
        for leader in players:
            # Step 3: select actions for all players from step 2 sequentially
            selected_action = controller.selectAction(leader, all_obstacles)

            # select action using Stackelberg game
            # selected_action = controller.selectStackelbergAction(leader, all_obstacles, reference_car)

            self.executeAction(selected_action, leader, all_obstacles)

        # Note that every player acts as a leader when selecting their actions
        return selected_action

    # execute the given action for the specified leader
    def executeAction(self, selected_action, leader, all_obstacles):
        if (selected_action == SCP.Action.ACCELERATE) and not leader.do_accelerate:
            self.accelerate(leader)
        elif (selected_action == SCP.Action.DECELERATE) and not leader.do_decelerate:
            self.decelerate(leader)
        elif (selected_action == SCP.Action.MAINTAIN) and not leader.do_maintain:
            self.maintain(leader, all_obstacles)

        leader.acceleration = max(-leader.max_acceleration, min(leader.acceleration, leader.max_acceleration))

        if (selected_action == SCP.Action.RIGHT) and not leader.right_mode:
            self.turn_right(leader)
        elif (selected_action == SCP.Action.LEFT) and not leader.left_mode:
            self.turn_left(leader)

        leader.steering = max(-leader.max_steering, min(leader.steering, leader.max_steering))
        
        return

    # these booleans are required to ensure the action is executed over a period of time
    def accelerate(self, car):
        car.do_accelerate = True
        car.do_decelerate = False
        car.do_maintain = False

    def maintain(self, car, all_obstacles):
        # only execute this function when required
        if car.do_maintain:
            return

        forward_obstacle = None
        for obstacle in all_obstacles:
            if obstacle == car:
                continue
            # obstacle in the same lane
            if obstacle.lane_id == car.lane_id:
                # obstacle has to be ahead the ego vehicle
                if obstacle.position.x > car.position.x:
                    if not forward_obstacle:
                        forward_obstacle = obstacle
                    # obstacle closest to the front
                    elif obstacle.position.x < forward_obstacle.position.x:
                        forward_obstacle = obstacle
        
        obstacle_velx = forward_obstacle.velocity.x if forward_obstacle else car.velocity.x
        car.setCruiseVel(obstacle_velx)
        car.do_maintain = True
        car.do_accelerate = False
        car.do_decelerate = False

    def decelerate(self, car):
        car.do_decelerate = True
        car.do_accelerate = False
        car.do_maintain = False

    def turn_right(self, car):
        car.lane_id = min(car.lane_id + 1, NUM_LANES)
        car.right_mode = True
        car.left_mode = False

    def turn_left(self, car):
        car.lane_id = max(car.lane_id - 1, 1)
        car.left_mode = True
        car.right_mode = False

    def run(self, cars_list, obstacle_list, is_manual=False, inf_obstacles=False, is_data_saved=False):

        bkgd = pygame.image.load('roadImg.png').convert()
        bkgd = pygame.transform.scale(bkgd, (WIDTH, HEIGHT))
        bkgd_x = 0

        TOTAL_AGENTS = 5
        TOTAL_RUNS = 100
        RUN_DURATION = 60 # 60 seconds

        files_dir = 'datafiles/NoDelay/dt_0.05/'
        file_name = files_dir+'model_IGA_0.05_%d_%d_%d.csv'%(TOTAL_AGENTS, TOTAL_RUNS, RUN_DURATION)

        df_columns = ['agent_count','run_count','obs_collisions','agent_collisions', 'avg_velocity', 'avg_distance']
        position_columns = ['agent_id','time','pos_x','pos_y']

        # track data across multiple runs and agents
        all_data = [[] for x in range(6)]

        for agent_count in range(TOTAL_AGENTS):

            # track data across multiple runs
            data_per_run = [[] for x in range(6)]
            position_tracker = [[] for x in range(4)]

            backup_file_name = files_dir+'backup_IGA_0.05_%d_%d_%d.csv'%(agent_count+1, TOTAL_RUNS, RUN_DURATION)
            position_file_name = files_dir+'pos_IGA_0.05_%d_%d.csv'%(agent_count+1, RUN_DURATION)

            for run_count in range(TOTAL_RUNS):
                # Stackelberg controller
                s_controller = SCP.StackelbergPlayer(CAR_WIDTH) if not is_manual else None

                all_agents = pygame.sprite.Group()
                all_obstacles = pygame.sprite.Group()
                all_coming_cars = pygame.sprite.Group()

                reference_car = None
                for data in cars_list[:agent_count+1]:
                # for data in cars_list[:3]:
                    new_car = Car(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=data['vel_y'], lane_id=data['lane_id'])
                    all_agents.add(new_car)
                    all_obstacles.add(new_car)

                    if not reference_car:
                        reference_car = new_car

                for data in obstacle_list:
                    new_obstacle = Obstacle(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=0.0, lane_id=data['lane_id'], color=data['color'])
                    all_coming_cars.add(new_obstacle)
                    all_obstacles.add(new_obstacle)

                action_timer = 0.0
                log_timer = 0.0
                continuous_time = 0.0
                current_action = SCP.Action.MAINTAIN.name
                num_obs_collisions = 0
                num_agent_collisions = 0

                total_velocity_per_run = []
                total_distance_per_run = []

                collision_count_lock = True
                run_time = 0.0

                is_paused = False

                # while not self.exit:
                while run_time <= RUN_DURATION and not self.exit:
                    dt = self.clock.get_time() / 1000
                    # dt = 50.0/1000.0
                    
                    # pause game when needed
                    for e in pygame.event.get():
                        if e.type == pygame.QUIT:
                            self.exit = True
                        if e.type == pygame.KEYDOWN:
                            if e.key == pygame.K_p: is_paused = True
                            if e.key == pygame.K_r: is_paused = False

                    if is_paused:
                        pygame.display.flip()
                        self.clock.tick(self.ticks)
                        continue

                    action_timer += dt
                    log_timer += dt
                    run_time += dt
                    continuous_time += dt

                    # Event queue
                    for event in pygame.event.get():
                        if event.type == pygame.QUIT:
                            self.exit = True

                    if is_manual:
                        self.manualControl(reference_car, all_obstacles)
                    else:
                        if action_timer >= ACTION_RESET_TIME:
                            selected_action = self.stackelbergControl(s_controller, reference_car, all_agents, all_obstacles)
                            current_action = selected_action.name
                            action_timer = 0.0

                    # collision check
                    if not collision_count_lock:
                        for agent in all_agents:
                            # get collisions with non reactive obstacles
                            car_collision_list = pygame.sprite.spritecollide(agent,all_coming_cars,False)
                            num_obs_collisions += len(car_collision_list)

                            collision_group = all_agents.copy()
                            collision_group.remove(agent)
                            car_collision_list = pygame.sprite.spritecollide(agent,collision_group,False)
                            num_agent_collisions += len(car_collision_list)

                    # update all sprites
                    all_agents.update(dt, reference_car)
                    all_coming_cars.update(dt, reference_car)

                    # log the velocity and distance travelled
                    if log_timer >= ACTION_RESET_TIME:
                        for agent in all_agents:
                            # log current velocity for each agent
                            total_velocity_per_run.append(agent.velocity.x)                    
                            # log distance travelled in ACTION_RESET_TIME at current velocity
                            total_distance_per_run.append(agent.velocity.x * ACTION_RESET_TIME)

                            if run_count == TOTAL_RUNS-1:
                                position_tracker[0].append(agent.id)
                                position_tracker[1].append(continuous_time)
                                position_tracker[2].append(agent.position.x)
                                position_tracker[3].append(agent.position.y)

                        log_timer = 0.0

                    sorted_agents = sorted(all_agents, key=lambda x: x.position.x, reverse=True)
                    reference_car = sorted_agents[0]

                    # generate new obstacles
                    if inf_obstacles:
                        for obstacle in all_coming_cars:
                            if obstacle.position.x < -CAR_WIDTH/32:
                                # remove old obstacle
                                all_coming_cars.remove(obstacle)
                                all_obstacles.remove(obstacle)
                                # obstacle_lanes.remove(obstacle.lane_id)

                                # add new obstacle
                                rand_pos_x = float(randrange(70, 80))
                                rand_pos_y = NEW_LANES[obstacle.lane_id-1]
                                rand_vel_x = float(randrange(5, 15))
                                rand_lane_id = obstacle.lane_id

                                new_obstacle = Obstacle(id=randrange(100,1000), x=rand_pos_x, y=rand_pos_y, vel_x=rand_vel_x, vel_y=0.0, lane_id=rand_lane_id, color=YELLOW)
                                all_coming_cars.add(new_obstacle)
                                all_obstacles.add(new_obstacle)

                    # Drawing
                    self.screen.fill((0, 0, 0))
                    
                    #Draw The Scrolling Road
                    rel_x = bkgd_x % bkgd.get_rect().width
                    self.screen.blit(bkgd, (rel_x - bkgd.get_rect().width, 0))
                    if rel_x < WIDTH:
                        self.screen.blit(bkgd, (rel_x, 0))
                    bkgd_x -= reference_car.velocity.x
                    # bkgd_x -= 1
                    # pygame.draw.line(self.screen, (255, 0, 0), (rel_x, 0), (rel_x, HEIGHT), 3)


                    # update the agent sprites
                    self.updateSprites(all_agents)
                    # update obstacle sprites
                    self.updateSprites(all_coming_cars)

                    # update collision display count
                    # new_lane_pos = (LANE_WIDTH * reference_car.lane_id - (LANE_WIDTH/2))/ppu
                    self.displayScore(num_obs_collisions, num_agent_collisions)

                    # display position of car
                    self.displayPos(reference_car.velocity.x)

                    # display selected action
                    # self.displayAction(current_action)

                    pygame.display.flip()

                    collision_count_lock = False

                    self.clock.tick(self.ticks)
                    # self.clock.tick()

                if not self.exit:
                    # log the number of agents and the current run count
                    data_per_run[0].append(agent_count+1)
                    data_per_run[1].append(run_count)
                    # total number of collisions for one run
                    data_per_run[2].append(num_obs_collisions)
                    data_per_run[3].append(num_agent_collisions)
                    # average velocity and distance across one run
                    data_per_run[4].append(sum(total_velocity_per_run) / float(len(total_velocity_per_run)))
                    data_per_run[5].append(sum(total_distance_per_run) / float(len(total_distance_per_run)))

            # log data across multiple runs and agents
            all_data[0].extend(data_per_run[0])
            all_data[1].extend(data_per_run[1])
            all_data[2].extend(data_per_run[2])
            all_data[3].extend(data_per_run[3])
            all_data[4].extend(data_per_run[4])
            all_data[5].extend(data_per_run[5])

            # save backups for each agent count in case of unexpected termination
            if not self.exit and is_data_saved:
                np_data_backup = numpy.array(data_per_run)
                np_data_backup = numpy.transpose(np_data_backup)
                np_data_backup_df = pd.DataFrame(np_data_backup, columns = df_columns)
                np_data_backup_df.to_csv(backup_file_name, index=False)

                # save position data for last run
                np_pos_data = numpy.array(position_tracker)
                np_pos_data = numpy.transpose(np_pos_data)
                np_pos_data_df = pd.DataFrame(np_pos_data, columns = position_columns)
                np_pos_data_df.to_csv(position_file_name, index=False)
                

        if not self.exit and is_data_saved:
            np_data_per_run = numpy.array(all_data)
            np_data_per_run = numpy.transpose(np_data_per_run)

            np_data_per_run_df = pd.DataFrame(np_data_per_run, columns = df_columns)
            np_data_per_run_df.to_csv(file_name, index=False)

        pygame.quit()

    def initObjects(self, data_point):

        left_lane_idx = [4,16,28]
        right_lane_idx = [12,14,32]

        all_coming_cars = pygame.sprite.Group()

        ego_car = Car(id=1, x=data_point[1], y=data_point[0], vel_x=data_point[3], vel_y=data_point[2], lane_id=2) # assume center lane for each case
        ego_car.max_velocity = 500.0
        ego_car.max_acceleration = 30.0

        for i in range(4, len(data_point), 4):

            # if obstacle has the same x and y positions as the ego, ignore this obstacle
            if data_point[i+3] == ego_car.position.x and data_point[i+2] == ego_car.position.y:
                continue

            laneId = 2 # center lane by default
            if i in left_lane_idx:
                laneId = 1
            elif i in right_lane_idx:
                laneId = 3

            new_obstacle = Obstacle(id=i*10, x=data_point[i+3], y=data_point[i+2], vel_x=data_point[i+1], vel_y=data_point[i], lane_id=laneId, color=YELLOW)
            new_obstacle.max_velocity = 500.0
            new_obstacle.max_acceleration = 30.0
            all_coming_cars.add(new_obstacle)

        return ego_car, all_coming_cars

    def relativeToAbsolute(self, data):
        # convert data to a dataframe
        data_df = pd.DataFrame(data)

        relative_idx = numpy.concatenate([numpy.arange(5, 8), numpy.arange(9, 12), \
            numpy.arange(13, 16), numpy.arange(17, 20), numpy.arange(21, 24), numpy.arange(25, 28), \
            numpy.arange(29, 32), numpy.arange(33, 36), numpy.arange(37, 40)]).ravel()

        for i in range(0, len(relative_idx), 3):
            data_df.loc[:,relative_idx[i]] = data_df.loc[:,3]-data_df.loc[:,relative_idx[i]]
            data_df.loc[:,relative_idx[i]+1] = data_df.loc[:,relative_idx[i]+1] + data_df.loc[:,0]
            data_df.loc[:,relative_idx[i]+2] = data_df.loc[:,relative_idx[i]+2] + data_df.loc[:,1]
        
        return data_df

    def runNgsim(self, ngsim_data):
        random.seed(2018)
        testing_sets = random.sample(range(len(ngsim_data)), 100)
        print(testing_sets)

        # Stackelberg controller
        s_controller = SCP.StackelbergPlayer(CAR_WIDTH)

        # holds average errors across each set of 100 samples
        set_errors = [[] for x in range(4)]

        for ts in testing_sets:

            # Step 1: load the 100 samples as a dataframe
            data_df = self.relativeToAbsolute(ngsim_data[ts])

            # Step 2: init ego and obstacles using sample[0]
            ego_car, all_coming_cars = self.initObjects(data_df.loc[0])
            all_obstacles = all_coming_cars.copy()
            all_obstacles.add(ego_car)

            # errors for x, y, vx, vy
            errors = [[] for x in range(4)]

            # for i in range(1, data_df.shape[0]):
            for i in range(10, data_df.shape[0], 10):
                
                # Step 3: get action for ego and project it 0.1s into future
                selected_action = s_controller.selectAction(ego_car, all_obstacles)

                # select action using Stackelberg game
                # s_controller.playerSets[ego_car] = [ego_car]
                # selected_action = s_controller.selectStackelbergAction(ego_car, all_obstacles, ego_car)

                # TODO: test commenting this section out
                self.executeAction(selected_action, ego_car, all_obstacles)
                # ego_car.updateNgsim(NGSIM_RESET_TIME)
                ego_car.updateNgsim(1.0)

                # Step 4: check state of ego with current sample
                new_ego_car, all_coming_cars = self.initObjects(data_df.loc[i])
                all_obstacles = all_coming_cars.copy()
                all_obstacles.add(new_ego_car)

                # Step 5: save error for x, y, vx, vy
                errors[0].append(abs(ego_car.position.x - new_ego_car.position.x)/abs(new_ego_car.position.x))
                errors[1].append(abs(ego_car.position.y - new_ego_car.position.y)/abs(new_ego_car.position.y))
                errors[2].append(abs(ego_car.velocity.x - new_ego_car.velocity.x)/max(abs(new_ego_car.velocity.x),1.0))
                errors[3].append(abs(ego_car.angular_velocity - new_ego_car.velocity.y)/max(abs(new_ego_car.velocity.y),1.0))

                # reset ego_car for next sample
                ego_car = new_ego_car

            # Step 6: average errors for each field at end
            set_errors[0].append(sum(errors[0]) / float(len(errors[0])))
            set_errors[1].append(sum(errors[1]) / float(len(errors[1])))
            set_errors[2].append(sum(errors[2]) / float(len(errors[2])))
            set_errors[3].append(sum(errors[3]) / float(len(errors[3])))

        np_set_errors = numpy.array(set_errors)
        np_set_errors = numpy.transpose(np_set_errors)

        np_set_errors_df = pd.DataFrame(np_set_errors, columns = ['e_x','e_y','e_vx','e_vy'])
        np_set_errors_df.to_csv('ngsim_errors_GIA_AV_1.0.csv', index=False)

# Load only the features required for evaluation
def loadNgsimData():

    features = None
    with open('features_test_0.data', 'rb') as file:
        features = pickle.load(file, encoding='latin1')

    # Indices of spatial data that needs to have its magnitude reduced.
    indices = numpy.concatenate([numpy.arange(0, 4), numpy.arange(5, 9), numpy.arange(11, 15), \
        numpy.arange(17, 21), numpy.arange(23, 27), numpy.arange(29, 33), numpy.arange(35, 39), \
        numpy.arange(41, 45), numpy.arange(47, 51), numpy.arange(53, 57)]).ravel()

    filteredData = features[:, :, indices]
    return filteredData

if __name__ == '__main__':

    # parse arguments
    parser = ArgumentParser()
    parser.add_argument("--manual", dest="manual", action='store_true', help="use manual driving mode; default is using Stackelberg driving model")
    parser.add_argument("--inf_obs", dest="inf_obs", action='store_true', help="produce new obstacle on a lane when current obstacle is out of window")
    parser.add_argument("--save", dest="save_data", action='store_true', help="save performance metrics")

    args = parser.parse_args()

    # initial positions of obstacles and agents
    obstacle_1 = {'id':100, 'x':-20, 'y':LANE_1_C, 'vel_x':13.0, 'lane_id':1, 'color':YELLOW}
    obstacle_2 = {'id':101, 'x':-25, 'y':LANE_2_C, 'vel_x':12.0, 'lane_id':2, 'color':YELLOW}
    obstacle_3 = {'id':102, 'x':-40, 'y':LANE_3_C, 'vel_x':10.0, 'lane_id':3, 'color':YELLOW}
    obstacle_list = [obstacle_1, obstacle_2, obstacle_3]

    car_1 = {'id':0, 'x':20, 'y':LANE_2_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':2}
    car_2 = {'id':1, 'x':5, 'y':LANE_1_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':1}
    car_3 = {'id':2, 'x':5, 'y':LANE_2_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':2}
    car_4 = {'id':3, 'x':20, 'y':LANE_3_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':3}
    car_5 = {'id':4, 'x':5, 'y':LANE_3_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':3}
    cars_list = [car_1, car_2, car_3, car_4, car_5]

    game = Game()
    # run the simulation
    game.run(cars_list, obstacle_list, args.manual, args.inf_obs, args.save_data)
