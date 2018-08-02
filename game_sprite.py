import os
import pygame
from math import tan, radians, degrees, copysign, ceil
from pygame.math import Vector2
import stackelbergPlayer as SCP
from random import randrange

WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
RED =   (255, 0, 0)
GREY = (210, 210 ,210)
PURPLE = (255, 0, 255)

WIDTH = 1900
HEIGHT = 360
# HEIGHT = 720
NUM_LANES = 3
LANE_WIDTH = int(HEIGHT/NUM_LANES)
ACTION_RESET_TIME = 0.25 # time till next action

ppu = 32
car_lane_ratio = 3.7/1.8
CAR_HEIGHT = int((HEIGHT/3.0)/car_lane_ratio)
CAR_WIDTH = int(CAR_HEIGHT*2)
# print(car_width, car_height)
# lane center positions
LANE_1_C = (LANE_WIDTH * 1 - (LANE_WIDTH/2))/ppu
LANE_2_C = (LANE_WIDTH * 2 - (LANE_WIDTH/2))/ppu
LANE_3_C = (LANE_WIDTH * 3 - (LANE_WIDTH/2))/ppu

NEW_LANES = [LANE_1_C, LANE_2_C, LANE_3_C]

# TEST:
lane_change_time = 0.0
lane_change_dist = 0.0

class Car(pygame.sprite.Sprite):
    def __init__(self, id, x, y, vel_x=0.0, vel_y=0.0, lane_id=1, color=RED, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):

        # init the sprite object
        super().__init__()

        self.id = id

        # Pass in the color of the car, and its x and y position, width and height.
        # Set the background color and set it to be transparent
        self.image = pygame.Surface([CAR_WIDTH, CAR_HEIGHT])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE)
 
        # Draw the car (a rectangle!)
        pygame.draw.rect(self.image, color, [0, 0, CAR_WIDTH, CAR_HEIGHT])
        
        # Instead we could load a proper pciture of a car...
        # self.image = pygame.image.load("car.png").convert_alpha()
 
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

    def update(self, dt, s_leader):
        
        if self.do_accelerate:
            self.accelerate(dt)
        elif self.do_decelerate:
            self.decelerate(dt)
        elif self.do_maintain:
            self.maintain(dt)

        self.velocity += (self.acceleration * dt, 0)
        self.velocity.x = max(0.0, min(self.velocity.x, self.max_velocity))
        # self.velocity.x = max(-self.max_velocity, min(self.velocity.x, self.max_velocity))

        # trigger movement
        new_lane_pos = (LANE_WIDTH * self.lane_id - (LANE_WIDTH/2))/ppu
        # print(new_lane_pos)
        if self.left_mode:
            self.moveLeft(dt, new_lane_pos)
        elif self.right_mode:
            self.moveRight(dt, new_lane_pos)

        if self.steering:
            turning_radius = self.length / tan(radians(self.steering))
            angular_velocity = self.velocity.x / turning_radius
        else:
            angular_velocity = 0

        # DEBUG: removed angle from car
        # TODO: added to make agent positions relative to eachother
        self.position += self.velocity.rotate(-self.angle) * dt
        self.position.y -= degrees(angular_velocity) * dt * dt

        if self.id == s_leader.id:
            self.position.x = 10
        else:
            self.position.x -= s_leader.velocity.x * dt

        # prevent the car from leaving the road
        if self.position.y < int((CAR_HEIGHT/2)/ppu):
            self.position.y = max(self.position.y, int((CAR_HEIGHT/2)/ppu))
        elif self.position.y > int((HEIGHT - int(CAR_HEIGHT/2))/ppu):
            self.position.y = min(self.position.y, int((HEIGHT - int((CAR_HEIGHT/2)/ppu))/ppu))

        # DEBUG: commented out the angle update
        # self.angle += degrees(angular_velocity) * dt

        # update rect for collision detection
        self.rect.x = self.position.x * ppu - self.rect.width / 2
        self.rect.y = self.position.y * ppu - self.rect.height / 2

    def setCruiseVel(self, cruise_vel):
        self.cruise_vel = cruise_vel

    def moveLeft(self, dt, new_lane_pos):
        self.steering += 30.0 * dt

        global lane_change_time, lane_change_dist
        # TODO: remove after testing 
        lane_change_time += dt

        if self.position.y <= new_lane_pos:
            self.steering = 0
            self.left_mode = False

            # TODO: remove after testing 
            lane_change_dist = self.position.x - lane_change_dist

    def moveRight(self, dt, new_lane_pos):
        self.steering -= 30.0 * dt

        global lane_change_time, lane_change_dist
        # TODO: remove after testing 
        lane_change_time += dt

        if self.position.y >= new_lane_pos:
            self.steering = 0
            self.right_mode = False

            # TODO: remove after testing 
            lane_change_dist = self.position.x - lane_change_dist

    def accelerate(self, dt):
        # the longitudinal velocity should never be less than 0
        if self.acceleration < 0.0:
            self.acceleration = 0.0 #self.brake_deceleration
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
        # if abs(self.velocity.x) > dt * self.brake_deceleration:
        #     self.acceleration = -self.brake_deceleration
        if self.acceleration > 0.0:
            self.acceleration = -self.max_acceleration #0.0
        else:
            # self.acceleration = -self.velocity.x / dt
            self.acceleration -= 1 * dt
        if self.velocity.x == 0.0:
            self.do_decelerate = False

class Obstacle(pygame.sprite.Sprite):
    def __init__(self, id, x, y, vel_x=0.0, vel_y=0.0, lane_id=1, color=RED, angle=0.0, length=4, max_steering=30, max_acceleration=5.0):

        # init the sprite object
        super().__init__()

        self.id = id

        # Pass in the color of the car, and its x and y position, width and height.
        # Set the background color and set it to be transparent
        self.image = pygame.Surface([CAR_WIDTH, CAR_HEIGHT])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE)
 
        # Draw the car (a rectangle!)
        pygame.draw.rect(self.image, color, [0, 0, CAR_WIDTH, CAR_HEIGHT])
        
        # Instead we could load a proper pciture of a car...
        # self.image = pygame.image.load("car.png").convert_alpha()
 
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
        text = font.render("accel: "+str(steering), True, WHITE)
        text_angle = font.render("velocity: "+str(car_angle), True, WHITE)
        self.screen.blit(text, (0,0))
        self.screen.blit(text_angle, (0,25))

    def displayPos(self, position):
        font = pygame.font.SysFont(None, 25)
        # text = font.render("X: "+str(position.x)+", Y: "+str(position.y), True, WHITE)
        text = font.render("Collisions: "+str(position), True, WHITE)
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
        # print(len(players))

        # Step 2. iterate over the set of players and execute their actions
        for leader in players:
            # Step 3: select actions for all players from step 2 sequentially
            selected_action = controller.selectAction(leader, all_obstacles)

            # TODO: test stackelberg copy:
            selected_action2 = controller.selectStackelbergAction(leader, all_obstacles, reference_car)

            self.executeAction(selected_action, leader, all_obstacles)

            # if selected_action != selected_action2:
            #     print("Different")
            # else:
            #     print("Different")


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

        # obstacle_velx = car.velocity.x
        # for obstacle in all_obstacles:
        #     if (obstacle.lane_id == car.lane_id) and (obstacle.position.x > car.position.x):
        #         obstacle_velx = obstacle.velocity.x

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
        global lane_change_time, lane_change_dist
        # TODO: remove after testing 
        lane_change_dist = car.position.x
        lane_change_time = 0.0

        car.lane_id = min(car.lane_id + 1, NUM_LANES)
        car.right_mode = True
        car.left_mode = False

    def turn_left(self, car):
        global lane_change_time, lane_change_dist
        # TODO: remove after testing 
        lane_change_dist = car.position.x
        lane_change_time = 0.0

        car.lane_id = max(car.lane_id - 1, 1)
        car.left_mode = True
        car.right_mode = False

    def run(self, cars_list, obstacle_list, is_Stackelberg=False, inf_obstacles=False):
        # current_dir = os.path.dirname(os.path.abspath(__file__))
        # image_path = os.path.join(current_dir, "car.png")
        # car_image = pygame.image.load(image_path)

        bkgd = pygame.image.load('roadImg.png').convert()
        bkgd = pygame.transform.scale(bkgd, (WIDTH, HEIGHT))
        bkgd_x = 0

        num_collisions = 0

        all_agents = pygame.sprite.Group()
        all_obstacles = pygame.sprite.Group()
        reference_car = None
        for data in cars_list:
            new_car = Car(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=data['vel_y'], lane_id=data['lane_id'])
            all_agents.add(new_car)
            all_obstacles.add(new_car)

            if not reference_car:
                reference_car = new_car

        # car = all_agents[0]

        # TODO: remove after testing
        test_counter = 0.0
        sprite_to_remove = None

        # obstacle_lanes = []
        all_coming_cars = pygame.sprite.Group()
        for data in obstacle_list:
            new_obstacle = Obstacle(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=0.0, lane_id=data['lane_id'], color=data['color'])
            all_coming_cars.add(new_obstacle)
            all_obstacles.add(new_obstacle)
            
            # obstacle_lanes.append(data['lane_id'])
        # obstacle_1 = Obstacle(x=30, y=2, vel_x=10.0, vel_y=0.0, color=GREY)

        for idx, obstacle in enumerate(all_coming_cars):
            if obstacle.lane_id == 2:
                sprite_to_remove = obstacle

        #This will be a list that will contain all the sprites we intend to use in our game.
        # all_sprites_list = pygame.sprite.Group()
        # Add the car to the list of objects
        # all_sprites_list.add(car)
        # all_sprites_list.add(obstacle_1)

        # Stackelberg controller
        s_controller = SCP.StackelbergPlayer(CAR_WIDTH) if is_Stackelberg else None

        # action timer
        action_timer = 0.0
        current_action = SCP.Action.MAINTAIN.name

        while not self.exit:
            dt = self.clock.get_time() / 1000

            action_timer += dt

            # Event queue
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.exit = True

            if not is_Stackelberg:
                self.manualControl(reference_car, all_obstacles)
            else:
                if action_timer >= ACTION_RESET_TIME:
                    selected_action = self.stackelbergControl(s_controller, reference_car, all_agents, all_obstacles)
                    current_action = selected_action.name
                    action_timer = 0.0

            # collision check
            for agent in all_agents:
                car_collision_list = pygame.sprite.spritecollide(agent,all_coming_cars,False)
                num_collisions += len(car_collision_list)
                # for accident in car_collision_list:
                #     num_collisions += 1
                    #End Of Game
                    # self.exit = True

            # update all sprites
            # car.update(dt)
            all_agents.update(dt, reference_car)
            all_coming_cars.update(dt, reference_car)

            # TODO: testing
            # test_counter += dt
            # if test_counter >= 5.0 and sprite_to_remove:
            #     all_coming_cars.remove(sprite_to_remove)
                # sprite_to_remove = None

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

            # update collision display count
            new_lane_pos = (LANE_WIDTH * reference_car.lane_id - (LANE_WIDTH/2))/ppu
            self.displayScore(reference_car.acceleration, reference_car.velocity.x)

            # update the agent sprites
            self.updateSprites(all_agents)
            # update obstacle sprites
            self.updateSprites(all_coming_cars)

            # display position of car
            self.displayPos(num_collisions)

            # display selected action
            self.displayAction(current_action)

            pygame.display.flip()

            self.clock.tick(self.ticks)
        pygame.quit()


if __name__ == '__main__':
    game = Game()

    # is_real = 15.0 == round(ceil(15.0023),3)
    # print(round(ceil(15.0003),3))
    # print(is_real)

    obstacle_1 = {'id':100, 'x':20, 'y':LANE_1_C, 'vel_x':13.0, 'lane_id':1, 'color':YELLOW}
    obstacle_2 = {'id':101, 'x':25, 'y':LANE_2_C, 'vel_x':12.0, 'lane_id':2, 'color':YELLOW}
    obstacle_3 = {'id':102, 'x':40, 'y':LANE_3_C, 'vel_x':10.0, 'lane_id':3, 'color':YELLOW}
    obstacle_list = [obstacle_1, obstacle_2, obstacle_3]

    car_1 = {'id':0, 'x':20, 'y':LANE_2_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':2}
    car_2 = {'id':1, 'x':5, 'y':LANE_1_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':1}
    car_3 = {'id':2, 'x':5, 'y':LANE_2_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':2}
    cars_list = [car_1, car_2, car_3]

    # run a Stackelberg game
    # game.run(cars_list, obstacle_list, True)

    # TODO: testing
    # players = [set() for x in range(3)]
    # for x in range(30):
    #     players[x%3].add(x)

    # for i in players:
    #     print(i)

    # del players[0]
    # print("\n Deleted first set")
    # print(players)
    # players.append(set())
    # print(players)
    # for i in players:
    #     print(i)

    # t = None
    # if t in players[0]:
    #     print("None")
    # else:
    #     print("Not none")

    # testRange = list(range(3))
    # print(testRange)
    # print(testRange[1:])
    # print(testRange[2:])


    # run a human controlled game
    game.run(cars_list, obstacle_list, True, True)
