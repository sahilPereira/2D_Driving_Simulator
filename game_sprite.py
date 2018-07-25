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

    def update(self, dt):
        
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
        # self.position += self.velocity.rotate(-self.angle) * dt
        self.position.y -= degrees(angular_velocity) * dt * dt
        self.position.x = 10

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
        
        # if is_speed and vel_ceil >= cruise_vel_ceil:
        #     self.velocity.x = self.cruise_vel
        #     self.acceleration = 0.0
        #     self.do_maintain = False
        # elif (not is_speed) and vel_ceil <= cruise_vel_ceil:
        #     self.velocity.x = self.cruise_vel
        #     self.acceleration = 0.0
        #     self.do_maintain = False
        # if vel_ceil == cruise_vel_ceil:
        #     self.acceleration = 0.0
        #     self.do_maintain = False

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
        text = font.render("X: "+str(position.x)+", Y: "+str(position.y), True, WHITE)
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

    def stackelbergControl(self, controller, all_car, all_obstacles):
        order_of_update = []
        # Step 1: select Stackelberg leader
        leader = None
        for idx, auto in enumerate(all_car):
            if leader is None:
                leader = auto
                order_of_update.append(idx)
            else:
                # leader is ahead of the other players
                if auto.position.x > leader.position.x:
                    leader = auto
                    order_of_update[0] = idx

        # Step 2: select up to 3 players involved in game
        # Skipped for now

        # Step 3: select actions for all players from step 2 sequentially
        selected_action = controller.selectAction(leader, all_obstacles)

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

        # Note that every player acts as a leader when selecting their actions
        return selected_action

    # these booleans are required to ensure the action is executed over a period of time
    def accelerate(self, car):
        car.do_accelerate = True
        car.do_decelerate = False
        car.do_maintain = False

    def maintain(self, car, all_obstacles):
        # only execute this function when required
        if car.do_maintain:
            return

        obstacle_velx = car.velocity.x
        for obstacle in all_obstacles:
            if (obstacle.lane_id == car.lane_id) and (obstacle.position.x > car.position.x):
                obstacle_velx = obstacle.velocity.x
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

        agents = []
        for data in cars_list:
            agents.append(Car(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=data['vel_y'], lane_id=data['lane_id']))
        # car = Car(x=10, y=9.375, vel_x=10.0, vel_y=0.0, lane_id=3)
        car = agents[0]

        # TODO: remove after testing
        test_counter = 0.0
        sprite_to_remove = None

        obstacle_lanes = []
        all_coming_cars = pygame.sprite.Group()
        for data in obstacle_list:
            all_coming_cars.add(Obstacle(id=data['id'], x=data['x'], y=data['y'], vel_x=data['vel_x'], vel_y=0.0, lane_id=data['lane_id'], color=data['color']))
            obstacle_lanes.append(data['lane_id'])
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
        s_controller = SCP.StackelbergPlayer() if is_Stackelberg else None

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
                self.manualControl(car, all_coming_cars)
            else:
                if action_timer >= ACTION_RESET_TIME:
                    selected_action = self.stackelbergControl(s_controller, [car], all_coming_cars)
                    current_action = selected_action.name
                    action_timer = 0.0

            # Logic
            car_collision_list = pygame.sprite.spritecollide(car,all_coming_cars,False)
            for accident in car_collision_list:
                num_collisions += 1
                #End Of Game
                # self.exit = True

            # update all sprites
            car.update(dt)
            all_coming_cars.update(dt, car)

            # TODO: testing
            # test_counter += dt
            # if test_counter >= 5.0 and sprite_to_remove:
            #     all_coming_cars.remove(sprite_to_remove)
                # sprite_to_remove = None

            # generate new obstacles
            for obstacle in all_coming_cars:
                if obstacle.position.x < -CAR_WIDTH/32:
                    # remove old obstacle
                    all_coming_cars.remove(obstacle)
                    # obstacle_lanes.remove(obstacle.lane_id)

                    # add new obstacle
                    rand_pos_x = float(randrange(70, 80))
                    rand_pos_y = NEW_LANES[obstacle.lane_id-1]
                    rand_vel_x = float(randrange(5, 15))
                    rand_lane_id = obstacle.lane_id
                    all_coming_cars.add(Obstacle(id=randrange(1,100), x=rand_pos_x, y=rand_pos_y, vel_x=rand_vel_x, vel_y=0.0, lane_id=rand_lane_id, color=YELLOW))
                    # obstacle_lanes.append(data['lane_id'])




            # Drawing
            self.screen.fill((0, 0, 0))
            
            #Draw The Scrolling Road
            rel_x = bkgd_x % bkgd.get_rect().width
            self.screen.blit(bkgd, (rel_x - bkgd.get_rect().width, 0))
            if rel_x < WIDTH:
                self.screen.blit(bkgd, (rel_x, 0))
            bkgd_x -= car.velocity.x
            # bkgd_x -= 1
            # pygame.draw.line(self.screen, (255, 0, 0), (rel_x, 0), (rel_x, HEIGHT), 3)

            # update collision display count
            new_lane_pos = (LANE_WIDTH * car.lane_id - (LANE_WIDTH/2))/ppu
            self.displayScore(car.acceleration, car.velocity.x)

            # update the agent sprites
            self.updateSprites([car])
            # update obstacle sprites
            self.updateSprites(all_coming_cars)

            # display position of car
            self.displayPos(car.position)

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

    obstacle_1 = {'id':0, 'x':20, 'y':LANE_1_C, 'vel_x':11.0, 'lane_id':1, 'color':YELLOW}
    obstacle_2 = {'id':1, 'x':25, 'y':LANE_2_C, 'vel_x':10.5, 'lane_id':2, 'color':YELLOW}
    obstacle_3 = {'id':2, 'x':40, 'y':LANE_3_C, 'vel_x':10.0, 'lane_id':3, 'color':YELLOW}
    obstacle_list = [obstacle_1, obstacle_2, obstacle_3]

    car_1 = {'id':0, 'x':20, 'y':LANE_2_C, 'vel_x':10.0, 'vel_y':0.0, 'lane_id':2}
    cars_list = [car_1]

    # run a Stackelberg game
    # game.run(cars_list, obstacle_list, True)

    # run a human controlled game
    game.run(cars_list, obstacle_list, True, True)
