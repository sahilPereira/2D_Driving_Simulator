import os
import pygame
from math import tan, radians, degrees, copysign
from pygame.math import Vector2
from enum import Enum

# TODO: probably set these values during init
VISIBLE_DIST = 2000 # pixels
LANE_DIFF = 120 # 120 pixels between centers of lanes

class Action(Enum):
    LEFT = 0
    RIGHT = 1
    MAINTAIN = 2
    ACCELERATE = 3
    DECELERATE = 4

class StackelbergPlayer():
    def __init__(self):
        pass

    def selectAction(self, leader, all_obstacles):
        current_lane = leader.lane_id
        all_actions = list(Action)

        # if in the left lane remove left action, same with right lane
        if current_lane == 1:
            del all_actions[Action.LEFT.value]
        elif current_lane == 3:
            del all_actions[Action.RIGHT.value]

        best_utility = 0.0
        selected_action = None
        for action in all_actions:
            # update the intended lane
            intended_lane = current_lane
            if action == Action.LEFT:
                intended_lane -= 1
            elif action == Action.RIGHT:
                intended_lane += 1

            # compute utility for the current action
            current_utility = self.positiveUtility(leader, intended_lane, all_obstacles)
            current_utility += self.negativeUtility(leader, intended_lane, all_obstacles)
            if current_utility > best_utility:
                best_utility = current_utility
                selected_action = action        
        
        return selected_action

    def pickPlayers(self):
        return 0

    # TODO: start with the simple positive utility
    def positiveUtility(self, ego, intended_lane, all_obstacles):
        # max visible distance
        ideal_velocity = ego.max_velocity
        for obstacle in all_obstacles:
            if obstacle.lane_id == intended_lane:
                dx = obstacle.position.x - ego.position.x
                # only consider vehicles ahead of ego vehicle
                if dx > 0:
                    stopping_dist = self.stoppingDist(ego)
                    tmp_val = dx + ego.velocity.x + min(dx - stopping_dist, 0)
                    ideal_velocity = min(tmp_val, VISIBLE_DIST + ideal_velocity)
        return ideal_velocity

    # compute stopping distance for ego vehicle
    def stoppingDist(self, ego):
        return 0.5*(ego.velocity.x ** 2)/ego.max_acceleration

    # TODO: start with the simple positive utility
    def positiveUtility_old(self, ego, intended_lane, all_obstacles):
        # max visible distance
        obstacle_dist = VISIBLE_DIST
        for obstacle in all_obstacles:
            if obstacle.lane_id == intended_lane:
                dx = obstacle.position.x - ego.position.x
                # only consider vehicles ahead of ego vehicle
                if dx > 0:
                    obstacle_dist = min(dx, obstacle_dist)
        return obstacle_dist

    def negativeUtility(self, ego, intended_lane, all_obstacles):
        neg_utility = 0.0
        for obstacle in all_obstacles:
            if obstacle.lane_id == intended_lane:
                dx = obstacle.position.x - ego.position.x
                # only consider vehicles behind of ego vehicle
                if dx < 0:
                    dv = obstacle.velocity.x - ego.velocity.x
                    time_lane_change = self.timeToChangeLane(ego)
                    dist_lane_change = ego.velocity.x * time_lane_change
                    # Negative utility formula
                    neg_utility = abs(dx) - dv*time_lane_change - dist_lane_change
        return neg_utility

    # Calculate lateral velocity assuming max steering for vehicle to get time to change lane
    def timeToChangeLane(self, ego):
        turning_radius = ego.length / tan(radians(ego.max_steering))
        angular_velocity = ego.velocity.x / turning_radius

        # assuming center of lanes, we know the distance is 120 pixels
        lane_change_time = LANE_DIFF/degrees(angular_velocity)
        return lane_change_time
