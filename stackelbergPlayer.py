import os
import pygame
from math import tan, radians, degrees, copysign
from pygame.math import Vector2
from enum import Enum

# TODO: probably set these values during init
VISIBLE_DIST = 1900.0 # pixels
LANE_DIFF = 120.0 # 120 pixels between centers of lanes
ACTION_HORIZON = 0.25
COMFORT_LVL = 0.0
NUM_PLAYERS = 3

class Action(Enum):
    LEFT = 0
    RIGHT = 1
    MAINTAIN = 2
    ACCELERATE = 3
    DECELERATE = 4

class StackelbergPlayer():
    def __init__(self, car_width):
        self.car_width = car_width/64
        self.players = [set() for x in range(NUM_PLAYERS)]

    def selectAction(self, leader, all_obstacles):
        selected_action = self.getActionUtilSet(leader, all_obstacles)[0][0]
        return selected_action

    def getActionUtilSet(self, leader, all_obstacles):
        current_lane = leader.lane_id
        all_actions = list(Action)

        # print(all_obstacles)

        # if in the left lane remove left action, same with right lane
        if current_lane == 1:
            del all_actions[Action.LEFT.value]
        elif current_lane == 3:
            del all_actions[Action.RIGHT.value]

        best_utility = 0.0
        selected_action = Action.MAINTAIN
        # selected_action = None
        action_util_dict = {}
        for action in all_actions:
            # update the intended lane
            intended_lane = current_lane
            if action == Action.LEFT:
                intended_lane -= 1
            elif action == Action.RIGHT:
                intended_lane += 1

            # update intended velocity
            intended_velocity = self.updatedVelocity(leader, action)
            # compute utility for the current action
            current_utility = self.positiveUtility(leader, intended_lane, intended_velocity, all_obstacles)
            current_utility += self.negativeUtility(leader, intended_lane, intended_velocity, all_obstacles)
            if current_utility > best_utility:
                best_utility = current_utility
                selected_action = action

            # save action and corresponding utility
            action_util_dict.update({action:current_utility})

        action_util_sorted = sorted(action_util_dict.items(), key=lambda kv: kv[1], reverse=True)
        # print(action_util_sorted[0][1])
        
        return action_util_sorted
        # return selected_action

    # select leaders to make a decision at this instance, and followers for next iteration
    def pickLeadersAndFollowers(self, all_agents, all_obstacles):
        # 1. rank all players based on position on road
        sorted_agents = self.sortByPosition(all_agents)
        
        # 2. select top agent as leader and add to leader/follower list
        # players = [set() for x in range(NUM_PLAYERS)]
        while sorted_agents:
            leader = sorted_agents.pop(0)

            # get the followers for this leader
            all_players = self.pickPlayers(leader, all_agents, all_obstacles)

            # 3. add the leaders and followers to the players list of sets
            for idx, agent in enumerate(all_players):
                self.players[idx].add(agent)

            # 4. remove all these from the original sorted list
            sorted_agents = [agent for agent in sorted_agents if agent not in all_players]        
        
        leader_list = self.players[0]
        # update players for next turn
        self.updatePlayersList()

        # return the leaders
        return leader_list

    # remove current leaders and make first followers the new leaders
    # TODO: might want to make this into a queue datastructure
    def updatePlayersList(self):
        # remove current leaders
        del self.players[0]
        # add empty set for new followers
        self.players.append(set())
        return

    # TODO: refactor this method
    def pickPlayers(self, ego, all_agents, all_obstacles):
        players = [ego]

        # by default it is the middle lane
        adversary_lane = 2
        
        # need to update adversary lane if leader is already in the middle lane
        if ego.lane_id == 2:
            action_util_set = self.getActionUtilSet(ego, all_obstacles)
            for action_tuple in action_util_set:
                if action_tuple[0] == Action.LEFT:
                    adversary_lane -= 1
                    break
                elif action_tuple[0] == Action.RIGHT:
                    adversary_lane += 1
                    break

        back_agent, side_agent = None, None
        for agent in all_agents:
            if agent != ego:
                if agent.lane_id == ego.lane_id:
                    if agent.position.x < ego.position.x:
                        if not back_agent:
                            back_agent = agent
                        elif agent.position.x > back_agent.position.x:
                            back_agent = agent
                elif agent.lane_id == adversary_lane:
                    if agent.position.x < ego.position.x:
                        if not side_agent:
                            side_agent = agent
                        elif agent.position.x > side_agent.position.x:
                            side_agent = agent
        if back_agent: players.append(back_agent)
        if side_agent: players.append(side_agent)

        # return players sorted by their longitudinal position in decending order
        return self.sortByPosition(players)

    def updatedVelocity(self, ego, action):
        intended_velocity = ego.velocity.x

        # increase or decrease velocity (vf = vi + accel*time), assume accel of 1
        if action == Action.ACCELERATE:
            intended_velocity += 1*ACTION_HORIZON
        elif action == Action.DECELERATE:
            intended_velocity -= 1*ACTION_HORIZON
        return max(0.0, min(intended_velocity, ego.max_velocity))

    # sort vehicles by longitudinal position (decreasing order)
    def sortByPosition(self, all_agents):
        sorted_agents = sorted(all_agents, key=lambda x: x.position.x, reverse=True)
        return sorted_agents

    # TODO: start with the simple positive utility
    def positiveUtility(self, ego, intended_lane, intended_velocity, all_obstacles):
        # max visible distance
        # ideal_velocity = VISIBLE_DIST + ego.max_velocity
        ideal_velocity = ego.max_velocity

        for obstacle in all_obstacles:
            if obstacle == ego:
                continue
            if obstacle.lane_id == intended_lane:
                dx = obstacle.position.x - ego.position.x
                # dx = (obstacle.position.x - (obstacle.rect[2]/64)) - (ego.position.x + (ego.rect[2]/64)) - COMFORT_LVL

                # only consider vehicles ahead of ego vehicle
                if dx >= 0:
                    # calculate actual difference between cars
                    dx = abs(obstacle.position.x - ego.position.x) - (ego.rect[2]/32) - self.car_width

                    stopping_dist = self.stoppingDist(ego, intended_velocity)
                    tmp_val = intended_velocity + min(dx - stopping_dist, 0)
                    # tmp_val = dx + intended_velocity + min(dx - stopping_dist, 0)
                    # tmp_val = dx + ego.velocity.x + min(dx - stopping_dist, 0)

                    ideal_velocity = min(tmp_val, ideal_velocity)
                    # ideal_velocity = min(tmp_val, VISIBLE_DIST + ideal_velocity)
        return ideal_velocity

    # compute stopping distance for ego vehicle
    def stoppingDist(self, ego, intended_velocity):
        return 0.5*(intended_velocity ** 2)/ego.max_acceleration

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

    def negativeUtility(self, ego, intended_lane, intended_velocity, all_obstacles):
        neg_utility = None
        for obstacle in all_obstacles:
            if obstacle == ego:
                continue
            if obstacle.lane_id == intended_lane:
                dx = obstacle.position.x - ego.position.x
                # dx = (obstacle.position.x + (obstacle.rect[2]/64)) - (ego.position.x - (ego.rect[2]/64)) + COMFORT_LVL

                # only consider vehicles behind of ego vehicle
                if dx <= 0:
                    dx = abs(obstacle.position.x - ego.position.x) - (ego.rect[2]/32) - self.car_width

                    dv = obstacle.velocity.x - intended_velocity    
                    # dv = obstacle.velocity.x - ego.velocity.x

                    time_lane_change = self.timeToChangeLane(ego, intended_velocity)
                    dist_lane_change = intended_velocity * time_lane_change
                    # dist_lane_change = ego.velocity.x * time_lane_change

                    # Negative utility formula
                    if not neg_utility:
                        neg_utility = dx - dv*time_lane_change - dist_lane_change
                    else:
                        neg_utility = min(dx - dv*time_lane_change - dist_lane_change, neg_utility)
                    # neg_utility = abs(dx) - dv*time_lane_change - dist_lane_change

        # set neg_utility to 0.0 if it was not assigned above
        neg_utility = neg_utility if neg_utility else 0.0
        return neg_utility

    # Calculate lateral velocity assuming max steering for vehicle to get time to change lane
    def timeToChangeLane(self, ego, intended_velocity):
        turning_radius = ego.length / tan(radians(ego.max_steering))
        angular_velocity = intended_velocity / turning_radius
        # angular_velocity = ego.velocity.x / turning_radius

        # assuming center of lanes, we know the distance is 120 pixels
        lane_change_time = LANE_DIFF/degrees(angular_velocity)
        return lane_change_time
