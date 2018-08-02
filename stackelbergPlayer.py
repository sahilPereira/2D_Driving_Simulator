import os
import pygame
import copy
from math import tan, radians, degrees, copysign
from pygame.math import Vector2
from enum import Enum

# TODO: probably set these values during init
VISIBLE_DIST = 1900.0 # pixels
LANE_DIFF = 120.0 # 120 pixels between centers of lanes
ACTION_HORIZON = 0.25
COMFORT_LVL = 0.0
NUM_PLAYERS = 3
NUM_LANES = 3

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
        self.playerSets = {}

    def selectAction(self, leader, all_obstacles):
        selected_action = self.getActionUtilSet(leader, all_obstacles)[0][0]
        return selected_action

    def selectStackelbergAction(self, leader, all_obstacles, reference_car):

        # Step 1: get the players involved in this game
        s_players = self.playerSets[leader]

        # Step 2: create copies of all the obstacles so we can project their actions
        mut_agents = []
        all_obstacles_copy = []

        for obst in all_obstacles:
            all_obstacles_copy.append(obst.simCopy())

        # all_obstacles_copy = copy.deepcopy(all_obstacles)

        p_ids = []
        for obstacle in all_obstacles:
            if obstacle in s_players:
                p_ids.append(obstacle.id)
                # create a copy of the obstacle
                # s_players_copy.append(obstacle.simCopy())

        for obstacle_copy in all_obstacles_copy:
            if obstacle_copy.id in p_ids:
                # all_obstacles_copy.remove(obstacle_copy)
                mut_agents.append(obstacle_copy)

        # print("All obstacle_copy len: "+str(len(all_obstacles_copy)))
        # print(all_obstacles_copy)

        # make sure the players are sorted
        mut_agents = self.sortByPosition(mut_agents)

        # use these for simulating different actions
        # mut_agents = []
        # for p_copy in s_players_copy:
        #     mut_agents.append(p_copy.simCopy())

        # all_obstacles_copy.add(mut_agents)

        p1_best, p2_best = 0.0, 0.0
        p1_best_action, p2_best_action, p3_best_action = Action.MAINTAIN, Action.MAINTAIN, Action.MAINTAIN

        all_actions = list(Action)

        p_range = list(range(len(s_players)))
        p_count = len(p_range)

        p1_action_list = self.getActionSubset(all_actions, mut_agents[0])
        if p_count > 1:
            p2_action_list = self.getActionSubset(all_actions, mut_agents[1])

        for p1_action in p1_action_list:
            # TODO remove after testing
            # print(p1_action.name)

            # 1a. select and execute an action for player[0]
            self.executeAction(p1_action, mut_agents[0], all_obstacles_copy)
            mut_agents[0].update(ACTION_HORIZON, reference_car)

            if p_count > 1:
                # 1b. observe how followers react to selected action
                for p2_action in p2_action_list:
                    
                    # 2a. select and execute an action for player[1]
                    self.executeAction(p2_action, mut_agents[1], all_obstacles_copy)
                    mut_agents[1].update(ACTION_HORIZON, reference_car)

                    if p_count > 2:
                        # 2b. observe how followers react to selected action
                        # 3a. select and execute an action which maximizes player[2] utility
                        p3_best_action = self.selectAction(mut_agents[2], all_obstacles_copy)
                        self.executeAction(p3_best_action, mut_agents[2], all_obstacles_copy)
                        mut_agents[2].update(ACTION_HORIZON, reference_car)

                    # 2c. calculate utility value for current state
                    p2_utility = self.positiveUtility(mut_agents[1], mut_agents[1].lane_id, mut_agents[1].velocity.x, all_obstacles_copy)
                    p2_utility += self.negativeUtility(mut_agents[1], mut_agents[1].lane_id, mut_agents[1].velocity.x, all_obstacles_copy)

                    # 2d. select action which results in the best utility value
                    if p2_utility > p2_best:
                        p2_best = p2_utility
                        p2_best_action = p2_action

                    # reset the state for agents 2 and 3
                    self.resetState(mut_agents, s_players, all_obstacles_copy, p_range[1:])

                # execute the best actions for player 2 and 3
                self.executeAction(p2_best_action, mut_agents[1], all_obstacles_copy)
                mut_agents[1].update(ACTION_HORIZON, reference_car)

                if p_count > 2:
                    self.executeAction(p3_best_action, mut_agents[2], all_obstacles_copy)
                    mut_agents[2].update(ACTION_HORIZON, reference_car)

            # 1c. calculate utility value for final state
            # TODO remove after testing
            # print(mut_agents[0].velocity.x)

            p1_utility = self.positiveUtility(mut_agents[0], mut_agents[0].lane_id, mut_agents[0].velocity.x, all_obstacles_copy)
            p1_utility += self.negativeUtility(mut_agents[0], mut_agents[0].lane_id, mut_agents[0].velocity.x, all_obstacles_copy)

            # 1d. select the action which results in the best utility value
            if p1_utility > p1_best:
                p1_best = p1_utility
                p1_best_action = p1_action

            # reset the state for agents 1, 2 and 3
            self.resetState(mut_agents, s_players, all_obstacles_copy, p_range)

        # TODO: remove after testing
        # ------------------------------------------------------------------
        # if s_players_copy[0] == mut_agents[0]:
        #     print("Leaders from copy and original are the same")
        # else:
        #     print("Copies are different")

        # # s_players_copy[0].position.x = 1000
        # same_pos = s_players_copy[0].position.x == mut_agents[0].position.x
        # same_pos &= s_players_copy[0].velocity.x == mut_agents[0].velocity.x
        # if same_pos:
        #     print("Still linked by reference")
        # else:
        #     print("Not linked, an actual copy")
        # ------------------------------------------------------------------

        # print(p1_best_action.name)
        return p1_best_action

    def resetState(self, mut_agents, s_players, all_obstacles_copy, resetList):
        # for obstacle_copy in mut_agents:
        #     # if obstacle_copy.id in p_ids:

        for i in resetList:
            all_obstacles_copy.remove(mut_agents[i])
            mut_agents[i] = s_players[i].simCopy()
            all_obstacles_copy.append(mut_agents[i])

        return

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

    def getActionSubset(self, actions, ego):
        # if in the left lane remove left action, same with right lane
        actions_subset = actions.copy()
        if ego.lane_id == 1:
            actions_subset.remove(Action.LEFT)
        elif ego.lane_id == 3:
            actions_subset.remove(Action.RIGHT)
        return actions_subset

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

            # save player sets for each leader
            # self.playerSets[leader] = all_players

            # 3. add the leaders and followers to the players list of sets
            for idx, agent in enumerate(all_players):
                self.players[idx].add(agent)

            # 4. remove all these from the original sorted list
            sorted_agents = [agent for agent in sorted_agents if agent not in all_players]        
        
        leader_list = self.players[0]

        # save player sets for each leader
        for leader in leader_list:
            # get the followers for this leader
            all_players = self.pickPlayers(leader, all_agents, all_obstacles)
            self.playerSets[leader] = all_players

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
        for agent in all_obstacles:
            if agent != ego:
                # agent in the same lane
                if agent.lane_id == ego.lane_id:
                    # agent has to be behind the ego vehicle
                    if agent.position.x < ego.position.x:
                        if not back_agent:
                            back_agent = agent
                        # agent closest to the back
                        elif agent.position.x > back_agent.position.x:
                            back_agent = agent
                # agent is in adjacent lane
                elif agent.lane_id == adversary_lane:
                    # agent has to be behind the ego vehicle
                    if agent.position.x < ego.position.x:
                        if not side_agent:
                            side_agent = agent
                        # agent closest the to back side
                        elif agent.position.x > side_agent.position.x:
                            side_agent = agent
        if back_agent in all_agents: players.append(back_agent)
        if side_agent in all_agents: players.append(side_agent)

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

    # TODO: really need to reuse these from Game(), need to understand inheritance in python
    # execute the given action for the specified leader
    def executeAction(self, selected_action, leader, all_obstacles):
        if (selected_action == Action.ACCELERATE) and not leader.do_accelerate:
            self.accelerate(leader)
        elif (selected_action == Action.DECELERATE) and not leader.do_decelerate:
            self.decelerate(leader)
        elif (selected_action == Action.MAINTAIN) and not leader.do_maintain:
            self.maintain(leader, all_obstacles)

        leader.acceleration = max(-leader.max_acceleration, min(leader.acceleration, leader.max_acceleration))

        if (selected_action == Action.RIGHT) and not leader.right_mode:
            self.turn_right(leader)
        elif (selected_action == Action.LEFT) and not leader.left_mode:
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
