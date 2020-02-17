#!/usr/bin/env python

import rospy
import numpy as np
import copy as cp

from directionEnum import Direction

from nav_msgs.msg._OccupancyGrid import OccupancyGrid

from path_planing.msg import PathPoint
from path_planing.msg import FullPath
from path_planing.srv import FindUnknown, FindUnknownResponse, FindUnseen, FindUnseenResponse, FindPathToGoal, FindPathToGoalResponse, FindShortestPath, FindPathToGoalResponse

class PathPlanningWayfront:

      def __init__(self):
            rospy.on_shutdown(self._shutdown)
            rospy.init_node('path_planning_wayfront')
            rospy.loginfo('Path Planning Wayfront node started')

            #Parameter
            self._value_goal = 2
            self._value_start = -2
            self._value_wall = 1
            self.received_map = False
            self.received_map_seen = False
            self.map = [[]]
            self.map_seen = [[]]

            self.mapSub = rospy.Subscriber('map', OccupancyGrid, self._map_callback)
            self.mapSeenSub = rospy.Subscriber('camera_seen_map', OccupancyGrid, self._map_seen_callback)

            self.find_unkown_service = rospy.Service('find_unkown_service', FindUnknown, self.handle_service_find_unknown)
            self.find_unseen_service = rospy.Service('find_unseen_service', FindUnseen, self.handle_service_find_unseen)
            self.find_path_to_goal_service = rospy.Service('find_path_to_goal_service', FindPathToGoal, self.handle_service_find_path_to_goal)
            self.find_shortest_path_service = rospy.Service('find_shortest_path_service', FindShortestPath, self.handle_service_find_shortest_path)

            #keep that shit running until shutdown
            rospy.loginfo('--- ready ---')
            rospy.spin()

      def _shutdown(self):
            rospy.loginfo('shutdown')

      def _map_callback(self, data):
            self.map = np.reshape(data.data, (data.info.height, data.info.width))
            self.received_map = True

      def _map_seen_callback(self, data):
            self.map_seen = np.reshape(data.data, (data.info.height, data.info.width))
            self.received_map_seen = True

      def _set_start_and_goal(self, map, xGoal, yGoal, xStart, yStart):
            """
            """
            map[yGoal][xGoal] = self._value_goal
            map[yStart][xStart] = self._value_start
            return map

      def _label_cells(self, map, neighbours):
            """
            Labels the map.

            Parameters:
            map: map to label
            start ([]): Array of Touple (x,y) of starting position

            Returns:
            map: the labeled map
            max_val: max value during labeling
            """

            max_val = 0
            while len(neighbours) != 0:
                  (x, y) = neighbours.pop(0)
   
                  if map[y - 1][x] == 0:
                        map[y - 1][x] = map[y][x] + 1
                        neighbours.append((x, y-1))
                        if map[y][x] + 1 > max_val:
                              max_val = map[y][x] + 1

                  if map[y][x + 1] == 0:
                        map[y][x + 1] = map[y][x] + 1
                        neighbours.append((x+1, y))
                        if map[y][x] + 1 > max_val:
                              max_val = map[y][x] + 1

                  if map[y + 1][x] == 0:
                        map[y + 1][x] = map[y][x] + 1
                        neighbours.append((x, y+1))
                        if map[y][x] + 1 > max_val:
                              max_val = map[y][x] + 1

                  if map[y][x - 1] == 0:
                        map[y][x - 1] = map[y][x] + 1
                        neighbours.append((x-1, y))
                        if map[y][x] + 1 > max_val:
                              max_val = map[y][x] + 1

            return map, max_val

      def _find_path(self, map, xStart, yStart, radius):
            """
            """
            currentX = xStart
            currentY = yStart
            lastX = xStart
            lastY = yStart
            waypoints = []
            allpoints = []
            # 0 -> No / 1 -> Top / 2 -> Bottom / 3 -> Left / 4 -> Right / Top Left -> 5 / Top Right -> 6 / Bottom Right -> 7 / Bottom Left -> 8
            lastDirection = Direction.Neutral

            run = True
            while run == True:
                  nextLowestAdjeacent = self._get_next_lowest_adjeacent(map, currentX, currentY)
                  if nextLowestAdjeacent[0] == 0 and nextLowestAdjeacent[1] == 0 and nextLowestAdjeacent[2] == 0:
                        return None, None, None

                  if nextLowestAdjeacent[2] != self._value_goal:
                        lastX = currentX
                        lastY = currentY
                        currentX = nextLowestAdjeacent[0]
                        currentY = nextLowestAdjeacent[1]
                        direction =nextLowestAdjeacent[3]
                        # currentValue = currentValue - 1
                        # map[currentY][currentX] = currentValue

                        #direction_changed, direction = self._detect_direction_change(direction, currentX, currentY, lastX, lastY)
                        if direction != lastDirection:
                              lastDirection = direction
                              waypoints.append((lastX, lastY))
                        allpoints.append((currentX, currentY))

                  elif nextLowestAdjeacent[2] == self._value_goal:
                        waypoints.append((currentX, currentY))
                        run = False
            return map, waypoints, allpoints

      def _detect_direction_change(self, direction, current_x, current_y, last_x, last_y):
            """
            Detects when movemend changes from vertical to horizontal or horizontal to vertical
            """
            direction_changed = False
            if current_x != last_x and direction == 'v':
                  direction = 'h'
                  direction_changed = True
            if current_y != last_y and direction == 'h':
                  direction = 'v'
                  direction_changed = True

            return direction_changed, direction

      def _get_first_adjeacent(self, map, currentX, currentY):
            """
            """
            tempX = 0
            tempY = 0
            tempValue = 147456

            # check left top
            if(map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1 and map[currentY - 1][currentX - 1] != -1:
                  tempX = currentX - 1
                  tempY = currentY - 1
                  tempValue = map[currentY - 1][currentX - 1]

            # check top
            if(map[currentY - 1][currentX] < tempValue) and map[currentY - 1][currentX] != 1 and map[currentY - 1][currentX] != -1:
                  tempX = currentX
                  tempY = currentY - 1
                  tempValue = map[currentY - 1][currentX]

            # check top right
            if(map[currentY - 1][currentX + 1] < tempValue) and map[currentY - 1][currentX + 1] != 1 and map[currentY - 1][currentX + 1] != -1:
                  tempX = currentX + 1
                  tempY = currentY - 1
                  tempValue = map[currentY - 1][currentX + 1]

            # check right
            if (map[currentY][currentX + 1] < tempValue) and map[currentY][currentX + 1] != 1 and map[currentY][currentX + 1] != -1:
                  tempX = currentX + 1
                  tempY = currentY
                  tempValue = map[currentY][currentX + 1]

            # check bottom right
            if (map[currentY + 1][currentX + 1] < tempValue) and map[currentY + 1][currentX + 1] != 1 and map[currentY + 1][currentX + 1] != -1:
                  tempX = currentX + 1
                  tempY = currentY + 1
                  tempValue = map[currentY + 1][currentX + 1]

            # check bottom
            if (map[currentY + 1][currentX] < tempValue) and map[currentY + 1][currentX] != 1 and map[currentY + 1][currentX] != -1:
                  tempX = currentX
                  tempY = currentY + 1
                  tempValue = map[currentY + 1][currentX]

            # check bottom left
            if (map[currentY + 1][currentX - 1] < tempValue) and map[currentY + 1][currentX - 1] != 1 and map[currentY + 1][currentX - 1] != -1:
                  tempX = currentX - 1
                  tempY = currentY + 1
                  tempValue = map[currentY + 1][currentX - 1]

            # check left
            if (map[currentY][currentX - 1] < tempValue) and map[currentY][currentX - 1] != 1 and map[currentY][currentX - 1] != -1:
                  tempX = currentX - 1
                  tempY = currentY
                  tempValue = map[currentY][currentX - 1]

            # check bottom left
            if (map[currentY - 1][currentX - 1] < tempValue) and map[currentY - 1][currentX - 1] != 1 and map[currentY - 1][currentX - 1] != -1:
                  tempX = currentX - 1
                  tempY = currentY - 1
                  tempValue = map[currentY - 1][currentX - 1]

            highestAdjeacent = [tempX, tempY, tempValue]

            return highestAdjeacent

      def _get_next_lowest_adjeacent(self, map, currentX, currentY):
            """
            """
            currentValue = 0

            nextX = 0
            nextY = 0
            nextValue = 0
            # 0 -> No
            # 1 -> Top
            # 2 -> Bottom
            # 3 -> Left
            # 4 -> Right
            # 5 -> Top Left
            # 6 -> Top Right
            # 7 -> Bottom Right
            # 8 -> Bottom Left
            direction = Direction.Neutral

            if map[currentY][currentX] == -2:
                  highestAdjeacent = self._get_first_adjeacent(map, currentX, currentY)
                  return [highestAdjeacent[0], highestAdjeacent[1], highestAdjeacent[2], Direction.Neutral]
            else:
                  nextValue = currentValue = map[currentY][currentX]

            # check top left
            if map[currentY - 1][currentX - 1] == currentValue - 2 and map[currentY - 1][currentX - 1] >=  self._value_goal:
                  nextX = currentX - 1
                  nextY = currentY - 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.NorthWest

            # check top
            if map[currentY - 1][currentX] == currentValue - 1 and map[currentY - 1][currentX] >=  self._value_goal and map[currentY - 1][currentX] < nextValue:
                  nextX = currentX
                  nextY = currentY - 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.North

            # check top right
            if map[currentY - 1][currentX + 1] == currentValue - 2 and map[currentY - 1][currentX + 1] >=  self._value_goal and map[currentY - 1][currentX + 1] < nextValue:
                  nextX = currentX + 1
                  nextY = currentY - 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.NorthEast

            # check right
            if map[currentY][currentX + 1] == currentValue - 1 and map[currentY][currentX + 1] >=  self._value_goal and map[currentY][currentX + 1] < nextValue:
                  nextX = currentX + 1
                  nextY = currentY
                  nextValue = map[nextY][nextX]
                  direction = Direction.East

            # check bottom right
            if map[currentY + 1][currentX + 1] == currentValue - 2 and map[currentY + 1][currentX + 1] >=  self._value_goal and map[currentY + 1][currentX + 1] < nextValue:
                  nextX = currentX + 1
                  nextY = currentY + 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.SouthEast

            # check bottom
            if map[currentY + 1][currentX] == currentValue - 1 and map[currentY + 1][currentX] >=  self._value_goal and map[currentY + 1][currentX] < nextValue:
                  nextX = currentX
                  nextY = currentY + 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.South

            # check bottom left
            if map[currentY + 1][currentX - 1] == currentValue - 2 and map[currentY + 1][currentX - 1] >=  self._value_goal and map[currentY + 1][currentX - 1] < nextValue:
                  nextX = currentX - 1
                  nextY = currentY + 1
                  nextValue = map[nextY][nextX]
                  direction = Direction.SouthWest

            # check left
            if map[currentY][currentX - 1] == currentValue - 1 and map[currentY][currentX - 1] >=  self._value_goal and map[currentY][currentX - 1] < nextValue:
                  nextX = currentX - 1
                  nextY = currentY
                  nextValue = map[nextY][nextX]
                  direction = Direction.West

            return [nextX, nextY, nextValue, direction]

      def _find_all_unknown(self, map, robot_radius):
            """
            Find coordinates of all spots that are unkown:

            Parameters:
            map: the unlabled map

            Returns:
            map: the updated map with the goals set
            unknown_spots ([]): Array of tuple (x,y) of the goals
            """
            num_rows = len(map)
            num_cols = len(map[0])

            list_unknown_spots = []

            for row in range(robot_radius, num_rows - robot_radius):
                  for col in range(robot_radius, num_cols - robot_radius):
                        if map[row][col] == 0:
                              addGoal = False
                              surrounding_any_wall = map[row - robot_radius: row + robot_radius + 1, col - robot_radius: col + robot_radius + 1] == self._value_wall
                              if map[row - 1][col] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                                    map[row][col] = self._value_goal
                                    addGoal = True
                              if map[row + 1][col] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                                    map[row][col] = self._value_goal
                                    addGoal = True
                              if map[row][col - 1] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                                    map[row][col] = self._value_goal
                                    addGoal = True
                              if map[row][col + 1] == -1 and not any(np.any(surrounding_any_wall, axis=0)) and not any(np.any(surrounding_any_wall, axis=1)):
                                    map[row][col] = self._value_goal
                                    addGoal = True

                              if addGoal:
                                    list_unknown_spots.append((col, row))

            return map, list_unknown_spots

      def _blow_up_wall(self, map, blowUpCellNum, robot_x, robot_y, robot_radius):
            #blow up walls
            tmp_map = cp.deepcopy(map)
            for row in range(0, len(tmp_map)):
                  for col in range(0 , len(tmp_map[0])):
                        # check outside boundaries and if it is a wall
                        if map[row,col] == 100 and row >= blowUpCellNum and col >= blowUpCellNum and row <= len(tmp_map) - blowUpCellNum and col <= len(tmp_map[0]) - blowUpCellNum:
                              # only blow up if not robot position
                              # if (robot_y < (row - blowUpCellNum) or robot_y > (row + blowUpCellNum)) and (robot_x < (col - blowUpCellNum) or robot_x > (col + blowUpCellNum)): 
                              tmp_map[row - blowUpCellNum : row + 1 + blowUpCellNum, col - blowUpCellNum : col + 1 + blowUpCellNum] = 100

            tmp_map = self._free_up_position_from_wall(robot_x, robot_y, robot_radius, map, tmp_map)
      
            return tmp_map

      def _free_up_position_from_wall(self, freeup_x, freeup_y, robot_radius, reference_map, map):
            # Free up robot top if no wall in org map
            if freeup_y - robot_radius >= 0 and  reference_map[freeup_y - robot_radius, freeup_x] != 100:
                  for y in range(freeup_y - robot_radius, freeup_y + 1):
                        for x in range(freeup_x - robot_radius, freeup_x + robot_radius + 1):
                              if y >= 0 and x >= 0 and y < len(map) and x < len(map[0]) and map[y, x] != reference_map[y, x]:
                                    map[y, x] = reference_map[y, x]

            # Free up robot right if no wall in org map
            if freeup_x + robot_radius <  len(map[0]) and reference_map[freeup_y, freeup_x + robot_radius] != 100:
                  for y in range(freeup_y - robot_radius, freeup_y + 1 + robot_radius):
                        for x in range(freeup_x, freeup_x + robot_radius + 1):
                              if y >= 0 and x >= 0 and y < len(map) and x < len(map[0]) and map[y, x] != reference_map[y, x]:
                                    map[y, x] = reference_map[y, x]

            # Free up robot bottom if no wall in org map
            if freeup_y + robot_radius < len(map) and  reference_map[freeup_y + robot_radius, freeup_x] != 100:
                  for y in range(freeup_y - robot_radius, freeup_y + 1):
                        for x in range(freeup_x - robot_radius, freeup_x + robot_radius + 1):
                              if y >= 0 and x >= 0 and y < len(map) and x < len(map[0]) and map[y, x] != reference_map[y, x]:
                                    map[y, x] = reference_map[y, x]

            # Free up robot left if no wall in org map
            if freeup_x - robot_radius >= 0 and reference_map[freeup_y, freeup_x - robot_radius] != 100:
                  for y in range(freeup_y - robot_radius, freeup_y + 1 + robot_radius):
                        for x in range(freeup_x - robot_radius, freeup_x + 1):
                              if y >= 0 and x >= 0 and y < len(map) and x < len(map[0]) and map[y, x] != reference_map[y, x]:
                                    map[y, x] = reference_map[y, x]
            
            return map

      def _create_msg(self, points):
            msg_pointlist = FullPath()

            for point in points:
                  msg_point = PathPoint()
                  msg_point.path_x = point[0]
                  msg_point.path_y = point[1]
                  msg_pointlist.fullpath.append(msg_point)
      
            return msg_pointlist

      def handle_service_find_shortest_path(self, data):
            """
            Handels the request to the service find shortest path
            """
            goals = data.goals.fullpath
            blowUpCellNum = data.blowUpCellNum
            xStart = data.xStart
            yStart = data.yStart
            radius = data.radius
         
            if self.received_map == True:
                  rospy.loginfo('Start find_unknown')
                  if blowUpCellNum > 0:
                        original_map = cp.deepcopy(self.map)
                        map = self._blow_up_wall(original_map, blowUpCellNum, xStart, yStart, radius)
                        for goal in goals:
                              map = self._free_up_position_from_wall(goal.path_x, goal.path_y, radius, original_map, map)
                  else:
                        map = cp.deepcopy(self.map)
                  # Walls = 100 | Unknown = -1 | Free Space = 0
                  # Walls need to be set to 1 to make algorithm work
                  map[map == 100] = self._value_wall
                  map[yStart][xStart] = self._value_start

                  np.savetxt("map.csv", self.map, delimiter=",", fmt='%1.3f')
                  

                  # set the goals
                  goals_extracted = []
                  for goal in goals:
                        goals_extracted.append((goal.path_x, goal.path_y))
                        map[goal.path_y][goal.path_x] = self._value_goal

                  np.savetxt("map_goals.csv", map, delimiter=",", fmt='%1.3f')
                  map, _ = self._label_cells(map, goals_extracted)
                  np.savetxt("map_labeled.csv", map, delimiter=",", fmt='%1.3f')

                  map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)
                  if waypoints == None and allpoints == None:
                        rospy.loginfo("ERROR --> No path found")
                        return None, None
                        #return FindUnknownResponse(None, None)
                  else:
                        rospy.loginfo("Success find_unknown")
                        allpoints_r = self._create_msg(allpoints)
                        waypoints_r = self._create_msg(waypoints)
                        #return FindUnknownResponse(waypoints_r, allpoints_r)
                        return waypoints_r, allpoints_r
            else:
                  rospy.loginfo("ERROR --> No map loaded")
                  #return FindUnknownResponse(None, None)
                  return None, None



      def handle_service_find_path_to_goal(self, sv_data):
            """
            """
            blowUpCellNum = sv_data.blowUpCellNum
            xGoal = sv_data.xGoal
            yGoal = sv_data.yGoal
            xStart = sv_data.xStart
            yStart = sv_data.yStart
            radius = sv_data.radius
            if self.received_map == True:
                  rospy.loginfo("Start find_path_to_goal")
                  if blowUpCellNum > 0:
                        map = self._blow_up_wall(cp.deepcopy(self.map), blowUpCellNum, xStart, yStart, radius)
                  else:
                        map = cp.deepcopy(self.map)
                  # Walls = 100 | Unknown = -1 | Free Space = 0
                  # Walls need to be set to 1 to make algorithm work
                  map[map == 100] = 1
                  map = self._set_start_and_goal( map, xGoal, yGoal, xStart, yStart)

                  map, _ = self._label_cells(map, [(xGoal, yGoal)])

                  map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)

                  if waypoints == None and allpoints == None:
                        rospy.loginfo("ERROR --> No path found")
                        return FindPathToGoalResponse(None, None)
                  else:
                        rospy.loginfo("Success find_path_to_goal")
                        allpoints_r = self._create_msg(allpoints)
                        waypoints_r = self._create_msg(waypoints)

                        return FindPathToGoalResponse(waypoints_r, allpoints_r)
            else:
                  rospy.loginfo("ERROR --> No map loaded")
                  return FindPathToGoalResponse(None, None)

      
      def handle_service_find_unseen(self, sv_data):
            """
            Search for space that was not discovered by the camera yet
            """
            if len(self.map_seen[0]) == 0:
                  self.map_seen = np.full((sv_data.info.height, sv_data.info.width), -1)
            map = self.map + self.map_seen
            # At this point Wall = 110, Free Space = 10, Unknown = -2
            map[map == 110] = 100
            map[map == 99] = 100
            map[map == 10] = 0
            map[map == -2] = -1
            # At this point Wall = 100, Free Space = 0, Unknown = -1
            waypoints, allpoints = self._find_unknown(sv_data, map)
            return FindUnseenResponse(waypoints, allpoints)


      def handle_service_find_unknown(self, sv_data):
            """
            Search for space that was not discovered by the Lidar yet
            """
            waypoints, allpoints = self._find_unknown(sv_data, self.map)
            return FindUnknownResponse(waypoints, allpoints)


      def _find_unknown(self, sv_data, map_in):
            """
            Find path to the nearest position

            Map needs following settings.
            Walls = 100 | Unknown = -1 | Free Space = 0
            """
            blowUpCellNum = sv_data.blowUpCellNum
            xStart = sv_data.xStart
            yStart = sv_data.yStart
            radius = sv_data.radius
         
            if self.received_map == True:
                  rospy.loginfo("--> Start find_unknown")
                  if blowUpCellNum > 0:
                        map = self._blow_up_wall(cp.deepcopy(map_in), blowUpCellNum, xStart, yStart, radius)
                  else:
                        map = cp.deepcopy(map_in)
                  # Walls = 100 | Unknown = -1 | Free Space = 0
                  # Walls need to be set to 1 to make algorithm work
                  map[map == 100] = self._value_wall
                  map[yStart][xStart] = self._value_start

                  np.savetxt("map.csv", map, delimiter=",", fmt='%1.3f')
                  map, goals = self._find_all_unknown(map, 0)
                  np.savetxt("map_goals.csv", map, delimiter=",", fmt='%1.3f')
                  map, _ = self._label_cells(map, goals)
                  np.savetxt("map_labeled.csv", map, delimiter=",", fmt='%1.3f')

                  map, waypoints, allpoints = self._find_path(map, xStart, yStart, radius)
                  if waypoints == None and allpoints == None:
                        rospy.loginfo("ERROR --> No path found")
                        return None, None
                        #return FindUnknownResponse(None, None)
                  else:
                        rospy.loginfo("--> Success find_unknown")
                        allpoints_r = self._create_msg(allpoints)
                        waypoints_r = self._create_msg(waypoints)
                        #return FindUnknownResponse(waypoints_r, allpoints_r)
                        return waypoints_r, allpoints_r
            else:
                  rospy.loginfo("ERROR --> No map loaded")
                  #return FindUnknownResponse(None, None)
                  return None, None

if __name__ == '__main__':
    try:
        ppw = PathPlanningWayfront()
    except rospy.ROSInterruptException:
        pass