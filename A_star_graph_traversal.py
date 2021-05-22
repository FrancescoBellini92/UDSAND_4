"""
Implementation of an A* algorithm for graph traversal
"""

def shortest_path(M,start,goal):
    frontier = MinHeapQueue() # the frontier is represented as a min heap (see below for the custom class used)
    frontier.add(start, 0) # starting point, whose cost is 0
    movement_cost_map = {start: 0}
    previous_node_map = {start: None} # map for keeping track of from_node -> to_node relations in our traversal

    # explore the frontier
    while not frontier.isEmpty:
        current_node = frontier.get() # get the node with lowest cost from the frontier
        if current_node == goal: # quick exit if we have arrived
            break

        neighbours = M.roads[current_node]
        for neighbour in neighbours:
            # calculate cost of neighbour
            neighbour_cost = movement_cost_map[current_node] + get_distance(M, current_node, neighbour)

            # if neighbour has not been reached yet, or the current route to it is cheaper than previous one
            if neighbour not in movement_cost_map or neighbour_cost < movement_cost_map[neighbour]:
                movement_cost_map[neighbour] = neighbour_cost # register the actual movement cost for this node

                # as we visit (or re-visit) nodes with lowest movement cost, we update their from_to relations
                previous_node_map[neighbour] = current_node

                heuristic_cost = get_distance(M, neighbour, goal) # heuristic cost is distance from goal
                estimated_cost = neighbour_cost + heuristic_cost
                frontier.add(neighbour, estimated_cost) # as priority use the estimated cost, not actual one

    # at this point, we can follow the breadcrumbs from goal to start
    # as the from_to map represents lowest_cost relationships between nodes
    path = [goal]
    while start not in path:
        current = path[-1]
        previous_node = previous_node_map[current]
        path.append(previous_node)

    path.reverse()
    return path



"""
Utility class that simulates a min heap queue,
using a sorted array and a priority: value map
"""
class MinHeapQueue:
    def __init__(self):
        self._priority_map = {} # priority: value
        self.priorities = []

    def add(self, value, priority):
        self._priority_map[priority] = value
        self.priorities.append(priority)
        self.priorities = sorted(self.priorities)

    def get(self):
        priority = self.priorities.pop(0)
        if priority in self._priority_map:
            return self._priority_map[priority]
    @property
    def isEmpty(self):
        return len(self.priorities) == 0


"""
Calculates manhattan distance between two points
"""
def get_distance(M, start, end):
    start_x, start_y = M.intersections[start]
    end_x, end_y = M.intersections[end]
    return abs(start_x - end_x) + abs(start_y - end_y)