import Queue as Q


class NodeState(object):
	# priority = 999999
	path = []
	goal_co_ords = [200,30]
	parent = 0
	# dist = 0

	def __init__(self, id, co_ords, links = []):
		self.id = id
		self.children = []
		# self.parent = parent
		self.co_ords = co_ords
		self.links = links[:]
		self.priority = 99999
		self.dist = 0
		self.goal_co_ords = [200,30]

	def calc_dist(self, origin, destination):
		d = ((destination[0] - origin[0])**2 + (destination[1] - origin[1])**2)**0.5
		return d

	def update_dist(self, parent):
		if parent:
			self.parent = parent
			self.dist = parent.dist + self.calc_dist(self.parent.co_ords, self.co_ords)
		else:
			self.dist = 0

	def update_priority(self):
			self.priority = self.dist + self.calc_dist(self.co_ords, self.goal_co_ords)
			# self.priority = self.calc_dist(self.co_ords, self.goal_co_ords)


	def get_priority(self):
		return self.priority

	def get_id(self):
		return self.id

	def update_path(self, parent):
		self.path = parent.path[:]
		self.path.append(self.co_ords)

	def update_links(self, node_list, obstruction_list)
		for node in node_list:
			if not self.co_ords == node.co_ords
				line_attempt = [self.co_ords, node.co_ords]
				for obstruction in obstruction_list:
					if line.crosses(obstruction):
						break
				self.links.append(node.id)


	# def get_priority(elem):
	# 	return elem.priority

	def create_children(self, nodes):
		if not self.children:
			self.nodes = nodes
			for x in self.links:
				# print 'aafkjsdlfk'
				child = self.nodes.get_node(x)
				child.parent = self
				child.update_dist(self)
				child.update_priority()
				child.update_path(self)
				# print 'Child Created: Node' + str(child.id)
				# print ('with details', str(child))

				# print str(child.id)
				self.children.append(child)
			# self.children.sort(key=get_priority())

	def __str__(self):
		return "[Node" + str(self.id) + "," + str(self.co_ords) + ", priority " + str(self.priority) +  "," + str(self.dist) + "," + str(self.links) + "]"

			# for i in xrange(len(self.goal)-1):
			# 	val = self.priority
			# 	val = val[:i] + val[i+1] + val[i] + val[i+2]
			# 	child = State_String(val, self)
			# 	self.children.append(child)

# class SateString(state):
# 	def __init__(self, id, co_ords, priority, parent, distance, links):
# 		super(State_String, self).__init__(id, co_ords, priority, parent, distance, links)
# 		self.distance = self.GetDist()

# 	def get_dist(self):
# 		# if self.co_ords == self.goal:
# 		# 	return 0
# 		dist = parent.getdist + ((self.co_ords[0]-parent.co_ords[0])**2+(self.co_ords[1]-parent.co_ords[1])**2)**(1/2.0)
# 		return dist

class NodeList(list):
	def __init__(self):
		self.node_list = []		
		self.node_list.append(NodeState(0, [10, 75]))
		self.node_list.append(NodeState(1, [10, 150]))
		self.node_list.append(NodeState(2, [10, 225]))
		self.node_list.append(NodeState(3, [590, 75]))
		self.node_list.append(NodeState(4, [590, 150]))
		self.node_list.append(NodeState(5, [590, 225]))
		self.count = 6

	def get_node(self, id):
		for j in self:
			if j.id == id:
				return j

	def print_list(self):
		print ('[%s]' % '\n '.join(map(str, self)))

	def add_node(self, id = 0, node_co_ords)
		# for node in block.block_nodes
		if not id:
			new_id = self.count
			self.count += 1
		else new_id = id
		new_node = NodeState(new_id, node)
		self.node_list.append(new_node)
		


class AStarSolver:

	def __init__(self, start, goal, nodes):
		self.path = []
		self.visited_queue = []
		self.priority_queue = Q.PriorityQueue()
		self.start = start
		self.goal = goal
		self.nodes = nodes
		self.dist = 0
		self.nodes.add_node(7000, start.co_ords)
		self.nodes.add_node(7001, goal.co_ords)

		for node in self.nodes:
			node.update_links(self.nodes, self.obstruction_list)

	def print_impossible(self):
		print ("Goal of " + str(self.goal.co_ords) + "is not possible!")



	def solve(self):
		start_state = self.start
		start_state.distance = 0
		start_state.priority = 0
		# start_state = NodeState(0, self.start, 0, 0, 0, [1, 2])
		count = 0
		self.priority_queue.put((0, count, start_state))
		while (not self.path and not self.priority_queue.empty()):
			closest_child = self.priority_queue.get()[2]
			print 'closest child is Node:' + str(closest_child.id) 
			print('with details', str(closest_child))
			closest_child.create_children(self.nodes)
			print 'Children of Closest Child are' + '[%s]' % ', '.join(map(str, closest_child.children))
			# print str(closest_child.children)
			self.visited_queue.append(closest_child.id)
			print ('Nodes visited so far: ' + str(self.visited_queue))
			print (' Distance travelled so far: ' + str(closest_child.dist))
			for child in closest_child.children:
				if child.calc_dist(child.co_ords, self.goal.co_ords) == 0.0:
					self.path.append(self.start.co_ords)
					self.path.extend(child.path[:])
					self.dist = child.dist
					break
				if child.id not in self.visited_queue:
					count += 1
					# self.priority_queue.put((child.priority, count, child))
					self.priority_queue.put((child.priority, count, child))


		if not self.path:
			self.print_impossible
		print ('HOORAY ROBBIE! YOURE THE GREATEST!!! Final Path: ' + str(self.path))
		print ('Total Distance Travelled: ' + str(self.dist))

puckstart = NodeState('start', [12, 57], [1,2])
target = NodeState(13, [200,30], [11,12])
node1 = NodeState(1, [50, 8], [3])
# print('this is node 1', str(node1))
node2 = NodeState(2, [50, 75], [5,6])
# print('this is node 2', str(node2))
# node2.cost = 15
# print('this is node 2', str(node2))
node3 = NodeState(3, [69, 8], [1,7])
node4 = NodeState(4, [69, 42], [5,7])
node5 = NodeState(5, [65, 42], [2,4])
# node5.cost = 18
node6 = NodeState(6, [65, 98], [2,8])
node7 = NodeState(7, [85, 23], [3,4,9,10])
node8 = NodeState(8, [85, 98], [6,9,10])
node9 = NodeState(9, [123, 12], [7,8,11])
node10 = NodeState(10, [123, 83], [7,8,12])
node11 = NodeState(11, [144, 12], [9, 13])
node12 = NodeState(12, [144, 83], [10, 13])

node_list = NodeList()
node_list.extend([node1, node2, node3, node4, node5, node6, node7, node8, node9, node10, node11, node12, target])
node_list.print_list()

solve_test = AStarSolver(puckstart, target, node_list)
solve_test.solve()
# nodelist.print_list()
# # print('Nodes Unsorted ', str(nodelist))
# print ('\nUnsorted list\n' + '\n'.join(map(str, nodelist)))
# nodelist.sort(key= getCost)
# print ('\nSorted list\n' + '\n'.join(map(str, nodelist)))
# # print('Nodes sorted by cost ', str(nodelist))