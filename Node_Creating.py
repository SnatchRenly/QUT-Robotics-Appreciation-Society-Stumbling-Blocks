import numpy
import itertools

class Block(object):
	"""A class for the foam blocks"""
	board = [600, 300]
	boundaries = [[[0,0],[600,0]],[[600,0],[600,300]],[[600,300],[0,300]],[[0,300],[0,0]]]

	def __init__(self, centroid, vertices):
		self.centroid = centroid
		self.vertices = vertices

	def expand_bbox(self, radius):
		"""Expands the bounding box of a block by the radius of the end effector"""
		new_vertices = []
		for i in self.vertices:
			# new_x = i[0] + (self.centroid[1]-i[1]/self.centroid[0]-i[0])*radius
			new_x = i[0] + numpy.sign(i[0]-self.centroid[0])*radius
			new_y = i[1] + numpy.sign(i[1]-self.centroid[1])*radius
			# new_y = i[1] + radius
			new_vertex = [new_x, new_y]
			new_vertices.append(new_vertex)
		self.vertices = new_vertices

	def __str__(self):
		"""Returns the string form of Block"""
		return str(self.centroid) + str(self.vertices)

	def get_edges(self):
		"""	Creates and returns a list of edges for the block bounding box"""
		self.edges = []
		for i in range(0, len(self.vertices) -1):
			new_edge = [self.vertices[i], self.vertices[i+1]]
			self.edges.append(new_edge)
		new_edge = [self.vertices[-1], self.vertices[0]]
		self.edges.append(new_edge)
		# return list(self.edges)
		return self.edges

	def cell_decomp(self, obstrucion_edges):
		"""Performs trapezoidal cell decomposition for individual block"""
		self.block_nodes = []
		self.obstrucion_edges = obstrucion_edges
		for x in range(0,1):
			new_line = [[self.vertices[x], [self.vertices[x][0], 0]]
			for edge in self.obstrucion_edges:
				if new_line intersects
				new_line = [[self.vertices[x], intersecting point of line]
			node_co_ords = 	[self.vertices[x][0], average (self.vertices[x][1], intersecting point of line[1]]
		self.block_nodes.append(node_co_ords)

		for x in range(2,3):
			new_line = [[self.vertices[x], [self.vertices[x][0], 300]]
			for lines in global_lines:
				if new_line intersects
				new_line = [[self.vertices[x], intersecting point of line]
			node_co_ords = 	[self.vertices[x][0], average (self.vertices[x][1], intersecting point of line[1]]
		self.block_nodes.append(node_co_ords)
		




Class Scene(self):

	end_effector_r = 5

	def __init__(self, pucks, targets, immovable_blocks = 0, movable_blocks = 0):
		"""initialises a scene ready for solving"""

		self.pucks = pucks
		self.targets = targets
		self.immovable_blocks = immovable_blocks
		self.movable_blocks = movable_blocks
		self.edges = []

	def expand_obstructions(self):
		for blocks in self.immovable_blocks:
			expand_bbox(self.end_effector_r)
		for blocks in self.movable_blocks:
			expand_bbox(self.end_effector_r)

	def get_edges(self)
		for block in self.immovable_blocks:
			new_edges = block.get_edges()
			self.edges.extend(new_edges)
		for block in self.movable_blocks:
			new_edges = block.get_edges()
			self.edges.extend(new_edges)

	def create_nodes(self):
		self.node_list = NodeList()
		for x in self.immovable_blocks:

		#add nodes from immovable blocks
		#add nodes from moveable blocks


end_effector_r = 5

block1 = Block([62,25], [[50,17],[70,17],[70,32],[50,32]])
print (str(block1))
block2 = Block([75,62], [[65,51],[85,51],[85,72],[65,72]])
print (str(block2))
block3 = Block([134,35], [[124,24],[144,24],[144,44],[124,44]])
print (str(block3))
block1.expand_bbox(end_effector_r)
print (str(block1))	
block2.expand_bbox(end_effector_r)
print (str(block1))	
# block3.expand_bbox(end_effector_r)
print (str(block3))	

print str(block3.get_edges())

# block1.get_lines()

# class NodeCreator:
	
# 	def __init__(self, block_list):
# 		self.block_list = block_list

# 	# def cell_decomp(self):
# 	# decomposes lines


# 	def solve(self):
# 		for i in block_list:
# 			for j in i.vertices
# 			# look at x co-ord and then see if any y vector woul





