import numpy as np
import networkx as nx
import pyqtree

class Discretization:

    def __init__(self, x_low, x_high, y_low, y_high, step_x = 0.7, step_y = 0.7):
        x_vals = np.arange(x_low, x_high + step_x, step_x)
        y_vals = np.arange(y_low, y_high + step_y, step_y)

        grid_tmp = list()

        self.removed = set()

        for x in list(x_vals):
            row = list()
            for y in list(y_vals):
                row.append([x, y])
            grid_tmp.append(row)
        
        self.grid = np.array(grid_tmp)
        self.G = nx.Graph()

        self.spindex = pyqtree.Index(bbox=(x_low - 2 * step_x, y_low - 2 * step_y, x_high + 2 * step_x, y_high + 2 * step_y))

        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                self.G.add_node(self.__get_node_num(i, j), pos=self.grid[i,j])
                self.spindex.insert((i, j), (self.grid[i, j][0] - step_x / 2.0, self.grid[i, j][1] - step_y / 2.0, self.grid[i, j][0] + step_x / 2.0, self.grid[i, j][1] + step_y / 2.0))

        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                if i + 1 < self.grid.shape[0]:
                    self.G.add_edge(self.__get_node_num(i, j), self.__get_node_num(i + 1, j), weight=self.__get_node_weight(i, j, i + 1, j))
                if j + 1 < self.grid.shape[1]:
                    self.G.add_edge(self.__get_node_num(i, j), self.__get_node_num(i, j + 1), weight=self.__get_node_weight(i, j, i, j + 1))

    def __get_node_num(self, i, j):
        return j * self.grid.shape[0] + i

    def __get_node_weight(self, i1, j1, i2, j2):
        return np.linalg.norm(self.grid[i1][j1] - self.grid[i2][j2])

    def get_nearest_node(self, pos):
        arr = self.spindex.intersect((pos[0], pos[1], pos[0], pos[1]))
        if len(arr) < 1:
            return -1
        return self.__get_node_num(arr[0][0], arr[0][1]) 

    def get_node_loc(self, p):
        return self.G.nodes[p]['pos'] 

    def draw(self):
        pos=nx.get_node_attributes(self.G,'pos')
        nx.draw(self.G,pos)
        labels = nx.get_edge_attributes(self.G,'weight')
        nx.draw_networkx_edge_labels(self.G,pos,edge_labels=labels)   

    def get_path(self, start_node, end_node):
        path = nx.astar_path(self.G, start_node, end_node, heuristic = lambda x,y: np.linalg.norm(self.G.nodes[x]["pos"] - self.G.nodes[y]["pos"]))
        return [self.G.nodes[p]['pos'] for p in path]
    
    def remove_node(self, node):
        print("Removed node", node)
        self.removed.add(node)
        for edge in self.G.edges:
            if edge[0] == node or edge[1] == node:
                self.G.edges[edge]['weight'] = len(self.G.nodes) * 2.0

    def is_removed(self, node):
        return node in self.removed
    
    def num_nodes(self):
        return len(self.G.nodes)