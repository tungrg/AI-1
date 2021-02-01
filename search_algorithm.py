import pygame
import graphUI
from node_color import white, yellow, black, red, blue, purple, orange, green
import queue
import math
import heapq
"""
Feel free print graph, edges to console to get more understand input.
Do not change input parameters
Create new function/file if necessary
"""
#dinh hien tai mau vang
#dinh trong tap mo mau do
#dinh trong tap dong mau xanh duong

def BFS(graph, edges, edge_id, start, goal):
    """
    BFS search
    """
    # TODO: your code
    #save explored node
    track = [0] * len(graph)
    #
    explore = []
    explore.append(start)
    hangdoi = queue.Queue()
    hangdoi.put(start)
    
    if(start == goal):
        print("Nothing to do.")
        return
    while hangdoi:
        temp = hangdoi.get()
        DoiMau(graph,temp, yellow)
        for i in graph[temp][1]:
            if i not in explore:
                track[i] = temp
                hangdoi.put(i)
                edges[edge_id(temp,i)][1] = white
                DoiMau(graph,i,red)
                # return path if neighbour is goal
                if i == goal:
                    TrackTrack(graph, edges, edge_id, start, goal, track)
                    return
            # mark node as explored
            explore.append(i)
        DoiMau(graph, temp, blue)
    print("Implement BFS algorithm.")
    pass
def DoiMau(graph, node, mau):
    graph[node][3] = mau
    graph[node][2] = white
    graphUI.updateUI()
    pygame.time.delay(500)

def TrackTrack(graph, edges, edge_id, start, goal, track):
    temp = goal
    graph[start][3] = orange
    graph[goal][3] = purple
    while(temp != start):
        edges[edge_id(temp, track[temp])][1] = green
        temp = track[temp]
    graphUI.updateUI()
def DFS(graph, edges, edge_id, start, goal):
    """
    DFS search
    """
    # TODO: your code
    print("Implement DFS algorithm.")
    #track = [0] * len(graph)
    #
    #explore = [False] * len(graph)
    stack = [(start, [start])]
    if(start == goal):
        print("Nothing to do.")
        return

    while stack:
        print(stack)
        (vertex, path) = stack.pop()
        #if goal in path:
        #    return DFStrack(graph, edges, edge_id, start, goal,path)
        DoiMau(graph,vertex, yellow)
        templist = list(set(graph[vertex][1]) - set(path))
        templist.sort(reverse=True)
        for i in templist:
            temp = list(path)
            temp.append(i)
                #TrackTrack(graph, edges, edge_id, start, goal,path)
                #print(path)
            stack.append(((i,temp)))
            DoiMau(graph, i, red)
            edges[edge_id(vertex,i)][1] = white
            if i == goal:
                path.append(i)
                return DFStrack(graph, edges, edge_id, start, goal,path)
        #edges[edge_id(vertex,stack[-1][0])][1] = white
        #DoiMau(graph, stack[-1], red)
        
        print("-------------------")
        DoiMau(graph, vertex, blue)
    pass
def DFStrack(graph, edges, edge_id, start, goal,path):
    graph[start][3] = orange
    graph[goal][3] = purple
    for i in range(0,len(path)-1):
        edges[edge_id(path[i], path[i+1])][1] = green
    graphUI.updateUI()
def weight(graph,a,b):
    return math.sqrt((graph[a][0][0] - graph[b][0][0])**2 + (graph[a][0][1] - graph[b][0][1])**2)

class MinHeap(object):
    def __init__(self):
        self.h = []
        self.nodes = {}
        self.counter = 0

    def is_empty(self):
        return not self.counter > 0
    def insert(self,node, weights):
        if node in self.nodes:
            self.remove(node)
        entry = [weights, node, True]
        self.counter += 1
        self.nodes[node] = entry
        heapq.heappush(self.h, entry)
    def remove(self, node):
        entry = self.nodes[node]
        entry[-1] = False
        self.counter -= 1
    def pop(self):
        while self.h:
            weight, node, is_active =  heapq.heappop(self.h)
            if is_active:
                self.counter -= 1
                del self.nodes[node]
                return (node, weight)
            
def UCS(graph, edges, edge_id, start, goal):
    """
    Uniform Cost Search search
    """
    # TODO: your code
    print("Implement Uniform Cost Search algorithm.")
    heap = MinHeap()
    heap.insert(start,0)
    min_weight = list(range(0,len(graph)))
    for i in range(0, len(graph)):
        min_weight[i] = float('inf')
    min_weight[start] = 0
    track = list(range(0,len(graph)))
    while not heap.is_empty():
        temp = heap.pop()
        DoiMau(graph, temp[0], yellow)
        if(temp[0] == goal):
            return TrackTrack(graph, edges, edge_id, start, goal, track)
        
        for i in graph[temp[0]][1]:
            if(min_weight[i] > min_weight[temp[0]] + weight(graph,temp[0], i)):
                min_weight[i] = min_weight[temp[0]] + weight(graph,temp[0],i)
                heap.insert(i,min_weight[i])
                edges[edge_id(temp[0],i)][1] = white
                DoiMau(graph, i, red)
                track[i] = temp[0]
        DoiMau(graph, temp[0], blue)
    pass

def AStar(graph, edges, edge_id, start, goal):
    """
    A star search
    """
    # TODO: your code
    print("Implement A* algorithm.")
    heap = MinHeap()
    
    min_weight = list(range(0,len(graph)))
    for i in range(0, len(graph)):
        min_weight[i] = float('inf')
    min_weight[start] = 0
    heuristic = list(range(0,len(graph)))
    for i in range(0,len(graph)):
        heuristic[i] = weight(graph,i,goal)
    min_combine = list(range(0,len(graph)))
    for i in range(0, len(graph)):
        min_combine[i] = float('inf')
    min_combine[start] = heuristic[start]
    heap.insert(start,min_combine[start])
    track = list(range(0,len(graph)))
    while not heap.is_empty():
        temp = heap.pop()
        DoiMau(graph, temp[0], yellow)
        if(temp[0] == goal):
            return TrackTrack(graph, edges, edge_id, start, goal, track)
        for i in graph[temp[0]][1]:
            if(min_weight[i] > min_weight[temp[0]] + weight(graph,temp[0], i)):
                min_weight[i] = min_weight[temp[0]] + weight(graph,temp[0],i)
                min_combine[i] = min_weight[i] + heuristic[i]
                heap.insert(i,min_combine[i])
                edges[edge_id(temp[0],i)][1] = white
                DoiMau(graph, i, red)
                track[i] = temp[0]
        DoiMau(graph, temp[0], blue)
    pass


def example_func(graph, edges, edge_id, start, goal):
    """
    This function is just show some basic feature that you can use your project.
    @param graph: list - contain information of graph (same value as global_graph)
                    list of object:
                     [0] : (x,y) coordinate in UI
                     [1] : adjacent node indexes
                     [2] : node edge color
                     [3] : node fill color
                Ex: graph = [
                                [
                                    (139, 140),             # position of node when draw on UI
                                    [1, 2],                 # list of adjacent node
                                    (100, 100, 100),        # grey - node edged color
                                    (0, 0, 0)               # black - node fill color
                                ],
                                [(312, 224), [0, 4, 2, 3], (100, 100, 100), (0, 0, 0)],
                                ...
                            ]
                It means this graph has Node 0 links to Node 1 and Node 2.
                Node 1 links to Node 0,2,3 and 4.
    @param edges: dict - dictionary of edge_id: [(n1,n2), color]. Ex: edges[edge_id(0,1)] = [(0,1), (0,0,0)] : set color
                    of edge from Node 0 to Node 1 is black.
    @param edge_id: id of each edge between two nodes. Ex: edge_id(0, 1) : id edge of two Node 0 and Node 1
    @param start: int - start vertices/node
    @param goal: int - vertices/node to search
    @return:
    """

    # Ex1: Set all edge from Node 1 to Adjacency node of Node 1 is green edges.
    node_1 = graph[1]
    for adjacency_node in node_1[1]:
        edges[edge_id(1, adjacency_node)][1] = green
    graphUI.updateUI()

    # Ex2: Set color of Node 2 is Red
    graph[2][3] = red
    graphUI.updateUI()

    # Ex3: Set all edge between node in a array.
    path = [4, 7, 9]  # -> set edge from 4-7, 7-9 is blue
    for i in range(len(path) - 1):
        edges[edge_id(path[i], path[i + 1])][1] = blue
    graphUI.updateUI()
