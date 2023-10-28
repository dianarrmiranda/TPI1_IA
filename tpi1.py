#STUDENT NAME: Diana Miranda
#STUDENT NUMBER: 107457

#DISCUSSED TPI-1 WITH: (names and numbers):


from tree_search import *
import math

class OrderDelivery(SearchDomain):

    def __init__(self,connections, coordinates):
        self.connections = connections
        self.coordinates = coordinates
        # ANY NEEDED CODE CAN BE ADDED HERE

    def actions(self,state):
        city = state[0]
        actlist = []
        for (C1,C2,D) in self.connections:
            if (C1==city):
                actlist += [(C1,C2)]
            elif (C2==city):
               actlist += [(C2,C1)]
        return actlist 

    def result(self, state, action):
        new_city = action[1]
        visited_cities = state[1] + [new_city]
        return new_city, visited_cities

    def satisfies(self, state, goal):
        # The goal is reached if all target cities have been visited and the delivery is back at the start city
        print(state[0], goal[0])
        print(set(state[1]), set(goal[1]))
        return state[0] == goal[0] and set(state[1]) == set(goal[1])

    def cost(self, state, action):
        for (C1, C2, Cost) in self.connections:
            if ((C1, C2) == action or (C2, C1) == action):
                return Cost

    def heuristic(self, state, goal):
        x1, y1 = self.coordinates[state[0]]
        x2, y2 = self.coordinates[goal[0]]
        return math.sqrt((x2-x1)**2 + (y2-y1)**2)


    


 
class MyNode(SearchNode):

    def __init__(self,state,parent,arg3=None,arg4=None,arg5=None,arg6=None):
        super().__init__(state,parent)
        #ADD HERE ANY CODE YOU NEED
        self.depth = arg3
        self.cost = arg4
        self.heuristic = arg5
        self.eval = arg6
        self.children = []
        self.markedForDel = False
    
    def in_parent(self, newstate):
        if self.parent == None:
            return False
        if self.parent.state == newstate:
            return True
        return self.parent.in_parent(newstate)
        


class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',maxsize=None):
        super().__init__(problem,strategy)
        #ADD HERE ANY CODE YOU NEED
        root = MyNode(problem.initial, None, 0, 0, 0)
        self.open_nodes = [root]
        self.terminals = 0
        self.maxsize = maxsize
        self.totalNodes = 0

    def astar_add_to_open(self,lnewnodes):
        for node in lnewnodes:
            self.open_nodes.append(node)
            self.open_nodes.sort(key= lambda node: (node.cost + node.heuristic, node.state))


    def search2(self):
        while self.open_nodes != []:
            self.terminals = len(self.open_nodes)
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.solution = node
                return self.get_path(node)
            
            lnewnodes = []
            self.non_terminals +=1
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if not node.in_parent(newstate):
                    newnode = MyNode(newstate,node, node.depth + 1, node.cost + self.problem.domain.cost(node.state, a), self.problem.domain.heuristic(newstate, self.problem.goal))
                    lnewnodes.append(newnode)

                    newnode.eval = newnode.cost + newnode.heuristic
                    node.children.append(newnode)  
                    self.totalNodes += 1

            self.add_to_open(lnewnodes)
            self.manage_memory()

        return None
    
    
    

    def manage_memory(self):
        if self.strategy != 'A*' or self.maxsize is None:
            return  
            
        while len(self.open_nodes) > self.maxsize:
            nodeToDel = self.open_nodes.pop(-1)
            nodeToDel.markedForDel = True

            print("Node to delete: ", nodeToDel.state)
            print("total nodes: ", self.totalNodes)
            allToDel = True
            for brother in nodeToDel.parent.children:
                if brother.markedForDel == False:
                    allToDel = False
                    break

            if allToDel:
                for brother in nodeToDel.parent.children:
                    if brother in self.open_nodes: 
                        self.open_nodes.remove(brother)
                    

            
def orderdelivery_search(domain, city, targetcities, strategy='breadth', maxsize=None):
    #ADD YOUR CODE HERE

    pass
    # if needed, auxiliary methods can be added here
