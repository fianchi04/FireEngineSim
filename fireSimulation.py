import networkx as nx
import math
import random
import matplotlib.pyplot as plt

#SETUP: DEFINE NODE NETWORK
def add_nodes():
    #add nodes
    G.add_nodes_from([0, 1, 2, 3, 4, 5, \
         6, 7, 8, 9, 10, 11, \
         12, 13, 14, 15, 16, 17, \
         18, 19, 20, 21, 22, 23, \
         24, 25, 26, 27, 28, 29,\
         30, 31, 32, 33, 34, 35])

    #add node coordinates
    for n in G:
        x, y = n // 6, n % 6
        G.node[n]['pos'] = (x, y) #save for graph to plot

        G.node[n]['x'] = x #save for heuristic to access
        G.node[n]['y'] = y

    #add edges
    for a in G.nodes():
        if a not in (5, 11, 17, 23,29, 35):
            G.add_edge(a, a+1)
        if a not in (30, 31, 32, 33, 34, 35):
            G.add_edge(a, a+6)
        if a not in (5, 11, 17, 23, 29, 30, 31, 32, 33, 34, 35):
            G.add_edge(a, a+7)
        if a not in (0, 6, 12, 18, 24, 30, 31, 32, 33, 34, 35):
            G.add_edge(a, a+5)

#DIAGONAL HEURISTIC
def heuristic(a, b):
    gx, gy = G.node[a]['x'], G.node[a]['y']
    nx, ny = G.node[b]['x'], G.node[b]['y']
    D = 1 #NWSE distance
    D2 = 1.41 #diag distance
    dx = abs(nx - gx)
    dy = abs(ny - gy)
    return (D * (dx + dy) + (D2 -2 * D) * min(dx, dy))

#CALCULATE DISTANCE BETWEEN NODES
def cost (from_node_x, from_node_y, to_node_x, to_node_y):
   xdist = to_node_x - from_node_x
   ydist = to_node_y - from_node_y
   return (math.sqrt(xdist*xdist + ydist*ydist))

#ALGORITHM TO FIND SHORTEST PATH
def a_star_search(G, start, end):   
    #initialise open list
    priorityList = {}
    #initialise closed list
    closed_nodes = {}
    #put starting node on open list
    priorityList[start]=0
    #initialise cost list
    cost_so_far = {}
    #no previous path
    closed_nodes[start] = None
    cost_so_far[start] = 0
    #lists for paths:
    eVisited= [] #edges which have been considered
    ePath = [] #edges added to final path

    
    #while open list is not empty
    while (len(priorityList) != 0):
        #get lowest value from priority list ie predicted shortest distance to goal
        minx = 1000
        for a in priorityList:
            if(priorityList[a] < minx):
                minx = priorityList[a]
                tempSave= a

        #pop item from dict
        current = tempSave
        priorityList.pop(tempSave)
        
        #stop search if we're at the goal
        if current==end:
            break

        #for each neighbour of the node we are currently considering
        for next in G.neighbors(current):
            #print("test current[x]: ", current('x'))
            new_cost = cost_so_far[current] + cost(G.node[current]['x'], G.node[current]['y'], G.node[next]['x'], G.node[next]['y'])
            #print('cost between '+ str(current) + ' and ' + str(next) + ' = ' + str(new_cost))
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                #add list to provisional final path if destination node is either not on node list, or the way to get to it is shortest than existing saved one
                cost_so_far[next] = new_cost
                #print('minimal cost for start to ' + str(next) +  ' found')
                #add edge to the list of considered edges
                eVisited.append((current, next))
                #calculate priority for using this edge in path
                priority = new_cost + heuristic(end, next)
                #add node to priority list
                priorityList[next] = priority
                #add node to list of nodes:shortest route to node from start
                closed_nodes[next] = current
            #call animate function to update paths
            animate(eVisited, ePath, start, end)


    #retrieve final path by working out linking pair relationships
    v = closed_nodes[end]
    ePath.append((end, closed_nodes[end]))
    while v != start:
        ePath.append((v, closed_nodes[v]))
        #call animate for updated final path
        animate(eVisited, ePath, start, end)
        v = closed_nodes[v]
    print(ePath)

    return ePath, eVisited




#animation function
def animate(eVisited, ePath, a, f):
    #draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=200, node_color="#6716A6")
    #tried routes dynamic
    nx.draw_networkx_edges(G, pos, edgelist=eVisited, width = 6, edge_color="#7be016")
    #path found dynamic
    nx.draw_networkx_edges(G, pos, edgelist=ePath, width = 6, edge_color="#072666")

    

    
    #draw actor
    nx.draw_networkx_nodes(G, pos, nodelist=[a],node_shape='s', node_color="#07EEF2")
    #draw fire
    if f!=None:
        nx.draw_networkx_nodes(G, pos, nodelist=[f],node_shape='^', node_color="#F24A07")
    
    plt.pause(0.01)
    return




#MAIN
G=nx.Graph()

#setup node network
add_nodes()
#get node positions
pos=nx.get_node_attributes(G, 'pos')
plt.grid('on')
#disable axis
plt.axis('off')
#draw nodes
nx.draw_networkx_nodes(G, pos, node_size=150, node_color="#6716A6")
#draw labels
nx.draw_networkx_labels(G, pos, font_size=6, font_family='sans-serif', font_color="#DDD5E3")
#draw all edges
#nx.draw_networkx_edges(G, pos, width=3)



        

actor = 0
fire = 35
#loop to iterate through repetitive animation cycle
for x in range(0,15):
    plt.grid('on')
    #disable axis
    plt.axis('off')
    #draw nodes
    nx.draw_networkx_nodes(G, pos, node_size=150, node_color="#6716A6")
    #draw labels
    nx.draw_networkx_labels(G, pos, font_size=6, font_family='sans-serif', font_color="#DDD5E3")

    #call main method              
    ePath, eVisited = a_star_search(G, actor, fire)

    #calculate movement of actor
    check = False
    z = len(ePath)
    for c in range (0,int(z)):
        print("for loop reached")
        a,b = ePath.pop()
        print(a,b)
        animate(eVisited, ePath, b, fire)
        animate(eVisited, ePath, a, fire)
        #update var holder for actor position
        actor=a


    #run animate once to let it clear
    animate([], [], a, None)

    #get new val for fire
    fire = random.randint(0, 35)

    #clear graph for next loop
    plt.clf()

    




#end of program clear graph
G.clear()
