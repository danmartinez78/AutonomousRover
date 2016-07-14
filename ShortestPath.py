#!/usr/bin/python

import numpy as np
import yaml

def dijkstras(occupancy_map,x_spacing,y_spacing,start,goal):
    """
    Implements Dijkstra's shortest path algorithm
    Input:
    occupancy_map - an N by M numpy array of boolean values (represented
        as integers 0 and 1) that represents the locations of the obstacles
        in the world
    x_spacing - parameter representing spacing between adjacent columns
    y_spacing - parameter representing spacing between adjacent rows
    start - a 3 by 1 numpy array of (x,y,theta) for the starting position 
    goal - a 3 by 1 numpy array of (x,y,theta) for the finishing position 
    Output: 
    path: list of the indices of the nodes on the shortest path found
        starting with "start" and ending with "end" (each node is in
        metric coordinates)
    """
    # Your  code  here
    [nrows,ncols] = occupancy_map.shape
    mymap = np.zeros((nrows,ncols))

    distanceFromStart = np.zeros((nrows,ncols))
    distanceFromStart = distanceFromStart + 100000000
    #print(distanceFromStart)
    mymap[occupancy_map==0] = 1
    mymap[occupancy_map==1] = 2
    #print(mymap)

    start_node_i = round((start[1,0] / y_spacing) - 0.5)
    start_node_j = round((start[0,0] / x_spacing) - 0.5)
    dest_node_i = round((goal[1,0] / y_spacing) - 0.5)
    dest_node_j = round((goal[0,0] / x_spacing) - 0.5)

    mymap[(start_node_i, start_node_j)] = 5
    mymap[(dest_node_i, dest_node_j)] = 6
    distanceFromStart[(start_node_i, start_node_j)] = 0

    #print(distanceFromStart)
    #print(occupancy_map)
    #print(mymap)

    parent = np.zeros((nrows, ncols))
    numExpanded = 0
    [imax, jmax] = occupancy_map.shape
    imax = imax-1
    jmax = jmax-1
    imin = 0
    jmin = 0
    delta_i = [1, -1, 0, 0]
    delta_j = [0, 0, 1, -1]
    edge = [y_spacing, y_spacing, x_spacing, x_spacing]

    while (True):
        min_dist = distanceFromStart.min()
        current = distanceFromStart.argmin()
        i,j = np.unravel_index(distanceFromStart.argmin(), distanceFromStart.shape)
        #print(i, j)
        #print(min_dist)
        #test = ravel_index(current, distanceFromStart.shape)
        if (i==dest_node_i and j==dest_node_j) or (distanceFromStart[(i, j)] > 1000000):
            print("DONE")
            break
        mymap[(i,j)] = 3
        distanceFromStart[(i, j)] = 100000000
        for index in range(0, len(delta_i)):
            if (i + delta_i[index] <= imax) and (i + delta_i[index] >= imin) and (j + delta_j[index] <= jmax) and (j + delta_j[index] >= jmin):
                if (mymap[i + delta_i[index], j + delta_j[index]] != 3 and mymap[i+delta_i[index], j+delta_j[index]] != 4 and mymap[i+delta_i[index], j+delta_j[index]] != 5 and mymap[i+delta_i[index], j+delta_j[index]] != 2):
                    if (distanceFromStart[i + delta_i[index], j + delta_j[index]] > min_dist + edge[index]):
                        distanceFromStart[i + delta_i[index], j + delta_j[index]] = min_dist + edge[index]
                        parent[i + delta_i[index], j + delta_j[index]] = current
                    mymap[i + delta_i[index], j + delta_j[index]] = 4
        numExpanded = numExpanded + 1

    print("Finding Route")
    print(parent)
    if (distanceFromStart[(dest_node_i, dest_node_j)]>1000):
        route = np.array
        print("No Path")
        return(route)
    else:
        route = np.array([(dest_node_i, dest_node_j)])
        while (parent[(route[0,0], route[0,1])] != 0):
            loc = int(parent[(route[0,0], route[0,1])])
            data_shape = occupancy_map.shape
            [pi,pj] = np.unravel_index(loc, data_shape)
            new_parents = np.array([(pi,pj)])
            route = np.vstack((new_parents, route))
    #print(route)
    [rows, cols] = route.shape
    routexy = np.zeros((rows,cols))
    for i in range(0, rows):
        routexy[(i, 0)] = (route[(i, 1)] + 0.5) * x_spacing
        routexy[(i, 1)] = (route[(i, 0)] + 0.5) * y_spacing
    routexy = np.vstack(([(start[0,0], start[1,0])], routexy))
    print(routexy)
    return(routexy)
    pass

def test():
    """
    Function that provides a few examples of maps and their solution paths
    """
    print("test 1")
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 0, 0, 0, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 0.13
    y_spacing1 = 0.2
    start1 = np.array([[0.3], [0.3], [0]])
    goal1 = np.array([[0.6], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    true_path1 = np.array([
        [ 0.3  ,  0.3  ],
        [ 0.325,  0.3  ],
        [ 0.325,  0.5  ],
        [ 0.325,  0.7  ],
        [ 0.455,  0.7  ],
        [ 0.455,  0.9  ],
        [ 0.585,  0.9  ],
        [ 0.600,  1.0  ]
        ])
    if np.array_equal(path1,true_path1):
      print("Path 1 passes")

    print("test 2")
    test_map2 = np.array([
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 1, 1, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.5], [1.0], [1.5707963267948966]])
    goal2 = np.array([[1.1], [0.9], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    true_path2 = np.array([[ 0.5,  1.0],
                           [ 0.5,  1.1],
                           [ 0.5,  1.3],
                           [ 0.5,  1.5],
                           [ 0.7,  1.5],
                           [ 0.9,  1.5],
                           [ 1.1,  1.5],
                           [ 1.1,  1.3],
                           [ 1.1,  1.1],
                           [ 1.1,  0.9]])
    if np.array_equal(path2,true_path2):
      print("Path 2 passes")

def test_for_grader():
    """
    Function that provides the test paths for submission
    """
    test_map1 = np.array([
              [1, 1, 1, 1, 1, 1, 1, 1, 1],
              [1, 0, 1, 0, 0, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 1, 0, 1, 0, 1, 0, 1],
              [1, 0, 0, 0, 1, 0, 0, 0, 1],
              [1, 1, 1, 1, 1, 1, 1, 1, 1]])
    x_spacing1 = 1
    y_spacing1 = 1
    start1 = np.array([[1.5], [1.5], [0]])
    goal1 = np.array([[7.5], [1], [0]])
    path1 = dijkstras(test_map1,x_spacing1,y_spacing1,start1,goal1)
    s = 0
    for i in range(len(path1)-1):
      s += np.sqrt((path1[i][0]-path1[i+1][0])**2 + (path1[i][1]-path1[i+1][1])**2)
    print("Path 1 length:")
    print(s)


    test_map2 = np.array([
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 0, 0, 0, 0, 0, 0, 1],
            [1, 1, 1, 1, 1, 1, 1, 1]])
    start2 = np.array([[0.4], [0.4], [1.5707963267948966]])
    goal2 = np.array([[0.4], [1.8], [-1.5707963267948966]])
    x_spacing2 = 0.2
    y_spacing2 = 0.2
    path2 = dijkstras(test_map2,x_spacing2,y_spacing2,start2,goal2)
    s = 0
    for i in range(len(path2)-1):
      s += np.sqrt((path2[i][0]-path2[i+1][0])**2 + (path2[i][1]-path2[i+1][1])**2)
    print("Path 2 length:")
    print(s)



def main():
    # Load parameters from yaml
    param_path = 'params.yaml' # rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    # Get params we need
    occupancy_map = np.array(params['occupancy_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']
    path = dijkstras(occupancy_map,x_spacing,y_spacing,pos_init,pos_goal)
    print(path)

if __name__ == '__main__':
    main()

