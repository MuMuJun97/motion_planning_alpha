#!/usr/bin/env python
import matplotlib.pyplot as plt


def show_tree():
    sums = []
    parents = []
    children = []

    with open( 'logs/tree.txt', 'rt' ) as tree_file :
        for line in tree_file :
            splits = line.split( '\n' )[0].split( ' ' )
            if len(splits) == 1:
                sums.append( int( splits[0] ) )
                continue
            parent_x = float( splits[0].split( ',' )[0] )
            parent_y = float( splits[0].split( ',' )[1] )
            parent = [ parent_x, parent_y ]
            parents.append( parent )

            child_x = float( splits[1].split( ',' )[0] )
            child_y = float( splits[1].split( ',' )[1] )
            child = [ child_x, child_y ]
            children.append( child )

    for no in range( len( sums ) ):
        gap = sum(sums[0:no])
        number = sums[no]
        p_parts = parents[gap: gap + number]
        c_parts = children[gap: gap + number]
        for parent, child in zip( p_parts, c_parts ) :
            plt.scatter( parent[0], parent[1] )
            plt.scatter( child[0], child[1] )
            plt.plot( [ parent[0], child[0] ], [ parent[1], child[1] ] , linewidth=2, color = 'black')


def show_waypoints():
    sums = []
    points = []

    with open( 'logs/waypoints.txt', 'rt' ) as wps_file:
        for line in wps_file:
            splits = line.split( '\n' )[0].split( ',' )
            if len(splits) == 1:
                sums.append( int( splits[0] ) )
                continue
            point_x = float( splits[0] )
            point_y = float( splits[1] )
            point = [ point_x, point_y ]
            points.append( point )

    for no in [1,2]:
        points_X = []
        points_Y = []
        gap = sum(sums[0:no])
        number = sums[no]
        parts = points[gap: gap + number]
        for point in parts:
            points_X.append( point[0] )
            points_Y.append( point[1] )
        plt.plot( points_X, points_Y ,linewidth=5)

def show_reference_path():
    nodes = []
    nodes_X = []
    nodes_Y = []
    sums = []
    
    with open( 'logs/reference_path.txt', 'rt' ) as pos_file:
        for line in pos_file:
            splits = line.split( '\n' )[0].split( ',' )
            if len(splits) == 1:
                sums.append( int( splits[0] ) )
                continue
            node_x = float( splits[0] )
            node_y = float( splits[1] )
            node = [ node_x, node_y ]

            nodes_X.append( node_x )
            nodes_Y.append( node_y )
            nodes.append( node )

    for index in range(len( sums) ) :
        node = nodes[ index ]
        if index == 0:
            plt.scatter( node[0], node[1], s=50, marker='x', c= 'r' , linewidths=2)
        else:
            plt.scatter( node[0], node[1], s=40, marker='s', c= 'r' )
    plt.plot(nodes_X, nodes_Y, linewidth=2, color='red')


def show_positions():
    positions = []
    positions_X = []
    positions_Y = []

    with open( 'logs/positions.txt', 'rt' ) as pos_file:
        for line in pos_file:
            splits = line.split( '\n' )[0].split( ',' )
            position_x = float( splits[0] ) 
            position_y = abs( float( splits[1] ) )
            position = [ position_x, position_y ]
            positions_X.append( position_x )
            positions_Y.append( position_y )
            positions.append( position )
        
    for position in positions:
        plt.scatter( position[0], position[1], s=40, marker='o', c= 'g' )
    plt.plot(positions_X, positions_Y, linewidth=2, color='green')
        


if __name__ == '__main__':
    show_reference_path()
    show_waypoints()
    show_tree()
    show_positions()
    plt.show()
