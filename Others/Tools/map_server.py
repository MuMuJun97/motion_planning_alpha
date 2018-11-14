#!/usr/bin/env python

import libGaussLocalGeographicCS as MapServer

mapserver = MapServer.GaussLocalGeographicCS( 0.0014744, -0.0017707 )

lattitude = -0.0014948 
longititude = -0.0016851
height = 0.0

xyz = mapserver.llh2xyz( lattitude, longititude, height)

print( "x: " + str(xyz[0]) + " y: " + str(xyz[1]) + " z: " + str(xyz[2]) )

x = xyz[0]
y = xyz[1]
z = xyz[2]
llh = mapserver.xyz2llh(x,y,z)

print( "l: " + str(llh[0]) + " l: " + str(llh[1]) + " h: " + str(llh[2]) )
