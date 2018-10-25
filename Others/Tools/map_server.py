#!/usr/bin/env python

import libGaussLocalGeographicCS as MapServer

mapserver = MapServer.GaussLocalGeographicCS( 0.0014744, -0.0017707 )

lattitude = -0.0014948
longititude = -0.000192
height = 0.0

xyz = mapserver.llh2xyz( lattitude, longititude, height)

print( "x: " + str(xyz[0]) + " y: " + str(xyz[1]) + " z: " + str(xyz[2]) )

lattitude = -0.0005519
longititude = -0.0009406
height = 0.0

xyz = mapserver.llh2xyz( lattitude, longititude, height)

print( "x: " + str(xyz[0]) + " y: " + str(xyz[1]) + " z: " + str(xyz[2]) )

x = 22.1813 / 1.0017
y = -330.5 / 1.0085
z = 0.0

llh = mapserver.xyz2llh(x,y,z)

print( "x: " + str(llh[0]) + " y: " + str(llh[1]) + " z: " + str(llh[2]) )

x = 14.14
y = -2.0
z = 0.0

llh = mapserver.xyz2llh(x,y,z)

print( "x: " + str(llh[0]) + " y: " + str(llh[1]) + " z: " + str(llh[2]) )


