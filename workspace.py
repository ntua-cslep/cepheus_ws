# References :

#Avoiding Dynamic Singularities in Cartesian Motions of Free-Floating Manipulators 
#(KOSTAS NANOS ,EVANGELOS PAPADOPOULOS, Senior Member, IEEE National Technical University of Athens Athens, Greece)


import math
import matplotlib.pyplot as plt

print "Starting the calculations of left arm's workspace "
print

m0 = 13
m1 = 0.06
m2 = 0.106
m3 = 0.08

l1 = 0.181004
l2 = 0.1605
l3 = 0.0499


I0 = 0.1196
I1 = 0.0002
I2 = 0.00027
I3 = 0.000037

r0 = 0.1746
r1 = 0.0817
r2 = 0.04386
r3 = 0.014


M = m0 + m1 + m2 + m3

a = m0 * r0 / M
b = (l1 * m0 + (m0 + m1) * r1) / M
c = (l2 * (m0 + m1) + r2 * (m0 + m1 + m2)) / M
d = l3 * (m0 + m1 + m2) / M + r3



a00 =  I0 + m0 * (m1 + m2 + m3) * r0**2 / M
a01 = m0 * r0 * (l1 * (m1 + m2 + m3) + r1 * (m2 + m3)) / M
a02 = m0 * r0 * (l2 * (m2 + m3) + r2 * m3) / M
a11 = I1 + (m0 * m1 * l1**2 + m1 * (m2 + m3) * r1**2 + m0 * (m2 + m3) * (l1 + r1)**2) / M
a21 = (l1 * m0 + (m0 + m1) * r1) * (l2 * (m2 + m3) + m3 * r2) / M
a22 = I2 + (m2 * m3 * r2**2 + m2 * (m0 + m1) * l2**2 + m3 * (m0 + m1) * (l2 + r2)**2) / M
a33 = I3 + m3 * (m0 + m1 + m2) * l3**2 / M



def k0(q):
	return ( b * (a * a22 - c * a02) * math.sin(q)  )

def k1(q):
	return ( -(a * b * a02 + a * c * a01 - 2 * b * c * a00)/2 + c * (-a * a11 + b * a01) * math.cos(q) + a * (b * a02 - c*a01) * math.cos(2*q)/2)

def k2(q):
	return( a * (b * a21 - c * a11)*math.sin(q) + a * (b * a02 - c * a01) * math.sin(2*q) / 2 )


#S(q1, q2) = S0
#the equation that zeroes the det of S matrix
#equals to zero cause we want to find the angles that singularities exist
def S0():
	return 0



def q2_1(q1):
	return ( math.pi - math.asin( (S0() - k0(q1))/(math.sqrt(k1(q1)**2 + k2(q1)**2)) ) - math.atan(k2(q1)/k1(q1)) )


def q2_2(q1):
	return ( math.asin( (S0() - k0(q1))/(math.sqrt(k1(q1)**2 + k2(q1)**2)) ) - math.atan(k2(q1)/k1(q1)) )




def R(q1, q2):
	return ( math.sqrt(a**2 + b**2 + c**2 + 2*a*b*math.cos(q1) + 2*a*c*math.cos(q1+q2) + 2*b*c*math.cos(q2)) )



q2_1_solutions = []
q2_2_solutions = []

q=0
while q <= 2*math.pi:
	#print q2_1(q)
	q2_1_solutions.append(q2_1(q))
	q = q + 0.01

q=0
while q <= 2*math.pi:
        #print q2_2(q)
        q2_2_solutions.append(q2_2(q))
        q = q + 0.01


q1_q2_mapping = []
q1=0
counter = 0
q1_values = []
while q1 <= 2*math.pi:
        
	q1_values.append(q1)
        q1_q2_mapping.append((q1, q2_1_solutions[counter], q2_2_solutions[counter]))
	q1 = q1 + 0.01
	counter = counter + 1


#plt.plot(q1_values, q2_1_solutions, 'r', q1_values, q2_2_solutions, 'b')
#plt.show()



# Mechanical restrictions of the second joint (elbow): -172 degs <= q2 <= 95 degs
# This means that independently from the value of q2_max that we found the q2_max value is 95 degs
# So the q2 = -172 degs ,is the value of the angle of the elbow in order to calculate the Rmin value of the PI Workspace
# The R_min value is calculates from the beggining of the left shouldre. NOT the COM of the system
Rmin = l1 + l2 * math.cos(-172 * math.pi / 180)

print "Rmin : "
print Rmin 
print
# Also we see that close to q2 = 0 we have some small angles that can cause sigularities
# We calculate the min and max values that make these restrictions
# So q2 = -5 degs

Rmax = l1 + l2 * math.cos(-5 * math.pi /180)

print "Rmax : "
print Rmax

