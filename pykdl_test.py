from PyKDL import *
print "Creating Robotic Chain"
chain=Chain()
joint0=Joint(Joint.RotZ) 
frame0=Frame(Vector(0.2,0.3,0))
segment0=Segment(joint0,frame0)
chain.addSegment(segment0) 
#Inertia zero (Don't want to mess with dynamics yet)
joint1=joint0 #Iqual joint
frame1=Frame(Vector(0.4,0,0))
segment1=Segment(joint1,frame1)
chain.addSegment(segment1)
joint2=joint1 #Iqual joint
frame2=Frame(Vector(0.1,0.1,0))
segment2=Segment(joint2,frame2)
chain.addSegment(segment2)
print "Forward kinematics"
jointAngles=JntArray(3)
jointAngles[0]=0.5236
jointAngles[1]=0.5236
jointAngles[2]=-1.5708
fk=ChainFkSolverPos_recursive(chain)
finalFrame=Frame()
fk.JntToCart(jointAngles,finalFrame)
print "Rotational Matrix of the final Frame: "
print  finalFrame.M
print "End-effector position: ",finalFrame.p