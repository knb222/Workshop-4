#!/usr/bin/env python
# coding: utf-8

# In[1]:


get_ipython().run_line_magic('pylab', 'inline')


# In[2]:


import sim as vrep
import time
import cv2
import numpy as np

vrep.simxFinish(-1)

clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
errcode,Handle=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)

if clientID!=-1:
    # Handler for the camera
    res, v1 = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_oneshot_wait)
    print ('Getting first image')
    err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_streaming)
    
    
    # Take p pictures
    p = 20
    i = 0
    
    resP, VP = vrep.simxGetObjectHandle(clientID, 'Plane', vrep.simx_opmode_oneshot_wait)
    pics=numpy.empty(p+1,dtype=numpy.ndarray)
    orientation=numpy.empty(p+1,dtype=list)
    
    while i <= p:     
        err, resolution, image = vrep.simxGetVisionSensorImage(clientID, v1, 0, vrep.simx_opmode_buffer)
        if err == vrep.simx_return_ok:
            print ("image %d OK!!!"%i)
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[1],resolution[0],3])
            
            #storing image in an array
            pics[i]=img
            
            #getting orientaion
            returnCodeOrientation,eulerAngles=vrep.simxGetObjectOrientation(clientID, Handle,-1,vrep.simx_opmode_streaming)
            
            #saving the orientation in an array
            orientation[i]=eulerAngles[2]
            
            
            # Show the images using Opencv (uncomment if you want to try it)
#            cv2.imshow('image',img)
#            if cv2.waitKey(1) & 0xFF == ord('q'):
#                break
            i+=1
        elif err == vrep.simx_return_novalue_flag:
            print ("no image yet")
            pass
        else:
          print (err)
        time.sleep(1)
else:
  print ("Failed to connect to remote API Server")
  vrep.simxFinish(clientID)


# In[6]:


print(orientation[7])


# In[8]:


#changing each pic in the array to gray scale
count=0
picsgray=numpy.empty(p+1,dtype=numpy.ndarray)
for x in pics:
    gray = cv2.cvtColor(x, cv2.COLOR_BGR2GRAY)
    picsgray[count]=gray
    imshow(gray, cmap='gray')
    count+=1


# In[9]:


imshow(picsgray[10])


# In[10]:


testray=numpy.empty(1,dtype=numpy.ndarray)
gray = cv2.medianBlur(picsgray[0],5)
circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, gray.shape[0] / 8, param1=30,param2=15,minRadius=10,maxRadius=0)
rmax=0
count=0
for x in circles[0, :]:
        r=x[2]
        if r>rmax:
            testray[count]=x
            rmax=r

for x in testray:
    print(x[2])
        


# In[11]:


circlesray=numpy.empty(p+1,dtype=numpy.ndarray)
blurpics=numpy.empty(p+1,dtype=numpy.ndarray)
count=0
for x in picsgray:
    gray = cv2.medianBlur(x,5)
    blurpics[count]=gray
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, gray.shape[0] / 8, param1=30,param2=15,minRadius=10,maxRadius=0)
    rmax=0
    for x in circles[0, :]:
        r=x[2]
        #only keeping the circle with the biggest radius
        if r>rmax:
            circlesray[count]=x
            rmax=r
    count=count+1
    

radiusray=numpy.empty(p+1,dtype=float)
centerray=numpy.empty(p+1,dtype=numpy.ndarray)    
count=0    
if circles is not None:
    for x in circlesray:
        center = (x[0], x[1])  # In pixels
        radius = x[2]  # In pixels
        imshow(blurpics[count], cmap='gray')
        radiusray[count]=radius
        centerray[count]=center
        count+=1
        # plot circle
        s = linspace(0,2*pi,100)
        cx = cos(s)*radius + center [0]
        cy = sin(s)*radius + center [1]
        # Plot center
        plot(center[0], center[1], '*')
        plot(cx,cy)


# In[12]:


print(circlesray)


# In[13]:


for x in radiusray:
    print(x)


# In[14]:


u0, v0 = 132, 132
xzcord=numpy.empty(p+1,dtype=numpy.ndarray)  
count=0
for x in circlesray:
    k = int(x[2]) / (1.5 / 2)  # Pixels per meters (radius in pixels / radius in  meters)
    fov = 60  # degrees

    # number of pixels in x-axis
    npx = 264

    # maximum value in the x-axis (in meters)
    xmax = npx / (2.* k)

    # Distance to the center of the sphere (z-axis coordinate)
    zc = xmax / tan(pi/6)
    

    # Location of the sphere in the x-axis coordinate
    xc = (x[0] - u0) / k
    
    print(xc, zc)
    xzcord[count]=[xc, zc, k, x[2]]
    count+=1


# In[29]:


#changing xc and zc to X and Y using orientation
import math
xycord=numpy.empty(p+1,dtype=numpy.ndarray)
count=0
for x in xzcord:
    D=sqrt(x[0]**2+x[1]**2)
    Y=D*sin(orientation[count])
    X=D*cos(orientation[count])
    
    xycord[count]=[X,Y,x[2],x[3],orientation[count]]
    count+=1


# In[30]:


print(xzcord)


# In[31]:


print(xycord)


# In[24]:


# Tha map is 20x20 sq meters
#Lets define a grid of nxn
n = 20
gmap = zeros(n*n) # the map is a grid of nxn

# x and y coordinates for the grid cells. Lowest and leftest point in the cell.
cell_w = 20/n
grid_x, grid_y = np.mgrid[-10:10:cell_w,-10:10:cell_w]
# Convert the matrix into a vector
grid_x = grid_x.flatten()
grid_y = grid_y.flatten()

plot(grid_x, grid_y, '+')


# In[25]:



# Computes the four points of the square that composes a cell
def points_cell(x, y, d):    
    X = [x, x+d, x+d, x]
    Y = [y, y, y+d, y+d]
    return X, Y

fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
# plot each cell
for x, y in zip(grid_x, grid_y):
    X, Y = points_cell(x, y, cell_w)
    cell = plt.Polygon([(xi, yi) for xi, yi in zip(X,Y)], color='0.9')
    
    ax.add_patch(cell)

    plot(X,Y, 'k-')
    plot(X,Y, 'b+')    
    
fig.canvas.draw()


# In[34]:


l0 = (0.3/(1-0.3))  # Initial belief
gmap = l0 * ones(n*n) # Initial belief
count=0
# For each cell, check if the circle is in it.
for i in range(n*n):
    count=0
    x, y = grid_x[i], grid_y[i]
    for values in xycord:
        # Corners of the cell
        X, Y = points_cell(x, y, cell_w)
        # check based on the ecuclidean distance
        dist = sqrt((values[0] - X)**2 + (values[1] - Y)**2)
    
        # Check if At least one of the borders is within the sphere
        if((dist < values[3]/values[2]).any()):
            print(dist)
            po = 0.8  # P(mi/zt) probability of having an obstacle 
            li = log(po / (1-po)) + gmap[i] - l0
            gmap[i] = li  # P(mi/zt) 
            print(li)
        else:
            po = 0.05  # P(mi/zt) probability of having an obstacle given a non-detected obstacle
            
            # Cells within the fov. Check if the four points are withing the FOV
            thetas = np.arctan2(Y,X) - pi/2
            if np.logical_and(-pi/6-values[4] <thetas, thetas < pi/6-values[4]).all():
                li = log(po / (1-po)) + gmap[i] - l0
                gmap[i] = li
            pass

# gmap


# In[35]:


print(gmap)


# In[36]:


fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')

# normalize gmap
gmap = gmap - min(gmap)
gmap = gmap / max(gmap)

# plot each cell
for c, x, y in zip(gmap, grid_x, grid_y):
    X, Y = points_cell(x, y, cell_w)
    cell = plt.Polygon([(xi, yi) for xi, yi in zip(X,Y)], color='%f'%(1-c))
    
    ax.add_patch(cell)

    plot(X,Y, 'k-')
    plot(X,Y, 'b+')    
    plot(0,0, 'go')
    
fig.canvas.draw()


# In[32]:





# In[ ]:




