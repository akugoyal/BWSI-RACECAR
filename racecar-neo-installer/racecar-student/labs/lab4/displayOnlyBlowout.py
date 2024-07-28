
import sys
sys.path.insert(1, "/Library/Developer/CommandLineTools/Library/Frameworks/Python3.framework/Versions/3.9/lib/python3.9/site-packages")
import numpy as np
import matplotlib.pyplot as plt
import math
import scipy

#f = open("BlowOuts/frontLeft/frontLeftCrazy_trial1_06.23_5:01pm", "r")
f = open("labs/lab4/frontRight_trial4_06.25_3.29pm.txt", "r")

d=f.read()
y1=[]
y2=[]
y3=[]
y4=[]
y5=[]
y6=[]
arr=[y1,y2,y3,y4,y5,y6,0]
i=0
while d.find("[")>0:
    start=d.find("[")
    end=d.find("]")
    #print(d[start+1:end])
    arr[i]=d[start+1:end].split(",")
    for j in range(len(arr[i])):
        arr[i][j]=float(arr[i][j])
    
        
    d=d[end+1:]
    i+=1

tireFell=float(d[d.find("(")+1:d.find(")")])

print(d[d.find("{")+1:d.find("}")])
start=float(d[d.find("{")+1:d.find("}")])
#tireFell=start
startIndex=arr[6].index(start)
tireFellIndex=arr[6].index(tireFell)
# tireFellIndex = 0
for i in range(len(arr)):
    arr[i] = np.array(arr[i], dtype='float')
    sos = scipy.signal.butter(5, 5, 'lowpass', fs=60, output='sos')
    arr[i] = scipy.signal.sosfiltfilt(sos, arr[i])
    arr[i]=list(arr[i])





def guassianBlur(y):
    kernel=[1,12,66,220,495,792,924,792,495,220,66,12,1]
    for i in range(len(y)-len(kernel)):
        val=0
        sum=0
        for j in range(len(kernel)):
            val+=kernel[j]*y[int(i+j)]
            sum+=kernel[j]
        y[i]=val
        y[i]/=sum


for i in range(6): 
    guassianBlur(arr[i])


plt.figure(figsize=(12, 8))  # Adjust the figure size as needed

# Plot 1
plt.subplot(2, 3, 1)  # (rows, columns, panel number)
#print("hi",arr[6].index(start))
plt.plot(arr[6][startIndex:], arr[0][startIndex:])

plt.scatter(tireFell, arr[0][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[0][arr[6].index(start)], color='green', label='Got input here') 
plt.title('x Angular Velocity ')
plt.xlabel("Time")


# Plot 2
plt.subplot(2, 3, 2)
plt.plot(arr[6][startIndex:], arr[1][startIndex:])
plt.scatter(tireFell, arr[1][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[1][arr[6].index(start)], color='green', label='Got input here') 
plt.title('y Angular Velocity')
plt.xlabel("Time")



# Plot 3
plt.subplot(2, 3, 3)
plt.plot(arr[6][startIndex:], arr[2][startIndex:])
plt.scatter(tireFell-0.5, arr[2][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[2][arr[6].index(start)], color='green', label='Got input here') 
plt.title('z Angular Velocity')
plt.xlabel("Time")



# Plot 4
plt.subplot(2, 3, 4)
plt.plot(arr[6][startIndex:], arr[3][startIndex:])
plt.scatter(tireFell, arr[3][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[3][arr[6].index(start)], color='green', label='Got input here') 
plt.title('x Linear Acceleration')
plt.xlabel("Time")



# Plot 5
plt.subplot(2, 3, 5)
plt.plot(arr[6][startIndex:], arr[4][startIndex:])
plt.xlim(0, 10)  # Setting x-axis limit from 0 to 6
plt.ylim(-1.5, 1.5)
plt.scatter(tireFell, arr[4][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[4][arr[6].index(start)], color='green', label='Got input here') 
plt.title('y Linear Acceleration')
plt.xlabel("Time")



# Plot 6
plt.subplot(2, 3, 6)
plt.plot(arr[6][startIndex:], arr[5][startIndex:])
plt.scatter(tireFell, arr[5][tireFellIndex], color='red', label='Tire Fell Here') 
#plt.scatter(start, arr[5][arr[6].index(start)], color='green', label='Got input here') 
plt.title('z Linear Acceleration')
plt.xlabel("Time")



# Adjust layout to prevent overlap of titles and labels
plt.tight_layout()

# Show plot
plt.show()


