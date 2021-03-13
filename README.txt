The Simulation can be run by opening the simulation.html file with a web browser.

To build the image analysis program you can use the Makefile or the file "RoboCupVisionProject" which is a Qt-Creator project file.

The executable needs to be executed with the following arguments: 
1. run program using hough circles algorithm:
RoboCupVisionProject.exe simulation.mp4 simulation.json 0

2. run program using find contours algorithm:
RoboCupVisionProject.exe simulation.mp4 simulation.json 1

3. run program and show the result of the ball detection in a window:
RoboCupVisionProject.exe simulation.mp4 simulation.json 1 ballPositions.json

