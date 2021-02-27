simulation.mp4
===============
This is the screen recording we used as input to our program.
It renders the different camera views at the same time.

detectedBallPositions.mp4
==========================
Here you can see the output of our image analysis program where we try to find a ball in every camera image.
the images are analysed separately and combined to create the view you can see in the video.
Since we calculate the radius and center of each ball we can draw a circle around it.
You can see that in the video.

PositionCalculationAndKalmanFilter.mp4
========================================
You can see here how the ball is tracked from multiple views.
The blue rectangles are the image planes.
The blue circles the pinhole points of the cameras and the red circles are the ball positions in the images.
The green line shows the ray that is used to calculate the intersection with the plane E (mentioned in the documentation).
The green circles show the locations of the calculated ball positions.
The black circle is the result from the kalman filter and the grey circle is the result from the simulation (for comparision).