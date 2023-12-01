# Sensor Calibration 

 ``table.npz`` and related GP function 

## ``table.npz`` data description

- Lookup table contains the (x pixel value in image frame) and corresponding (polar angle in degree). These are extracted from the image with 800x600 pixel value. 
  
- Center_x and center_y contains center of the sensor in image coordinates (x,y). 

## ``sensor_calibration.py``

This code contains GP estimation fuction that converts pixel value from the center of the image into the corresponding polar angle in real tactile sensor. 

Tactile sensor has the 25.5mm radius and rotational symmetry, so the function can convert both x and y coordinate into corresponding polar angle. 

