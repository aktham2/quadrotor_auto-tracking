
A program for autonomous flight of a quadrotor based on (red) object tracking. Face tracking is supported although this requires a more powerful computer. Run ballMain.py to do tracking of a red object, using openCV. Run main.py to track a face using dlib, a useful machine learning python module.

### System: 
- 250 mm size quadrotor
- Raspberry Pi 3
- Raspberry Pi camera
- Naze32 FC

### Logic:
1. Arm quadrotor
2. Tune roll, pitch, yaw, throttle through the keyboard
3. Initiate tracking loop
4. Read frame in through camera
5. Locate object/face in frame
6. Calculate error of object location and size (desired is in the center of the frame with specified radius)
7. Compute PD control from the error for roll, pitch, throttle
8. Return to Step 4
9. If tracking is lost for 100 consecutive loops, then start landing procedure (reduced throttle for 5 seconds)
