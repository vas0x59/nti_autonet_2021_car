# **YASK** team 

### Deploy script
```shell
./deploy.sh 
```

### Run in sim 
```shell
./run_sim.sh
```


### TODO
 - [ ] Simple test
 - [ ] Video recorder
 - [ ] Video uploader
 - [ ] Lane detection
 - [ ] "Camera matrix/model"
 - [ ] Navigation 
 - [ ] Crossroads 
 - [ ] Road sign detection 
 - [ ] Traffic lights detection 
 - [ ] Pedestrian detection 
 - [ ] Optical flow


### Callibrate camera ?, Get approximate camera matrix ?
Get camera matrix from known information without special actions, for example: detect points on the ground with known coordinates.


### Lane/Stopline detection


### Crossroads
While the vehicle is driving through crossroads the stopline/lane detection algorithm is used for precise coordination of maneuvers.
And optical flow algorithms is used for addition accuracy improvemnts in velocity control.


### Stopline/lane detection/analysis algorithm for crossroads 


### Optical flow
Optical flow is used when the vehicle is driving through crossroads.


### Signs/Traffic/Pedestrian detection
One Yolo for all? yes
Grab Yolo model from НТИ АТС 2020? yes



