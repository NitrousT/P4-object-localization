# Scan Matching Localization

This project is my implementation of Scan Matching Localization algorithms NDT(Normal Distributions Transform) and ICP (Iterative Closest Point) algorithms for localizing an autonomous vehicle in Carla.

## How to Run:
1) All the important files are inside the Files folder.
2) There are two additional files which hold the ICP and NDT algorithm implementations named scan_matching.cpp/.h
3) Run the following commands: 
```
$ mkdir build && cd build
$ cmake ..
$ make
```
4) After everything has compiled, your code now resides in the build folder. Now simply run ```./run_carla.sh``` in one terminal and  ```./cloud_loc```in another terminal to see the results.

## Implementation Details
* Using the default scans, was resulting in misalignement between the vehicle's scan and the map. So I had to implemented a pre-processing (90deg counter-clock rotation about the z-axis and mirror image about x-y plane) step which handled the co-ordinate transform of the input point clouds. This correctly transforms the input point cloud to match the map environment.
<table>
  <tr>
    <td><img src="./assets/Screenshot%202025-06-30%20at%208.57.36 PM.png" alt="Without pre-process" width="100%"></td>
    <td><img src="./assets/Screenshot%202025-06-30%20at%209.12.08 PM.png" alt="After Pre-process" width="100%"></td>
  </tr>
</table>
* Secondly, Both of the algorithms come with their pros and cons. NDT is fast and can handle poor intialization but it may not produce very fine alignment. ICP on the other hand is very precise and finds optimal solution but at the cost of speed and it is very sensitive to inital guess.
* So my approach was to fuse both of these algorithms together because they complimented each other. This localization algorithm is a two step process. First coarse alignment with NDT: Since NDT is fast and can take on poor inital guess, we first do localization using NDT. For the second step, we use the aligned output provided by the NDT and feed it to the ICP algorithm. ICP takes in the NDT aligned scan and tries to optimize it even further. This two step process produces better overall aligned scan which results in better performance.

### Results
The algorithm taking in 3 throttle steps is able to pass the test. Some improvements could be to implement SLAM which provides more robust performance. Full video of a Demo-Run is available here: [👉 Click Here](https://youtu.be/m5QTFf8FRg0)
![plot](./assets/Screenshot%202025-07-01%20at%202.17.54 PM.png)

