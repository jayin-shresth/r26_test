# R26_test 

The code basically used the provided datasheet which was in UBX form then parsed NAV-POSLLH payload into classid structures.

the UGX raw data was used to map out the starting and the ending points using gps structure then it was convered into lats/lon/height.

the gridmap converts the gps coordinates to a 2d grid map which has obstacles 

then a pathplanner was created to go from starting point to ending point without hitting the obstacles and having least deviation.

An odometer was used to compute the motion commands for example time, distance and rotation for changing angles.

The whole program was integrated using my_program.cpp which acts as the main.cpp

the whole code :
1. reads ubx gps data.
2. converts the gps to a grid map.
3. plans a path using euclidean heuristics.
4. odometer is used to compute the motions of the rover.



another thing is while I was trying to run the code, it had some compilation errors so i copied the code to another folder and then copied it back to the R26_test folder, idk if this helps but if it shows an error saying


ld.exe: cannot open output file my_program.exe: No such file or directory
collect2.exe: error: ld returned 1 exit status
make: *** [makefile:9: my_program] Error 1


please copy the folder contents and paste it into another folder and run the code, that will work.

















<p align="center">
  <img src="https://github.com/teamrudra/r25-test/blob/main/datasheets/feynman-simple.jpg" width="600" height="600"/
</p>
     
