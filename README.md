# rsky

Team RaspberrySky's drone code for the 2025-2026 PSU autonomous vehicle capstone



A "ext\_libs" folder will need to be created for this code to work. It must be called 'ext\_libs', or else the main CMakeLists will need to be modified. 3 libraries need to be added to this list. Firstly, Eigen (Matrix library) can be found via the files on the right of the page https://eigen.tuxfamily.org/. It will need to be extracted into the libs folder, and renamed to 'eigen'. Next, clone this repo into the libs folder: https://github.com/gcdavis26/thermal-camera. Lastly, clone https://github.com/gcdavis26/Navio2 into the folder. After all this has been done, you should hopefully be able to build the project by running CMake on the main CMakeLists.



To run the simulator, run the "simulation" executable. To change simulation parameters, change values in the simulation.cpp file. 



After the simulation runs, csv files with data will be created in the build folder, called sim\_truths.csv and sim\_estimates.csv. These files will be automatically found by the postprocessing code, located in the postprocessing folder. Just run sim\_visualizer.m to visualize the simulation results. 

