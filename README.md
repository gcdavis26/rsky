# rsky

Team RaspberrySky's drone code for the 2025-2026 PSU autonomous vehicle capstone



A "libs" folder will need to be created for this code to work. It must be called libs, or else the main CMakeLists will need to be modified. 3 libraries need to be added to this list. Firstly, Eigen (Matrix library) can be found via the files on the right of the page https://eigen.tuxfamily.org/. It will need to be extracted into the libs folder. Next, clone this repo into the libs folder: https://github.com/gcdavis26/thermal-camera. Lastly, clone https://github.com/gcdavis26/Navio2 into the folder. After all this has been done, you should hopefully be able to build the project by running CMake on the main CMakeLists. 

