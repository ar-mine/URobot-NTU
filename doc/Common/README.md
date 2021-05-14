+ Use ` sudo apt-get install python3-catkin-tools ` instead of `sudo apt-get install python-catkin-tools` to install `catkin build`.

+ When some shared libs cannot find some symbols, it is possible that there are many libs with same name.
Use `ldd` to find and copy the right one.