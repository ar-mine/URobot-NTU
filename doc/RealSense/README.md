## RealSense Installation
Reference in details: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md

### Installing the packages:

+ Register the server's public key:

    sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE 


+ Add the server to the list of repositories:

    Ubuntu 20 LTS:
    sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u

+ Install the libraries:

    sudo apt-get install librealsense2-dkms
    sudo apt-get install librealsense2-utils

    The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.

+ Optionally install the developer and debug packages:

    sudo apt-get install librealsense2-dev
    sudo apt-get install librealsense2-dbg

    With dev package installed, you can compile an application with librealsense using g++ -std=c++11 filename.cpp -lrealsense2 or an IDE of your choice.

+ Reconnect the Intel RealSense depth camera and run: realsense-viewer to verify the installation.
