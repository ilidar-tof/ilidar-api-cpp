# iLidar API Example Projects
The following example project illustrate how to use our sensor functionalities and help you integrate them into your application.

# Project List
|Project|Library|Description|Usage|
|:---:|:---:|:---|:---|
|Helloworld|C++|Illustrate how to use our sensor functionalities by using iLidar APIs|Sensor test|
|OpenCV Example|[OpenCV]|Illustrate how to read the depth and intensity data and convert it to the image format|2D depth image|
|PCL Example|[PCL]|Illustrate how to transform the depth images to the point cloud|3D point cloud|

# Build
Please check the belows to build the projects in your development environment.

## Case 1: Visual Studio (Windows 10)
1. Open the `ilidar-api-cpp.sln` file with Visual Studio.
2. Build the proejcts. OpenCV and PCL examples may require additional solution settings depending on your development environment.

## Case 2: CMake (Windows / Linux)
1. Open and change `CMakeList.txt` file to your development environment.
2. Use the following CMake command to build the projects:
    ```bash
    cmake .
    cmake --build . --config Release
    ```
3. See executables in output folder. You may need to copy DLL files or "iTFS-110.dat" files to the output folder.

# Implementation
All example projects have the same structure and users can implement it with the following code block:
```cpp
// Include ilidar library
#include "../src/ilidar.hpp"

// img_t definition for global data holder
static iTFS::img_t lidar_img_data[iTFS::max_device];

// Define lidar data handler function
static void lidar_data_handler(iTFS::device_t *device) {
    // This handler is called when the depth and intensity data have been properly received.
    /* ADD YOUR DATA PIPELINE CODE HERE */
    // For example, we can use memcpy function to copy the recived data to the global data holder
    memcpy((void*)&lidar_img_data[device->idx], (const void*)device->data.img, sizeof(device->data.img));
}

// Define lidar status packet handler function
static void status_packet_handler(iTFS::device_t* device) {
    // This handler is called when the status packet has been received.
    /* ADD YOUR STATUS HANDLER CODE HERE */
}

// Define lidar info packet handler function
static void info_packet_handler(iTFS::device_t* device) {
    // This handler is called when the info packet has been received.
    /* ADD YOUR INFO HANDLER CODE HERE */

}

// Main entry point
int main(int argc, char* argv[]) {
    // Create iTFS LiDAR class with defined handler functions
    iTFS::LiDAR* ilidar;
    ilidar = new iTFS::LiDAR(lidar_data_handler, status_packet_handler, info_packet_handler);

    // Check the sensor driver is ready
    while (ilidar->Ready() != true) {
        /* ADD YOUR SLEEP CODE HERE */
    }

    // After the sensor is ready, each handler will be called immediately when the related packet is received.

    // Main loop
    while (true) {
        // Do post-processing here
        // For example, we can access each depth point as the below
        for (int _i = 0; _i < iTFS::max_col; _i++) {
            for (int _j = 0; _j < iTFS::max_row; _j++) {
                // Get distance and direction
                float depth = ((float)(lidar_data->img[_j][_i])) * 0.001f;    // Convert mm to meter

                /* DO SOMETHING HERE */
            }
        }
    }

    // Stop and delete iTFS LiDAR class
    delete ilidar;

    return 0;
}
```

# License
All example projects are licensed under the MIT License. Copyright 2022-Present HYBO Inc.  
See LICENSE file to check the licenses of all open source libraries used in each project.

[OpenCV]: https://opencv.org/
[PCL]: https://pointclouds.org/
