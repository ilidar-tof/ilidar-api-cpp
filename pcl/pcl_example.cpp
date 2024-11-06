#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

#include <iostream>

#pragma warning(disable: 4996)
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

// Include ilidar library
#include "../src/ilidar.hpp"

// PCL point cloud with XYZ RGB A
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan[iTFS::max_device];

// Read transformation vectors
static float vec[240][320][3];

// Read transformation vectors from intrinsic calibration results
int readIntrinsic(std::string file) {
	FILE* fp = fopen((const char*)file.c_str(), "rb");
	fread(vec, 1, sizeof(vec), fp);
	fclose(fp);
	return 0;
}

// Get the depth vector of the pixel
inline pcl::PointXYZ getDepthVector(int x, int y) {
	int xIndex = x;
	int yIndex = y + (240 - 160) / 2;
	return pcl::PointXYZ(vec[yIndex][xIndex][0], vec[yIndex][xIndex][1], vec[yIndex][xIndex][2]);
}

// Height colormap
constexpr float zMax = 1.5f;
constexpr float zMin = -1.5f;

// Transform 2D depth image to 3D point cloud
void transform(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dst, iTFS::img_t* lidar_data) {
	for (int _i = 0; _i < iTFS::max_col; _i++) {
		for (int _j = 0; _j < iTFS::max_row; _j++) {
			// Get distance and direction
			float depth = ((float)(lidar_data->img[_j][_i])) * 0.001f;    // Convert mm to meter
			pcl::PointXYZ depthVector = getDepthVector(_i, _j);

			// Transform image coordinates to Cartesian
			int idx = _j * iTFS::max_col + _i;
			dst.get()->points[idx].x = depth * depthVector.z;
			dst.get()->points[idx].y = -depth * depthVector.x;
			dst.get()->points[idx].z = -depth * depthVector.y;

			// Color
			float heightLevel = (dst.get()->points[idx].z - zMin) / (zMax - zMin);
			if (heightLevel > 0.99f) { heightLevel = 1.0f; }
			if (heightLevel < 0.01f) { heightLevel = 0.0f; }

			dst.get()->points[idx].r = 0xFF;
			dst.get()->points[idx].g = (int)(255.0f * heightLevel);
			dst.get()->points[idx].b = (int)(255.0f * heightLevel);
			dst.get()->points[idx].a = 0xFF;
		}
	}
}

// Draw basic grid
void drawGrid(pcl::visualization::PCLVisualizer::Ptr p, int x, int y, int z) {
	for (int i = 0; i < (2 * x + 1); i++) {
		char name[32] = "L00";
		name[2] = 0x21 + i;
		pcl::PointXYZ p1, p2;
		p1.x = i - x;
		p1.y = y;
		p1.z = z;

		p2.x = i - x;
		p2.y = -y;
		p2.z = z;

		p->addLine(p1, p2, 0.3, 0.3, 0.3, name);
	}
	for (int i = 0; i < (2 * y + 1); i++) {
		char name[32] = "L10";
		name[2] = 0x21 + i;
		pcl::PointXYZ p1, p2;
		p1.x = x;
		p1.y = i - y;
		p1.z = z;

		p2.x = -x;
		p2.y = i - y;
		p2.z = z;

		p->addLine(p1, p2, 0.3, 0.3, 0.3, name);
	}
}

// img_t definition for global data holder
static iTFS::img_t lidar_img_data[iTFS::max_device];

// Synchronization variables
static std::condition_variable	lidar_cv;
static std::mutex				lidar_cv_mutex;
static std::queue<int>			lidar_q;

// Lidar data handler function for PCL viewer
static void lidar_data_handler(iTFS::device_t* device) {
	// Print message
	//printf("[MESSAGE] iTFS::LiDAR image  | D# %d  M %d  F# %2d  %d.%d.%d.%d:%d\n",
	//	device->idx, device->data.mode, device->data.frame,
	//	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);

	// Deep-copy the lidar data to img_t
	memcpy((void*)&lidar_img_data[device->idx],
		(const void*)device->data.img,
		sizeof(device->data.img));

	// Notify the reception to the main thread
	int idx = device->idx;
	std::lock_guard<std::mutex> lk(lidar_cv_mutex);
	lidar_q.push(idx);
	lidar_cv.notify_one();
}

// Basic lidar status packet handler function
static void status_packet_handler(iTFS::device_t* device) {
	// Print message
	printf("[MESSAGE] iTFS::LiDAR status | D#%d mode %d frame %2d time %lld us temp %.2f from %3d.%3d.%3d.%3d:%5d\n",
		device->idx, device->status.capture_mode, device->status.capture_frame, get_sensor_time_in_us(&device->status), (float)(device->status.sensor_temp_core) * 0.01f,
		device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);
}

// Basic lidar info packet handler function
static void info_packet_handler(iTFS::device_t* device) {
	// Check info packet version
	if (device->info.sensor_sn != 0) {
		// Print message
		printf("[MESSAGE] iTFS::LiDAR info packet was received.\n");
		printf("[MESSAGE] iTFS::LiDAR info   | D# %d  lock %d\n",
			device->idx, device->info.lock);

		printf("\tSN #%d mode %d, rows %d, period %d\n",
			device->info.sensor_sn,
			device->info.capture_mode,
			device->info.capture_row,
			device->info.capture_period);

		printf("\tshutter [ %d, %d, %d, %d, %d ]\n",
			device->info.capture_shutter[0],
			device->info.capture_shutter[1],
			device->info.capture_shutter[2],
			device->info.capture_shutter[3],
			device->info.capture_shutter[4]);

		printf("\tlimit [ %d, %d ]\n",
			device->info.capture_limit[0],
			device->info.capture_limit[1]);

		printf("\tip   %d.%d.%d.%d\n",
			device->info.data_sensor_ip[0],
			device->info.data_sensor_ip[1],
			device->info.data_sensor_ip[2],
			device->info.data_sensor_ip[3]);

		printf("\tdest %d.%d.%d.%d:%d\n",
			device->info.data_dest_ip[0],
			device->info.data_dest_ip[1],
			device->info.data_dest_ip[2],
			device->info.data_dest_ip[3],
			device->info.data_port);

		printf("\tsync %d, syncBase %d autoReboot %d, autoRebootTick %d\n",
			device->info.sync,
			device->info.sync_delay,
			device->info.arb,
			device->info.arb_timeout);

		printf("\tFW version: V%d.%d.%d - ",
			device->info.sensor_fw_ver[2],
			device->info.sensor_fw_ver[1],
			device->info.sensor_fw_ver[0]);
		printf((const char*)device->info.sensor_fw_time);
		printf(" ");
		printf((const char*)device->info.sensor_fw_date);
		printf("\n");
	}
	else if (device->info_v2.sensor_sn != 0) {
		printf("[MESSAGE] iTFS::LiDAR info_v2 packet was received.\n");
		printf("[MESSAGE] iTFS::LiDAR info_v2| D# %d  lock %d\n",
			device->idx, device->info_v2.lock);

		printf("\tSN #%d mode %d, rows %d, period %d\n",
			device->info_v2.sensor_sn,
			device->info_v2.capture_mode,
			device->info_v2.capture_row,
			device->info_v2.capture_period_us);

		printf("\tshutter [ %d, %d, %d, %d, %d ]\n",
			device->info_v2.capture_shutter[0],
			device->info_v2.capture_shutter[1],
			device->info_v2.capture_shutter[2],
			device->info_v2.capture_shutter[3],
			device->info_v2.capture_shutter[4]);

		printf("\tlimit [ %d, %d ]\n",
			device->info_v2.capture_limit[0],
			device->info_v2.capture_limit[1]);

		printf("\tip   %d.%d.%d.%d\n",
			device->info_v2.data_sensor_ip[0],
			device->info_v2.data_sensor_ip[1],
			device->info_v2.data_sensor_ip[2],
			device->info_v2.data_sensor_ip[3]);

		printf("\tdest %d.%d.%d.%d:%d\n",
			device->info_v2.data_dest_ip[0],
			device->info_v2.data_dest_ip[1],
			device->info_v2.data_dest_ip[2],
			device->info_v2.data_dest_ip[3],
			device->info_v2.data_port);

		printf("\tsync %d, syncBase %d autoReboot %d, autoRebootTick %d\n",
			device->info_v2.sync,
			device->info_v2.sync_trig_delay_us,
			device->info_v2.arb,
			device->info_v2.arb_timeout);

		printf("\tFW version: V%d.%d.%d - ",
			device->info_v2.sensor_fw_ver[2],
			device->info_v2.sensor_fw_ver[1],
			device->info_v2.sensor_fw_ver[0]);
		printf((const char*)device->info_v2.sensor_fw_time);
		printf(" ");
		printf((const char*)device->info_v2.sensor_fw_date);
		printf("\n");

		printf("\tFW0: V%d.%d.%d,  FW1: V%d.%d.%d,  FW2: V%d.%d.%d\n",
			device->info_v2.sensor_fw0_ver[2],
			device->info_v2.sensor_fw0_ver[1],
			device->info_v2.sensor_fw0_ver[0],
			device->info_v2.sensor_fw1_ver[2],
			device->info_v2.sensor_fw1_ver[1],
			device->info_v2.sensor_fw1_ver[0],
			device->info_v2.sensor_fw2_ver[2],
			device->info_v2.sensor_fw2_ver[1],
			device->info_v2.sensor_fw2_ver[0]);

		if (device->info_v2.sensor_boot_mode == 0) {
			printf("\tSENSOR IS IN SAFE-MODE\n");
		}
	}
	else {
		printf("[MESSAGE] iTFS::LiDAR info   | INVALID PACKET\n");
	}
}

// Helloworld example starts here
int main(int argc, char* argv[]) {
	// Create iTFS LiDAR class
	iTFS::LiDAR* ilidar;
	ilidar = new iTFS::LiDAR(
		lidar_data_handler,
		status_packet_handler,
		info_packet_handler);

	// Read the intrinsic calibration file for iTFS
	std::string calibFile = "../src/iTFS-110.dat";
	//std::string calibFile = "../src/iTFS-80.dat";
	readIntrinsic(calibFile);

	// Create the viewer
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer Example"));

	// Create the point cloud holder
	int point_cloud_size = iTFS::max_col * iTFS::max_row;
	for (int _i = 0; _i < iTFS::max_device; _i++) {
		scan[_i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan[_i].get()->points.resize(point_cloud_size);
	}

	// Draw grid
	drawGrid(viewer, 20, 20, -1);

	viewer->addCoordinateSystem(1.0);
	// Check the sensor driver is ready
	while (ilidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
	printf("[MESSAGE] iTFS::LiDAR is ready.\n");

	// Main loop starts here
	int recv_device_idx = 0;
	while (true) {
		// Wait for new data
		std::unique_lock<std::mutex> lk(lidar_cv_mutex);
		lidar_cv.wait(lk, [] { return !lidar_q.empty(); });
		recv_device_idx = lidar_q.front();
		lidar_q.pop();

		// Check the main loop underrun
		if (!lidar_q.empty()) {
			/* The main loop is slower than data reception handler */
			printf("[WARNING] iTFS::LiDAR The main loop seems to be slower than the LiDAR data reception handler.\n");

			// Flush the queue
			while (!lidar_q.empty()) { recv_device_idx = lidar_q.front(); lidar_q.pop(); }
		}

		// Convert depth image to point cloud
		transform(scan[recv_device_idx], &lidar_img_data[recv_device_idx]);
			
		// Update the viewer
		std::string cloud_name = "scan" + std::to_string(recv_device_idx);
		if (!viewer->updatePointCloud(scan[recv_device_idx], cloud_name)) {
			viewer->addPointCloud(scan[recv_device_idx], cloud_name);
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
		}

		// Sleep for display
		viewer->spinOnce(10);

		// Check the viewer is closed
		if (viewer->wasStopped()) {
			break;
		}
	}

	// Stop and delete iTFS LiDAR class
	delete ilidar;
	printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");
}

