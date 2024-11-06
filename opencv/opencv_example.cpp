#include <thread>
#include <stdio.h>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

// Include ilidar library
#include "../src/ilidar.hpp"

// cv::Mat definition for global data holder
static cv::Mat lidar_img_data[iTFS::max_device];

// Synchronization variables
static std::condition_variable	lidar_cv;
static std::mutex				lidar_cv_mutex;
static std::queue<int>			lidar_q;

// Lidar data handler function for OpenCV viewer
static void lidar_data_handler(iTFS::device_t* device) {
	// Print message
	//printf("[MESSAGE] iTFS::LiDAR image  | D# %d  M %d  F# %2d  %d.%d.%d.%d:%d\n",
	//	device->idx, device->data.mode, device->data.frame,
	//	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);

	// Deep-copy the lidar data to cv::Mat
	memcpy((void *)lidar_img_data[device->idx].data,
		(const void *)device->data.img,
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
	int key_input = 0;

	// Initialize cv::Mat
	for (int _i = 0; _i < iTFS::max_device; _i++) { lidar_img_data[_i] = cv::Mat::zeros(2 * iTFS::max_row, iTFS::max_col, CV_16SC1); }
	cv::Mat scaled_img = cv::Mat::zeros(2 * iTFS::max_row, iTFS::max_col, CV_8UC1);
	cv::Mat color_img = cv::Mat::zeros(2 * iTFS::max_row, iTFS::max_col, CV_8UC3);

	// Create iTFS LiDAR class
	iTFS::LiDAR* ilidar;
	ilidar = new iTFS::LiDAR(
		lidar_data_handler,
		status_packet_handler,
		info_packet_handler);

	// Check the sensor driver is ready
	while (ilidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
	printf("[MESSAGE] iTFS::LiDAR is ready.\n");

	// Send first sync packet
	iTFS::packet::cmd_t sync = { 0, };
	sync.cmd_id = iTFS::packet::cmd_sync;
	sync.cmd_msg = 0;
	ilidar->Send_cmd_to_all(&sync);
	printf("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.\n");

	// Sync packet period
	int sync_packet_period = 20;	// [s] 20 sec

	// Get program start time
	auto pri_time = std::chrono::system_clock::now();

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

		// Check the time for sync packet
		auto cur_time = std::chrono::system_clock::now();
		auto after_last_sync = std::chrono::duration_cast<std::chrono::seconds>(cur_time - pri_time);
		if (after_last_sync.count() > sync_packet_period) {
			/* Time to send sync packet */
			pri_time = cur_time;

			// Send sync command packet
			ilidar->Send_cmd_to_all(&sync);
			printf("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.\n");
		}

		// Set the openCV window name
		std::string window_name = "iTFS D" + \
			std::to_string(recv_device_idx) + " " + \
			std::to_string(ilidar->device[recv_device_idx].ip[0]) + "." + \
			std::to_string(ilidar->device[recv_device_idx].ip[1]) + "." + \
			std::to_string(ilidar->device[recv_device_idx].ip[2]) + "." + \
			std::to_string(ilidar->device[recv_device_idx].ip[3]) + ":" + \
			std::to_string(ilidar->device[recv_device_idx].port);

		// Scale the image to visualize up to 20000mm (20m)
		lidar_img_data[recv_device_idx].convertTo(scaled_img, CV_8UC1, 255.0 / 20000.0);	// Reduced scale (20m)

		// Apply colormap
		cv::applyColorMap(scaled_img, color_img, cv::COLORMAP_JET);

		// Check the viewer is closed
		static int cv_window = 0;
		if (cv_window == 0) {
			// Create window
			cv::namedWindow(window_name, cv::WINDOW_NORMAL);
			cv_window = 1;
		}

		// Call imshow
		cv::imshow(window_name, color_img);
		key_input = cv::waitKey(10);

		// Check window property
		if (key_input=='s'|| key_input == 'S') {
			break;
		}
	}

	// Destroy all openCV windows
	cv::destroyAllWindows();
	cv::waitKey(10);

	// Stop and delete iTFS LiDAR class
	delete ilidar;
	printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");

	return 0;
}

