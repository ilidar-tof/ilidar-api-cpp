#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

// Include ilidar library
#include "../src/ilidar.hpp"

// img_t definition for global data holder
static iTFS::img_t lidar_img_data[iTFS::max_device];

// Synchronization variables
static std::condition_variable	lidar_cv;
static std::mutex				lidar_cv_mutex;
static std::queue<int>			lidar_q;

// Basic lidar data handler function
static void lidar_data_handler(iTFS::device_t *device) {
	// Print message
	printf("[MESSAGE] iTFS::LiDAR image  | D# %d  M %d  F# %2d  %d.%d.%d.%d:%d\n",
		device->idx, device->data.mode, device->data.frame,
		device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);

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

// Example keyboard input run in seperate thread 
static void keyboard_input_run(iTFS::LiDAR* ilidar) {
	// Wait fot the sensor
	while (ilidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }

	// Check keyboard input
	while (ilidar->Ready() == true) {
		// Get char input
		char ch = getchar();

		if (ch == 'q' || ch == 'Q') {
			/* Send config packet */
			for (int _i = 0; _i < ilidar->device_cnt; _i++) {
				// Check the info packet was received or not
				if (ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info.sensor_sn) {
					/* The serial number is matched */

					/* This example shows how to configure the sensor with API functions */
					/* See the manual or synchronization docs for details */

					// Send info packet to configure the LiDAR
					ilidar->device[_i].info.capture_mode = 2;

					ilidar->device[_i].info.capture_shutter[0] = 400;
					ilidar->device[_i].info.capture_shutter[1] = 80;
					ilidar->device[_i].info.capture_shutter[2] = 16;
					ilidar->device[_i].info.capture_shutter[3] = 8;
					ilidar->device[_i].info.capture_shutter[4] = 8000;

					ilidar->device[_i].info.capture_period = 80;

					ilidar->device[_i].info.sync = iTFS::packet::sync_strobe_on | iTFS::packet::sync_mode_udp;
					ilidar->device[_i].info.sync_delay = 0;
					ilidar->device[_i].info.data_output = iTFS::packet::data_output_status_full | iTFS::packet::data_output_depth_on | iTFS::packet::data_output_intensity_on;
					ilidar->Send_config(_i, &(ilidar->device[_i].info));
					printf("[MESSAGE] iTFS::LiDAR config(info) packet was sent to D#%d.\n", _i);
				}
				else if (ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info_v2.sensor_sn) {
					/* The serial number is matched */

					/* This example shows how to configure the sensor with API functions */
					/* See the manual or synchronization docs for details */

					// Send info_v2 packet to configure the LiDAR
					ilidar->device[_i].info_v2.capture_mode = 1;

					ilidar->device[_i].info_v2.capture_shutter[0] = 400;
					ilidar->device[_i].info_v2.capture_shutter[1] = 80;
					ilidar->device[_i].info_v2.capture_shutter[2] = 16;
					ilidar->device[_i].info_v2.capture_shutter[3] = 8;
					ilidar->device[_i].info_v2.capture_shutter[4] = 8000;

					ilidar->device[_i].info_v2.capture_period_us = 200000;
					ilidar->device[_i].info_v2.sync = iTFS::packet::sync_strobe_on | iTFS::packet::sync_mode_udp;

					ilidar->device[_i].info_v2.capture_seq = 0;
					ilidar->device[_i].info_v2.sync_trig_delay_us = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[0] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[1] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[2] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[3] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[4] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[5] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[6] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[7] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[8] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[9] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[10] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[11] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[12] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[13] = 0;
					ilidar->device[_i].info_v2.sync_ill_delay_us[14] = 0;

					ilidar->device[_i].info_v2.data_output = iTFS::packet::data_output_status_full | iTFS::packet::data_output_depth_on | iTFS::packet::data_output_intensity_on;

					ilidar->Send_config(_i, &(ilidar->device[_i].info_v2));
					printf("[MESSAGE] iTFS::LiDAR config(info_v2) packet was sent to D#%d.\n", _i);
				}
			}
		}
		else if (ch == 'w' || ch == 'W') {
			/* Send store command packet */
			for (int _i = 0; _i < ilidar->device_cnt; _i++) {
				// Check the info packet was received or not
				if (ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info.sensor_sn ||
					ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info_v2.sensor_sn) {
					/* The serial number is matched */

					// Send store command
					iTFS::packet::cmd_t store = { 0, };
					store.cmd_id = iTFS::packet::cmd_store;
					store.cmd_msg = 0;
					ilidar->Send_cmd(_i, &store);
					printf("[MESSAGE] iTFS::LiDAR cmd_store packet was sent.\n");
				}
			}
		}
		else if (ch == 'l' || ch == 'L') {
			/* Send lock command packet */
			for (int _i = 0; _i < ilidar->device_cnt; _i++) {
				// Check the info packet was received or not
				if (ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info.sensor_sn ||
					ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info_v2.sensor_sn) {
					/* The serial number is matched */

					// Send lock command
					iTFS::packet::cmd_t lock = { 0, };
					lock.cmd_id = iTFS::packet::cmd_lock;
					lock.cmd_msg = ilidar->device[_i].status.sensor_sn;
					ilidar->Send_cmd(_i, &lock);
					printf("[MESSAGE] iTFS::LiDAR cmd_lock packet was sent.\n");
				}
			}
		}
		else if (ch == 'u' || ch == 'U') {
			/* Send unlock command packet */
			for (int _i = 0; _i < ilidar->device_cnt; _i++) {
				// Check the info packet was received or not
				if (ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info.sensor_sn ||
					ilidar->device[_i].status.sensor_sn == ilidar->device[_i].info_v2.sensor_sn) {
					/* The serial number is matched */

					// Send unlock command
					iTFS::packet::cmd_t unlock = { 0, };
					unlock.cmd_id = iTFS::packet::cmd_unlock;
					unlock.cmd_msg = ilidar->device[_i].status.sensor_sn;
					ilidar->Send_cmd(_i, &unlock);
					printf("[MESSAGE] iTFS::LiDAR cmd_unlock packet was sent.\n");
				}
			}
		}
		else if (ch == 'f' || ch == 'F') {
			/* Send sync command packet */
			iTFS::packet::cmd_t sync = { 0, };
			sync.cmd_id = iTFS::packet::cmd_sync;
			sync.cmd_msg = 0;
			ilidar->Send_cmd_to_all(&sync);
			printf("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.\n");
		}
		else if (ch == 'r' || ch == 'R') {
			/* Send reboot command packet */
			iTFS::packet::cmd_t reboot = { 0, };
			reboot.cmd_id = iTFS::packet::cmd_reboot;
			reboot.cmd_msg = 0;
			ilidar->Send_cmd_to_all(&reboot);
			printf("[MESSAGE] iTFS::LiDAR cmd_reboot packet was sent.\n");
		}
		else if (ch == 'p' || ch == 'P') {
			/* Send pause command packet */
			iTFS::packet::cmd_t pause = { 0, };
			pause.cmd_id = iTFS::packet::cmd_pause;
			pause.cmd_msg = 0;
			ilidar->Send_cmd_to_all(&pause);
			printf("[MESSAGE] iTFS::LiDAR cmd_pause packet was sent.\n");
		}
		else if (ch == 'o' || ch == 'O') {
			/* Send measure command packet */
			iTFS::packet::cmd_t measure = { 0, };
			measure.cmd_id = iTFS::packet::cmd_measure;
			measure.cmd_msg = 0;
			ilidar->Send_cmd_to_all(&measure);
			printf("[MESSAGE] iTFS::LiDAR cmd_measure packet was sent.\n");
		}
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

	// Check the sensor driver is ready
	while (ilidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
	printf("[MESSAGE] iTFS::LiDAR is ready.\n");

	// Create keyboard input thread
	std::thread keyboard_input_thread = std::thread([=] { keyboard_input_run(ilidar); });

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

		// Do something here
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	// Stop and delete iTFS LiDAR class
	delete ilidar;
	printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");

	// Wait for keyboard input thread
	keyboard_input_thread.join();

	return 0;
}

