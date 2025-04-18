#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

#include <iostream>
#include <cstring>

#if defined(_WIN32) || defined(_WIN64)
// Windows headers
#include <conio.h>
#include <tchar.h>
#else
// Non-windows headers
/* Assume that any non-Windows platform uses POSIX-style sockets instead */
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

#pragma warning(disable: 4996)

// Include ilidar library
#include "../src/ilidar.hpp"

// Definitions for shared memory
int		shared_memory_id;
int		shared_memory_size;
void*	shared_memory_ptr = NULL;

// Definitions for global data pointer
static iTFS::img_t lidar_img_data[iTFS::max_device];
static std::mutex lidar_img_mutex;

// Definitions for output data holder
static uint16_t *lidar_output_raw;		// Place holder

// Synchronization variables
static std::condition_variable	lidar_cv;
static std::mutex				lidar_cv_mutex;
static std::queue<int>			lidar_q;

static std::condition_variable	output_cv;
static std::mutex				output_cv_mutex;
static std::queue<int>			output_q;

// control variables
static int device_sn[iTFS::max_device];
static int device_idx[iTFS::max_device];

typedef enum {
	status_normal			= 0,
	status_underrun			= (0x01 << 0),
	status_overrun			= (0x01 << 1),
	//status_disconnected	= (0x01 << 2),
	status_missing_frames	= (0x01 << 3),
	status_missing_rows		= (0x01 << 4),
	status_sendfail			= (0x01 << 7)
}thread_status;

static int ilidar_thread_frame[32];
static int ilidar_thread_status[32];
static int ilidar_frame_status[32];

static int output_thread_frame;
static int output_thread_idx;
static int output_thread_status;

typedef struct {
	int		ilidar_idx[iTFS::max_device];
	int		ilidar_serial_number[iTFS::max_device];
	int		ilidar_update[iTFS::max_device];
	int		ilidar_num;
	int		ilidar_sync;

	int		shm_size;

	int		display_period;
	int		print_period;

	uint8_t		dest_ip[4];
	uint16_t	dest_port;

	uint16_t	output_src_port;
	uint16_t	output_dest_port;
}multi_ilidar;

static multi_ilidar ilidar_set;

static int read_ilidar_set(std::string file) {
	// Read the file
	FILE* fc = fopen(file.c_str(), "r");

	// Check the file
	if (fc == NULL) { return (-1); }

	// Read configuration
	int	shm_size = 4;

	int lidar_num = 0;
	int ilidar_sync = 0;
	int serial_num = 0;

	int display_period = 0;
	int print_period = 0;

	int dest_ip[4];
	int dest_port;

	int output_src_port;
	int output_dest_port;

	fscanf(fc, "LN = %d\n", &lidar_num);
	fscanf(fc, "SHM_SIZE = %d\n", &shm_size);
	fscanf(fc, "SYNC = %d\n", &ilidar_sync);
	fscanf(fc, "DISPLAY = %d\n", &display_period);
	fscanf(fc, "PRINT = %d\n", &print_period);
	fscanf(fc, "DEST_IP = %d.%d.%d.%d\n", &dest_ip[0], &dest_ip[1], &dest_ip[2], &dest_ip[3]);
	fscanf(fc, "DEST_PORT = %d\n", &dest_port);
	fscanf(fc, "OUTPUT_SRC_PORT = %d\n", &output_src_port);
	fscanf(fc, "OUTPUT_DEST_PORT = %d\n", &output_dest_port);

	if (lidar_num < 1) { fclose(fc); return (-2); }
	else if (lidar_num > iTFS::max_device) { fclose(fc); return (-2); }

	ilidar_set.ilidar_num = lidar_num;
	ilidar_set.ilidar_sync = ilidar_sync;
	ilidar_set.shm_size = shm_size;

	if (display_period < 33) { display_period = 33; }
	if (print_period < 100) { print_period = 100; }

	ilidar_set.display_period = display_period;
	ilidar_set.print_period = print_period;

	ilidar_set.dest_ip[0] = dest_ip[0];
	ilidar_set.dest_ip[1] = dest_ip[1];
	ilidar_set.dest_ip[2] = dest_ip[2];
	ilidar_set.dest_ip[3] = dest_ip[3];
	ilidar_set.dest_port = dest_port;

	ilidar_set.output_src_port = output_src_port;
	ilidar_set.output_dest_port = output_dest_port;

	for (int _i = 0; _i < iTFS::max_device; _i++) {
		ilidar_set.ilidar_idx[_i] = -1;
		ilidar_set.ilidar_serial_number[_i] = 0;
		ilidar_set.ilidar_update[_i] = 0;
	}

	for (int _i = 0; _i < lidar_num; _i++) {
		fscanf(fc, "SN = %d\n", &serial_num);
		ilidar_set.ilidar_idx[_i] = -1;
		ilidar_set.ilidar_serial_number[_i] = serial_num;
		ilidar_set.ilidar_update[_i] = 0;
	}

	fclose(fc);
	return (0);
}

// Basic lidar data handler function
static void lidar_data_handler(iTFS::device_t* device) {
	int idx = ilidar_set.ilidar_idx[device->idx];
	if (idx < 0) { return; }

	// Print message
	//printf("[MESSAGE] iTFS::LiDAR image  | D# %d  M %d  F# %2d  %d.%d.%d.%d:%d\n",
	//	device->idx, device->data.mode, device->data.frame,
	//	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);

	// Deep-copy the lidar data
	memcpy((void *)lidar_img_data[idx].img,
		(const void *)device->data.img,
		sizeof(device->data.img));

	lidar_img_data[idx].frame = device->data.frame;

	// Copy row_frame status
	ilidar_frame_status[idx] = device->data.frame_status;

	// Notify the reception to the main thread
	std::lock_guard<std::mutex> lk(lidar_cv_mutex);
	lidar_q.push(idx);
	lidar_cv.notify_one();
}

// Basic lidar status packet handler function
static void status_packet_handler(iTFS::device_t* device) {
	// Print message
	//printf("[MESSAGE] iTFS::LiDAR status | D#%d mode %d frame %2d time %lld us temp %.2f from %3d.%3d.%3d.%3d:%5d\n",
	//	device->idx, device->status.capture_mode, device->status.capture_frame, get_sensor_time_in_us(&device->status), (float)(device->status.sensor_temp_core) * 0.01f,
	//	device->ip[0], device->ip[1], device->ip[2], device->ip[3], device->port);
}

// Basic lidar info packet handler function
static void info_packet_handler(iTFS::device_t* device) {
	// Check info packet version
	if (device->info.sensor_sn != 0) {
		// Print message
		printf("[ ERROR ] iTFS::LiDAR info packet was received. (lower version lidar)\n");
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

		// Check idx
		for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
			if (ilidar_set.ilidar_serial_number[_i] == (int)device->info_v2.sensor_sn) {
				ilidar_set.ilidar_idx[device->idx] = _i;
				ilidar_set.ilidar_update[_i] = 1;

				// Add
				printf("[MESSAGE] iTFS::LiDAR info_v2| D#%d SN#%d : Added to L#%d\n",
					device->idx, device->info_v2.sensor_sn, _i);

				return;
			}
		}

		// Error
		printf("[MESSAGE] iTFS::LiDAR info_v2| D#%d SN#%d : NOT FOUND ON THE LIST!!!\n",
			device->idx, device->info_v2.sensor_sn);
	}
	else {
		printf("[ ERROR ] iTFS::LiDAR info   | INVALID PACKET!!!\n");
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
	}
}

bool ilidar_thread_run = true;

void lidar_img_handler() {
	// Initialize read status
	for (int _i = 0; _i < 32; _i++) {
		ilidar_thread_status[_i] = status_normal;
		ilidar_thread_frame[_i] = 0;
	}

	// status for condition variable
	bool wait_status;

	// frame number for overrun check
	int recv_idx;

	// Main loop starts here
	while (ilidar_thread_run) {
		// Wait for new data
		std::unique_lock<std::mutex> lk(lidar_cv_mutex);
		wait_status = lidar_cv.wait_for(lk, std::chrono::milliseconds(500), [=] { return !lidar_q.empty(); });

		// Check the wait status
		if (!wait_status) {
			// Set status to underrun
			output_thread_status |= status_underrun;	// warning

			/* The main loop is slower than data reception handler */
			// printf("[WARNING] iTFS::LiDAR Thread #%d does not receive the LiDAR data for 0.5 sec!\n", idx);
			continue;
		}

		// Pop the front value
		recv_idx = lidar_q.front();
		lidar_q.pop();

		// Check the main loop underrun
		if (!lidar_q.empty()) {
			// Set status to overrun
			ilidar_thread_status[recv_idx] |= status_overrun;	// warning

			/* The main loop is slower than data reception handler */
			// printf("[WARNING] iTFS::LiDAR Thread #%d seems to be slower than the LiDAR data reception handler.\n", idx);

			// Flush the queue
			while (!lidar_q.empty()) {
				recv_idx = lidar_q.front();
				lidar_q.pop();
			}
		}

		// Check frame status
		if (ilidar_frame_status[recv_idx] != 0) { ilidar_thread_status[recv_idx] |= status_missing_rows; }

		// Check the frame number
		if ((lidar_img_data[recv_idx].frame - ilidar_thread_frame[recv_idx] + 64) % 64 > 1) {
			ilidar_thread_status[recv_idx] |= status_missing_frames;
		}
		ilidar_thread_frame[recv_idx] = lidar_img_data[recv_idx].frame;

		// Copy the data to the output buffer
		lidar_img_mutex.lock();

		memcpy((void *)&lidar_output_raw[recv_idx * iTFS::max_row * iTFS::max_col],
			(const void *)lidar_img_data[recv_idx].img,
			sizeof(lidar_img_data[recv_idx].img) / 2);

		memcpy((void*)&lidar_output_raw[(ilidar_set.ilidar_num + recv_idx) * iTFS::max_row * iTFS::max_col],
			(const void*)&lidar_img_data[recv_idx].img[iTFS::max_row],
			sizeof(lidar_img_data[recv_idx].img) / 2);

		lidar_img_mutex.unlock();

		// Check the last sensor data was received
		if (recv_idx == (ilidar_set.ilidar_num - 1)) {
			// Notify to the output thread
			std::lock_guard<std::mutex> output_lk(output_cv_mutex);
			output_q.push(ilidar_thread_frame[recv_idx]);
			output_cv.notify_one();
		}
	}
}

void lidar_output_run() {
	output_thread_status = status_normal;	// normal
	output_thread_frame = 0;
	output_thread_idx = 0;

	// status for condition variable
	bool wait_status;

	// frame number for overrun check
	int frame;

	// Create socket for notification
	SOCKET_TYPE			local_sockfd;
	struct sockaddr_in	addr, addr_app;

	// Sender message
	uint8_t output_buffer[256];
	output_buffer[0] = iTFS::packet::stx0;
	output_buffer[1] = iTFS::packet::stx1;
	output_buffer[2] = (iTFS::packet::stream_status_id >> 0) & 0xFF;
	output_buffer[3] = (iTFS::packet::stream_status_id >> 8) & 0xFF;
	output_buffer[4] = (iTFS::packet::stream_status_len >> 0) & 0xFF;
	output_buffer[5] = (iTFS::packet::stream_status_len >> 8) & 0xFF;
	output_buffer[iTFS::packet::header_len + iTFS::packet::stream_status_len] = iTFS::packet::etx0;
	output_buffer[iTFS::packet::header_len + iTFS::packet::stream_status_len + 1] = iTFS::packet::etx1;

	iTFS::packet::stream_status_t stream_status;
	stream_status.sensor_cnt = ilidar_set.ilidar_num;

#if defined (_WIN32) || defined( _WIN64)
	// WSA Startup
	WSADATA wsa_data;
	if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != NO_ERROR) {
		printf("[ ERROR ] iTFS::LiDAR WSA Loading Failed!\n");
		return;
	}
#endif // WSA

	// Open the socket
	if ((local_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		printf("[ ERROR ] iTFS::LiDAR Local Socket Opening Failed\n");
		return;
	}

	// Initialize outgoing address
	memset((void*)&addr, 0x00, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = inet_addr("127.0.0.1");
	addr.sin_port = htons(ilidar_set.output_src_port);

	memset(&addr_app, 0, sizeof(addr_app));
	addr_app.sin_family = AF_INET;
	addr_app.sin_addr.s_addr = inet_addr("127.0.0.1");
	addr_app.sin_port = htons(ilidar_set.output_dest_port);

	int enable = 1;

	// Set the socket option to reuse address
	if (setsockopt(local_sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&enable, sizeof(enable)) < 0) {
		printf("[ ERROR ] iTFS::LiDAR Sender Socket Setsockopt Failed\n");
		closesocket(local_sockfd);
		return;
	}

	// Bind the socket
	if (bind(local_sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		printf("[ ERROR ] iTFS::LiDAR Socket Bind Failed\n");
		closesocket(local_sockfd);
		return;
	}

	// Main loop starts here
	while (ilidar_thread_run) {
		// Reset thread status
		for (int _i = 0; _i < 32; _i++) {
			ilidar_thread_status[_i] = status_normal;
		}

		// Wait for new output
		std::unique_lock<std::mutex> output_lk(output_cv_mutex);
		wait_status = output_cv.wait_for(output_lk, std::chrono::milliseconds(500), [=] { return !output_q.empty(); });

		// Check the wait status
		if (!wait_status) {
			// Set status to underrun
			output_thread_status |= status_underrun; // warning
			continue;
		}

		// Pop the front value
		frame = output_q.front();
		output_q.pop();

		// Check the main loop underrun
		if (!output_q.empty()) {
			// Set status to overrun
			output_thread_status |= status_overrun; // warning

			// Flush the queue
			while (!output_q.empty()) { frame = output_q.front(); output_q.pop(); }
		}

		// Check the frame number
		if ((frame - output_thread_frame + 64) % 64 > 1) {
			output_thread_status |= status_missing_frames;
		}
		output_thread_frame = frame;

		// Copy the output data in shared memory
		lidar_img_mutex.lock();
		void *target_ptr = (void *)((uint8_t *)shared_memory_ptr + output_thread_idx * (4 * sizeof(uint8_t) * iTFS::max_row * iTFS::max_col * ilidar_set.ilidar_num));
		memcpy(target_ptr,
			(const void*)lidar_output_raw,
			4 * sizeof(uint8_t) * iTFS::max_row * iTFS::max_col * ilidar_set.ilidar_num);
		lidar_img_mutex.unlock();

		// Store output message
		stream_status.shm_idx = output_thread_idx;
		for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
			stream_status.sensor_sn[_i] = device_sn[_i];
			stream_status.sensor_frame[_i] = ilidar_thread_frame[_i];
			stream_status.read_status[_i] = ilidar_thread_status[_i];
		}
		iTFS::packet::encode_stream_status(&stream_status, &output_buffer[6]);

		// Send udp packet to application
		if (sendto(local_sockfd, (const char*)output_buffer, (iTFS::packet::overheader_len + iTFS::packet::stream_status_len), 0, (struct sockaddr*)&addr_app, sizeof(addr_app)) < 0) {
			output_thread_status |= status_sendfail;
		}

		if (++output_thread_idx >= ilidar_set.shm_size) { output_thread_idx = 0; }
	}
}

// Helloworld example starts here
int main(int argc, char* argv[]) {
	// Get program start time
	auto pri_time = std::chrono::system_clock::now();

	// Print version
	printf("[MESSAGE] iTFS::LiDAR This is multi_thread_read process ");
	printf("(SW package V %d.%d.%d)", iTFS::ilidar_lib_ver[2], iTFS::ilidar_lib_ver[1], iTFS::ilidar_lib_ver[0]);
	printf("\n");

	// Check lib version
	int ilidar_lib_ver_check = iTFS::ilidar_lib_ver[2] * 10000 + iTFS::ilidar_lib_ver[1] * 100 + iTFS::ilidar_lib_ver[0];
	if (ilidar_lib_ver_check < 11202) {
		printf("[ ERROR ] iTFS::LiDAR LIB version is not macthed !!");
		printf("\n");
		printf("\tRequired ( V 1.12.2+ ), but current ( V %d.%d.%d )", iTFS::ilidar_lib_ver[2], iTFS::ilidar_lib_ver[1], iTFS::ilidar_lib_ver[0]);
		printf("\n");

		exit(-1);
	}

	// Paramter files
	std::string ilidar_num_file_name;

	// Display flags
	bool print_info = false;

	// Check the input arguments
	if (argc > 0) {
		for (int _i = 0; _i < argc; _i++) {
			// Check the ilidar_num file name
			if (std::strcmp(argv[_i], "-i") == 0 && ((_i + 1) < argc)) {
				/* Success to get ilidar_num file name */
				ilidar_num_file_name = argv[_i + 1];
			}

			// Check the print flag
			if (std::strcmp(argv[_i], "-p") == 0) {
				/* Set print info flag to true */
				print_info = true;
			}
		}
	}

	// Check the file names
	if (ilidar_num_file_name.empty()) {
		ilidar_num_file_name = "cfg.txt";
	}

	// Read ilidar_num file
	int read_ilidar = read_ilidar_set(ilidar_num_file_name);
	if (read_ilidar == (-1)) {
		printf("[ ERROR ] iTFS::LiDAR There is no file for ilidar serial number (%s)", ilidar_num_file_name.c_str());
		printf("\n");
		return 0;
	}
	else if (read_ilidar == (-2)) {
		printf("[ ERROR ] iTFS::LiDAR Invalid ilidar serial number in (%s)", ilidar_num_file_name.c_str());
		printf("\n");
		return 0;
	}
	else {
		printf("[MESSAGE] iTFS::LiDAR From the ilidar serial number file (%s),\n", ilidar_num_file_name.c_str());
		printf("\tThis program receive the data from %d LiDARs,\n\t[", ilidar_set.ilidar_num);
		for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
			printf(" %d ", ilidar_set.ilidar_serial_number[_i]);
		}
		printf("]\n");
		printf("\tSynchronization period = %d sec\n", ilidar_set.ilidar_sync);

		printf("\tPrint period = %d msec\n", ilidar_set.print_period);

		printf("\t PC IP : %d.%d.%d.%d   PORT: %d",
			ilidar_set.dest_ip[0], ilidar_set.dest_ip[1], ilidar_set.dest_ip[2], ilidar_set.dest_ip[3], ilidar_set.dest_port);
		printf("\n");

		printf("\tOUTPUT : 127.0.0.1  [%d] --> [%d]",
			ilidar_set.output_src_port, ilidar_set.output_dest_port);
		printf("\n");
	}

	lidar_output_raw = (uint16_t *)malloc(sizeof(uint16_t) * 2 * iTFS::max_row * iTFS::max_col * ilidar_set.ilidar_num);

	// Create threads
	ilidar_thread_run = true;
	std::thread output_thread = std::thread([=] { lidar_output_run(); });
	std::thread ilidar_thread = std::thread([=] { lidar_img_handler(); });
	printf("[MESSAGE] iTFS::LiDAR success to create threads.\n");

	// Check print flag
	if (print_info) {
		printf("[MESSAGE] iTFS::LiDAR ");
		printf("print info ON");
		printf("\n");
	}
	else {
		printf("[MESSAGE] iTFS::LiDAR ");
		printf("print info OFF");
		printf("\n");
	}

	// Create shared memory
#if defined(_WIN32) || defined( _WIN64)
	// Windows
	shared_memory_size = ilidar_set.shm_size * sizeof(uint8_t) * 4 * iTFS::max_row * iTFS::max_col * ilidar_set.ilidar_num;

	printf("[MESSAGE] iTFS::LiDAR Try to create new ");
	printf("shared memory %d KB (%d buffers) with key %d", shared_memory_size / 1024, ilidar_set.shm_size, ilidar_set.output_src_port);
	printf("\n\tThis process will delete all non-attached shared memory before create new one\n");

	HANDLE	shm_handle;
	char	shm_name[17] = "iLidar_shm_00000";
	shm_name[11] = shm_name[11] + (ilidar_set.output_src_port % 100000) / 10000;
	shm_name[12] = shm_name[12] + (ilidar_set.output_src_port % 10000) / 1000;
	shm_name[13] = shm_name[13] + (ilidar_set.output_src_port % 1000) / 100;
	shm_name[14] = shm_name[14] + (ilidar_set.output_src_port % 100) / 10;
	shm_name[15] = shm_name[15] + ilidar_set.output_src_port % 10;

	shm_handle = CreateFileMappingA(INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, shared_memory_size, shm_name);

	if (shm_handle == NULL) {
		printf("Could not create file mapping object (%d).\n", GetLastError());
		return 0;
	}

	shared_memory_ptr = (void *)MapViewOfFile(shm_handle, FILE_MAP_ALL_ACCESS, 0, 0, shared_memory_size);

	if (shared_memory_ptr == NULL) {
		printf("Could not map view of file (%d).\n", GetLastError());
		CloseHandle(shm_handle);
		return 0;
	}

	CopyMemory(shared_memory_ptr, TEXT("shm_test"), (_tcslen(TEXT("shm_test")) * sizeof(TCHAR)));

#else
	// Non-windows
	shared_memory_size = ilidar_set.shm_size * sizeof(uint8_t) * 4 * iTFS::max_row * iTFS::max_col * ilidar_set.ilidar_num;

	printf("[MESSAGE] iTFS::LiDAR Try to create new ");
	printf("shared memory %d KB (%d buffers) with key %d", shared_memory_size / 1024, ilidar_set.shm_size, ilidar_set.output_src_port);
	printf("\n\tThis process will delete all non-attached shared memory before create new one\n");

	struct shmid_ds shm_info, shm_seg;
	int max_shmid = shmctl(0, SHM_INFO, &shm_info);
	if (max_shmid > 0) {
		for (int _i = 0; _i < max_shmid; _i++) {
			int shmid = shmctl(_i, SHM_STAT, &shm_seg);
			if (shmid <= 0) {
				continue;
			}
			else if (shm_seg.shm_nattch == 0) {
				if ((shmctl(shmid, IPC_RMID, 0)) == -1) {
					printf("[ ERROR ] iTFS::LiDAR Fail to remove shared memory pointer with key %d", ilidar_set.output_src_port);
					printf("\n");
					return 0;
				}
			}
		}
	}
	if ((shared_memory_id = shmget(ilidar_set.output_src_port, shared_memory_size, IPC_CREAT | 0666)) == -1) {
		printf("[ ERROR ] iTFS::LiDAR Fail to create shared memory with key %d", ilidar_set.output_src_port);
		printf("\n");
		return 0;
	}
	if ((shared_memory_ptr = shmat(shared_memory_id, NULL, 0)) == (void*)-1) {
		printf("[ ERROR ] iTFS::LiDAR Fail to get shared memory pointer with key %d", ilidar_set.output_src_port);
		printf("\n");
		return 0;
	}
	printf("[MESSAGE] iTFS::LiDAR Success to link to the shared memory %d KB with key %d\n", shared_memory_size / 1024, ilidar_set.output_src_port);
#endif

	// Delay
	printf("[MESSAGE] iTFS::LiDAR process starts in\n");
	for (int _i = 3; _i > 0; _i--) {
		printf("\t%d..\n", _i);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
	// Create iTFS LiDAR class
	iTFS::LiDAR* ilidar;
	uint8_t broadcast_ip[4] = { ilidar_set.dest_ip[0], ilidar_set.dest_ip[1], ilidar_set.dest_ip[2], 255 };
	ilidar = new iTFS::LiDAR(
		lidar_data_handler,
		status_packet_handler,
		info_packet_handler,
		broadcast_ip,
		ilidar_set.dest_ip,
		ilidar_set.dest_port);

	// Check the sensor driver is ready
	while (ilidar->Ready() != true) { std::this_thread::sleep_for(std::chrono::milliseconds(100)); }
	printf("[MESSAGE] iTFS::LiDAR is ready ");
	printf("(LIB V %d.%d.%d )", iTFS::ilidar_lib_ver[2], iTFS::ilidar_lib_ver[1], iTFS::ilidar_lib_ver[0]);
	printf("\n");

	// Create keyboard input thread
	std::thread keyboard_input_thread = std::thread([=] { keyboard_input_run(ilidar); });

	// Send first sync packet
	iTFS::packet::cmd_t sync = { 0, };
	sync.cmd_id = iTFS::packet::cmd_sync;
	sync.cmd_msg = 0;

	// Sync packet period
	int sync_packet_period = ilidar_set.ilidar_sync;	// [s]

	// First sync
	while (true) {
		// Sleep
		std::this_thread::sleep_for(std::chrono::milliseconds(ilidar_set.print_period));

		// Check the update status
		int cnt = 0;
		for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) { cnt += ilidar_set.ilidar_update[_i]; }
		if (cnt != ilidar_set.ilidar_num) { continue; }

		// Send sync command packet
		ilidar->Send_cmd_to_all(&sync);
		printf("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent (first_sync).\n");
		break;
	}

	// Get index array
	for (int _i = 0; _i < iTFS::max_device; _i++) {
		device_idx[_i] = -1;
		device_sn[_i] = -1;
	}

	for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
		for (int _j = 0; _j < iTFS::max_device; _j++) {
			if (ilidar_set.ilidar_idx[_j] == _i) {
				device_idx[_i] = _j;
				device_sn[_i] = ilidar->device[_j].info_v2.sensor_sn;
			}
		}
	}

	// Reset thread status
	for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
		ilidar_thread_status[_i] = status_normal;
	}

	print_info = true;

	// Main loop starts here
	while (true) {
		// Print ilidar info
		if (print_info) {
			std::time_t print_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			printf("------------------------------------------------------------\n");
			printf(" LiDAR | S/N | FN | TIME                 | CORE   | CASE  \n");
			printf("------------------------------------------------------------\n");
			for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
				uint16_t	sensor_sn = ilidar->device[device_idx[_i]].info_v2.sensor_sn;
				uint64_t	sensor_time_th = ilidar->device[device_idx[_i]].status_full.sensor_time_th;
				uint16_t	sensor_time_tl = ilidar->device[device_idx[_i]].status_full.sensor_time_tl;;
				float		sensor_temp_core = (float)(ilidar->device[device_idx[_i]].status_full.sensor_temp_core) * 0.01f;
				float		sensor_temp_enclosure = (float)(ilidar->device[device_idx[_i]].status_full.sensor_temp[0]) * 0.01f;

				uint64_t	day = sensor_time_th / (uint64_t)(24 * 60 * 60 * 1000);
				uint64_t	day_time = sensor_time_th % (uint64_t)(24 * 60 * 60 * 1000);	

				printf("   %02d  | %03d | %02d | D%3d + %9.03f sec | %04.01f C | %04.01f C\n",
					_i, sensor_sn, ilidar_thread_frame[_i], (int)day, iTFS::packet::get_sensor_time(day_time, sensor_time_tl), sensor_temp_core, sensor_temp_enclosure);
			}
			printf("------------------------------------------------------------\n");
			printf("  OUTPUT  |  LiDAR %02d  |  FRAME %02d  |  INDEX %d\n",
				(ilidar_set.ilidar_num - 1), output_thread_frame, output_thread_idx);
		}

		// Print ilidar status
		for (int _i = 0; _i < ilidar_set.ilidar_num; _i++) {
			if (ilidar_thread_status[device_idx[_i]] == status_underrun) {
				printf("[WARNING] iTFS::LiDAR Read thread does not receive the LiDAR data for 0.5 sec! (L# %d)", device_idx[_i]);
				printf("\n");
			}

			if (ilidar_thread_status[device_idx[_i]] == status_overrun) {
				printf("[WARNING] iTFS::LiDAR Read thread seems to be slower than the LiDAR data reception handler. (L# %d)", device_idx[_i]);
				printf("\n");
			}

			if (ilidar_thread_status[device_idx[_i]] == status_missing_frames) {
				printf("[WARNING] iTFS::LiDAR Frame skipping occurred! (L# %d)", device_idx[_i]);
				printf("\n");
			}

			if (ilidar_thread_status[device_idx[_i]] == status_missing_rows) {
				printf("[WARNING] iTFS::LiDAR Row skipping occurred! (L# %d)", device_idx[_i]);
				printf("\n");
			}

			ilidar_thread_status[device_idx[_i]] = status_normal;
		}

		// Print output status
		if (output_thread_status == status_underrun) {
			printf("[WARNING] iTFS::LiDAR Output thread does not receive the signal for 0.5 sec!");
			printf("\n");
		}
		
		if (output_thread_status == status_overrun) {
			printf("[WARNING] iTFS::LiDAR Output thread seems to be slower than the LiDAR data reception handler.");
			printf("\n");
		}

		if (output_thread_status == status_missing_frames) {
			printf("[WARNING] iTFS::LiDAR Output thread frame skipping occurred!");
			printf("\n");
		}

		if (output_thread_status == status_missing_rows) {
			printf("[WARNING] iTFS::LiDAR Output thread row skipping occurred!");
			printf("\n");
		}

		if (output_thread_status == status_sendfail) {
			printf("[WARNING] iTFS::LiDAR Output thread fails to send the output message!");
			printf("\n");
		}

		output_thread_status = status_normal;

		// Check the time for sync packet
		if (sync_packet_period > 0) {
			auto cur_time = std::chrono::system_clock::now();
			auto after_last_sync = std::chrono::duration_cast<std::chrono::seconds>(cur_time - pri_time);
			if (after_last_sync.count() > sync_packet_period) {
				/* Time to send sync packet */
				pri_time = cur_time;

				// Send sync command packet
				ilidar->Send_cmd_to_all(&sync);
				printf("[MESSAGE] iTFS::LiDAR cmd_sync packet was sent.\n");
			}
		}

		// Sleep
		std::this_thread::sleep_for(std::chrono::milliseconds(ilidar_set.print_period));
	}

	// Stop and delete iTFS LiDAR class
	delete ilidar;
	printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");

	// Wait for keyboard input thread
	keyboard_input_thread.join();

#if defined (_WIN32) || defined( _WIN64)
	// WSA Cleanup
	WSACleanup();

	// Unmap shm
	UnmapViewOfFile(shared_memory_ptr);
	CloseHandle(shm_handle);
#endif // WSA

	return 0;
}