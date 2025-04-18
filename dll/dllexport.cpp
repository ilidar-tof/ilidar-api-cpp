#include "dllexport.h"

// Callback function for python script
static callback_t python_callback = NULL;

// Definition for global data holder
static iTFS::LiDAR* ilidar = NULL;
static uint16_t* ilidar_data = NULL;

// Additional connection variables
static std::thread				lidar_connection_thread;
static bool						lidar_connection = false;
static int						lidar_connection_idx = -1;
static bool                     lidar_data_stream = false;

// Basic lidar data handler function
static void lidar_data_handler(iTFS::device_t* device) {
	// Check the output ptr
	if (ilidar_data != NULL) {
		// Check the ID
		if (lidar_connection_idx == device->idx) {
			// Deep-copy the lidar data to img_t
			memcpy((void*)ilidar_data, (const void*)device->data.img, sizeof(device->data.img));

			// Callback function
            if (python_callback != NULL && lidar_data_stream == true) {
                python_callback(ilidar_data);
            }
		}
	}
}

// Basic lidar status packet handler function
static void status_packet_handler(iTFS::device_t* device) {
	// EMPTY
}

// Basic lidar info packet handler function
static void info_packet_handler(iTFS::device_t* device) {
    // EMPTY
}

// Connection thread
static void lidar_connection_run(void) {
    // Creat cmd_take
    iTFS::packet::cmd_t take = { 0, };
    take.cmd_id = iTFS::packet::cmd_take;

    // Keep send cmd_take for connect
	while (lidar_connection == true) {
		// Send the take message to keep connect
		ilidar->Send_cmd(lidar_connection_idx, &take);

        // Sleep a second
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}

// External C interface for python
extern "C" {
    // Initialize
    EXPORT int ilidar_init(uint16_t* data, callback_t cb) {
        if (data == NULL) {
            return (-1);
        }

        ilidar = NULL;
        ilidar_data = NULL;
        lidar_connection = false;
        lidar_connection_idx = -1;
        ilidar_data = data;
        python_callback = cb;
        return 0;
    }

	// Create
	EXPORT int ilidar_create(uint8_t* broadcast_ip, uint8_t* dest_ip, uint16_t dest_port) {
		// Create iTFS LiDAR class
		ilidar = new iTFS::LiDAR(
			lidar_data_handler,
			status_packet_handler,
			info_packet_handler,
			broadcast_ip,
			dest_ip,
			dest_port);

		// Check the sensor driver is ready
        auto pri_time = std::chrono::system_clock::now();
		while (ilidar->Ready() != true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            auto cur_time = std::chrono::system_clock::now();
            auto after_last_check = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pri_time);
            if (after_last_check.count() > 500) {
                return (-1);
            }
        }
		
        printf("[MESSAGE] iTFS::LiDAR is ready.\n");
        return 0;
	}

	// Destroy
	EXPORT int ilidar_destroy(void) {
        lidar_data_stream = false;

        if (lidar_connection == true) {
            lidar_connection = false;
            lidar_connection_thread.join();
        }

        if (ilidar != NULL) {
            delete ilidar;
        }

		printf("[MESSAGE] iTFS::LiDAR has been deleted.\n");
        return 0;
	}

	EXPORT int ilidar_connect(uint8_t *ip, uint16_t port) {
        // Check the old connection
        for (int _d = 0; _d < ilidar->device_cnt; _d++) {
            if (ip[0] == ilidar->device[_d].ip[0] &&
                ip[1] == ilidar->device[_d].ip[1] &&
                ip[2] == ilidar->device[_d].ip[2] &&
                ip[3] == ilidar->device[_d].ip[3]) {

                // Found
                lidar_connection_idx = _d;
                break;
            }

        }

		// Try to connect to the new sensor
        if (lidar_connection_idx < 0) {
            auto start_time = std::chrono::system_clock::now();
            auto pri_time = std::chrono::system_clock::now();
            iTFS::packet::cmd_t redirect = { 0, };
            redirect.cmd_id = iTFS::packet::cmd_redirect;
            ilidar->Send_cmd(ip, &redirect);
            printf("[MESSAGE] iTFS::LiDAR try to connect to the sensor: %d.%d.%d.%d\n",
                ip[0], ip[1], ip[2], ip[3]);
            while (true) {
                // Send redirect to the sensor
                auto cur_time = std::chrono::system_clock::now();
                auto after_last_check = std::chrono::duration_cast<std::chrono::seconds>(cur_time - pri_time);
                if (after_last_check.count() > 3) {
                    pri_time = cur_time;
                    ilidar->Send_cmd(ip, &redirect);
                }

                // Check the IP of all devices
                for (int _d = 0; _d < ilidar->device_cnt; _d++) {
                    if (ip[0] == ilidar->device[_d].ip[0] &&
                        ip[1] == ilidar->device[_d].ip[1] &&
                        ip[2] == ilidar->device[_d].ip[2] &&
                        ip[3] == ilidar->device[_d].ip[3]) {

                        // Found
                        lidar_connection_idx = _d;
                        break;
                    }

                }

                if (lidar_connection_idx >= 0) { break; }

                // Wait for a second
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                printf(".");
                fflush(stdout);
                
                // Timeout
                auto end_time = std::chrono::system_clock::now();
                auto timeout_check = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
                if (timeout_check.count() > 15) {
                    break;
                }
            }
        }
        printf("\n");

        // Check the connection
        if (lidar_connection_idx < 0) {
            // Fail to connect
            printf("[MESSAGE] iTFS::LiDAR fail to connect to the sensor %d.%d.%d.%d. with 15-s timeout.\n",
                ip[0], ip[1], ip[2], ip[3]);
            return (-1);
        }

        printf("[MESSAGE] iTFS::LiDAR success to connect!\n");

        // Success to connect
        lidar_connection = true;
        lidar_connection_thread = std::thread([=] { lidar_connection_run(); });
        
        return 0;
	}

	EXPORT int ilidar_disconnect(void) {
        if (lidar_connection == true) {
            lidar_connection = false;
            lidar_connection_thread.join();
        }

        return 0;
	}

    EXPORT int ilidar_get_params(uint8_t* params) {
        if (lidar_connection == true) {
            iTFS::packet::encode_info_v2(&ilidar->device[lidar_connection_idx].info_v2, params);
            return 0;
        }
        else {
            return (-1);
        }
    }

	EXPORT int ilidar_set_params(uint8_t* params) {
        if (lidar_connection == true) {
            iTFS::packet::info_v2_t input;
            iTFS::packet::decode_info_v2(params, &input);

            ilidar->device[lidar_connection_idx].info_v2.capture_mode = input.capture_mode;
            ilidar->device[lidar_connection_idx].info_v2.capture_row = input.capture_row;
            ilidar->device[lidar_connection_idx].info_v2.capture_shutter[0] = input.capture_shutter[0];
            ilidar->device[lidar_connection_idx].info_v2.capture_shutter[1] = input.capture_shutter[1];
            ilidar->device[lidar_connection_idx].info_v2.capture_shutter[2] = input.capture_shutter[2];
            ilidar->device[lidar_connection_idx].info_v2.capture_shutter[3] = input.capture_shutter[3];
            ilidar->device[lidar_connection_idx].info_v2.capture_shutter[4] = input.capture_shutter[4];
            ilidar->device[lidar_connection_idx].info_v2.capture_limit[0] = input.capture_limit[0];
            ilidar->device[lidar_connection_idx].info_v2.capture_limit[1] = input.capture_limit[1];
            ilidar->device[lidar_connection_idx].info_v2.capture_period_us = input.capture_period_us;
            ilidar->device[lidar_connection_idx].info_v2.capture_seq = input.capture_seq;

            ilidar->device[lidar_connection_idx].info_v2.data_output = input.data_output;
            ilidar->device[lidar_connection_idx].info_v2.data_baud = input.data_baud;

            ilidar->device[lidar_connection_idx].info_v2.sync = input.sync;
            ilidar->device[lidar_connection_idx].info_v2.sync_trig_delay_us = input.sync_trig_delay_us;
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[0] = input.sync_ill_delay_us[0];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[1] = input.sync_ill_delay_us[1];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[2] = input.sync_ill_delay_us[2];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[3] = input.sync_ill_delay_us[3];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[4] = input.sync_ill_delay_us[4];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[5] = input.sync_ill_delay_us[5];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[6] = input.sync_ill_delay_us[6];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[7] = input.sync_ill_delay_us[7];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[8] = input.sync_ill_delay_us[8];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[9] = input.sync_ill_delay_us[9];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[10] = input.sync_ill_delay_us[10];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[11] = input.sync_ill_delay_us[11];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[12] = input.sync_ill_delay_us[12];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[13] = input.sync_ill_delay_us[13];
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_delay_us[14] = input.sync_ill_delay_us[14];
            ilidar->device[lidar_connection_idx].info_v2.sync_trig_trim_us = input.sync_trig_trim_us;
            ilidar->device[lidar_connection_idx].info_v2.sync_ill_trim_us = input.sync_ill_trim_us;
            ilidar->device[lidar_connection_idx].info_v2.sync_output_delay_us = input.sync_output_delay_us;

            ilidar->device[lidar_connection_idx].info_v2.arb = input.arb;
            ilidar->device[lidar_connection_idx].info_v2.arb_timeout = input.arb_timeout;

            ilidar->Send_config(lidar_connection_idx, &(ilidar->device[lidar_connection_idx].info_v2));
            return 0;
        }
        else {
            return (-1);
        }
	}

	EXPORT int ilidar_store(void) {
        if (lidar_connection == true) {
            iTFS::packet::cmd_t store = { 0, };
            store.cmd_id = iTFS::packet::cmd_store;
            ilidar->Send_cmd(lidar_connection_idx, &store);
            return 0;
        }
        else {
            return (-1);
        }
	}

    EXPORT int ilidar_lock(void) {
        if (lidar_connection == true) {
            iTFS::packet::cmd_t lock = { 0, };
            lock.cmd_id = iTFS::packet::cmd_lock;
            lock.cmd_msg = ilidar->device[lidar_connection_idx].info_v2.sensor_sn;
            ilidar->Send_cmd(lidar_connection_idx, &lock);
            return 0;
        }
        else {
            return (-1);
        }
    }

    EXPORT int ilidar_unlock(void) {
        if (lidar_connection == true) {
            iTFS::packet::cmd_t unlock = { 0, };
            unlock.cmd_id = iTFS::packet::cmd_unlock;
            unlock.cmd_msg = ilidar->device[lidar_connection_idx].info_v2.sensor_sn;
            ilidar->Send_cmd(lidar_connection_idx, &unlock);
            return 0;
        }
        else {
            return (-1);
        }
    }

    EXPORT int ilidar_start(void) {
        if (lidar_connection == true) {
            lidar_data_stream = true;
            return 0;
        }
        else {
            return (-1);
        }
    }

    EXPORT int ilidar_stop(void) {
        if (lidar_connection == true) {
            lidar_data_stream = false;
            return 0;
        }
        else {
            return (-1);
        }
    }
}

