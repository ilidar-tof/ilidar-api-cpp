/**
 * @see ilidar.hpp
 * @author JSon (json@hybo.co)
 * @date 2025-02-03
 * @version 1.12.3
 */

#include "ilidar.hpp"

namespace iTFS {
	LiDAR::LiDAR(
			callback_handler	img_data_handler_func,
			callback_handler	status_packet_handler_func,
			callback_handler	info_packet_handler_func,
			uint8_t				*brodcast_ip,
			uint8_t				*listening_ip,
			uint16_t			listening_port) {

		this->img_data_handler_func = img_data_handler_func;
		this->status_packet_handler_func = status_packet_handler_func;
		this->info_packet_handler_func = info_packet_handler_func;

		memset((void*)this->device, 0x00, sizeof(this->device));

		this->is_ready = false;
		this->exit = false;

		this->is_read_ready = false;
		this->is_send_ready = false;

		this->device_cnt = 0;

		if (brodcast_ip == NULL) {
			this->broadcast_ip[0] = 192;
			this->broadcast_ip[1] = 168;
			this->broadcast_ip[2] = 5;
			this->broadcast_ip[3] = 255;
		}
		else {
			this->broadcast_ip[0] = brodcast_ip[0];
			this->broadcast_ip[1] = brodcast_ip[1];
			this->broadcast_ip[2] = brodcast_ip[2];
			this->broadcast_ip[3] = brodcast_ip[3];
			printf("[MESSAGE] iTFS::LiDAR unique broadcast IP has been set: %d.%d.%d.%d\n",
				this->broadcast_ip[0], this->broadcast_ip[1], this->broadcast_ip[2], this->broadcast_ip[3]);
		}

		if (listening_ip == NULL) {
			this->listening_ip[0] = 0;
			this->listening_ip[1] = 0;
			this->listening_ip[2] = 0;
			this->listening_ip[3] = 0;
		}
		else {
			this->listening_ip[0] = listening_ip[0];
			this->listening_ip[1] = listening_ip[1];
			this->listening_ip[2] = listening_ip[2];
			this->listening_ip[3] = listening_ip[3];
			printf("[MESSAGE] iTFS::LiDAR unique listening IP has been set: %d.%d.%d.%d\n",
				this->listening_ip[0], this->listening_ip[1], this->listening_ip[2], this->listening_ip[3]);
		}

		this->listening_port = listening_port;

		if (this->img_data_handler_func == NULL || \
			this->status_packet_handler_func == NULL || \
			this->info_packet_handler_func == NULL) {
			printf("[ ERROR ] iTFS::LiDAR callback functions must be defined.\n");
			this->is_ready = false;
			return;
		}

		this->read_thread = std::thread([=] { this->Read_run(); });
		this->send_thread = std::thread([=] { this->Send_run(); });

        auto pri_time = std::chrono::system_clock::now();
		while (true) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			if (this->is_read_ready == true && \
				this->is_send_ready == true) {
				break;
			}

            auto cur_time = std::chrono::system_clock::now();
            auto try_time = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pri_time);
            if (try_time.count() > 500) {
                this->is_ready = false;
                return;
            }
		}

		this->is_ready = true;
	}

	LiDAR::~LiDAR() {
		this->is_ready = false;
		this->Try_exit();
		this->Join();
	}

	void LiDAR::Read_run() {
		// Receiver socket definitions
		SOCKET_TYPE			sockfd;
		int					read_length;
		struct sockaddr_in	addr, addr_client;
		SOCKET_LEN			length = sizeof(addr_client);

		long		recv_addr_bin;
		uint8_t*	recv_addr = (uint8_t*)&recv_addr_bin;
		uint16_t	recv_port;
		int			recv_device;

#if defined (_WIN32) || defined( _WIN64)
		// WSA Startup
		WSADATA wsa_data;
		if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != NO_ERROR) {
			printf("[ERROR] iTFS::LiDAR WSA Loading Failed!\n");
			return;
		}
#endif // WSA
		
		// Open the socket
		if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			printf("[ERROR] iTFS::LiDAR Receiver Socket Opening Failed\n");
			return;
		}

		// Initialize incomming address
		memset((void*)&addr, 0x00, sizeof(addr));
		addr.sin_family = AF_INET;
		if (this->listening_ip[0] == 0 && \
			this->listening_ip[1] == 0 && \
			this->listening_ip[2] == 0 && \
			this->listening_ip[3] == 0) {
			addr.sin_addr.s_addr = htonl(INADDR_ANY);
		}
		else {
			char addr_str[32];
			sprintf(addr_str, "%d.%d.%d.%d",
				this->listening_ip[0], this->listening_ip[1], this->listening_ip[2], this->listening_ip[3]);
			addr.sin_addr.s_addr = inet_addr(addr_str);
		}
		addr.sin_port = htons(this->listening_port);

		// Set the socket option to reuse address
		int enable = 1;
		if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&enable, sizeof(enable)) < 0) {
			printf("[ERROR] iTFS::LiDAR Receiver Socket Setsockopt Failed\n");
			closesocket(sockfd);
			return;
		}

		// Set the socket option to use timeout = 500 ms
		struct timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = 500000;
		if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0) {
			printf("[ERROR] iTFS::LiDAR Receiver Socket Setsockopt Failed\n");
			closesocket(sockfd);
			return;
		}

		// Bind the socket
		if (bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
			printf("[ERROR] iTFS::LiDAR Receiver Socket Bind Failed\n");
			closesocket(sockfd);
			return;
		}

		// Set the ready flag to true
		this->is_read_ready = true;

		// Reaceiver buffer definitions
		unsigned char buffer[2000] = { 0, };

		// Timeout print cnt
		int timeout_cnt = 0;

		/* The main loop starts here */
		while (this->exit == false) {
			// Try to read incomming data using recvfrom
			if ((read_length = recvfrom(sockfd, (char*)buffer, 2000, 0, (struct sockaddr*)&addr_client, &length)) < 0) {
				if (++timeout_cnt > 10) {
					printf("[MESSAGE] iTFS::LiDAR recvfrom: No data has been received in the last 5 seconds\n");
					timeout_cnt = 0;
				}
			}
			else {
				timeout_cnt = 0;

				// Get IP and port
				recv_addr_bin = addr_client.sin_addr.s_addr;
				recv_port = htons(addr_client.sin_port);

				// Get registered device number
				recv_device = this->Get_device_num(recv_addr, recv_port);
				if (recv_device >= 0) {
					/* Success to get the device number */

					// Check the data is valid or not using length and ID
					uint16_t	message_id = (buffer[3] << 8) | (buffer[2]);
					uint16_t	payload_len = (buffer[5] << 8) | (buffer[4]);
					bool		valid = (buffer[0] == packet::stx0) && (buffer[1] == packet::stx1) && \
										(buffer[read_length-2] == packet::etx0) && (buffer[read_length-1] == packet::etx1) && \
										(payload_len == (read_length - packet::overheader_len));

					if (valid && (message_id == packet::img_data_id) && (payload_len == packet::img_data_len)) {
						/* Image data packet was received, start to decode the data */

						int row_idx = buffer[6];
						int mode = (buffer[7] >> 6) & packet::capture_mode_bin_mask;
						int frame = (buffer[7] & 0x3F);

						this->device[recv_device].data.mode = mode;
						this->device[recv_device].data.frame = frame;

						// Copy the data for monitoring
						if (mode == 1) {
							// Depth and intensity image from mode1(NB)
							if (row_idx < max_row) {
								for (int _j = 0; _j < 2; _j++) {
									for (int _i = 0; _i < max_col; _i++) {
										uint16_t pixel = (uint16_t)(((buffer[8 + _i * 2 + _j * 2 * max_col + 1]) << 8) | (buffer[8 + _i * 2 + _j * 2 * max_col + 0]));
										this->device[recv_device].data.img[2 * row_idx + _j][_i] = pixel;
									}
								}
							}

							// Update row_frame
							this->device[recv_device].data.row_frame[row_idx] = frame;

							// Check the last index was received
							if ((row_idx == (this->device[recv_device].data.capture_row / 2 - 1)) && \
								((this->device[recv_device].info.data_output & packet::data_output_intensity_mask) == 0)) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row / 2; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row / 2)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
							else if ((row_idx == (this->device[recv_device].data.capture_row - 1))) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
						}
						else if (mode == 2) {
							// Depth and intensity image from mode2(NB)
							if (row_idx < (max_row / 2)) {
								for (int _j = 0; _j < 2; _j++) {
									for (int _i = 0; _i < max_col; _i++) {
										uint16_t pixel = (uint16_t)(((buffer[8 + _i * 2 + _j * 2 * max_col + 1]) << 8) | (buffer[8 + _i * 2 + _j * 2 * max_col + 0]));
										this->device[recv_device].data.img[2 * (2 * row_idx + _j) + 0][_i] = pixel;
										this->device[recv_device].data.img[2 * (2 * row_idx + _j) + 1][_i] = pixel;
									}
								}
							}

							// Update row_frame
							this->device[recv_device].data.row_frame[2 * row_idx + 0] = frame;
							this->device[recv_device].data.row_frame[2 * row_idx + 1] = frame;

							// Check the last index was received
							if ((row_idx == (this->device[recv_device].data.capture_row / 4 - 1)) && \
								((this->device[recv_device].info.data_output & packet::data_output_intensity_mask) == 0)) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row / 4; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row / 4)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
							else if ((row_idx == (this->device[recv_device].data.capture_row / 2 - 1))) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row / 2; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row / 2)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
						}
						else if (mode == 3) {
							// Depth and intensity image from mode3(HV)
							if (row_idx < (max_row / 4)) {
								// copy data
								for (int _j = 0; _j < 4; _j++) {
									for (int _i = 0; _i < (max_col / 2); _i++) {
										uint16_t pixel = (uint16_t)(((buffer[8 + _i * 2 + _j * max_col + 1]) << 8) | (buffer[8 + _i * 2 + _j * max_col + 0]));
										this->device[recv_device].data.img[2 * (4 * row_idx + _j) + 0][2 * _i + 0] = pixel;
										this->device[recv_device].data.img[2 * (4 * row_idx + _j) + 1][2 * _i + 0] = pixel;
										this->device[recv_device].data.img[2 * (4 * row_idx + _j) + 0][2 * _i + 1] = pixel;
										this->device[recv_device].data.img[2 * (4 * row_idx + _j) + 1][2 * _i + 1] = pixel;
									}
								}
							}

							// Update row_frame
							this->device[recv_device].data.row_frame[4 * row_idx + 0] = frame;
							this->device[recv_device].data.row_frame[4 * row_idx + 1] = frame;
							this->device[recv_device].data.row_frame[4 * row_idx + 2] = frame;
							this->device[recv_device].data.row_frame[4 * row_idx + 3] = frame;

							// Check the last index was received
							if ((row_idx == (this->device[recv_device].data.capture_row / 8 - 1)) && \
								((this->device[recv_device].info.data_output & packet::data_output_intensity_mask) == 0)) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row / 8; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row / 8)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
							else if ((row_idx == (this->device[recv_device].data.capture_row / 4 - 1))) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < this->device[recv_device].data.capture_row / 4; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * this->device[recv_device].data.capture_row / 4)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
						}
						else {
							// Depth and intensity image from mode0(GRAY)
							if (row_idx < (gray_row / 2)) {
								for (int _j = 0; _j < 2; _j++) {
									for (int _i = 0; _i < max_col; _i++) {
										uint16_t pixel = (uint16_t)(((buffer[8 + _i * 2 + _j * 2 * max_col + 1]) << 8) | (buffer[8 + _i * 2 + _j * 2 * max_col + 0]));
										this->device[recv_device].data.img[2 * row_idx + _j][_i] = pixel;
									}
								}
							}
							
							this->device[recv_device].data.row_frame[row_idx] = frame;

							// Check the last index was received
							if (row_idx == gray_row / 2 - 1) {

								// Get sum of row_frames
								int sum_row_frames = 0;
								for (int _i = 0; _i < gray_row / 2; _i++) {
									sum_row_frames += this->device[recv_device].data.row_frame[_i];
								}

								// Check sum of row_frames
								if (sum_row_frames != (frame * gray_row / 2)) {
									// There are missing rows
									this->device[recv_device].data.frame_status = (-1);
								}
								else {
									this->device[recv_device].data.frame_status = 0;
								}

								// Call callback function
								this->img_data_handler_func(&this->device[recv_device]);
							}
						}
						continue;
					}
					else if (valid && (message_id == packet::status_id) && (payload_len == packet::status_len)) {
						// Decode message
						packet::decode_status((uint8_t*)&buffer[6], &(this->device[recv_device].status));

						// Check lidar info
						if ((this->device[recv_device].status.sensor_sn != this->device[recv_device].info.sensor_sn) &&
							(this->device[recv_device].status.sensor_sn != this->device[recv_device].info_v2.sensor_sn)) {
							// Send read_info command
							iTFS::packet::cmd_t read_info;
							read_info.cmd_id = iTFS::packet::cmd_read_info;
							read_info.cmd_msg = 0;
							this->Send_cmd(recv_device, &read_info);
							printf("[MESSAGE] iTFS::LiDAR cmd_read_info packet was sent.\n");
						}

						// Call handler
						this->status_packet_handler_func(&this->device[recv_device]);
						continue;
					}
					else if (valid && (message_id == packet::status_full_id) && (payload_len == packet::status_full_len)) {
						// Decode message
						packet::decode_status_full((uint8_t*)&buffer[6], &(this->device[recv_device].status_full));

						// Copy the data to the normal status packet
						this->Copy_status(&(this->device[recv_device]));

						// Check lidar info
						if ((this->device[recv_device].status.sensor_sn != this->device[recv_device].info.sensor_sn) &&
							(this->device[recv_device].status.sensor_sn != this->device[recv_device].info_v2.sensor_sn)) {
							// Send read_info command
							iTFS::packet::cmd_t read_info;
							read_info.cmd_id = iTFS::packet::cmd_read_info;
							read_info.cmd_msg = 0;
							this->Send_cmd(recv_device, &read_info);
							printf("[MESSAGE] iTFS::LiDAR cmd_read_info packet was sent.\n");
						}

						// Call handler
						this->status_packet_handler_func(&this->device[recv_device]);
						continue;
					}
					else if (valid && (message_id == packet::sync_ack_id) && (payload_len == packet::sync_ack_len)) {
						// Decode message
						packet::decode_sync_ack((uint8_t*)&buffer[6], &(this->device[recv_device].sync_ack));
						continue;
					}
					else if (valid && (message_id == packet::info_id) && (payload_len == packet::info_len)) {
						// Decode message
						packet::decode_info((uint8_t*)&buffer[6], &(this->device[recv_device].info));

						// Update capture row in image data receiver
						this->device[recv_device].data.capture_row = this->device[recv_device].info.capture_row;

						// Call handler
						this->info_packet_handler_func(&this->device[recv_device]);
						continue;
					}
					else if (valid && (message_id == packet::info_v2_id) && (payload_len == packet::info_v2_len)) {
						// Decode message
						packet::decode_info_v2((uint8_t*)&buffer[6], &(this->device[recv_device].info_v2));
						
						// Update capture row in image data receiver
						this->device[recv_device].data.capture_row = this->device[recv_device].info_v2.capture_row;
						this->device[recv_device].info.data_output = this->device[recv_device].info_v2.data_output;

						// Call handler
						this->info_packet_handler_func(&this->device[recv_device]);
						continue;
					}
					else if (valid && (message_id == packet::ack_id) && (payload_len == packet::ack_len)) {
						// Decode message
						packet::decode_ack((uint8_t*)&buffer[6], &(this->device[recv_device].ack));
						continue;
					}
				}
				else {
					/* Fail to get the device number */

					// Check the data is valid or not using length and ID
					uint16_t	message_id = (buffer[3] << 8) | (buffer[2]);
					uint16_t	payload_len = (buffer[5] << 8) | (buffer[4]);
					bool		valid = (buffer[0] == packet::stx0) && (buffer[1] == packet::stx1) && \
										(buffer[read_length-2] == packet::etx0) && (buffer[read_length-1] == packet::etx1) && \
										(payload_len == (read_length - packet::overheader_len)) && \
										(((message_id == packet::status_id) && (payload_len == packet::status_len)) || \
											((message_id == packet::status_full_id) && (payload_len == packet::status_full_len)));

					// Time to register a new device
					if (valid) {
						recv_device = this->Set_device_num(recv_addr, recv_port);

						if (recv_device >= 0) {
							/* Success to get the device number */

							printf("[MESSAGE] iTFS::LiDAR A new device is registered. D#%d: %3d.%3d.%3d.%3d:%5d\n",
								recv_device, recv_addr[0], recv_addr[1], recv_addr[2], recv_addr[3], recv_port);

							// Copy the status message to the device
							if (valid && (message_id == packet::status_id) && (payload_len == packet::status_len)) {
								packet::decode_status((uint8_t*)&buffer[6], &(this->device[recv_device].status));
							}
							else if (valid && (message_id == packet::status_full_id) && (payload_len == packet::status_full_len)) {
								packet::decode_status_full((uint8_t*)&buffer[6], &(this->device[recv_device].status_full));
								this->Copy_status(&(this->device[recv_device]));
							}

							// Initialize roi
							this->device[recv_device].data.capture_row = max_row;
							continue;
						}
						else if (recv_device == (-1)) {
							/* The device is already registered */
							printf("[WARNNING] iTFS::LiDAR The device is already registered.\n");
							continue;
						}
						else if (recv_device == (-2)) {
							/* The number of the devices reached to it's maximum value */
							printf("[WARNNING] iTFS::LiDAR The number of the devices reached to it's maximum value.\n");
							continue;
						}
					}
				}
			}
		}

		// Set the ready flag to false
		this->is_read_ready = false;

		// Close the socket
		closesocket(sockfd);

#if defined (_WIN32) || defined( _WIN64)
		// WSA Cleanup
		WSACleanup();
#endif // WSA
	}

	void LiDAR::Send_run() {
		// Wait for Read_run() thread initialization
		while (this->is_read_ready == false) {
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Sender socket definitions
		struct sockaddr_in	addr;

		// Skip to call WSA startup - already called in Read_run()

		// Open the socket
		if ((this->send_sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
			printf("[ERROR] iTFS::LiDAR Sender Socket Opening Failed\n");
			return;
		}

		// Initialize outgoing address
		memset((void*)&addr, 0x00, sizeof(addr));
		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = htonl(INADDR_ANY);
		addr.sin_port = htons(user_config_port);

		int enable = 1;

		// Set the socket option to reuse address
		if (setsockopt(this->send_sockfd, SOL_SOCKET, SO_REUSEADDR, (char*)&enable, sizeof(enable)) < 0) {
			printf("[ERROR] iTFS::LiDAR Sender Socket Setsockopt Failed\n");
			closesocket(this->send_sockfd);
			return;
		}

		// Set the socket option to enable broadcasting
		if (setsockopt(this->send_sockfd, SOL_SOCKET, SO_BROADCAST, (char*)&enable, sizeof(enable)) < 0) {
			printf("[ERROR] iTFS::LiDAR Sender Socket Setsockopt Failed\n");
			return;
		}

		// Bind the socket
		if (bind(this->send_sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
			printf("[ERROR] iTFS::LiDAR Socket Bind Failed\n");
			closesocket(this->send_sockfd);
			return;
		}

		// Set the ready flag to true
		this->is_send_ready = true;

		/* The main loop starts here */
		while (this->exit == false) {
			// This thread do nothing but hold the sender socket
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Set the ready flag to false
		this->is_send_ready = false;

		// Close the socket
		closesocket(this->send_sockfd);

#if defined (_WIN32) || defined( _WIN64)
		// WSA Cleanup
		WSACleanup();
#endif // WSA
	}

	int LiDAR::Get_device_num(uint8_t ip[4], uint16_t port) {
		for (int _i = 0; _i < this->device_cnt; _i++) {
			if ((this->device[_i].ip[0] == ip[0]) && \
				(this->device[_i].ip[1] == ip[1]) && \
				(this->device[_i].ip[2] == ip[2]) && \
				(this->device[_i].ip[3] == ip[3]) && \
				(this->device[_i].port == port)) {
				return _i;
			}
		}

		return (-1);
	}

	int LiDAR::Set_device_num(uint8_t ip[4], uint16_t port) {
		if (Get_device_num(ip, port) != (-1)) { return (-1); }

		if (this->device_cnt >= max_device) { return (-2); }

		int _i = this->device_cnt;

		this->device[_i].ip[0] = ip[0];
		this->device[_i].ip[1] = ip[1];
		this->device[_i].ip[2] = ip[2];
		this->device[_i].ip[3] = ip[3];
		this->device[_i].port = port;

		this->device[_i].idx = _i;

		this->device[_i].data.capture_row = max_row;

		this->device_cnt++;

		return _i;
	}

	void LiDAR::Copy_status(device_t *device) {
		device->status.capture_mode = device->status_full.capture_mode;
		device->status.capture_frame = device->status_full.capture_frame;

		device->status.sensor_sn = device->status_full.sensor_sn;
		device->status.sensor_time_th = device->status_full.sensor_time_th;
		device->status.sensor_time_tl = device->status_full.sensor_time_tl;
		device->status.sensor_frame_status = device->status_full.sensor_frame_status;
		device->status.sensor_temp_rx = device->status_full.sensor_temp_rx;
		device->status.sensor_temp_core = device->status_full.sensor_temp_core;
		device->status.sensor_vcsel_level = device->status_full.sensor_vcsel_level;
		device->status.sensor_power_level = device->status_full.sensor_power_level;
		device->status.sensor_warning = device->status_full.sensor_warning;
	}

	int LiDAR::Send_cmd(int device_idx, packet::cmd_t* cmd) {
		// Get device address
		if (device_idx >= this->device_cnt) {
			// There is no matched device
			return (0);
		}

		// Initialize address
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		uint32_t ip	= (this->device[device_idx].ip[3] << 24) | \
					(this->device[device_idx].ip[2] << 16) | \
					(this->device[device_idx].ip[1] <<  8) | \
					(this->device[device_idx].ip[0] <<  0);

		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = ip;
		addr.sin_port = htons(lidar_config_port);

		// Copy command packet to buffer
		uint8_t buffer[16];
		buffer[0] = packet::stx0;
		buffer[1] = packet::stx1;
		buffer[2] = (packet::cmd_id >> 0) & 0xFF;
		buffer[3] = (packet::cmd_id >> 8) & 0xFF;
		buffer[4] = (packet::cmd_len >> 0) & 0xFF;
		buffer[5] = (packet::cmd_len >> 8) & 0xFF;
		buffer[packet::header_len + packet::cmd_len] = packet::etx0;
		buffer[packet::header_len + packet::cmd_len + 1] = packet::etx1;
		packet::encode_cmd(cmd, &buffer[packet::header_len]);

		// Send the packet
		this->send_mutex.lock();
		int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::cmd_len), 0, (struct sockaddr*)&addr, sizeof(addr));
		this->send_mutex.unlock();

		// Return sendto result
		return result;
	}

    int LiDAR::Send_cmd(uint8_t* device_ip, packet::cmd_t* cmd) {
        // Initialize address
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));

        uint32_t ip = (device_ip[3] << 24) | \
            (device_ip[2] << 16) | \
            (device_ip[1] << 8) | \
            (device_ip[0] << 0);

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = ip;
        addr.sin_port = htons(lidar_config_port);

        // Copy command packet to buffer
        uint8_t buffer[16];
        buffer[0] = packet::stx0;
        buffer[1] = packet::stx1;
        buffer[2] = (packet::cmd_id >> 0) & 0xFF;
        buffer[3] = (packet::cmd_id >> 8) & 0xFF;
        buffer[4] = (packet::cmd_len >> 0) & 0xFF;
        buffer[5] = (packet::cmd_len >> 8) & 0xFF;
        buffer[packet::header_len + packet::cmd_len] = packet::etx0;
        buffer[packet::header_len + packet::cmd_len + 1] = packet::etx1;
        packet::encode_cmd(cmd, &buffer[packet::header_len]);

        // Send the packet
        this->send_mutex.lock();
        int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::cmd_len), 0, (struct sockaddr*)&addr, sizeof(addr));
        this->send_mutex.unlock();

        // Return sendto result
        return result;
    }

	int LiDAR::Send_flash_block(int device_idx, packet::flash_block_t* fb) {
		// Get device address
		if (device_idx >= this->device_cnt) {
			// There is no matched device
			return (0);
		}

		// Initialize address
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		uint32_t ip = (this->device[device_idx].ip[3] << 24) | \
			(this->device[device_idx].ip[2] << 16) | \
			(this->device[device_idx].ip[1] << 8) | \
			(this->device[device_idx].ip[0] << 0);

		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = ip;
		addr.sin_port = htons(lidar_config_port);

		// Copy command packet to buffer
		uint8_t buffer[2048];
		buffer[0] = packet::stx0;
		buffer[1] = packet::stx1;
		buffer[2] = (packet::flash_block_id >> 0) & 0xFF;
		buffer[3] = (packet::flash_block_id >> 8) & 0xFF;
		buffer[4] = (packet::flash_block_len >> 0) & 0xFF;
		buffer[5] = (packet::flash_block_len >> 8) & 0xFF;
		buffer[packet::header_len + packet::flash_block_len] = packet::etx0;
		buffer[packet::header_len + packet::flash_block_len + 1] = packet::etx1;
		packet::encode_flash_block(fb, &buffer[packet::header_len]);

		// Send the packet
		this->send_mutex.lock();
		int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::flash_block_len), 0, (struct sockaddr*)&addr, sizeof(addr));
		this->send_mutex.unlock();

		// Return sendto result
		return result;
	}

	int LiDAR::Send_cmd_to_all(packet::cmd_t* cmd) {
		// Initialize address
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		uint32_t ip = (this->broadcast_ip[3] << 24) | \
					(this->broadcast_ip[2] << 16) | \
					(this->broadcast_ip[1] << 8) | \
					(this->broadcast_ip[0] << 0);

		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = ip;
		addr.sin_port = htons(lidar_config_port);

		// Copy command packet to buffer
		uint8_t buffer[16];
		buffer[0] = packet::stx0;
		buffer[1] = packet::stx1;
		buffer[2] = (packet::cmd_id >> 0) & 0xFF;
		buffer[3] = (packet::cmd_id >> 8) & 0xFF;
		buffer[4] = (packet::cmd_len >> 0) & 0xFF;
		buffer[5] = (packet::cmd_len >> 8) & 0xFF;
		buffer[packet::header_len + packet::cmd_len] = packet::etx0;
		buffer[packet::header_len + packet::cmd_len + 1] = packet::etx1;
		packet::encode_cmd(cmd, &buffer[packet::header_len]);

		// Send the packet
		this->send_mutex.lock();
		int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::cmd_len), 0, (struct sockaddr*)&addr, sizeof(addr));
		this->send_mutex.unlock();

		// Return sendto result
		return result;
	}

	int LiDAR::Send_config(int device_idx, packet::info_t* config) {
		// Get device address
		if (device_idx >= this->device_cnt) {
			// There is no matched device
			return (0);
		}

		// Initialize address
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		uint32_t ip = (this->device[device_idx].ip[3] << 24) | \
			(this->device[device_idx].ip[2] << 16) | \
			(this->device[device_idx].ip[1] << 8) | \
			(this->device[device_idx].ip[0] << 0);

		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = ip;
		addr.sin_port = htons(lidar_config_port);

		// Copy command packet to buffer
		uint8_t buffer[256];
		buffer[0] = packet::stx0;
		buffer[1] = packet::stx1;
		buffer[2] = (packet::info_id >> 0) & 0xFF;
		buffer[3] = (packet::info_id >> 8) & 0xFF;
		buffer[4] = (packet::info_len >> 0) & 0xFF;
		buffer[5] = (packet::info_len >> 8) & 0xFF;
		buffer[packet::header_len + packet::info_len] = packet::etx0;
		buffer[packet::header_len + packet::info_len + 1] = packet::etx1;
		packet::encode_info(config, &buffer[packet::header_len]);

		// Send the packet
		this->send_mutex.lock();
		int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::info_len), 0, (struct sockaddr*)&addr, sizeof(addr));
		this->send_mutex.unlock();

		// Return sendto result
		return result;
	}

	int LiDAR::Send_config(int device_idx, packet::info_v2_t* config) {
		// Get device address
		if (device_idx >= this->device_cnt) {
			// There is no matched device
			return (0);
		}

		// Initialize address
		struct sockaddr_in addr;
		memset(&addr, 0, sizeof(addr));

		uint32_t ip = (this->device[device_idx].ip[3] << 24) | \
			(this->device[device_idx].ip[2] << 16) | \
			(this->device[device_idx].ip[1] << 8) | \
			(this->device[device_idx].ip[0] << 0);

		addr.sin_family = AF_INET;
		addr.sin_addr.s_addr = ip;
		addr.sin_port = htons(lidar_config_port);

		// Copy command packet to buffer
		uint8_t buffer[256];
		buffer[0] = packet::stx0;
		buffer[1] = packet::stx1;
		buffer[2] = (packet::info_v2_id >> 0) & 0xFF;
		buffer[3] = (packet::info_v2_id >> 8) & 0xFF;
		buffer[4] = (packet::info_v2_len >> 0) & 0xFF;
		buffer[5] = (packet::info_v2_len >> 8) & 0xFF;
		buffer[packet::header_len + packet::info_v2_len] = packet::etx0;
		buffer[packet::header_len + packet::info_v2_len + 1] = packet::etx1;
		packet::encode_info_v2(config, &buffer[packet::header_len]);

		// Send the packet
		this->send_mutex.lock();
		int result = sendto(this->send_sockfd, (const char*)buffer, (packet::overheader_len + packet::info_v2_len), 0, (struct sockaddr*)&addr, sizeof(addr));
		this->send_mutex.unlock();

		// Return sendto result
		return result;
	}

	void LiDAR::Set_broadcast_ip(uint8_t* ip) {
		this->broadcast_ip[0] = ip[0];
		this->broadcast_ip[1] = ip[1];
		this->broadcast_ip[2] = ip[2];
		this->broadcast_ip[3] = ip[3];
	}
}

