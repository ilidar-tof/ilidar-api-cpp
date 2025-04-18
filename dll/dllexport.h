#ifndef ILIDAR_DLL_EXPORT_H
#define ILIDAR_DLL_EXPORT_H

#include <thread>
#include <stdio.h>
#include <chrono>
#include <condition_variable>	// Data synchronization
#include <mutex>				// Data synchronization
#include <queue>				// Data synchronization

// Include ilidar library
#include "../src/ilidar.hpp"

#ifdef _WIN32
    #define EXPORT __declspec(dllexport)
#else
    #define EXPORT __attribute__((visibility("default")))
#endif

// Callback function
typedef void (*callback_t)(uint16_t *img_ptr);

// Export functions
extern "C" {
    EXPORT int ilidar_init(uint16_t* data, callback_t cb);

    EXPORT int ilidar_create(uint8_t* broadcast_ip, uint8_t* dest_ip, uint16_t dest_port);
    EXPORT int ilidar_destroy(void);

    EXPORT int ilidar_connect(uint8_t *sensor_ip, uint16_t sensor_port);
    EXPORT int ilidar_disconnect(void);

    EXPORT int ilidar_get_params(uint8_t *params);
    EXPORT int ilidar_set_params(uint8_t* params);
    EXPORT int ilidar_store(void);

    EXPORT int ilidar_lock(void);
    EXPORT int ilidar_unlock(void);

    EXPORT int ilidar_start(void);
    EXPORT int ilidar_stop(void);
}

#endif // ILIDAR_DLL_EXPORT_H