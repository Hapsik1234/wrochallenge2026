#include <libusb-1.0/libusb.h>
#include <iostream>

uint64_t get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ull + ts.tv_nsec;
}



int main() {

    uint64_t timestamp_last = get_time_ns();
    uint64_t timestamp = get_time_ns()+1;

    libusb_context* ctx = nullptr;
    libusb_device_handle* handle = nullptr;

    libusb_init(&ctx);

    uint16_t vid = 0x046d; // Your mouse VID
    uint16_t pid = 0xc24a; // Your mouse PID

    handle = libusb_open_device_with_vid_pid(ctx, vid, pid);
    if (!handle) {
        std::cerr << "Mouse not found!\n";
        libusb_exit(ctx);
        return 1;
    }

    // Detach kernel driver if active
    if (libusb_kernel_driver_active(handle, 0) == 1) {
        if (libusb_detach_kernel_driver(handle, 0) != 0) {
            std::cerr << "Failed to detach kernel driver\n";
            libusb_close(handle);
            libusb_exit(ctx);
            return 1;
        }
    }

    if (libusb_claim_interface(handle, 0) != 0) {
        std::cerr << "Cannot claim interface\n";
        libusb_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    unsigned char data[8];
    int actual_length;

    std::cout << "Reading mouse data...\n";

    while (true) {
        int r = libusb_interrupt_transfer(handle, 0x81, data, sizeof(data), &actual_length, 0);
        if (r == 0 && actual_length > 0) {
            timestamp = get_time_ns();
            std::cout << data
                      << "\n";

            
            uint64_t dt = timestamp - timestamp_last;
            long double frequency = 1.0 / dt;
            frequency *= 1000000000;
            std::cout << "Delta time: " << dt / 1000000 << "ms" << std::endl;
            std::cout << "Frequency: " << frequency << "Hz" << std::endl;
            timestamp_last = timestamp;
        }
    }

    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(ctx);
    return 0;
}