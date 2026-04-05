
#pragma once

#include <string>
#include <unistd.h>
#include <stdexcept>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <libudev.h>
#include <sys/stat.h>
#include <sys/sysinfo.h>


namespace fs = std::filesystem;



// https://stackoverflow.com/questions/49207803/how-to-get-the-usb-vid-pid-and-serial-number-of-a-device-in-ubuntu-using-c-fr
void get_vid_pid_from_device_filepath(const std::string& device_path, int& vid_result, int& pid_result, std::string& serial_number_result) {
    udev* udev = udev_new();
    if (!udev) return;

    struct stat statbuf;
    if (stat(device_path.c_str(), &statbuf) < 0) { 
        udev_unref(udev);
        return; 
    }

    char type = S_ISBLK(statbuf.st_mode) ? 'b' : S_ISCHR(statbuf.st_mode) ? 'c' : 0;
    if (type == 0) {
        udev_unref(udev);
        return;
    }

    auto opened_device = udev_device_new_from_devnum(udev, type, statbuf.st_rdev);
    auto device = opened_device;

    while (device != nullptr) {
        auto serial = udev_device_get_sysattr_value(device, "serial");
        if (!serial) {
            device = udev_device_get_parent(device);
        } 

        else {
            vid_result = strtol(udev_device_get_sysattr_value(device, "idVendor"), NULL, 16);
            pid_result = strtol(udev_device_get_sysattr_value(device, "idProduct"), NULL, 16);
            serial_number_result = serial;
            break;
        }
    }

    if (opened_device) 
        udev_device_unref(opened_device);
    
    udev_unref(udev);
}



std::string get_device_filepath_from_vid_pid_and_serial_number(uint16_t vid, uint16_t pid, const std::string& serial_number) {
    const std::string base_path = "/dev/serial/by-id";

    if (!fs::exists(base_path)) {
        throw std::runtime_error("/dev/serial/by-id does not exist");
    }

    while (true) {
        for (const auto& entry : fs::directory_iterator(base_path)) {
            fs::path symlink_path = entry.path();
            fs::path real_path;

            try {
                real_path = fs::read_symlink(symlink_path);
            } 
            catch (...) {
                continue; // broken symlink, skip
            }

            std::string device_filename = real_path.filename().string();
            std::string device_path = "/sys/class/tty/" + device_filename + "/device";
            
            int current_vid, current_pid;
            std::string current_serial_number;

            get_vid_pid_from_device_filepath("/dev/" + device_filename, current_vid, current_pid, current_serial_number);

            if (vid == current_vid && pid == current_pid && serial_number == current_serial_number) {
                return "/dev/" + device_filename;
            }
        }
    }

}



void print_cpu_and_ram_stats() {
    std::ifstream statm("/proc/self/statm");
    long size, resident, share, text, lib, data, dt;
    statm >> size >> resident >> share >> text >> lib >> data >> dt;

    long page_size_kb = sysconf(_SC_PAGESIZE) / 1024; // in KB
    std::cout << "Resident RAM: " << resident * page_size_kb << " KB\n";

    // --- CPU time ---
    std::ifstream stat("/proc/self/stat");
    std::string tmp;
    long utime_ticks, stime_ticks;
    for (int i=0; i<13; ++i) stat >> tmp; // skip first 13 fields
    stat >> utime_ticks >> stime_ticks;     // user and kernel time in ticks

    long ticks_per_sec = sysconf(_SC_CLK_TCK);
    double utime_sec = (double)utime_ticks / ticks_per_sec;
    double stime_sec = (double)stime_ticks / ticks_per_sec;

    std::cout << "User CPU time: " << utime_sec << " s\n";
    std::cout << "System CPU time: " << stime_sec << " s\n";
}