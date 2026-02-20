#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <random>
#include <chrono>
#include <csignal>
#include <cmath>

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>


enum {
    LEFT,
    RIGHT
};

uint64_t get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

struct Packet {
    uint64_t timestamp;
    int source;
    int value;
    int dimension;
};


class SafeQueue {
    std::queue<Packet> q;
    std::mutex m;
    std::condition_variable cv;

public:
    std::atomic<bool> running{true};

    void push(const Packet& p) {
        {
            std::lock_guard<std::mutex> lock(m);
            q.push(p);
        }
        cv.notify_one();
    }

    bool pop(Packet& out) {
        std::unique_lock<std::mutex> lock(m);

        cv.wait(lock, [&]{
            return !q.empty() || !running;
        });

        if (q.empty())
            return false;     // shutting down

        out = q.front();
        q.pop();
        return true;
    }

    void stop() {
        running = false;
        cv.notify_all();
    }

    uint64_t size() {
        return q.size();
    } 
};

std::atomic<bool> global_running{true};
SafeQueue queue;

void signal_handler(int) {
    global_running = false;
    queue.stop();
}

void get_data(int id, std::string device) {

    int fd = open(device.c_str(), O_RDONLY);
    if (fd < 0) {
        perror((std::string("Cannot open event device ") + device).c_str());
        return;
    }

    struct input_event ev;

    while (global_running && (read(fd, &ev, sizeof(ev)) > 0)) {
        Packet p;
        p.timestamp = get_time_ns();
        p.source = id;

        if (ev.type == EV_REL 
        && (ev.code == REL_X  || ev.code == REL_Y)) {
            p.value = ev.value;
            p.dimension = ev.code;
            queue.push(p);
        }

        // std::cout << "Got data: " << ev.value << ", of type: " << ev.type << std::endl;

        // std::this_thread::sleep_for(
        //     std::chrono::milliseconds(delay_ms)
        // );
    }

    std::cout << "Producer " << id << " stopped\n";
}

void analyze_data() {
    Packet packet;

    float theta = 0;
    float x = 0;
    float y = 0;


    while (queue.pop(packet)) {
        // std::cout
        //     << "time=" << packet.timestamp
        //     << " | source=" << packet.source
        //     << " | value=" << packet.value
        //     << " | dimension=" << packet.dimension
        //     << " | lenght=" << queue.size()+1
        //     << " | theta=" << theta
        //     << std::endl;
        
        
        if (packet.dimension == REL_Y) {

            float dx_mm = packet.value; // TODO: Convert units properly

            const float distance_mm = 4000.0;

            if (packet.source == LEFT) {
                theta -= (packet.value / distance_mm);
            } else if (packet.source == RIGHT) {
                theta += (packet.value / distance_mm);
            }

        
            x += dx_mm * cos(theta);
            y += dx_mm * sin(theta);

            std::cout << "x: " << x << ", y: " << y << ", theta: " << theta << std::endl;

        }

        if (packet.dimension == REL_Y) {

            float dy_mm = packet.value; // TODO: Convert units properly

        
            x += dy_mm * sin(theta);
            y += dy_mm * cos(theta);

            std::cout << "x: " << x << ", y: " << y << ", theta: " << theta << std::endl;

        }

        // Measure polling rate
        
        // if ((p.source == 0) && (p.dimension == REL_X)) {
        //     double dx = p.value / 1800.0;
        //     dx *= 25.4;
        //     x += dx;
        //     uint64_t dt = p.timestamp - lastping;
        //     long double frequency = 1.0 / dt;
        //     frequency *= 1000000000;
        //     std::cout << "Delta time: " << dt / 1000000 << "ms" << std::endl;
        //     std::cout << "Frequency: " << frequency << "Hz" << std::endl;
        //     std::cout << "So far: " << x << "mm" << std::endl;
        //     lastping = p.timestamp;
        // }
    }

    std::cout << "Consumer stopped\n";
}

int main(int argc, char *argv[]) {
    signal(SIGINT, signal_handler);

    std::string mouse1 = "/dev/input/event4";
    std::string mouse2 = "/dev/input/event7";

    // TODO: get them from json file and write a simple program that automatically determines event_X

    if (argc >= 3) {
        mouse1 = argv[1];
        mouse2 = argv[2];
    }

    std::thread t1(get_data, 0, mouse1);
    std::thread t2(get_data, 1, mouse2);
    std::thread t3(analyze_data);


    std::cout << "Press Ctrl+C to stop...\n";

    t1.join();
    t2.join();
    t3.join();

    std::cout << "Clean exit!\n";

    return 0;
}