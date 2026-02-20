#include <iostream>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cmath>
#include <csignal>

#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

constexpr double CPI = 1600.0;        // counts per inch (adjust!)
constexpr double INCH_TO_M = 0.0254; // meters per inch
constexpr double WHEEL_BASE = 0.20;  // distance between mice (meters)

struct Packet {
    uint64_t timestamp;
    int source;     // 0 = left, 1 = right
    int value;
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
            return false;

        out = q.front();
        q.pop();
        return true;
    }

    void stop() {
        running = false;
        cv.notify_all();
    }
};

std::atomic<bool> global_running{true};
SafeQueue queue;

void signal_handler(int) {
    global_running = false;
    queue.stop();
}

uint64_t get_time_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000ull + ts.tv_nsec;
}

void get_data(int id, std::string device) {

    int fd = open(device.c_str(), O_RDONLY);
    if (fd < 0) {
        perror(("Cannot open " + device).c_str());
        return;
    }

    struct input_event ev;

    while (global_running && read(fd, &ev, sizeof(ev)) > 0) {

        if (ev.type == EV_REL && ev.code == REL_Y) {
            Packet p;
            p.timestamp = get_time_ns();
            p.source = id;
            p.value = ev.value;
            queue.push(p);
        }
    }

    close(fd);
    std::cout << "Producer " << id << " stopped\n";
}

void analyze_data() {

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    Packet packet;

    while (queue.pop(packet)) {

        // Convert counts → meters
        double distance_m = (packet.value / CPI) * INCH_TO_M;

        double d_left = 0.0;
        double d_right = 0.0;

        if (packet.source == 0)
            d_left = distance_m;
        else
            d_right = distance_m;

        double d_center = (d_left + d_right) / 2.0;
        double d_theta  = (d_right - d_left) / WHEEL_BASE;

        // Update pose
        theta += d_theta;

        x += d_center * std::cos(theta);
        y += d_center * std::sin(theta);

        std::cout
            << "x=" << x
            << " y=" << y
            << " theta=" << theta
            << std::endl;
    }

    std::cout << "Consumer stopped\n";
}

int main(int argc, char *argv[]) {

    signal(SIGINT, signal_handler);

    std::string mouse1 = "/dev/input/event4";
    std::string mouse2 = "/dev/input/event7";

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