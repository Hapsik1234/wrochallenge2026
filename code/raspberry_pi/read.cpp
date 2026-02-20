#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/input.h>

int main() {

    std::cout << "Hello, World" << std::endl;

    int fd = open("/dev/input/event4", O_RDONLY);
    if (fd == -1) {
        perror("Cannot open event device");
        return 1;
    }

    struct input_event ev;

    while (read(fd, &ev, sizeof(ev)) > 0) {

        // std::cout << "Got data: " << ev.value << ", of type: " << ev.type << std::endl;

        if (ev.type == EV_REL) {
            if (ev.code == REL_X)
                std::cout << "Move X: " << ev.value << std::endl;

            if (ev.code == REL_Y)
                std::cout << "Move Y: " << ev.value << std::endl;

            if (ev.code == REL_WHEEL)
                std::cout << "Wheel: " << ev.value << std::endl;
        }

        if (ev.type == EV_KEY) {
            if (ev.code == BTN_LEFT)
                std::cout << "Left button: " << ev.value << std::endl;

            if (ev.code == BTN_RIGHT)
                std::cout << "Right button: " << ev.value << std::endl;
        }
    }

    close(fd);
}