#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "hytech.pb.h"

int main() {

    auto period = std::chrono::milliseconds(4); // 250hz

    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    strcpy(ifr.ifr_name, "can_primary");
    ioctl(s, SIOCGIFINDEX, &ifr);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    bind(s, (struct sockaddr *)&addr, sizeof(addr));

    hytech::acu_ok acu_ok;

    can_frame torque_request_msg = {
        .can_id = 241,
        .can_dlc = 8
    };

    can_frame speed_request_msg = {
        .can_id = 242,
        .can_dlc = 8
    };

    while (true) {
        int nbytes_torque = write(s, &torque_request_msg, sizeof(struct can_frame));
        int nbytes_speed = write(s, &speed_request_msg, sizeof(struct can_frame));
        std::this_thread::sleep_for(period);
    }

    return 0;
}
