#include <CANComms.hpp> 
#include <hytech.pb.h>
#include <thread> 
#include <iostream>
#include <chrono>

comms::CANComms* primary_can;

void thread1_send() {
    while (true) {
        std::shared_ptr<hytech::drivebrain_torque_lim_input> torque_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
        torque_msg->set_drivebrain_torque_fl(10.0);
        torque_msg->set_drivebrain_torque_fr(20.0);
        torque_msg->set_drivebrain_torque_rl(30.0);
        torque_msg->set_drivebrain_torque_rr(40.0);
        primary_can->send_message(torque_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void thread2_send() {
    while (true) {
        std::shared_ptr<hytech::drivebrain_speed_set_input> speed_msg = std::make_shared<hytech::drivebrain_speed_set_input>(); 
        speed_msg->set_drivebrain_set_rpm_fl(1.0);
        speed_msg->set_drivebrain_set_rpm_fr(2.0);
        speed_msg->set_drivebrain_set_rpm_rl(4.0);
        speed_msg->set_drivebrain_set_rpm_rr(8.0);
        primary_can->send_message(speed_msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

int main(int argc, char* argv[]) {

    // // Initialize can driver (second command line argument is the dbc file path)
    primary_can = new comms::CANComms("can_primary", argv[1]);

    std::thread thread1(thread1_send);
    std::thread thread2(thread2_send);

    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    thread1.join(); 
    thread2.join(); 

    delete primary_can;

    return 0; 
}
