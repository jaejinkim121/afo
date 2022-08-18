#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <iostream>

#define DORSI_ID 0x01
#define PLANTAR_ID 0x02
#define CAN_FRAME_MAX_LEN 8

//#define DEBUG_CAN

using namespace std;

class Controller{
    private:
        bool stopsign = false;

        bool trigger_plantar=false, trigger_dorsi=false;

        int gain_p_plantar, gain_d_plantar;
        int gain_p_dorsi, gain_d_dorsi;

        int pos_plantar, vel_plantar, tor_plantar;
        int pos_dorsi, vel_dorsi, tor_dorsi;

        int rising_time, falling_time;

        int sock;

    public:
        Controller(int* plantar_parameter, int* dorsi_parameter, int rising_time);
        ~Controller();

        int initiate_control();
        
        int test_parameter();
        int update_parameter(int* plantar_parameter, int* dorsi_parameter, int rising_time);
        
        void wait_motor(bool is_rising);
        int increment(int origin);
        int motor_zero();
        
        void set_trigger(bool trigger_plantar, bool trigger_dorsi);
        int controller_trigger(int p_to_d, int d_to_p);
        
        void set_stopsign(bool is_stop);
        bool get_stopsign();

        // Can Interface
        int InitCanInterface(const char *ifname);
        int TransmitCanFrame(const uint32_t id, const uint8_t *data, const size_t data_len);
        int ReceiveCanFrame();

};

#endif
