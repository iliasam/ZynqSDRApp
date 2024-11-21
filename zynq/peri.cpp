#include "types.h"
#include "config.h"
#include "kiwi.h"
#include "misc.h"
#include "peri.h"
#include "coroutines.h"
#include "clk.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <atomic>
#include <semaphore.h>

#include "arm_math.h"

#include "ioctl.h"

static bool init;

static const uint8_t buss_id = 0;
static const uint8_t chip_addr = 0x60;



static int ad8370_fd;

static sem_t wf_sem;
static std::atomic<int> wf_using[4];
static int wf_channels;

struct iq2_t {
    s2_t i, q;
} __attribute__((packed));

s4_t  rnd_snd_data[1024*16];
iq2_t  rnd_wf_data[1024*8];


//*****************************************************

void peri_init() {
    if (init)
        return;

    int int_clk;
    if (clk.ext_ADC_clk) {
        int_clk = 0;
    }
    else {
        int_clk = 1;
    }

    bool use_13ch = (kiwi.airband) || (ADC_CLOCK_NOM < 100.0 * MHz);

    // load fpga bitstream
    //int status = blocking_system("cat /media/mmcblk0p1/websdr_%s.bit > /dev/xdevcfg", use_13ch ? "vhf" : "hf");


    //scall("/dev/zynqsdr", ad8370_fd = open("/dev/zynqsdr", O_RDWR | O_SYNC));
    //if (ad8370_fd <= 0) {
    //    sys_panic("Failed to open kernel driver");
    //}

    //fcntl(ad8370_fd, F_SETFD, FD_CLOEXEC);
    //ioctl(ad8370_fd, CLK_SET, int_clk);

    // set airband mode
    rf_enable_airband(kiwi.airband);

    // set default attn to 0
    rf_attn_set(0);

    wf_channels = (fpga_signature() >> 8) & 0x0f;
    sem_init(&wf_sem, 0, wf_channels);

    for (int i = 0; i < 1024*16; i++)
    {
        rnd_snd_data[i] = (s4_t)(random() % 1024) - (s4_t)512;
    }

    for (int i = 0; i < 1024*8; i++)
    {
        rnd_wf_data[i].i = (s2_t)(random() % 256) - (s2_t)128;
        rnd_wf_data[i].q = (s2_t)(random() % 256) - (s2_t)128;
    }

    init = TRUE;
}

void rf_attn_set(float f) {
    if (f > 0)
        return;

    int gain = (int)(-f * 2);

    printf("Set PE4312 with %d/0x%x\n", gain, gain);
    //if (ioctl(ad8370_fd, AD8370_SET, gain) < 0) {
    //    printf("AD8370 set RF failed: %s\n", strerror(errno));
   // }

    return;
}

void rf_enable_airband(bool enabled) {
    //if (ioctl(ad8370_fd, MODE_SET, (int)enabled) < 0) {
    //    printf("AD8370 set mode failed %s\n", strerror(errno));
    //}

    return;
}

void peri_free() {
    assert(init);
    //close(ad8370_fd);
}

static std::atomic<int> write_enabled(0);
void sd_enable(bool write) {
    if (write) {
    }
    else {
    }
}

static int last = 100;


////////////////////////////////
// FPGA DNA
////////////////////////////////

u64_t fpga_dna() {
    int rc;
    uint64_t signature = 0;
    //rc = ioctl(ad8370_fd, GET_DNA, &signature);
    //if (rc)
    //    sys_panic("Get FPGA Signature failed");

    return signature;
}

uint32_t fpga_signature() {
    int rc;
    uint32_t signature = 13 + (2 << 8);
    //rc = ioctl(ad8370_fd, GET_SIGNATURE, &signature);
    //if (rc)
    //    sys_panic("Get FPGA Signature failed");

    return signature;
}

void fpga_start_rx() {
    //uint32_t decim = uint32_t(ADC_CLOCK_NOM / 12000 / 256);
    //int rc = ioctl(ad8370_fd, RX_START, decim);
    //if (rc)
    //    sys_panic("Start RX failed");
}

void fpga_rxfreq(int rx_chan, uint64_t i_phase) {
    //int rc;
    //struct rx_param_op param = { (__u8)rx_chan, freq };
    //rc = ioctl(ad8370_fd, RX_PARAM, &param);
    //if (rc)
    //    lprintf("Set RX freq failed");
}

void fpga_read_rx2(void* buf, uint32_t size, uint32_t nsamples)
{
    int total_size_bytes = size;
    int single_size_bytes = 0;

    void *desp_p = buf;

    do
    {
        if (total_size_bytes > sizeof(rnd_snd_data))
            single_size_bytes = sizeof(rnd_snd_data);
        else
            single_size_bytes = total_size_bytes;

        memcpy(desp_p, rnd_snd_data, single_size_bytes);
        desp_p += single_size_bytes;

        total_size_bytes -= single_size_bytes;
    } while (total_size_bytes > 0);

    //memset(buf, 0, size);
    TaskSleepUsec(nsamples * 83); //12 khz
}

void fpga_read_rx(void* buf, uint32_t size) {
    memset(buf, 0, size);
    /*
    int rc;
    struct rx_read_op read_op = { (__u32)buf, size }; //address, length

    while (true) {
        // printf("In: 0x%x %d\t", read_op.address, read_op.length);
        rc = ioctl(ad8370_fd, RX_READ, &read_op);
        if (rc)
            break;
        // printf("OUT: 0x%x %d -> %d\n", read_op.address, read_op.length, read_op.readed);

        if (read_op.readed != read_op.length) {
            read_op.address += read_op.readed;
            read_op.length -= read_op.readed;
            TaskSleepMsec(10);
        }
        else {
            break;
        }
    }

    if (rc < 0) {
        lprintf("Read RX failed");
    }
    */
}

void fpga_start_pps() {
    //int rc = ioctl(ad8370_fd, PPS_START, 1);
    //if (rc)
    //    sys_panic("Start PPS failed");
}

uint64_t fpga_read_pps() {
    uint32_t pps = 0;
    /*
    int rc;

    rc = ioctl(ad8370_fd, PPS_READ, &pps);
    if (rc && errno == EBUSY) {
        return 0;
    }

    if (rc)
        lprintf("read PPS failed");
    */

    return pps;
}

int fpga_set_antenna(int mask) {
    /*
    uint32_t gpio;
    int rc = ioctl(ad8370_fd, GET_GPIO_MASK, &gpio);
    if (rc)
        lprintf("Get GPIO failed");

    gpio &= ~GPIO_ANNENNA_MASK;
    gpio |= (mask & GPIO_ANNENNA_MASK);

    rc = ioctl(ad8370_fd, SET_GPIO_MASK, gpio);
    if (rc)
        lprintf("Set GPIO failed");
    */


    return 0;
}

static int fpga_set_bit(bool enabled, int bit) {
    /*
    uint32_t gpio;
    int rc = ioctl(ad8370_fd, GET_GPIO_MASK, &gpio);
    if (rc)
        lprintf("Get GPIO failed");

    if (enabled)
        gpio |= bit;
    else
        gpio &= ~bit;

    rc = ioctl(ad8370_fd, SET_GPIO_MASK, gpio);
    if (rc)
        lprintf("Set GPIO failed");
    */


    return 0;
}

int fpga_set_pga(bool enabled) {
    return fpga_set_bit(enabled, GPIO_PGA);
}

int fpga_set_dither(bool enabled) {
    return fpga_set_bit(enabled, GPIO_DITHER);
}

int fpga_set_led(bool enabled) {
    return fpga_set_bit(enabled, GPIO_LED);
}



void fpga_setovmask(uint32_t mask) {
    /// TODO
}

void fpga_setadclvl(uint32_t val) {
    /// TODO
}

int fpga_reset_wf(int wf_chan, bool cont) {
    int rc = 0;
    int data = wf_chan;

    if (cont) {
        data |= WF_READ_CONTINUES;
    }

    //rc = ioctl(ad8370_fd, WF_START, wf_chan);
    //if (rc)
    //    lprintf("WF Start failed");

    // printf("WF %d started[%d]\n", wf_chan, cont);

    return rc;
}

int fpga_wf_param(int wf_chan, int decimate, uint64_t i_phase) {
    int rc = 0;
    //wf_param_op param = { (__u16)wf_chan, (__u16)decimate, freq };
    //rc = ioctl(ad8370_fd, WF_PARAM, &param);
    //if (rc)
    //    lprintf("WF Parameter failed");

    return rc;
}

int fpga_get_wf(int rx_chan) {
    int ret = -1;

    while (ret) {
        ret = sem_wait(&wf_sem);
    }

    for (int i = 0; i < wf_channels; i++) {
        int empty = 0;
        bool exchanged = wf_using[i].compare_exchange_strong(empty, rx_chan + 10);

        if (exchanged) {
            return i;
        }
    }

    panic("Run out of wf channels");

    return -1;
}

void fpga_free_wf(int wf_chan, int rx_chan) {
    if (rx_chan > 0) {
        rx_chan += 10;

        bool exchanged = wf_using[wf_chan].compare_exchange_strong(rx_chan, 0);
        assert(exchanged);
    }
    else {
        wf_using[wf_chan].store(0);
    }

    int ret = sem_post(&wf_sem);
    if (ret) panic("sem_post failed");
}

void fpga_read_wf2(int wf_chan, void* buf, uint32_t size, uint32_t nsamples)
{
    //memset(buf, 0, size);
    memcpy(buf, rnd_wf_data, size);
    TaskSleepUsec(100);
}

void fpga_read_wf(int wf_chan, void* buf, uint32_t size) 
{
    memset(buf, 0, size);
    /*
    int rc;
    struct wf_read_op read_op = { (__u16)wf_chan, (__u32)buf, (__u32)size };
    while (true) {
        // printf("In: 0x%x %d\t", read_op.address, read_op.length);
        rc = ioctl(ad8370_fd, WF_READ, &read_op);

        if (rc)
            break;
        // printf("OUT: 0x%x %d -> %d\n", read_op.address, read_op.length, read_op.readed);

        if (read_op.readed != read_op.length) {
            read_op.address += read_op.readed;
            read_op.length -= read_op.readed;
            TaskSleepMsec(1);
            continue;
        }
        else {
            break;
        }
    }

    if (rc)
        lprintf("Read WF failed");
    */

}
