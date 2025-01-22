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
#include <linux/spi/spidev.h>

#define MAX_WF_CHANNELS         2

#define SPI_CMD_SET_RX_FREQ     1
#define SPI_CMD_SET_WF_FREQ     2
#define SPI_CMD_SET_WF_DECIM    3

#define SDRDMA_NAME             "/dev/sdrdma"

#define RX_READ_BAD_SIZE	10
#define RX_READ_NO_DATA		11
#define RX_READ_OK			20

static bool init;

static const uint8_t buss_id = 0;
static const uint8_t chip_addr = 0x60;

static uint32_t SPI_SPEED = 5000000;
static uint8_t SPI_BITS = 8;//per word


static int sdrdma_fd;

static int sdr_spi_fd;

static sem_t wf_sem;
static std::atomic<int> wf_using[4];
static int wf_channels;

typedef struct
{
    uint64_t freq_code;
    int decim_code;
} waterfall_state_t;

waterfall_state_t wf_buf[MAX_WF_CHANNELS];

struct iq2_t {
    s2_t i, q;
} __attribute__((packed));


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

    scall("/dev/spidev0.0", sdr_spi_fd = open("/dev/spidev0.0", O_RDWR));

    scall(SDRDMA_NAME, sdrdma_fd = open(SDRDMA_NAME, O_RDWR | O_SYNC));
    if (sdrdma_fd <= 0) {
        sys_panic("Failed to open kernel driver");
    }
    else
    {
        printf("Succesfiully opened kernel device driver!\n");
    }

    //fcntl(sdrdma_fd, F_SETFD, FD_CLOEXEC);
    //ioctl(sdrdma_fd, CLK_SET, int_clk);

    // set airband mode
    rf_enable_airband(kiwi.airband);

    // set default attn to 0
    rf_attn_set(0);

    wf_channels = (fpga_signature() >> 8) & 0x0f;
    sem_init(&wf_sem, 0, wf_channels);

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
    //if (ioctl(sdrdma_fd, AD8370_SET, gain) < 0) {
    //    printf("AD8370 set RF failed: %s\n", strerror(errno));
    // }

    return;
}

void rf_enable_airband(bool enabled) {
    //if (ioctl(sdrdma_fd, MODE_SET, (int)enabled) < 0) {
    //    printf("AD8370 set mode failed %s\n", strerror(errno));
    //}

    return;
}

void peri_free() {
    assert(init);
    close(sdrdma_fd);
    close(sdr_spi_fd);
}

static std::atomic<int> write_enabled(0);
void sd_enable(bool write) {
    if (write) {
    }
    else {
    }
}

static int last = 100;

void spi_transfer(int fd, uint8_t *tx, uint8_t *rx, size_t len)
{
	int ret;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = len,
        .speed_hz = SPI_SPEED,
		.delay_usecs = 0,
		.bits_per_word = SPI_BITS,
	};

    /*
	if (mode & SPI_TX_QUAD)
		tr.tx_nbits = 4;
	else if (mode & SPI_TX_DUAL)
		tr.tx_nbits = 2;
	if (mode & SPI_RX_QUAD)
		tr.rx_nbits = 4;
	else if (mode & SPI_RX_DUAL)
		tr.rx_nbits = 2;
	if (!(mode & SPI_LOOP)) {
		if (mode & (SPI_TX_QUAD | SPI_TX_DUAL))
			tr.rx_buf = 0;
		else if (mode & (SPI_RX_QUAD | SPI_RX_DUAL))
			tr.tx_buf = 0;
	}
    */

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message");
}
////////////////////////////////
// FPGA DNA
////////////////////////////////

u64_t fpga_dna() {
    int rc;
    uint64_t signature = 0;
    //rc = ioctl(sdrdma_fd, GET_DNA, &signature);
    //if (rc)
    //    sys_panic("Get FPGA Signature failed");

    return signature;
}

uint32_t fpga_signature() {
    int rc;
    uint32_t signature = 8 + (1 << 8);
    //rc = ioctl(sdrdma_fd, GET_SIGNATURE, &signature);
    //if (rc)
    //    sys_panic("Get FPGA Signature failed");

    return signature;
}

void fpga_start_rx() {
    //uint32_t decim = uint32_t(ADC_CLOCK_NOM / 12000 / 256);
    //int rc = ioctl(sdrdma_fd, RX_START, decim);
    //if (rc)
    //    sys_panic("Start RX failed");
}

void fpga_rxfreq(int rx_chan, uint64_t i_phase) {

    uint8_t payload_tx[6];
    uint8_t payload_rx[6];

    payload_tx[0] = SPI_CMD_SET_RX_FREQ;
    payload_tx[1] = (uint8_t)rx_chan;

    //printf("SPI1: %lu\n", (uint32_t)i_phase);

    payload_tx[2] = (uint8_t)(((uint32_t)i_phase & 0xFF000000) >> 24);
    payload_tx[3] = (uint8_t)(((uint32_t)i_phase & 0x00FF0000) >> 16);
    payload_tx[4] = (uint8_t)(((uint32_t)i_phase & 0x0000FF00) >> 8);
    payload_tx[5] = (uint8_t)(((uint32_t)i_phase & 0x000000FF) >> 0);

    spi_transfer(sdr_spi_fd, payload_tx, payload_rx, sizeof(payload_tx));

    printf("SPI: CMD=%x CH=%x %x %x %x %x\n", payload_tx[0], payload_tx[1], payload_tx[2], payload_tx[3], payload_tx[4], payload_tx[5]);
}

void fpga_read_rx(void* buf, uint32_t size) {
    memset(buf, 0, size);

    static uint32_t test_cnt = 0;
    test_cnt++;
    
    int rc;
    struct rx_read_op read_op = { (__u32)buf, size }; //address, length

    while (true) {
        // printf("In: 0x%x %d\t", read_op.destination, read_op.length);
        rc = ioctl(sdrdma_fd, RX_READ, &read_op);
        if (rc)
            break;

        //if (test_cnt < 20)
        //    printf("OUT: 0x%x len=%d res= %d \n", read_op.destination, read_op.length, read_op.result);

        if (read_op.result != RX_READ_OK)
        {
            TaskSleepMsec(10);
        }
        else {
            break;
        }
    }

    if (rc < 0) {
        lprintf("Read RX failed");
    }
}

void fpga_start_pps() {
    //int rc = ioctl(sdrdma_fd, PPS_START, 1);
    //if (rc)
    //    sys_panic("Start PPS failed");
}

uint64_t fpga_read_pps() {
    uint32_t pps = 0;
    /*
    int rc;

    rc = ioctl(sdrdma_fd, PPS_READ, &pps);
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
    int rc = ioctl(sdrdma_fd, GET_GPIO_MASK, &gpio);
    if (rc)
        lprintf("Get GPIO failed");

    gpio &= ~GPIO_ANNENNA_MASK;
    gpio |= (mask & GPIO_ANNENNA_MASK);

    rc = ioctl(sdrdma_fd, SET_GPIO_MASK, gpio);
    if (rc)
        lprintf("Set GPIO failed");
    */


    return 0;
}

static int fpga_set_bit(bool enabled, int bit) {
    /*
    uint32_t gpio;
    int rc = ioctl(sdrdma_fd, GET_GPIO_MASK, &gpio);
    if (rc)
        lprintf("Get GPIO failed");

    if (enabled)
        gpio |= bit;
    else
        gpio &= ~bit;

    rc = ioctl(sdrdma_fd, SET_GPIO_MASK, gpio);
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

//WATERFALL *********************************************

/// @brief Start WF channel read
/// @param wf_chan 
/// @param cont - continues
/// @return 
int fpga_reset_wf(int wf_chan, bool cont) {
    int rc = 0;
    int data = wf_chan;

    if (cont) {
        data |= WF_READ_CONTINUES;
    }

    //rc = ioctl(sdrdma_fd, WF_START, wf_chan);
    //if (rc)
    //    lprintf("WF Start failed");

    // printf("WF %d started[%d]\n", wf_chan, cont);

    return rc;
}

void fpga_set_wf_freq(int wf_chan, uint64_t i_phase) 
{
    uint8_t payload_tx[6];
    uint8_t payload_rx[6];

    payload_tx[0] = SPI_CMD_SET_WF_FREQ;
    payload_tx[1] = (uint8_t)wf_chan + 1;

    payload_tx[2] = (uint8_t)(((uint32_t)i_phase & 0xFF000000) >> 24);
    payload_tx[3] = (uint8_t)(((uint32_t)i_phase & 0x00FF0000) >> 16);
    payload_tx[4] = (uint8_t)(((uint32_t)i_phase & 0x0000FF00) >> 8);
    payload_tx[5] = (uint8_t)(((uint32_t)i_phase & 0x000000FF) >> 0);

    spi_transfer(sdr_spi_fd, payload_tx, payload_rx, sizeof(payload_tx));

    printf("SPI: CMD=%x CH=%x %x %x %x %x\n", payload_tx[0], payload_tx[1], payload_tx[2], payload_tx[3], payload_tx[4], payload_tx[5]);
}

void fpga_set_wf_cic_decim(int wf_chan, int decimation) 
{
    uint8_t payload_tx[6];
    uint8_t payload_rx[6];

    payload_tx[0] = SPI_CMD_SET_WF_DECIM;
    payload_tx[1] = (uint8_t)wf_chan + 1;
    payload_tx[2] = 0;
    payload_tx[3] = 0;
    payload_tx[4] = (uint8_t)(((uint32_t)decimation & 0x0000FF00) >> 8);
    payload_tx[5] = (uint8_t)(((uint32_t)decimation & 0x000000FF) >> 0);

    spi_transfer(sdr_spi_fd, payload_tx, payload_rx, sizeof(payload_tx));

    printf("SPI: CMD=%x CH=%x %x %x %x %x\n", payload_tx[0], payload_tx[1], payload_tx[2], payload_tx[3], payload_tx[4], payload_tx[5]);
}

int fpga_wf_param(int wf_chan, int decimate, uint64_t i_phase) 
{
    int rc = 0;

    if (wf_chan >= MAX_WF_CHANNELS)
        return 0;

    waterfall_state_t *wf_state = &wf_buf[wf_chan];

    if (wf_state->freq_code != i_phase)
    {
        wf_state->freq_code = i_phase;

        float freq = (float)i_phase / (float)pow(2, RX_WF_DDS_RESOLUTION) * (float)40.0f;//mhz
        printf("WF ch=%d changed dec=%d  FREQ=%f\n", wf_chan, decimate, freq);
        fpga_set_wf_freq(wf_chan, i_phase);
    }

    if (wf_state->decim_code != decimate)
    {
        wf_state->decim_code = decimate;
        float freq = (float)(wf_state->freq_code) / (float)pow(2, RX_WF_DDS_RESOLUTION) * (float)40.0f;//mhz
        printf("WF ch=%d changed DEC=%d  freq=%f\n", wf_chan, decimate, freq);
        fpga_set_wf_cic_decim(wf_chan, decimate);
    }

    return rc;
}

void fpga_read_wf2(int wf_chan, void* buf, uint32_t size, uint32_t nsamples)
{
    //memset(buf, 0, size);
    memcpy(buf, rnd_wf_data, size);
    TaskSleepUsec(100);
}

void fpga_read_wf(int wf_chan, void* buf, uint32_t size) 
{
    //memset(buf, 0, size);

    int rc;
    struct wf_read_op read_op = { (__u16)wf_chan, (__u32)buf, (__u32)size };
    while (true) {
        // printf("In: 0x%x %d\t", read_op.destination, read_op.length);
        rc = ioctl(sdrdma_fd, WF_READ, &read_op);

        if (rc)
            break;
        // printf("OUT: 0x%x %d -> %d\n", read_op.destination, read_op.length, read_op.result);

        if (read_op.result != RX_READ_OK) 
        {
            TaskSleepMsec(10);
            continue;
        }
        else 
        {
            break;
        }
    }

    if (rc)
        lprintf("Read WF failed");

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
