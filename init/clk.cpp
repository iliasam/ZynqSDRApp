/*
--------------------------------------------------------------------------------
This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Library General Public
License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version.
This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Library General Public License for more details.
You should have received a copy of the GNU Library General Public
License along with this library; if not, write to the
Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
Boston, MA  02110-1301, USA.
--------------------------------------------------------------------------------
*/

// Copyright (c) 2017 John Seamons, ZL4VO/KF6VO

#include "types.h"
#include "config.h"
#include "kiwi.h"
#include "clk.h"
#include "gps_.h"
#include "timing.h"
#include "web.h"
#include "misc.h"
#include "peri.h"
#include "eeprom.h"

#include <time.h>

#define CLK_PRINTF
#ifdef CLK_PRINTF
#define clk_printf(fmt, ...) \
    if (clk_printfs) printf(fmt, ##__VA_ARGS__)
#else
#define clk_printf(fmt, ...)
#endif

clk_t clk;

static int outside_window;
static bool clk_printfs;

double adc_clock_hz;

void clock_init() {
    bool err; // NB: all CFG_OPTIONAL because don't get defaulted early enough

    if (kiwi.airband) {
        adc_clock_hz = ADC_CLOCK_VHF;
    }
    else {
        adc_clock_hz = ADC_CLOCK_HF;
        //if (kiwi.narrowband)
        //    adc_clock_hz /= 2;
    }

    clk.do_corrections = cfg_int("ADC_clk2_corr", &err, CFG_OPTIONAL);
    if (err) clk.do_corrections = ADC_CLK_CORR_CONTINUOUS;
    clk.ext_ADC_clk = cfg_bool("ext_ADC_clk", &err, CFG_OPTIONAL);
    if (err) clk.ext_ADC_clk = false;
    clk.gpsdo_ext_clk = cfg_int("ext_ADC_freq", &err, CFG_OPTIONAL);
    if (err) clk.gpsdo_ext_clk = 0; // external is always 10Mhz

    if (clk.ext_ADC_clk)
        clk.clock_ref = 10 * MHz;
    else
        clk.clock_ref = eeprom_refclock();

    clk.adc_clock_base = ADC_CLOCK_TYP;
    printf("ADC_CLOCK: %.6f MHz %s\n",
           clk.adc_clock_base / MHz, clk.ext_ADC_clk ? "(ext clk connector)" : "");

    clk.manual_adj = 0;
    clk_printfs = kiwi_file_exists(DIR_CFG "/clk.debug");
}

void clock_conn_init(conn_t* conn) {
    conn->adc_clock_corrected = clk.adc_clock_base;
    conn->manual_offset = 0;
    conn->adjust_clock = true;
}

double adc_clock_system() {
    double new_clk = clk.adc_clock_base + clk.manual_adj;

    // apply effect of any manual clock corrections
    if (clk.do_corrections == ADC_CLK_CORR_DISABLED ||
        (clk.do_corrections >= ADC_CLK_CORR_CONTINUOUS && clk.adc_gps_clk_corrections == 0)) {
        for (conn_t* c = conns; c < &conns[N_CONNS]; c++) {
            if (!c->valid) continue;

            // adc_clock_corrected is what the sound and waterfall channels use for their NCOs
            c->adc_clock_corrected = new_clk;
        }

        if (clk.adc_clk_corrections != clk.last_adc_clk_corrections) {
            clk_printf("%-12s adc_clock_system() base=%.6lf man_adj=%d clk=%.6lf(%d)\n", "CLK",
                       clk.adc_clock_base / 1e6, clk.manual_adj, new_clk / 1e6, clk.adc_clk_corrections);
            clk.last_adc_clk_corrections = clk.adc_clk_corrections;
        }
    }

    return new_clk;
}

void clock_manual_adj(int manual_adj) {
    clk.manual_adj = manual_adj;
    adc_clock_system();
    clk.adc_clk_corrections++;
}

// Compute corrected ADC clock based on GPS time.
// Called on each GPS solution.
void clock_correction(double t_rx, u64_t ticks) {
    // record stats
    clk.gps_secs = t_rx;
    clk.ticks = ticks;

    bool initial_temp_correction = (clk.adc_clk_corrections <= 5);

    if (clk.do_corrections == ADC_CLK_CORR_DISABLED) {
        clk.is_corr = false;
        clk_printf("CLK CORR DISABLED\n");
        return;
    }

    u64_t diff_ticks = ticks;
    double gps_secs = 1.0;
    double new_adc_clock = diff_ticks / gps_secs;

// First correction allows wider window to capture temperature error.
// Subsequent corrections use a much tighter window to remove bad GPS solution outliers.
// Also use wider window if too many sequential solutions outside window.
#define MAX_OUTSIDE 8
    bool first_time_temp_correction = (clk.adc_clk_corrections == 0);

    double offset_window =
        (first_time_temp_correction || outside_window > MAX_OUTSIDE) ? PPM_TO_HZ(ADC_CLOCK_TYP, ADC_CLOCK_PPM_TYP) : PPM_TO_HZ(ADC_CLOCK_TYP, 1);
    double offset = new_adc_clock - clk.adc_clock_base; // offset from previous clock value

    // limit offset to a window to help remove outliers
    if (offset < -offset_window || offset > offset_window) {
        outside_window++;
        clk_printf("CLK BAD %4d offHz %2.0f winHz %2.0f SYS %.6f NEW %.6f GT %6.3f ticks %08x|%08x\n",
                   outside_window, offset, offset_window,
                   clk.adc_clock_base / 1e6, new_adc_clock / 1e6, gps_secs, PRINTF_U64_ARG(ticks));
        return;
    }
    outside_window = 0;

    // first correction handles XO temperature offset
    if (first_time_temp_correction) {
        clk.temp_correct_offset = offset;
    }

    // Perform modified moving average which seems to keep up well during
    // diurnal temperature drifting while dampening-out any transients.
    // Keeps up better than a simple cumulative moving average.

    static double adc_clock_mma;
#define MMA_PERIODS 32
    if (adc_clock_mma == 0) adc_clock_mma = new_adc_clock;
    adc_clock_mma = ((adc_clock_mma * (MMA_PERIODS - 1)) + new_adc_clock) / MMA_PERIODS;
    clk.adc_clk_corrections++;
    clk.adc_gps_clk_corrections++;

#ifdef CLK_PRINTF
    double diff_mma = adc_clock_mma - clk.adc_clock_base;
#endif
    clk_printf("CLK CORR %3d offHz %2.0f winHz %4.0lf MMA %.3lf(%6.3f) %4.1f NEW %.3lf\n",
               clk.adc_clk_corrections, offset, offset_window,
               adc_clock_mma, diff_mma, offset, new_adc_clock);

    clk.manual_adj = 0; // remove any manual adjustment now that we're automatically correcting
    clk.adc_clock_base = adc_clock_mma;


#if 0
        if (!ns_nom) ns_nom = clk.adc_clock_base;
        int bin = ns_nom - clk.adc_clock_base;
        ns_bin[bin+512]++;
#endif

    // If in one of the clock correction-limiting modes we still maintain the corrected clock values
    // for the benefit of GPS timestamping (i.e. clk.* values updated).
    // But don't update the conn->adc_clock_corrected values which keeps the audio and waterfall
    // NCOs from changing during the blocked intervals.
    if (clk.do_corrections > ADC_CLK_CORR_CONTINUOUS) {
        bool ok = false;
        const char* s;
        int min, sec;
        utc_hour_min_sec(NULL, &min, &sec);

        switch (clk.do_corrections) {

        case ADC_CLK_CORR_EVEN_2_MIN:
            s = "even 2 min";
            ok = ((min & 1) && sec > 51);
            break;

        case ADC_CLK_CORR_5_MIN:
            s = "5 min";
            ok = ((min % 5) == 4 && sec > 50);
            break;

        case ADC_CLK_CORR_15_MIN:
            s = "15 min";
            ok = ((min % 15) == 14 && sec > 50);
            break;

        case ADC_CLK_CORR_30_MIN:
            s = "30 min";
            ok = ((min % 30) == 29 && sec > 52);
            break;

        default:
            s = "???";
            break;
        }

        if (!initial_temp_correction) {
            clk_printf("CLK %02d:%02d every %s, %s\n", min, sec, s, ok ? "OK" : "skip");
            clk.is_corr = ok;
            if (!ok) return;
        }
        else {
            clk_printf("CLK %02d:%02d every %s, but initial temp correction\n", min, sec, s);
        }
    }

    clk.is_corr = true;

    // Even if !adjust_clock mode is set adjust for first_time_temp_correction.
    // If any of the correction-limiting modes are in effect, apply during the entire window
    // to make sure correction is propagated.
    u4_t now = timer_sec();
    static u4_t last;
    if (now > (last + 10) || clk.do_corrections > ADC_CLK_CORR_CONTINUOUS) {
        clk_printf("%-12s CONN ", "CLK");
        for (conn_t* c = conns; c < &conns[N_CONNS]; c++) {
            if (!c->valid || (!c->adjust_clock && !first_time_temp_correction)) continue;

            // adc_clock_corrected is what the sound and waterfall channels use for their NCOs
            c->adc_clock_corrected = clk.adc_clock_base;
            clk_printf("%d ", c->self_idx);
        }
        clk_printf("\n");
        clk_printf("%-12s APPLY clk=%.3lf(%d)\n", "CLK", clk.adc_clock_base, clk.adc_clk_corrections);
        last = now;
    }
}
