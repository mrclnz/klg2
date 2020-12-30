/*
    KL-G2 Printer Utility

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <libusb.h>
#include "config.h"

/* USB constants */
const uint16_t KLG2_VID = 0x07CF;
const uint16_t KLG2_PID = 0x4112;
const uint8_t KLG2_IFACE = 0;
const uint8_t KLG2_EPOUT = 0x01;
const uint8_t KLG2_EPIN = 0x82;
const uint8_t KLG2_EPSIZE = 0x40;

/* Valid output endpoint transfer sizes */
enum EPSIZE_T {
    EPSIZE_1 = 1,
    EPSIZE_16 = 16,
    EPSIZE_64 = 64
};

/* Tape codes */
enum TAPECODE_T {
    TAPECODE_NOTAPE = 0x0000,
    TAPECODE_6MM = 0x8100,
    TAPECODE_9MM = 0x8500,
    TAPECODE_12MM = 0x8303,
    TAPECODE_18MM = 0x8703,
    TAPECODE_24MM = 0x8603
};

/* Margin/feed codes */
enum MARGINCODE_T {
    MARGINCODE_SMALL = 0x40,
    MARGINCODE_MEDIUM = 0x80,
    MARGINCODE_LARGE = 0x02,
    MARGINCODE_NOFEED = 0x01
};

/* Print density codes */
enum DENSITYCODE_T {
    DENSITYCODE_1 = 0xFE,
    DENSITYCODE_2 = 0xFF,
    DENSITYCODE_3 = 0x00,
    DENSITYCODE_4 = 0x01,
    DENSITYCODE_5 = 0x02
};

/* Cutter mode codes */
enum CUTTERCODE_T {
    CUTTERCODE_FULLCUT = 0x00,
    CUTTERCODE_HALFCUT = 01,
    CUTTERCODE_NOCUT = 0xFF
};

/* Options */
_Bool dump_comm = false;
enum DENSITYCODE_T opt_density = DENSITYCODE_3;
enum MARGINCODE_T opt_margin = MARGINCODE_SMALL;
enum TAPECODE_T opt_tape = TAPECODE_12MM;
enum CUTTERCODE_T opt_cutter = CUTTERCODE_HALFCUT;
enum OPERMODE_T  {
    OPERATION_PRINT,
    OPERATION_FEED,
    OPERATION_CUT,
    OPERATION_HALFCUT
} opt_operation = OPERATION_PRINT;

#define PRINTER_ACK 0x06
#define PRINTER_NAK 0x1E
#define PRINTER_STX 0x02

/* Printer handle */
libusb_device_handle *devhnd;

/* Image buffer */
#define IMAGE_ROWS 128
unsigned image_w;
uint8_t *image_stripes[IMAGE_ROWS];

/* Print pattern */
unsigned pattern_size;
uint8_t *pattern;

/*======================================================================
  Debug dump
*/
void debug_dump(char marker, const uint8_t *frame, unsigned flen)
{
    if (dump_comm) {
        putc(marker, stderr);
        int i;
        for (i = 0; i < flen; ++i) {
            fprintf(stderr, "%02X ", frame[i]);
        }
        putc('\n', stderr);
    }
}

/*======================================================================
  Receive a frame from the printer
*/
int recv_from_printer(uint8_t *d)
{
    uint8_t in[KLG2_EPSIZE];
    memset(in, 0, KLG2_EPSIZE);
    int rxcnt = 0;

    /* Endpoint buffer is 64 bytes */
    int rc = libusb_bulk_transfer(devhnd, KLG2_EPIN, in, KLG2_EPSIZE,
            &rxcnt, 0);
    if (rc) {
        fprintf(stderr, "Error receiving frame (%d)\n", rc);
        abort();
    }
    memcpy(d, in, rxcnt);
    debug_dump('<', in, rxcnt);
    return rxcnt;
}

/*======================================================================
  Send a frame to the printer
*/
int send_to_printer(const uint8_t *d, uint8_t cnt, enum EPSIZE_T epsize)
{
    uint8_t out[KLG2_EPSIZE];

    /* Important: keep padded with zeros */
    memset(out, 0, KLG2_EPSIZE);

    /* Important (2): it only accepts transfers of 1, 16 or 64 bytes
       depending on the command, not on the amount of data transferred.
       EXAMPLE: an incomplete raster transfer must be of 64 bytes even
       if it fits in 16 */
    if (cnt > epsize) {
        fprintf(stderr, "Shouldn't happen: output too large (%d)\n",
                cnt);
        abort();
    }
    memcpy(out, d, cnt);
    int txcnt = 0;
    debug_dump('>', out, cnt);
    int rc = libusb_bulk_transfer(devhnd, KLG2_EPOUT, out, epsize,
            &txcnt, 0);
    if (rc) {
        fprintf(stderr, "Error sending frame (%d)\n", rc);
        abort();
    }
    if (txcnt != epsize) {
        fprintf(stderr, "Incomplete transfer (%d/%d)\n",
                txcnt, epsize);
        abort();
    }
    return txcnt;
}

/*======================================================================
  Check printer readiness (can be slow)
*/
int printer_check_status(void)
{
    uint8_t rsp[KLG2_EPSIZE];
    static const uint8_t idcheck[] = {
        PRINTER_STX, 0x1D
    };
    send_to_printer(idcheck, 2, EPSIZE_16);
    int rc = recv_from_printer(rsp);
    if (rc != 6) {
        fprintf(stderr, "Unexpected status response length (%d)\n", rc);
        return 1;
    }
    if (rsp[0] != PRINTER_STX || rsp[1] != 0x80 || rsp[2] != 0x02 ||
            rsp[3] != 0x00 || rsp[4] != 0x00 || rsp[5] != 0xa6) {
        fputs("Status response mismatch\n", stderr);
        return 1;
    }
    return 0;
}

/*======================================================================
  Common case for receive PRINTER_ACK
*/
static int printer_recv_ack(const char *msg)
{
    uint8_t rsp[KLG2_EPSIZE];
    int rc = recv_from_printer(rsp);
    if (rc != 1 || rsp[0] != PRINTER_ACK) {
        fputs(msg, stderr);
        return 1;
    }
    return 0;
}

/*======================================================================
  Printer reset
*/
int printer_reset(void)
{
    static const uint8_t reset[] = {
        0x02, 0x01
    };
    send_to_printer(reset, 2, EPSIZE_16);
    return printer_recv_ack("Printer reset failed\n");
}

/*======================================================================
  Tape cut
*/
int printer_tape_cut(void)
{
    static const uint8_t tapecut[] = {
        0x08
    };
    send_to_printer(tapecut, 1, EPSIZE_1);
    return printer_recv_ack("Tape cut failed\n");
}

/*======================================================================
  Tape halfcut
*/
int printer_tape_halfcut(void)
{
    static const uint8_t tapecut[] = {
        0x09
    };
    send_to_printer(tapecut, 1, EPSIZE_1);
    return printer_recv_ack("Tape half cut failed\n");
}

/*======================================================================
  Tape feed
*/
int printer_tape_feed(void)
{
    static const uint8_t tapefeed[] = {
        0x0A
    };
    send_to_printer(tapefeed, 1, EPSIZE_1);
    return printer_recv_ack("Tape feed failed\n");
}

/*======================================================================
  Cancel job
*/
int printer_cancel_job(void)
{
    static const uint8_t canceljob[] = {
        0x18
    };
    send_to_printer(canceljob, 1, EPSIZE_1);

    /* No answer expected */
    return 0;
}

/*======================================================================
  Pre-print config (no idea of what it does)
*/
int printer_prejob(void)
{
    static const uint8_t cfg1[] = {
        PRINTER_STX, 0x02, 0x04, 0x00, 0x00, 0x09, 0x09, 0x01
    };
    send_to_printer(cfg1, 8, EPSIZE_16);

    if (printer_recv_ack("Prejob failed\n")) {
        return 1;
    }

    static const uint8_t cfg2[] = {
        PRINTER_STX, 0x82
    };
    send_to_printer(cfg2, 2, EPSIZE_16);
    uint8_t rsp[KLG2_EPSIZE];
    int rc = recv_from_printer(rsp);
    if (rc != 5) {
        fprintf(stderr, "Unexpected response length (%d)\n", rc);
        return 1;
    }
    if (rsp[0] != PRINTER_STX || rsp[1] != 0x80 || rsp[2] != 0x01 ||
            rsp[3] != 0x00 || rsp[4] != 0x01) {
        fputs("Response mismatch\n", stderr);
        return 1;
    }
    return 0;
}

/*======================================================================
  Printer Speed Adjust
*/
int printer_set_speed(void)
{
    static const uint8_t psa[] = {
        PRINTER_STX, 0x1C, 0x01, 0x00, 0x00
    };
    send_to_printer(psa, 5, EPSIZE_16);
    return printer_recv_ack("Speed adjust failed\n");
}

/*======================================================================
  Tape check
*/
int printer_check_tape(enum TAPECODE_T tapeid)
{
    /* For some reason there is an extra byte after the tape code proper
       (either 0 or 3), no idea of what it means */
    static uint8_t mtc[] = {
        PRINTER_STX, 0x17, 0x02, 0x00, 0x00, 0x00
    };
    mtc[4] = tapeid >> 8;
    mtc[5] = tapeid & 0xFF;
    send_to_printer(mtc, 6, EPSIZE_16);
    return printer_recv_ack("Tape check failed\n");
}

/*======================================================================
  Margin select
*/
int printer_set_margin(enum MARGINCODE_T marginid)
{
    static uint8_t afs[] = {
        PRINTER_STX, 0x0d, 0x01, 0x00, 0x00
    };
    afs[4] = marginid;
    send_to_printer(afs, 5, EPSIZE_16);
    return printer_recv_ack("Margin select failed\n");
}

/*======================================================================
  Density select (deployment mode select)
*/
int printer_set_density(enum DENSITYCODE_T densityid)
{
    static uint8_t dms[] = {
        PRINTER_STX, 0x09, 0x06, 0x00,
        0x00, 0x00, 0x01, 0x00, 0x00, 0x00
    };
    dms[8] = densityid;
    send_to_printer(dms, 10, EPSIZE_16);
    return printer_recv_ack("Print density select failed\n");
}

/*======================================================================
  Cutter mode
*/
int printer_set_cutter(enum CUTTERCODE_T cutterid)
{
    static uint8_t cms[] = {
        PRINTER_STX, 0x19, 0x01, 0x00, 0x00
    };
    cms[4] = cutterid;
    send_to_printer(cms, 5, EPSIZE_16);
    return printer_recv_ack("Cutter mode select failed\n");
}

/*======================================================================
  Get the mounted tape
*/
int printer_get_tape(enum TAPECODE_T *tapeid)
{
    uint8_t rsp[KLG2_EPSIZE];
    static const uint8_t idcheck[] = {
        PRINTER_STX, 0x1A
    };
    send_to_printer(idcheck, 2, EPSIZE_16);
    int rc = recv_from_printer(rsp);

    *tapeid = TAPECODE_NOTAPE;
    if (rc == 5) {
        switch (rsp[4]) {
        case 0x81:
            *tapeid = TAPECODE_6MM;
            break;
        case 0x85:
            *tapeid = TAPECODE_9MM;
            break;
        case 0x83:
            *tapeid = TAPECODE_12MM;
            break;
        case 0x87:
            *tapeid = TAPECODE_18MM;
            break;
        case 0x86:
            *tapeid = TAPECODE_24MM;
        }
        return 0;
    } else {
        return 1;
    }
}

/*======================================================================
  Pre feed the tape
*/
int printer_prefeed_tape(uint8_t amount)
{
    static uint8_t feed[] = {
        PRINTER_STX, 0x1B, 0x01, 0x00, 0x00
    };
    feed[4] = amount;
    send_to_printer(feed, 5, EPSIZE_16);
    return printer_recv_ack("Prefeed failed\n");
}

/*======================================================================
  Raster end
*/
int printer_raster_end(void)
{
    static const uint8_t raster_end[] = {
        PRINTER_STX, 0x04
    };
    send_to_printer(raster_end, 2, EPSIZE_16);
    return printer_recv_ack("Raster end failed\n");
}

/*======================================================================
  Print page
*/
int printer_print_page(void)
{
    static const uint8_t print_page[] = {
        0x0C
    };
    send_to_printer(print_page, 1, EPSIZE_1);
    return printer_recv_ack("Print page failed\n");
}


/*======================================================================
  Send raster block
*/
int printer_raster_block(uint8_t *blk, uint8_t blksize)
{
    blk[0] = PRINTER_STX;
    blk[1] = 0xFE;
    blk[2] = blksize;
    blk[3] = 0;
    send_to_printer(blk, blksize+4, EPSIZE_64);
    return printer_recv_ack("Raster block failed\n");
}


/*======================================================================
  Send raster data
  The printhead on the KL-G2 gives 8 points/mm (standard thermal 200dpi)
*/
int printer_send_raster(const uint8_t *raw, unsigned rawsize)
{
    unsigned sent_size = 0;
    unsigned page_size = 0;
    unsigned block_size = 0;
    uint8_t block[64];
    do {
        block[4 + block_size] = raw[sent_size];
        ++block_size;
        ++sent_size;
        ++page_size;
        if (block_size == 60
                || page_size == 8192
                || sent_size == rawsize) {
            if (printer_raster_block(block, block_size)) {
                return 1;
            }
            block_size = 0;
            if (sent_size == rawsize) {
                if (printer_raster_end()) {
                    return 1;
                }
            }
            if (page_size == 8192 || sent_size == rawsize) {
                if (printer_print_page()) {
                    return 1;
                }
                page_size = 0;
            }
        }
    } while (sent_size < rawsize);
    return 0;
}

/*======================================================================
  PBM Loader
*/
int load_image(FILE *fin)
{
    /* Check signature */
    if (getc(fin) != 'P' ||
            getc(fin) != '4' ||
            getc(fin) != '\n') {
        fputs("Input is not a packed PBM\n", stderr);
        return 1;
    }
    /* Check for comment */
    uint8_t ch = getc(fin);
    while (ch == '#') {
        do {
            ch = getc(fin);
        } while (ch != '\n');
        ch = getc(fin);
    }
    ungetc(ch, fin);
    unsigned img_w, img_h, pad_h;
    if (fscanf(fin, "%u %u\n", &img_w, &img_h) != 2) {
        fputs("PBM image size error\n", stderr);
        return 1;
    }

    if (img_h > IMAGE_ROWS) {
        fputs("WARNING: Image truncated\n", stderr);
        img_h = IMAGE_ROWS;
    }
    pad_h = (IMAGE_ROWS - img_h) / 2;

    image_w = img_w;
    img_w = (image_w + 7)/8;

    int i;
    for (i = 0; i < IMAGE_ROWS; ++i) {
        uint8_t *stripe = calloc(1, img_w);
        if (!stripe) {
            fputs("malloc failed\n", stderr);
            return 1;
        }
        image_stripes[i] = stripe;
    }
    for (i = 0; i < img_h; ++i) {
        if (fread(image_stripes[i+pad_h], img_w, 1, fin) != 1) {
            fputs("PBM ended unexpectedly\n", stderr);
            return 1;
        }
    }

    pattern_size = IMAGE_ROWS/8 * image_w;
    pattern = calloc(1, pattern_size);
    if (!pattern) {
        fputs("malloc failed\n", stderr);
        return 1;
    }

    /* Transpose to pattern */
    for (i = 0; i < IMAGE_ROWS; ++i) {
        unsigned x = 0;
        int w, b;
        for (w = 0; w < img_w; ++w) {
            for (b = 0; b < 8; ++b) {
                if ((image_stripes[i][w] << b) & 0x80) {
                    pattern[x * (IMAGE_ROWS/8) + i/8] |=
                        1 << (i%8);
                }
                ++x;
            }
        }
    }

    if (dump_comm) {
        for (i = 0; i < image_w; ++i) {
            fprintf(stderr, "%5d [", i);
            int j;
            for (j = 0; j < 16; ++j) {
                fprintf(stderr, "%02X", pattern[i*IMAGE_ROWS/8+j]);
            }
            fputs("]\n", stderr);
        }
    }
    return 0;
}

/*======================================================================
  Option handling
*/
void handle_options(int argc, char **argv)
{
    int opt, oval;
    while ((opt = getopt(argc, argv, "hvFCHm:t:c:d:")) != -1) {
        switch (opt) {
        case 'v':
            dump_comm = true;
            break;
        case 'F':
            opt_operation = OPERATION_FEED;
            break;
        case 'C':
            opt_operation = OPERATION_CUT;
            break;
        case 'H':
            opt_operation = OPERATION_HALFCUT;
            break;
        case 'm':
            oval = atoi(optarg);
            switch (oval) {
            case 0: opt_margin = MARGINCODE_NOFEED; break;
            case 1: opt_margin = MARGINCODE_SMALL; break;
            case 2: opt_margin = MARGINCODE_MEDIUM; break;
            case 3: opt_margin = MARGINCODE_LARGE; break;
            default:
                fputs("Invalid margin setting\n", stderr);
                exit(1);
            }
            break;
        case 'c':
            oval = atoi(optarg);
            switch (oval) {
            case 0: opt_cutter = CUTTERCODE_NOCUT; break;
            case 1: opt_cutter = CUTTERCODE_HALFCUT; break;
            case 2: opt_cutter = CUTTERCODE_FULLCUT; break;
            default:
                fputs("Invalid cutter setting\n", stderr);
                exit(1);
            }
            break;
        case 'd':
            oval = atoi(optarg);
            switch (oval) {
            case 1: opt_density = DENSITYCODE_1; break;
            case 2: opt_density = DENSITYCODE_2; break;
            case 3: opt_density = DENSITYCODE_3; break;
            case 4: opt_density = DENSITYCODE_4; break;
            case 5: opt_density = DENSITYCODE_5; break;
            default:
                fputs("Invalid print density setting\n", stderr);
                exit(1);
            }
            break;
        case 't':
            oval = atoi(optarg);
            switch (oval) {
            case 6: opt_tape = TAPECODE_6MM; break;
            case 9: opt_tape = TAPECODE_9MM; break;
            case 12: opt_tape = TAPECODE_12MM; break;
            case 18: opt_tape = TAPECODE_18MM; break;
            case 24: opt_tape = TAPECODE_24MM; break;
            default:
                fputs("Invalid tape size\n", stderr);
                exit(1);
            }
            break;

        case 'h':
        default:
            fprintf(stderr, "Usage: %s [OPTION]...\n", argv[0]);
            fputs("Prints the PBM on the standard input\n", stderr);
            fputs("  -F          Feed the tape an exit\n", stderr);
            fputs("  -C          Cut the tape an exit\n", stderr);
            fputs("  -H          Half-cut the tape an exit\n", stderr);
            fputs("  -m margin   Margin (0 none, *1 small, 2 medium, 3 large)\n", stderr);
            fputs("  -t tapesize Tape width in mm (6, 9, *12, 18, 24)\n", stderr);
            fputs("  -c cutmode  Cut more (0 no cut, *1 half-cut, 2 full-cut)\n", stderr);
            fputs("  -d density  Set print density (1-5, default 3)\n", stderr);
            fputs("  -v          Verbose (dump USB communications)\n", stderr);
            fputs("  -h          Display this help and exit\n", stderr);
            exit(1);
        }
    }
}

/*======================================================================
  Main
*/
int main(int argc, char **argv)
{
    handle_options(argc, argv);

    _Bool need_cancel = false;
    int rc = libusb_init(NULL);
    if (rc < 0)
        return rc;

    /* Apre la comunicazione usando VID e PID */
    devhnd = libusb_open_device_with_vid_pid(NULL, KLG2_VID, KLG2_PID);
    if (!devhnd) {
        fputs("Can't find or access printer\n", stderr);
        return 1;
    }
    rc = libusb_claim_interface(devhnd, KLG2_IFACE);
    if (rc) {
        fputs("Can't claim printer interface\n", stderr);
        return 1;
    }

    /* Sequenza standard */
    printer_check_status();
    printer_reset();
    switch (opt_operation) {
    case OPERATION_FEED:
        printer_tape_feed();
        break;
    case OPERATION_CUT:
        printer_tape_cut();
        break;
    case OPERATION_HALFCUT:
        printer_tape_halfcut();
        break;
    case OPERATION_PRINT:
        /* Read and prepare the image to be printed */
        rc = load_image(stdin);
        if (rc)
            return 1;

        if (!need_cancel)
            need_cancel = printer_prejob();
        if (!need_cancel)
            need_cancel = printer_check_tape(opt_tape);
        if (!need_cancel)
            need_cancel = printer_reset();
        if (!need_cancel)
            need_cancel = printer_set_speed();
        if (!need_cancel)
            need_cancel = printer_set_margin(opt_margin);
        if (!need_cancel)
            need_cancel = printer_set_density(opt_density);
        if (!need_cancel)
            need_cancel = printer_set_cutter(opt_cutter);
        if (!need_cancel)
            need_cancel = printer_check_status();
        if (!need_cancel)
            need_cancel = printer_send_raster(pattern, pattern_size);

        /* The standard program does this even in the success case */
        printer_cancel_job();
        break;
    }

    /* Cleanup */
    libusb_release_interface(devhnd, KLG2_IFACE);
    libusb_close(devhnd);
    libusb_exit(NULL);

    return 0;
}
