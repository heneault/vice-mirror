/*
 * chat64.c - Cartridge handling, chat64 cart.
 *
 * Written by
 *  Yannick Heneault <yheneaul@gmail.com>
 *
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

#include "vice.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alarm.h"
#define CARTRIDGE_INCLUDE_SLOTMAIN_API
#include "c64cartsystem.h"
#undef CARTRIDGE_INCLUDE_SLOTMAIN_API
#include "cartio.h"
#include "cartridge.h"
#include "chat64.h"
#include "export.h"
#include "log.h"
#include "machine.h"
#include "maincpu.h"
#include "monitor.h"
#include "snapshot.h"
#include "types.h"
#include "util.h"
#include "crt.h"
#include "rs232drv.h"

/*
   "chat64"

   - 8kb ROM, mapped to $8000 in 8k game config that contain the communication firmware.

   A serial protocol is used to communicate with a real microcontroller (Esp32, pi core...)
  
   The commodore 64 use the io1 and io2 lines to perform read in write transaction with
   the microcontroller.
   For this vice emulator, we convert these to serial communication with a
   real microcontroller.
  
   The commodore use io1 line to send data and io2 to receive data

   -- C64 receiving from microcontroller --
   In the real cartridge, when the microcontroller send a value, it puts it
   on the data on the data bus during an io2 line access
   When the commodore 64 read the the data, the io2 lines trigger an interrupt
   to ack the read to the microcontroller.

   In the vice emulation, the microcontroller sends a byte to the c64 with this command:
     "$D" "123" 0

   Thus the command start with dollar sign and letter 'D' followed by
   the byte value in plain ascii and ending with a null byte.
   The value is kept in a buffer until read via access on the io2 line.
   When read, a ack command is sent from the c64
   to the controller like this:
     "$J" 0

   -- C64 sending to microcontroller --
   In the real cartridge a write on io1 send the data to TTL logic chips to be 
   stored and an interrupt is send to the controller to signal it.
   The controller read the data from the TTL chips. This allow more clock cycle 
   for the controller to read the value than its presence on the data bus

   In this vice emulation, a write access to io1 will send the serial command:
     "$I" "123" 0

   Thus the command start with dollar sign and letter 'I' followed by
   the byte value in plain ascii and ending with a null byte.

   -- Handshake commands --

   The microcontroller can send the command "$R" 0 to trigger a reset of the C64.

   The microcontroller can send the command "$N" 0 to trigger an NMI interrupt.
   This is usually to tell the commodore that a new byte is ready to be read on the io2 line.

   -- Protocol synchronization --
   The data value 0x80 is used as a synchronized flag. The commodore 64 will wait for a 0x80
   byte (via io2 read access) before sending a new byte if the controller is expecting one.

   The first byte sent by the commodore is the command byte. From that, 
   it is hardcoded what to expect to send and receive until transmission is finished with a 0x80.

   you can check the firmware at https://github.com/bvenneker/Chat64 for details.

*/

#define CHAT64_READ_IO_CYCLES 5000

struct alarm_s *chat64_alarm;

static CLOCK chat64_alarm_time;

static int chat64_fd = -1;
static uint8_t chat64_io2_byte = 0x80;
static bool chat64_received_data = false;

static void process_rs232(void)
{
    static bool receiving_command = false; 
    static bool receiving_data = false;
    static int data = 0;
    uint8_t buf;
    while (rs232drv_getc(chat64_fd, &buf) == 1)
    {
        if (receiving_data)
        {
            if (buf >= '0' && buf <= '9')
            {
                data = 10 * data + buf -'0';
            }
            else
            {
                chat64_io2_byte = data;
                chat64_received_data = true;
                receiving_data = false;
            }
        }
        else if (!receiving_command && buf == '$')
        {
            receiving_command = true;
        }
        else
        {
            if (buf == 'N')
            {
                cart_trigger_nmi();
            }
            else if (buf == 'R')
            {
                machine_trigger_reset(MACHINE_RESET_MODE_RESET_CPU);
            }
            else if (buf == 'D')
            {
                //read int and convert
                data = 0;
                receiving_data = true;
            }
            else
            {
                log_warning(LOG_DEFAULT, "chat64: unknown command %02x", buf);
            }

            receiving_command = false;
        }
    }
}

static void chat64_alarm_handler(CLOCK offset, void *data)
{
    process_rs232();
    chat64_alarm_time = maincpu_clk + CHAT64_READ_IO_CYCLES;
    alarm_set(chat64_alarm, chat64_alarm_time);
}

static void chat64_io1_write(uint16_t addr, uint8_t data)
{
    //send data to the esp32
    char buff[5];
    int i = 0;
    sprintf(buff, "%d", data);
    rs232drv_putc(chat64_fd, '$');
    rs232drv_putc(chat64_fd, 'I');
    do {
        rs232drv_putc(chat64_fd, buff[i]);
    } while(buff[i++]);
    chat64_io2_byte = 0;
    chat64_received_data = false;
}

static uint8_t chat64_io2_read(uint16_t addr)
{
    //receive data incoming from esp32 data bus
    if (chat64_received_data){
        rs232drv_putc(chat64_fd, '$');
        rs232drv_putc(chat64_fd, 'J');
        rs232drv_putc(chat64_fd, 0);
        chat64_received_data = false;
    }
    return chat64_io2_byte;
}

/* ---------------------------------------------------------------------*/

static io_source_t chat64_io1_device = {
    CARTRIDGE_NAME_CHAT64,        /* name of the device */
    IO_DETACH_CART,               /* use cartridge ID to detach the device when involved in a read-collision */
    IO_DETACH_NO_RESOURCE,        /* does not use a resource for detach */
    0xde00, 0xdeff, 0xff,         /* range for the device, address is ignored, reg:$de00, mirrors:$de01-$deff */
    0,                            /* read is never valid */
    chat64_io1_write,             /* write function */
    NULL,                         /* NO poke funtion */
    NULL,                         /* NO read function */
    NULL,                         /* NO peek function */
    NULL,                         /* NO dump function */
    CARTRIDGE_CHAT64,             /* cartridge ID */
    IO_PRIO_NORMAL,               /* normal priority, device read needs to be checked for collisions */
    0,                            /* insertion order, gets filled in by the registration function */
    IO_MIRROR_NONE                /* NO mirroring */
};

static io_source_t chat64_io2_device = {
    CARTRIDGE_NAME_CHAT64,        /* name of the device */
    IO_DETACH_CART,               /* use cartridge ID to detach the device when involved in a read-collision */
    IO_DETACH_NO_RESOURCE,        /* does not use a resource for detach */
    0xdf00, 0xdfff, 0xff,         /* range for the device, regs:$df00-$dfff */
    1,                            /* read is always valid */
    NULL,                         /* NO store function */
    NULL,                         /* NO poke function */
    chat64_io2_read,              /* read function */
    NULL,                         /* NO peek function */
    NULL,                         /* NO dump function */
    CARTRIDGE_CHAT64,             /* cartridge ID */
    IO_PRIO_NORMAL,               /* normal priority, device read needs to be checked for collisions */
    0,                            /* insertion order, gets filled in by the registration function */
    IO_MIRROR_NONE                /* NO mirroring */
};

static io_source_list_t *chat64_io1_list_item = NULL;
static io_source_list_t *chat64_io2_list_item = NULL;

static const export_resource_t export_res_chat64 = {
    CARTRIDGE_NAME_CHAT64, 0, 1, &chat64_io1_device, &chat64_io2_device, CARTRIDGE_CHAT64
};

/* ---------------------------------------------------------------------*/

uint8_t chat64_roml_read(uint16_t addr)
{
    return roml_banks[(addr & 0x1fff)];
}

void chat64_freeze(void)
{
    cartridge_release_freeze();
}

/* ---------------------------------------------------------------------*/

void chat64_reset(void)
{
    if (chat64_fd >= 0) {
        rs232drv_close(chat64_fd);
    }

    chat64_fd = rs232drv_open(0);
    if (chat64_fd < 0)
    {
        log_error(LOG_DEFAULT, "chat64: cannot open rs232 device");
        archdep_vice_exit(1);
    }

    chat64_io2_byte = 0x80;
    chat64_received_data = false;
}

void chat64_config_init(void)
{
    cart_config_changed_slotmain(CMODE_8KGAME, CMODE_8KGAME, CMODE_READ);
}

void chat64_config_setup(uint8_t *rawcart)
{
    memcpy(roml_banks, rawcart, 0x2000);
    cart_config_changed_slotmain(CMODE_8KGAME, CMODE_8KGAME, CMODE_READ);
}

/* ---------------------------------------------------------------------*/

static int chat64_common_attach(void)
{
    if (export_add(&export_res_chat64) < 0) {
        return -1;
    }

    chat64_alarm = alarm_new(maincpu_alarm_context, "Chat64Rs232Alarm", chat64_alarm_handler, NULL);
    chat64_alarm_time = maincpu_clk + CHAT64_READ_IO_CYCLES;
    alarm_set(chat64_alarm, chat64_alarm_time);

    chat64_io1_list_item = io_source_register(&chat64_io1_device);
    chat64_io2_list_item = io_source_register(&chat64_io2_device);

    return 0;
}

int chat64_bin_attach(const char *filename, uint8_t *rawcart)
{
    if (util_file_load(filename, rawcart, 0x2000, UTIL_FILE_LOAD_SKIP_ADDRESS) < 0) {
        return -1;
    }
    return chat64_common_attach();
}

int chat64_crt_attach(FILE *fd, uint8_t *rawcart)
{
    crt_chip_header_t chip;

    if (crt_read_chip_header(&chip, fd)) {
        return -1;
    }

    if (chip.size != 0x2000) {
        return -1;
    }

    if (crt_read_chip(rawcart, 0, &chip, fd)) {
        return -1;
    }

    return chat64_common_attach();
}

void chat64_detach(void)
{
    alarm_destroy(chat64_alarm);

    if (chat64_fd >= 0) {
        rs232drv_close(chat64_fd);
    }
    chat64_fd = -1;
    
    export_remove(&export_res_chat64);
    io_source_unregister(chat64_io1_list_item);
    io_source_unregister(chat64_io2_list_item);
    chat64_io1_list_item = NULL;
    chat64_io2_list_item = NULL;
}
