/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          vehicle systems.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Implementation of the Seagate ST-01 / ST-02 8-bit ISA SCSI
 *          host adapters.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/rom.h>
#include <86box/pic.h>
#include <86box/scsi.h>
#include <86box/scsi_device.h>

#define ST01_ROM_PATH       "roms/scsi/seagate/st01-02-bios-v3-2.bin"
#define ST01_ROM_SIZE       0x4000  /* 16 KB ROM window */
#define ST01_MEM_SIZE       0x4000  /* 16 KB total memory-mapped window */

/* Seagate ST-01/ST-02 Memory Offsets */
#define ST01_ROM_END        0x17FF
#define ST01_RAM_MIN        0x1800
#define ST01_RAM_MAX        0x19FF
#define ST01_STAT_CTRL_MIN  0x1A00
#define ST01_STAT_CTRL_MAX  0x1BFF
#define ST01_DATA_MIN       0x1C00
#define ST01_DATA_MAX       0x1DFF

/* Seagate Status Register Bits (Read at +0x1A00) */
#define STAT_BSY            0x01    /* SCSI Bus Busy */
#define STAT_IO             0x02    /* Input/Output (1 = Target -> Initiator) */
#define STAT_CD             0x04    /* Command/Data (1 = Command/Status) */
#define STAT_MSG            0x08    /* Message (Active-Low: 1 = No Message) */
#define STAT_REQ            0x10    /* SCSI Request */
#define STAT_SEL            0x20    /* Selection / Arbitration Lost */
#define STAT_IRQ            0x40    /* Interrupt Pending */
#define STAT_PARITY         0x80    /* Parity Error / Arbitration Complete */



/* Seagate Control Register Bits (Write at +0x1A00) */
#define CTRL_RST            0x01    /* Assert SCSI RST */
#define CTRL_SEL            0x02    /* Assert SEL */
#define CTRL_BSY            0x04    /* Assert BSY */
#define CTRL_ATN            0x08    /* Assert ATN */
#define CTRL_ARB            0x10    /* Start Hardware Arbitration */
#define CTRL_PARITY_EN      0x20    /* Enable Parity */
#define CTRL_IRQ_EN         0x40    /* Enable Interrupts */
#define CTRL_DATA_EN        0x80    /* Enable SCSI Data Bus */

typedef struct st01_t {
    uint8_t       bus;
    int           irq;
    uint32_t      bios_addr;
    uint8_t       ctrl;
    uint8_t       data_out;
    uint8_t       ram[0x80];        /* 128-byte Scratchpad RAM (+0x1800..+0x187F) */
    scsi_bus_t    scsibus;
    rom_t         bios_rom;
    mem_mapping_t mem_mapping;
} st01_t;

static int
st01_get_host_signals(st01_t *dev)
{
    int sig = 0;

    if (dev->ctrl & CTRL_SEL)
        sig |= BUS_SEL;
    if (dev->ctrl & CTRL_BSY)
        sig |= BUS_BSY;
    if (dev->ctrl & CTRL_ATN)
        sig |= BUS_ATN;
    if (dev->ctrl & CTRL_RST)
        sig |= BUS_RST;
    if (dev->ctrl & CTRL_DATA_EN)
        sig |= BUS_SETDATA(dev->data_out);

    return sig;
}

static void
st01_update_irq(st01_t *dev, uint8_t status)
{
    if ((dev->ctrl & CTRL_IRQ_EN) && (status & STAT_IRQ))
        picint(1 << dev->irq);
    else
        picintc(1 << dev->irq);
}

static uint8_t
st01_get_status(st01_t *dev)
{
    int     bus = scsi_bus_read(&dev->scsibus);
    uint8_t st  = 0;

if ((bus & BUS_BSY) || (dev->ctrl & CTRL_BSY)) {
    st |= STAT_BSY;
    if (!(bus & BUS_MSG))
        st |= STAT_MSG;
}
if (bus & BUS_REQ)
    st |= STAT_REQ;
if (bus & BUS_IO)
    st |= STAT_IO;
if (bus & BUS_CD)
    st |= STAT_CD;


    /* Bit 0x80 = Arbitration Complete when CTRL_ARB (0x10) is asserted */
    if (dev->ctrl & CTRL_ARB)
        st |= 0x80;

    if ((dev->ctrl & CTRL_IRQ_EN) && (bus & BUS_REQ))
        st |= STAT_IRQ;

    st01_update_irq(dev, st);
    return st;
}


static uint8_t
st01_mem_read(uint32_t addr, void *priv)
{
    st01_t  *dev    = (st01_t *) priv;
    uint32_t offset = addr & 0x3FFF;

    /* 0x0000 - 0x17FF and 0x2000 - 0x3FFF: Option ROM */
    if (offset <= ST01_ROM_END || offset >= 0x2000) {
        return dev->bios_rom.rom[offset & (ST01_ROM_SIZE - 1)];
    }

    /* 0x1800 - 0x19FF: 128 B Scratchpad RAM (mirrored every 128 bytes) */
    if (offset >= ST01_RAM_MIN && offset <= ST01_RAM_MAX) {
        return dev->ram[offset & 0x7F];
    }

    /* 0x1A00 - 0x1BFF: Status Register */
    if (offset >= ST01_STAT_CTRL_MIN && offset <= ST01_STAT_CTRL_MAX) {
        uint8_t st = st01_get_status(dev);
        pclog("ST01: READ  STAT offset=%04X -> %02X (ctrl=%02X)\n", offset, st, dev->ctrl);
        return st;
    }

    /* 0x1C00 - 0x1DFF: SCSI Data Port */
    if (offset >= ST01_DATA_MIN && offset <= ST01_DATA_MAX) {
        int     bus = scsi_bus_read(&dev->scsibus);
        uint8_t val = (dev->ctrl & CTRL_DATA_EN) ? dev->data_out : BUS_GETDATA(bus);

        pclog("ST01: READ  DATA offset=%04X -> %02X\n", offset, val);

        /* ST-01 auto-handshakes ACK on every data read when REQ is active */
        if (bus & BUS_REQ) {
            int host = st01_get_host_signals(dev);
            scsi_bus_update(&dev->scsibus, host | BUS_ACK);
            scsi_bus_update(&dev->scsibus, host & ~BUS_ACK);
        }
        return val;
    }

    return 0xFF;
}

static void
st01_mem_write(uint32_t addr, uint8_t val, void *priv)
{
    st01_t  *dev    = (st01_t *) priv;
    uint32_t offset = addr & 0x3FFF;

    /* 0x1800 - 0x19FF: 128 B Scratchpad RAM */
    if (offset >= ST01_RAM_MIN && offset <= ST01_RAM_MAX) {
        dev->ram[offset & 0x7F] = val;
        return;
    }

    /* 0x1A00 - 0x1BFF: Control Register */
    if (offset >= ST01_STAT_CTRL_MIN && offset <= ST01_STAT_CTRL_MAX) {
        pclog("ST01: WRITE CTRL offset=%04X <- %02X\n", offset, val);
        dev->ctrl = val;
        scsi_bus_update(&dev->scsibus, st01_get_host_signals(dev));
        st01_get_status(dev);
        return;
    }

    /* 0x1C00 - 0x1DFF: SCSI Data Port */
    if (offset >= ST01_DATA_MIN && offset <= ST01_DATA_MAX) {
        int host;
        pclog("ST01: WRITE DATA offset=%04X <- %02X\n", offset, val);
        dev->data_out = val;
        host          = st01_get_host_signals(dev);
        scsi_bus_update(&dev->scsibus, host);

        /* ST-01 auto-handshakes ACK on data write when REQ is active and SEL is not asserted */
        if (!(dev->ctrl & CTRL_SEL) && (scsi_bus_read(&dev->scsibus) & BUS_REQ)) {
            scsi_bus_update(&dev->scsibus, host | BUS_ACK);
            scsi_bus_update(&dev->scsibus, host & ~BUS_ACK);
        }
        return;
    }
}

static void
st01_timer_on_auto(void *priv, double period)
{
    (void) priv;
    (void) period;
}

static void *
st01_init(const device_t *info)
{
    st01_t *dev = calloc(1, sizeof(st01_t));
    if (!dev)
        return NULL;

    dev->bus       = scsi_get_bus();
    dev->bios_addr = device_get_config_hex20("bios_addr");
    dev->irq       = device_get_config_int("irq");

    if (rom_init(&dev->bios_rom, ST01_ROM_PATH, dev->bios_addr,
                 ST01_ROM_SIZE, ST01_ROM_SIZE - 1, 0, MEM_MAPPING_EXTERNAL) != 0) {
        free(dev);
        return NULL;
    }

    mem_mapping_disable(&dev->bios_rom.mapping);

    mem_mapping_add(&dev->mem_mapping, dev->bios_addr, ST01_MEM_SIZE,
                    st01_mem_read, NULL, NULL,
                    st01_mem_write, NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, dev);

    
    dev->scsibus.bus_device = dev->bus;
    dev->scsibus.timer      = st01_timer_on_auto;
    dev->scsibus.priv       = dev;

    scsi_bus_set_speed(dev->bus, 5000000.0);
    dev->scsibus.speed   = 0.2;
    dev->scsibus.divider = 1.0;
    dev->scsibus.multi   = 1.0;

    for (int i = 0; i < 8; i++)
        scsi_device_reset(&scsi_devices[dev->bus][i]);

    return dev;
    
}    

static void
st01_close(void *priv)
{
    st01_t *dev = (st01_t *) priv;
    if (dev)
        free(dev);
}

static int
st01_available(void)
{
    return rom_present(ST01_ROM_PATH);
}

static const device_config_t st01_config[] = {
    {
        .name           = "bios_addr",
        .description    = "BIOS address",
        .type           = CONFIG_HEX20,
        .default_string = "",
        .default_int    = 0xC8000,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "C800H", .value = 0xC8000 },
            { .description = "CA00H", .value = 0xCA000 },
            { .description = "CC00H", .value = 0xCC000 },
            { .description = "CE00H", .value = 0xCE000 },
            { .description = "D000H", .value = 0xD0000 },
            { .description = "D800H", .value = 0xD8000 },
            { .description = "DC00H", .value = 0xDC000 },
            { .description = "" }
        }
    },
    {
        .name           = "irq",
        .description    = "IRQ",
        .type           = CONFIG_SELECTION,
        .default_string = "",
        .default_int    = 5,
        .file_filter    = "",
        .spinner        = { 0 },
        .selection      = {
            { .description = "Disabled", .value = -1 },
            { .description = "IRQ 3",    .value = 3  },
            { .description = "IRQ 5",    .value = 5  },
            { .description = "" }
        }
    },
    { .name = "", .description = "", .type = CONFIG_END }
};

const device_t scsi_st01_device = {
    .name          = "Seagate ST-01",
    .internal_name = "st01",
    .flags         = DEVICE_ISA,
    .local         = 0,
    .init          = st01_init,
    .close         = st01_close,
    .reset         = NULL,
    .available     = st01_available,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = st01_config
};
