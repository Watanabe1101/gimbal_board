// version: 2.0.0
#pragma once

#ifndef SPICREATE_Master
#define SPICREATE_Master

#include <driver/spi_master.h>
#include <deque>
#include <esp_log.h>

class SPICreate
{
    spi_bus_config_t bus_cfg = {};
    spi_device_handle_t handle[4];
    int deviceNum{0};
    spi_host_device_t host = SPI2_HOST;
    uint8_t mode = 3;   // must be 1 or 3
    int max_size{4094}; // default size
    uint32_t frequency{SPI_MASTER_FREQ_8M};

    std::deque<spi_transaction_t> transactions;
    int queue_size{1};

public:
    bool begin(spi_host_device_t host_in = SPI2_HOST,
               int sck = -1,
               int miso = -1,
               int mosi = -1,
               uint32_t f = SPI_MASTER_FREQ_8M);

    bool end();

    int addDevice(spi_device_interface_config_t *if_cfg, int cs);
    bool rmDevice(int deviceHandle);

    uint8_t readByte(uint8_t addr, int deviceHandle);
    void sendCmd(uint8_t cmd, int deviceHandle);
    void setReg(uint8_t addr, uint8_t data, int deviceHandle);

    void transmit(uint8_t *tx, int size, int deviceHandle);
    void transmit(uint8_t *tx, uint8_t *rx, int size, int deviceHandle);
    void transmit(spi_transaction_t *transaction, int deviceHandle);

    void pollTransmit(spi_transaction_t *transaction, int deviceHandle);
};

#endif