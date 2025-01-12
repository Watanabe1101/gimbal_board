// version: 2.0.0
#include "SPICREATE.h" // 2.0.0

bool SPICreate::begin(spi_host_device_t host_in, int sck, int miso, int mosi, uint32_t f)
{
    frequency = f;

    bus_cfg.sclk_io_num = (sck < 0) ? 14 : sck;
    bus_cfg.miso_io_num = (miso < 0) ? 12 : miso;
    bus_cfg.mosi_io_num = (mosi < 0) ? 13 : mosi;
    bus_cfg.max_transfer_sz = max_size;

    if (mode != 1 && mode != 3) {
        mode = 3;
    }

    host = host_in; 
    if (host_in == SPI2_HOST) {
        dma_chan = 1; // 例: SPI2はDMAチャンネル1を使う
    } else if (host_in == SPI3_HOST) {
        dma_chan = 2; 
    }

    esp_err_t e = spi_bus_initialize(host, &bus_cfg, dma_chan);
    if (e != ESP_OK) {
        return false;
    }
    return true;
}

bool SPICreate::end()
{
    esp_err_t e = spi_bus_free(host);
    if (e != ESP_OK)
    {
        // printf("[ERROR] SPI bus free failed : %d\n", e);
        return false;
    }

    return true;
}
int SPICreate::addDevice(spi_device_interface_config_t *if_cfg, int cs)
{
    deviceNum++;

    if_cfg->spics_io_num = cs;
    
    CSs[deviceNum] = cs;

    if (deviceNum > 2) {
        ESP_LOGE("SPICreate", "Device number over");
        return 0; // handle[0..2]までしか用意していないので
    }

    // デバイス追加
    esp_err_t e = spi_bus_add_device(host, if_cfg, &handle[deviceNum]);
    if (e != ESP_OK) {
        return 0;
    }
    return deviceNum;
}

bool SPICreate::rmDevice(int deviceHandle)
{
    esp_err_t e = spi_bus_remove_device(handle[deviceHandle]);
    if (e != ESP_OK)
    {
        // printf("[ERROR] SPI bus remove device failed : %d\n", e);
        ESP_LOGI("SPICreate", "spi bus remove device failed : %d", e);
        return false;
    }
    return true;
}
void SPICreate::sendCmd(uint8_t cmd, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_TXDATA;
    comm.length = 8;
    comm.tx_data[0] = cmd;
    comm.user = (void *)CSs[deviceHandle];
    pollTransmit(&comm, deviceHandle);
}
uint8_t SPICreate::readByte(uint8_t addr, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA;
    comm.tx_data[0] = addr;
    comm.length = 16;
    comm.user = (void *)CSs[deviceHandle];
    pollTransmit(&comm, deviceHandle);
    return comm.rx_data[1];
}
void SPICreate::setReg(uint8_t addr, uint8_t data, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.flags = SPI_TRANS_USE_TXDATA;
    comm.length = 16;
    comm.tx_data[0] = addr;
    comm.tx_data[1] = data;
    transmit(&comm, deviceHandle);
}
void SPICreate::transmit(uint8_t *tx, int size, int deviceHandle)
{
    transmit(tx, NULL, size, deviceHandle);
}
void SPICreate::transmit(uint8_t *tx, uint8_t *rx, int size, int deviceHandle)
{
    spi_transaction_t comm = {};
    comm.length = size * 2 * 8;
    comm.rxlength = size * 8;
    comm.tx_buffer = tx;
    comm.rx_buffer = rx;
    transmit(&comm, deviceHandle);
}

void SPICreate::transmit(spi_transaction_t *transaction, int deviceHandle)
{
    esp_err_t e = spi_device_transmit(handle[deviceHandle], transaction);
    return;
}
void SPICreate::pollTransmit(spi_transaction_t *transaction, int deviceHandle)
{
    spi_device_polling_transmit(handle[deviceHandle], transaction);
    return;
}