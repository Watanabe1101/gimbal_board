#include "SPICREATE.h"

static const char *TAG_SPICREATE = "SPICreate";

//-------------------------------------------
// コールバック関数 (CS制御用)
//-------------------------------------------
void IRAM_ATTR csReset(spi_transaction_t *t)
{
    // t->user にデバイス追加時に設定した csPin が格納されている想定
    gpio_num_t csPin = (gpio_num_t)(uint32_t)(t->user);
    gpio_set_level(csPin, 0);
}

void IRAM_ATTR csSet(spi_transaction_t *t)
{
    gpio_num_t csPin = (gpio_num_t)(uint32_t)(t->user);
    gpio_set_level(csPin, 1);
}

//-------------------------------------------
// SPICreate 実装
//-------------------------------------------
SPICreate::SPICreate()
{
    _host = (spi_host_device_t)-1;
    _initialized = false;
}

SPICreate::~SPICreate()
{
    // end()を呼ばずにデストラクタが呼ばれたら、あとでend()を呼ぶ
    if (_initialized)
    {
        end();
    }
}

esp_err_t SPICreate::begin(spi_host_device_t host,
                           gpio_num_t sck,
                           gpio_num_t miso,
                           gpio_num_t mosi,
                           uint32_t freq,
                           int dma_chan)
{
    if (_initialized)
    {
        ESP_LOGW(TAG_SPICREATE, "SPI bus already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // GPIO設定 (SCK, MISO, MOSIピンをSPIとして使用)
    // → spi_bus_initialize() 内部で自動的に設定されますが、
    //   必要に応じて gpio_config() でPull設定などしておくとよい

    // SPIバス設定
    spi_bus_config_t buscfg = {};
    buscfg.sclk_io_num = sck;
    buscfg.miso_io_num = miso;
    buscfg.mosi_io_num = mosi;
    buscfg.quadwp_io_num = -1;  // 未使用なら -1
    buscfg.quadhd_io_num = -1;  // 未使用なら -1
    buscfg.max_transfer_sz = 4096; // 必要に応じて調整

    // バス初期化
    esp_err_t ret = spi_bus_initialize(host, &buscfg, dma_chan);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPICREATE, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    _host = host;
    _initialized = true;
    ESP_LOGI(TAG_SPICREATE, "SPI bus initialized (host=%d, freq=%u)", host, freq);
    return ESP_OK;
}

esp_err_t SPICreate::end()
{
    if (!_initialized)
    {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = spi_bus_free(_host);
    if (ret == ESP_OK)
    {
        _initialized = false;
        ESP_LOGI(TAG_SPICREATE, "SPI bus freed (host=%d)", _host);
    }
    else
    {
        ESP_LOGE(TAG_SPICREATE, "spi_bus_free failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

spi_device_handle_t SPICreate::addDevice(const spi_device_interface_config_t *if_cfg,
                                         gpio_num_t csPin)
{
    if (!_initialized)
    {
        ESP_LOGE(TAG_SPICREATE, "SPI bus not initialized");
        return nullptr;
    }

    // CSピンのGPIO設定
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << csPin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // CSピンをデフォルトHIGHに
    gpio_set_level(csPin, 1);

    // デバイスインターフェース設定をコピーして user に csPin を設定
    spi_device_interface_config_t devcfg = *if_cfg;
    devcfg.spics_io_num = -1; // ハードウェアCSは使わず、ソフト制御する場合は -1
    devcfg.pre_cb = (devcfg.pre_cb) ? devcfg.pre_cb : csReset;
    devcfg.post_cb = (devcfg.post_cb) ? devcfg.post_cb : csSet;
    devcfg.user = (void *)(uint32_t)csPin; // CSピン番号を記録

    // デバイス追加
    spi_device_handle_t handle = nullptr;
    esp_err_t ret = spi_bus_add_device(_host, &devcfg, &handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPICREATE, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return nullptr;
    }

    ESP_LOGI(TAG_SPICREATE, "Device added (CS=%d)", csPin);
    return handle;
}

esp_err_t SPICreate::removeDevice(spi_device_handle_t handle)
{
    if (!handle)
    {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t ret = spi_bus_remove_device(handle);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPICREATE, "spi_bus_remove_device failed: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG_SPICREATE, "Device removed");
    }
    return ret;
}

void SPICreate::sendCmd(uint8_t cmd, spi_device_handle_t handle)
{
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.length = 8;    // 1 byte
    trans.tx_data[0] = cmd;

    pollTransmit(&trans, handle);
}

uint8_t SPICreate::readByte(uint8_t addr, spi_device_handle_t handle)
{
    // アドレス(8bit) + データ(8bit) の合計16bitを想定
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    trans.length = 16;
    trans.tx_data[0] = addr;

    pollTransmit(&trans, handle);

    return trans.rx_data[1]; // 上位8bit=tx_data(0)に被るので、受信データはrx_data[1]
}

void SPICreate::writeByte(uint8_t addr, uint8_t data, spi_device_handle_t handle)
{
    spi_transaction_t trans = {};
    trans.flags = SPI_TRANS_USE_TXDATA;
    trans.length = 16;
    trans.tx_data[0] = addr;
    trans.tx_data[1] = data;

    transmit(&trans, handle);
}

void SPICreate::transmit(const uint8_t *tx, uint8_t *rx, size_t len, spi_device_handle_t handle)
{
    // 全二重 (txとrxが同時に送受信) を想定
    spi_transaction_t trans = {};
    memset(&trans, 0, sizeof(trans));
    trans.length = len * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;

    transmit(&trans, handle);
}

void SPICreate::transmit(spi_transaction_t *trans, spi_device_handle_t handle)
{
    esp_err_t ret = spi_device_transmit(handle, trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPICREATE, "spi_device_transmit failed: %s", esp_err_to_name(ret));
    }
}

void SPICreate::pollTransmit(spi_transaction_t *trans, spi_device_handle_t handle)
{
    // 割り込みを使わないポーリング送受信
    esp_err_t ret = spi_device_polling_transmit(handle, trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG_SPICREATE, "spi_device_polling_transmit failed: %s", esp_err_to_name(ret));
    }
}
