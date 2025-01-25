#pragma once

#include "CanComm.h"
#include <cstring>

//---------------------------------------
// ID変換
//---------------------------------------
uint16_t CanComm::makeStdId(BoardID board, ContentID content)
{
    uint16_t b = static_cast<uint8_t>(board) & 0x07;
    uint16_t c = static_cast<uint8_t>(content);
    return (b << 8) | c; // 3bit + 8bit = 11bit
}
void CanComm::parseStdId(uint16_t sid, BoardID &bOut, ContentID &cOut)
{
    uint8_t b = (sid >> 8) & 0x07;
    uint8_t c = sid & 0xFF;
    bOut = static_cast<BoardID>(b);
    cOut = static_cast<ContentID>(c);
}

//---------------------------------------
// コンストラクタ
//---------------------------------------
CanComm::CanComm(BoardID self_board_id,
                 int tx_gpio,
                 int rx_gpio,
                 BoardID filter_board1,
                 BoardID filter_board2,
                 uint32_t tx_queue_len,
                 uint32_t rx_queue_len, )
    : self_board_id_(self_board_id),
      fb1_(filter_board1),
      fb2_(filter_board2),
      driver_installed_(false)
{
    // config初期化
    g_config_ = {
        .controller_id = 0,
        .mode = TWAI_MODE_NORMAL,
        .tx_io = (gpio_num_t)tx_gpio,
        .rx_io = (gpio_num_t)rx_gpio,
        .clkout_io = GPIO_NUM_NC,
        .bus_off_io = GPIO_NUM_NC,
        .tx_queue_len = tx_queue_len,
        .rx_queue_len = rx_queue_len,
        .alerts_enabled = TWAI_ALERT_RX_DATA | TWAI_ALERT_BUS_OFF,
        .clkout_divider = 0,
        .intr_flags = ESP_INTR_FLAG_LOWMED,
        .sleep_allow_pd = 0};

    t_config_ = TWAI_TIMING_CONFIG_500KBITS();

    f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
}

esp_err_t CanComm::begin()
{
    if (driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    // フィルタ設定を反映
    setupFilter();

    // ドライバインストール
    esp_err_t err = twai_driver_install(&g_config_, &t_config_, &f_config_);
    if (err != ESP_OK)
    {
        return err;
    }
    // スタート
    err = twai_start();
    if (err == ESP_OK)
    {
        driver_installed_ = true;
    }
    else
    {
        twai_driver_uninstall();
    }
    return err;
}

//---------------------------------------
// send: 自BoardID <<8 | content
//---------------------------------------
esp_err_t CanComm::send(ContentID content, const uint8_t *data, size_t len)
{
    if (!driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 8)
    {
        return ESP_ERR_INVALID_ARG;
    }
    uint16_t sid = makeStdId(self_board_id_, content);

    twai_message_t tx_msg = {};
    tx_msg.extd = 0; // standard
    tx_msg.rtr = 0;  // data frame
    tx_msg.identifier = sid;
    tx_msg.data_length_code = len;
    if (data && len > 0)
    {
        memcpy(tx_msg.data, data, len);
    }

    return twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
}

//---------------------------------------
// readFrameNoWait
//---------------------------------------
esp_err_t CanComm::readFrameNoWait(CanRxFrame &outFrame)
{
    if (!driver_installed_)
    {
        return ESP_ERR_INVALID_STATE;
    }
    twai_message_t rx_msg;
    esp_err_t err = twai_receive(&rx_msg, 0);
    if (err != ESP_OK)
    {
        return err; // ESP_ERR_TIMEOUTなど
    }
    if (rx_msg.extd)
    {
        // 拡張IDは未対応
        return ESP_ERR_NOT_SUPPORTED;
    }

    parseStdId(rx_msg.identifier, outFrame.sender_board_id, outFrame.content_id);
    outFrame.dlc = rx_msg.data_length_code;
    memcpy(outFrame.data, rx_msg.data, rx_msg.data_length_code);
    for (int i = rx_msg.data_length_code; i < 8; i++)
    {
        outFrame.data[i] = 0;
    }
    return ESP_OK;
}

//---------------------------------------
// バスオフリカバリ
//---------------------------------------
esp_err_t CanComm::recover()
{
    return twai_initiate_recovery();
}

//---------------------------------------
// setupFilter(): fb1_, fb2_ の状況で切替
//---------------------------------------
void CanComm::setupFilter()
{
    // ===============
    // 0枚指定 → accept all
    // ===============
    if (fb1_ == BoardID::UNKNOWN && fb2_ == BoardID::UNKNOWN)
    {
        f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
        return;
    }

    // ===============
    // 1枚指定 → single filter
    // ===============
    if (fb1_ != BoardID::UNKNOWN && fb2_ == BoardID::UNKNOWN)
    {
        // Standard Filter (全11bit中 上位3bitだけ厳密に一致, 下位8bit無視)
        f_config_.single_filter = true;

        // 11bit上位3bitが fb1_ に一致するように code と mask を作る
        uint16_t boardSid = (static_cast<uint8_t>(fb1_) & 0x07) << 8;

        // SingleFilterMode のビット配置:
        //  - bits[0..10] = SFF ID
        //  - bits[11..12] = RTR,IDE
        //  → acceptance_code は下位11bitにIDを詰める
        // ここでは下位8bitを無視するので mask でそこを0に
        uint16_t maskVal = 0x700; // bits[10..8]だけ見る -> 0b11100000000
        uint32_t code = (boardSid & 0x7FF) << 21;
        uint32_t mask = (maskVal & 0x7FF) << 21;

        f_config_.acceptance_code = code;
        f_config_.acceptance_mask = mask;
        return;
    }

    // ===============
    // 2枚指定 → dual filter
    // ===============
    if (fb1_ != BoardID::UNKNOWN && fb2_ != BoardID::UNKNOWN)
    {
        f_config_.single_filter = false;

        auto encodeBoard = [&](CanBoardID b)
        {
            return static_cast<uint16_t>((static_cast<uint8_t>(b) & 0x07) << 8);
        };
        uint16_t sidA = encodeBoard(fb1_);
        uint16_t sidB = encodeBoard(fb2_);

        // DualFilterMode:
        //  - bits[29..19] = Filter A for SFF
        //  - bits[13..3]  = Filter B for SFF
        //  - mask も同様
        uint16_t maskVal = 0x700; // 上位3bitだけ見る

        uint32_t codeA = (sidA & 0x7FF) << 19;
        uint32_t codeB = (sidB & 0x7FF) << 3;
        uint32_t code = codeA | codeB;

        uint32_t maskA = (maskVal & 0x7FF) << 19;
        uint32_t maskB = (maskVal & 0x7FF) << 3;
        uint32_t mask = maskA | maskB;

        f_config_.acceptance_code = code;
        f_config_.acceptance_mask = mask;
        return;
    }

    // 上記以外のケース
    f_config_ = TWAI_FILTER_CONFIG_ACCEPT_ALL();
}