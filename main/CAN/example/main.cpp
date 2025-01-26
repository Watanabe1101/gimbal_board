#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "CanComm.hpp"

extern "C" void app_main(void)
{
    // 例: 自分は GIMBAL(0b011) という基板として動作
    //     フィルタで1枚だけ: COM を受信したい
    //       → single filter
    //  (filter_board2=UNKNOWN(0xFF))
    //  + RX_DATAアラートを有効化
    CanComm can(
        /*self_board_id=*/BoardID::GIMBAL,
        /*tx_gpio=*/21,
        /*rx_gpio=*/22,
        /*filter_board1=*/BoardID::COM);

    if (can.begin() == ESP_OK)
    {
        printf("[MAIN] CAN driver started.\n");
    }
    else
    {
        printf("[MAIN] can.begin() failed.\n");
        return;
    }

    // 例: 送信 (ContentID=BOARD_STATE)
    //     => ID= (board=GIMBAL(0b011) <<8 | 0x02(BOARD_STATE)) => 0x302
    uint8_t data[1] = {0xAA};
    esp_err_t tx_err = can.send(ContentID::BOARD_STATE, data, 1);
    if (tx_err == ESP_OK)
    {
        printf("[MAIN] Tx success.\n");
    }
    else
    {
        printf("[MAIN] Tx failed.\n");
    }

    // 例: イベントドリブンで受信
    while (true)
    {
        // アラート待ち
        uint32_t alerts = 0;
        esp_err_t err = twai_read_alerts(&alerts, portMAX_DELAY); // rx_dataが到着するまで他のタスクを実行
        if (err == ESP_OK)
        {
            if (alerts & TWAI_ALERT_RX_DATA)
            {
                // RXキューに何か到着
                while (true)
                {
                    CanRxFrame rx;
                    esp_err_t e = can.readFrameNoWait(rx);
                    if (e == ESP_OK)
                    {
                        // 受信成功
                        printf("[MAIN] Rx from Board=%d, Content=0x%02X, dlc=%d\n",
                               (int)rx.sender_board_id,
                               (int)rx.content_id,
                               rx.dlc);
                    }
                    else if (e == ESP_ERR_TIMEOUT)
                    {
                        // キュー空
                        break;
                    }
                    else
                    {
                        // None
                        break;
                    }
                }
            }

            if (alerts & TWAI_ALERT_BUS_OFF)
            {
                // バスオフ検知 → リカバリ
                printf("[MAIN] BUS_OFF -> attempt recover.\n");
                can.recover();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED)
            {
                printf("[MAIN] BUS recovered.\n");
            }
        }
        else
        {
            // None
        }
    }
}
