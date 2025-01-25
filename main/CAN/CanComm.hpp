#include "driver/twai.h"
#include "73CANConfig.hpp"

/**
 * @brief CAN (TWAI) 通信クラス
 *
 * - コンストラクタで自分の送信BoardID (self_board_id) を指定
 * - フィルタ用に受信 boardID を最大2つまで指定可能
 *    - 全て受け入れ (filter_board1=UNKNOWN, filter_board2=UNKNOWN) → AcceptAll
 *    - 特定の１基板からのみ受信 (filter_board1だけ有効) → Single Filter Mode
 *    - 特定の２基板からのみ受信 → Dual Filter Mode
 * - begin()/end() でドライバをインストール/アンインストール
 * - send() では contentID と data を指定して送信
 * - readFrameNoWait() は非ブロッキング受信(呼び出しじにRX FIFOが空ならば即座に返る)
 * - recover() でCANバス復帰を試みる
 */
class CanComm
{
public:
    /**
     * @param self_board_id     自分の送信BoardID
     * @param tx_gpio           TWAI TXピン
     * @param rx_gpio           TWAI RXピン
     * @param filter_board1(option)     1つ目のフィルタ対象BoardID (UNKNOWN=指定なし)
     * @param filter_board2(option)     2つ目 (UNKNOWN=指定なし)
     * @param tx_queue_len (default=10) 送信キュー長
     * @param rx_queue_len (default=10) 受信キュー長
     */
    CanComm(BoardID self_board_id,
            int tx_gpio,
            int rx_gpio,
            BoardID filter_board1 = CanBoardID::UNKNOWN,
            BoardID filter_board2 = CanBoardID::UNKNOWN,
            uint32_t tx_queue_len = 10,
            uint32_t rx_queue_len = 10, );

    /**
     * @brief ドライバインストール & スタート
     */
    esp_err_t begin();

    /**
     * @brief ドライバストップ & アンインストール
     */
    esp_err_t end();

    /**
     * @brief 送信 (BoardIDはコンストラクタ指定)
     */
    esp_err_t send(ContentID content, const uint8_t *data, size_t len);

    /**
     * @brief 非ブロッキング受信
     * @return
     *  - ESP_OK: 1フレーム取り出し
     *  - ESP_ERR_TIMEOUT: キュー空
     *  - ESP_ERR_NOT_SUPPORTED: 拡張ID受信 etc
     *  - ESP_ERR_INVALID_STATE: ドライバ未開始
     */
    esp_err_t readFrameNoWait(/*out*/ CanRxFrame &outFrame);

    /**
     * @brief バスオフリカバリ開始
     */
    esp_err_t recover();

private:
    BoardID self_board_id_;

    // フィルタ対象 (最大2つ)
    BoardID fb1_;
    BoardID fb2_;

    twai_general_config_t g_config_;
    twai_timing_config_t t_config_;
    twai_filter_config_t f_config_;

    bool driver_installed_;

private:
    // ID変換
    static uint16_t makeStdId(BoardID board, ContentID content);
    static void parseStdId(uint16_t sid, BoardID &bOut, ContentID &cOut);

    // フィルタ設定 (内部で呼ぶ)
    void setupFilter();
};