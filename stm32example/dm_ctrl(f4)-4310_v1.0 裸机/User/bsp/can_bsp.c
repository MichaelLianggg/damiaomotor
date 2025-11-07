#include "can_bsp.h"
/**
************************************************************************
* @brief:       can_bsp_init(void)
* @param:       void
* @retval:      void
* @details:    Enable and start CAN peripherals.
************************************************************************
*/
void can_bsp_init(void)
{ 
	can_filter_init();
	HAL_CAN_Start(&hcan1);   
	HAL_CAN_Start(&hcan2);   
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);	
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
************************************************************************
* @brief:       can_filter_init(void)
* @param:       void
* @retval:      void
* @details:    Initialize CAN filters for both CAN interfaces.
************************************************************************
*/
void can_filter_init(void)
{
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
	HAL_CAN_Start(&hcan2);
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}
/**
************************************************************************
* @brief:       canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
* @param:       hcan: CAN handle
* @param:       id:   CAN device ID (standard ID)
* @param:       data: pointer to data buffer to send
* @param:       len:  number of bytes to send
* @retval:      status (0 = OK)
* @details:    Transmit a CAN frame using the HAL CAN API; tries mailboxes
*               in order until successful.
************************************************************************
*/
uint8_t canx_bsp_send_data(CAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
/**
************************************************************************
* @brief:       canx_bsp_receive(CAN_HandleTypeDef *hcan, uint16_t *rec_id, uint8_t *buf)
* @param:       hcan: CAN handle
* @param[out]:  rec_id: pointer to receive the source CAN ID
* @param:       buf: buffer to store received data
* @retval:      number of bytes received (DLC) or 0 on failure
* @details:    Receive a CAN frame from FIFO0 using HAL and return length.
************************************************************************
*/
uint8_t canx_bsp_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
			HAL_CAN_AddTxMessage(hcan, &tx_header, data, (uint32_t*)CAN_TX_MAILBOX2);
	/**
	************************************************************************
	* @brief:       HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
	* @param:       han: CAN handle
	* @retval:      void
	* @details:    HAL-layer callback for CAN RX FIFO0 pending interrupts; it
	*              dispatches to user-defined weak callback handlers for each
	*              CAN instance.
	************************************************************************
	*/
	void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan)
* @retval:     	���յ����ݳ���
* @details:    	��������
************************************************************************
**/
uint8_t canx_bsp_receive(hcan_t *hcan, uint16_t *rec_id, uint8_t *buf)
/**
************************************************************************
* @brief:       can1_rx_callback(void)
* @param:       void
* @retval:      void
* @details:    Weak user callback invoked when CAN1 receives a message.
************************************************************************
*/
__weak void can1_rx_callback(void)
/**
************************************************************************
* @brief:       can2_rx_callback(void)
* @param:       void
* @retval:      void
* @details:    Weak user callback invoked when CAN2 receives a message.
************************************************************************
*/
__weak void can2_rx_callback(void)
**/
void HAL_CAN_RxFifo0MsgPendingCallback(hcan_t *hcan)
{
	if(hcan == &hcan1) {
		can1_rx_callback();
	}
	if(hcan == &hcan2) {
		can2_rx_callback();
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
__weak void can1_rx_callback(void)
{

}
/**
************************************************************************
* @brief:      	can2_rx_callback(void)
* @param:       void
* @retval:     	void
* @details:    	���û����õĽ���������
************************************************************************
**/
__weak void can2_rx_callback(void)
{

}


