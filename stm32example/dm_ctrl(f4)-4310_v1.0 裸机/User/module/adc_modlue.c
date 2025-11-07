#include "adc_modlue.h"
#include "drivers.h"
#include "math.h"

uint16_t key_value;
float key_filter_val;
uint8_t key_up;
uint8_t key_down;
uint8_t key_mid;
uint8_t key_left;
uint8_t key_right;

/**
**********************************************************************
* @brief:      	antijamming_filter: ���Ŷ��˲�����
* @param[in]: 	value_real: ָ�����ʵ����ֵ��ָ�룬ͬʱ��Ϊ���룬���ڴ洢�˲����ֵ
* @param[in]:   value_new :�������ֵ�����ڽ����˲�
* @param[in]:   err_ware: ������Ŷ������������ж��Ƿ�����˲�
* @retval:      void
* @details:    	�ú������ڶ��������ֵ���п��Ŷ��˲��������˲�����洢��value_realָ��ĵ�ַ�С�
***********************************************************************
**/
void antijamming_filter(float *value_real, float value_new, uint16_t err_ware)
{
    float temp = 0.1f;

    // �����ǰʵ����ֵ����ֵ֮������Ŷ���������С�ڸ��Ŷ�������������˲�����
    if ((*value_real - value_new > err_ware) || (*value_real - value_new < (-1) * err_ware))
        *value_real += (value_new - *value_real) * temp;
}


/**
***********************************************************************
* @brief:      get_key_adc
* @param:			 void
* @retval:     void
* @details:    ��ȡ����adc��ֵ��ת��Ϊ 0 1 �ź�
***********************************************************************
**/
void get_key_adc(void)
{
	key_value = (float)adc1_median_filter(KEY);

	/* Apply anti-jamming filter to raw ADC value with threshold = 10 */
	antijamming_filter(&key_filter_val, key_value, 10);
	
	if (key_value>0 && key_value<100)
		key_mid = 0;
	else
		key_mid = 1;
	if (key_value>2200 && key_value<2500)
		key_up = 0;
	else
		key_up = 1;
	if (key_value>2800 && key_value<3500)
		key_down = 0;
	else
		key_down = 1;
	if (key_value>1500 && key_value<1800)
		key_left = 0;
	else
		key_left = 1;
	if (key_value>700 && key_value<1000)
		key_right = 0;
	else
		key_right = 1;
}







