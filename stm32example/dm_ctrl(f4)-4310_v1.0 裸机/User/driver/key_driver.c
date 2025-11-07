#include "key_driver.h"
#include "key_bsp.h"
#include "string.h"
#include "adc_modlue.h"
#include "key_modlue.h"

key_driver_t key_array[key_num];

/* �ص�ʾ��demo */
uint8_t ssy1 = 0, ssy2 = 0, ssy3 = 0;
void key_click_one_fun(void)
{
	ssy1++;
}
void key_click_double_fun(void)
{
	ssy2++;
}
void key_long_press_fun(void)
{
	ssy3++;
}
/**
***********************************************************************
* @brief:      key_driver_init(void)
* @param:      void
* @retval:     void
* @details:    Initialize key driver and related board support.
***********************************************************************
*/
void key_driver_init(void)
{
	key_bsp_init();
	key_array_init();
}
/**
***********************************************************************
* @brief:      key_check(void)
* @param:      void
* @retval:     void
* @details:    Poll raw BSP inputs and update debounce/hold filter state.
***********************************************************************
*/
void key_check(void)
{
	key_driver_t *key_p;
	key_p = &key_array[key_sw1];
	hold_filter(&key_p->bsp_hold_filter, key_1); // ��Ӳ�������İ���״̬�˲�
	
	key_p = &key_array[key_sw2];
	hold_filter(&key_p->bsp_hold_filter, key_mid); // ��Ӳ�������İ���״̬�˲�
	
	key_p = &key_array[key_sw3];
	hold_filter(&key_p->bsp_hold_filter, key_up); // ��Ӳ�������İ���״̬�˲�
	
	key_p = &key_array[key_sw4];
	hold_filter(&key_p->bsp_hold_filter, key_down); // ��Ӳ�������İ���״̬�˲�
	
	key_p = &key_array[key_sw5];
	hold_filter(&key_p->bsp_hold_filter, key_right); // ��Ӳ�������İ���״̬�˲�
	
	key_p = &key_array[key_sw6];
	hold_filter(&key_p->bsp_hold_filter, key_left); // ��Ӳ�������İ���״̬�˲�
}
/**
***********************************************************************
* @brief:      key_array_init(void)
* @param:      void
* @retval:     void
* @details:    Initialize key_driver_t structures and register callbacks.
***********************************************************************
*/
void key_array_init(void)
/**
***********************************************************************
* @brief:      key_attach(void)
* @param:      num: enum index of the key
* @param:      value: key event value
* @param:      callback: function to call when event occurs
* @retval:     void
* @details:    Attach a callback function to a key event for the given key.
***********************************************************************
*/
void key_attach(key_num_e num, key_value_e value, key_callback_t callback)
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	/**
	***********************************************************************
	* @brief:      hold_filter(hold_filter_t *filter_p, uint32_t variable)
	* @param:      filter_p: pointer to the hold_filter structure
	* @param:      variable: current input state marker
	* @retval:     uint8_t: filtered result (0 or 1)
	* @details:    Hold-type debounce/filter for button inputs. This function
	*              implements time-based state filtering to avoid jitter and
	*              to detect stable press/release events.
	***********************************************************************
	*/
	uint8_t hold_filter(hold_filter_t *filter_p, uint32_t variable)
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	/* ע�ᰴ�������ص� */
	key_p->key_callback[key_click_one] 		= motor_enter_set;
	key_p->key_callback[key_click_double] = key_click_double_fun;
	key_p->key_callback[key_long_press] 	= motor_set;
	
	key_p = &key_array[key_sw3];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;	// �͵�ƽ����
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	/* ע�ᰴ�������ص� */
	key_p->key_callback[key_click_one] 		= key_click_one_fun;
	key_p->key_callback[key_click_double] = key_click_double_fun;
	key_p->key_callback[key_long_press] 	= motor_clear_err;
	
	key_p = &key_array[key_sw4];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;	// �͵�ƽ����
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	/* ע�ᰴ�������ص� */
	key_p->key_callback[key_click_one] 		= key_click_one_fun;
	key_p->key_callback[key_click_double] = key_click_double_fun;
	key_p->key_callback[key_long_press] 	= motor_clear_para;
	
	key_p = &key_array[key_sw5];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;	// �͵�ƽ����
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	/* ע�ᰴ�������ص� */
	key_p->key_callback[key_click_one] 		= minus_key;
	key_p->key_callback[key_click_double] = key_click_double_fun;
	key_p->key_callback[key_long_press] 	= key_long_press_fun;
	
	key_p = &key_array[key_sw6];
	key_p->bsp_hold_filter.hold_time = 10;
	key_p->bsp_hold_filter.time_point = 0;
	key_p->bsp_hold_filter.trigger_variable = 0;	// �͵�ƽ����
	key_p->bsp_hold_filter.result = 0;
	key_p->shield = 0;
	key_p->press_time = 0;
	key_p->release_time = 0;
	key_p->state_machine = key_release;
	key_p->key_value = key_none;
	
	/* ע�ᰴ�������ص� */
	key_p->key_callback[key_click_one] 		= add_key;
	key_p->key_callback[key_click_double] = key_click_double_fun;
	key_p->key_callback[key_long_press] 	= key_long_press_fun;
}
/**
***********************************************************************
* @brief:      key_attach(void)
* @param:		   void
* @retval:     void
* @details:    ע���ֵ�����ص�����
***********************************************************************
**/
void key_attach(key_num_e num, key_value_e value, key_callback_t callback)
{
	key_driver_t *key_p;
	key_p = &key_array[num];
	key_p->key_callback[value] = callback;
}

/**
***********************************************************************
* @brief:      hold_filter(hold_filter_t *filter_p, uint32_t variable)
* @param:		   filter_p: ���������ṹ����
* @param:		   variable: ����״̬��־
* @retval:     void
* @details:    �����������˲�
***********************************************************************
**/
uint8_t hold_filter(hold_filter_t *filter_p, uint32_t variable)
{
	uint8_t variable_state = 0;
	uint32_t pass_time, system_time_temp;
	
	system_time_temp = get_key_bsptick();
	pass_time = system_time_temp - filter_p->time_point;

	if (filter_p->trigger_variable == variable)
	{
		variable_state = 1;
	}

	if (variable_state == filter_p->last_variable_state) 
	{
		if (filter_p->last_variable_state) 
		{
			if (pass_time > filter_p->hold_time && filter_p->result == 0) 
			{
				filter_p->result = 1;
			}
		} 
		else 
		{
			if (pass_time > filter_p->hold_time && filter_p->result == 1) 
			{
				filter_p->result = 0;
			}
		}
	} 
	else 
	{
		if (pass_time >= filter_p->hold_time) 
		{
			filter_p->time_point = system_time_temp;
		} 
		else 
		{
			filter_p->time_point = system_time_temp - (filter_p->hold_time - pass_time);
		}
	}
	filter_p->last_variable_state = variable_state;
	
	return filter_p->result;
}
/**
***********************************************************************
* @brief:      key_process(void)
* @param:      void
* @retval:     void
* @details:    High-level key processing: handles short press, long press,
*              single click and double click state transitions and invokes
*              registered callbacks when events occur.
***********************************************************************
*/
void key_process(void)
{
	uint8_t i;
	key_driver_t *key_p;

	key_check();
	
	for (i = 0; i < key_num; i++) 
	{
		key_p = &key_array[i];
		key_p->key_value = key_none;
		if (get_key_bsptick() > key_p->shield && key_p->shield != key_alway_shield) 
		{
			switch (key_p->state_machine) 
			{
				case key_release:
				if (key_p->bsp_hold_filter.result)  // ��������
				{
					key_p->press_time = get_key_bsptick();
					key_p->state_machine = key_short_pressing;
				}
				break;
				
				case key_short_pressing:
					if (key_p->bsp_hold_filter.result)  // ��Ȼ����
					{
						if (get_key_bsptick() - key_p->press_time > long_press_time)  // ��ס��ʱ�����������涨ʱ��
						{
							key_p->press_time = get_key_bsptick();
							key_p->key_value = key_long_press; // ������Ч
							key_p->state_machine = key_long_pressing;
						}
					} 
					else // ����
					{ 
						key_p->release_time = get_key_bsptick();
						key_p->state_machine = key_click_one_wait_for_double;
					}
					break;
					
				case key_click_one_wait_for_double:
					if (get_key_bsptick() - key_p->press_time < click_one_wait_for_double_time)  // ��˫���ȴ�ʱ����
					{
						if (key_p->bsp_hold_filter.result)  // �ٴΰ���
						{
							key_p->key_value = key_click_double; // ˫����Ч
							key_p->state_machine = key_double_pressing;
						}
					} 
					else  // ��ʱ
					{
						key_p->key_value = key_click_one; // ������Ч
						key_p->state_machine = key_release;
					}
					break;

				case key_double_pressing: // ˫����δ����
					if (!key_p->bsp_hold_filter.result) 
					{
						key_p->state_machine = key_release;
					}
					break;

				case key_long_pressing:
					if (key_p->bsp_hold_filter.result)  // ��Ȼ����
					{	
						if ((get_key_bsptick() - key_p->press_time) % long_press_effective_interval_time == 0) 
						{
							key_p->key_value = key_long_press; // ������Ч
						}
					} 
					else  // ���ֺ�λ
					{	
						key_p->state_machine = key_release;
					}
					break;

				default:
					break;
			}
		} 
		else 
		{
			if (!key_p->bsp_hold_filter.result) 
			{
				key_p->state_machine = key_release;
			}
		}
		key_callback(key_p);
	}
}

void key_callback(key_driver_t *key_p)
{
	switch(key_p->key_value)
	{
		case key_click_one:
			key_p->key_callback[key_click_one]();
			break;
		case key_click_double:
			key_p->key_callback[key_click_double]();
			break;
		case key_long_press:
			key_p->key_callback[key_long_press]();
			break;
		default:
			break;
	}
}





