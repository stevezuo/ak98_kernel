#include <linux/string.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <mach/map.h>
#include <mach/ak880x_gpio.h>
#include <mach/lib_l2.h>

//************************** gpio *********************************//

bool gpio_set_dir(int gpiopin /*0~31,32~64,64~95,96~127 */ ,
		  int set /*0:out,1:in */ )
{
	unsigned int group_index;
	unsigned int bit_offset;

	group_index = (gpiopin >> 5) & 0x3;

	if (group_index > AK88_GPIO_MAX_PIN_NUM)
		return AK_FALSE;

	bit_offset = gpiopin & 0x1f;
	if (set)		//direction is IN
		AKSET_BITS(1 << bit_offset, AK88_GPIO_DIR(group_index));
	else			//direction is OUT
		AKCLR_BITS(1 << bit_offset, AK88_GPIO_DIR(group_index));

	//return AK_TRUE;
	return true;
}

bool gpio_set_level(int gpiopin /*0~31,32~64,64~95,96~127 */ , int set)
{
	unsigned int group_index;
	unsigned int bit_offset;

	group_index = (gpiopin >> 5) & 0x3;

	if (group_index > AK88_GPIO_MAX_PIN_NUM)
		return AK_FALSE;

	bit_offset = gpiopin & 0x1f;
	if (set)		//pull high 
		AKSET_BITS(1 << bit_offset, AK88_GPIO_OUT(group_index));
	else			//pull low 
		AKCLR_BITS(1 << bit_offset, AK88_GPIO_OUT(group_index));

	//return AK_TRUE;
	return true;
}

bool gpio_get_level(int gpiopin /*0~31,32~64,64~95,96~127 */ )
{
	unsigned int group_index;
	unsigned int bit_offset;
	bool ret;

	group_index = (gpiopin >> 5) & 0x3;

	if (group_index > AK88_GPIO_MAX_PIN_NUM)
		return AK_FALSE;

	bit_offset = gpiopin & 0x1f;

	ret = AKGET_BIT(1 << bit_offset, AK88_GPIO_IN(group_index));

	return ret;
}

bool gpio_set_output(int gpiopin /*0~31,32~64,64~95,96~127 */ ,
		     int level /*output level */ )
{
	unsigned int group_index;
	unsigned int bit_offset;

	group_index = (gpiopin >> 5) & 0x3;

	if (group_index > AK88_GPIO_MAX_PIN_NUM)
		return AK_FALSE;

	//set dir to output
	bit_offset = gpiopin & 0x1f;

	//direction is OUT, bit=0 for output
	AKCLR_BITS(1 << bit_offset, AK88_GPIO_DIR(group_index));

	//set output level
	if (level)		//pull high 
		AKSET_BITS(1 << bit_offset, AK88_GPIO_OUT(group_index));
	else			//pull low 
		AKCLR_BITS(1 << bit_offset, AK88_GPIO_OUT(group_index));

	return true;

}

bool gpio_get_input(int gpiopin /*0~31,32~64,64~95,96~127 */ )
{
	unsigned int group_index;
	unsigned int bit_offset;
	bool ret;

	group_index = (gpiopin >> 5) & 0x3;

	if (group_index > AK88_GPIO_MAX_PIN_NUM)
		return AK_FALSE;

	//set dir to input
	bit_offset = gpiopin & 0x1f;

	//direction is IN, bit=1 for output
	AKSET_BITS(1 << bit_offset, AK88_GPIO_DIR(group_index));

	//get input level
	ret = AKGET_BIT(1 << bit_offset, AK88_GPIO_IN(group_index));

	return ret;

}
