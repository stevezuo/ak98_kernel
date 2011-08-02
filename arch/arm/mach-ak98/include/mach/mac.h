/* platfrom data for platfrom device structure's platfrom_data field */

#define MAC_ADDR_LEN 6
#define MAC_ADDR_STRING_LEN (MAC_ADDR_LEN * 3 - 1)

struct ak98_mac_data {
	unsigned int	flags;
	unsigned char	dev_addr[MAC_ADDR_LEN];
};
