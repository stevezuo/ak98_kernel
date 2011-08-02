/* 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

/* struct ak98_nand_set
 *
 * define an set of one or more nand chips registered with an unique mtd
 *
 * nr_chips	 = number of chips in this set
 * nr_partitions = number of partitions pointed to be partitoons (or zero)
 * name		 = name of set (optional)
 * nr_map	 = map for low-layer logical to physical chip numbers (option)
 * partitions	 = mtd partition list
*/

struct ak98_nand_set {
	int nr_chips;
	int nr_partitions;
	char *name;
	int *nr_map;
	struct mtd_partition *partitions;

	/* timing information for controller */
	unsigned int cmd_len;
	unsigned int data_len;
	unsigned char col_cycle;
	unsigned char row_cycle;
};

struct ak98_platform_nand {
	int nr_sets;
	struct ak98_nand_set *sets;

	void (*select_chip) (struct ak98_nand_set *, int chip);
};
