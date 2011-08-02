#ifndef		_FHA_ASA_H_
#define		_FHA_ASA_H_

#define ASA_FILE_FAIL   	0
#define ASA_FILE_SUCCESS    1
#define ASA_FILE_EXIST   	2
#define ASA_MODE_OPEN		0
#define ASA_MODE_CREATE		1

 
#define ASA_FORMAT_NORMAL   0
#define ASA_FORMAT_EWR      1
#define ASA_FORMAT_RESTORE  2


#define ASA_MAIN_VER  1
#define ASA_SUB_VER   5

T_U32  FHA_asa_scan(T_BOOL  bMount);

T_U32  FHA_asa_format(T_U32 type);

T_U32  FHA_set_bad_block(T_U32 block);

T_BOOL  FHA_check_bad_block(T_U32 block);

T_U32  FHA_get_bad_block(T_U32 start_block, T_U8 *pData, T_U32 blk_cnt);

T_U32  FHA_asa_write_file(T_U8 file_name[], const T_U8 *pData, T_U32 data_len, T_U8 mode);

T_U32  FHA_asa_read_file(T_U8 file_name[], T_U8 *pData, T_U32 data_len);

#endif    //

