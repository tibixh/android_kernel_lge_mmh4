#ifndef __MACH_EFUSE_MT6735_53_H__
#define __MACH_EFUSE_MT6735_53_H__

// -------------------------------
// Structure
// -------------------------------
typedef enum {
	EFUSE_IDX_COMMON_CTRL       =  0,
	EFUSE_IDX_SECURE_CTRL       =  1,
//	EFUSE_IDX_C_CTRL_1          = 2,    // do not use
	EFUSE_IDX_C_CTRL_0          =  2,
	EFUSE_IDX_C_TRLM            =  3,
	EFUSE_IDX_C_LOCK            =  4,
	EFUSE_IDX_SECURE_LOCK       =  9,
	EFUSE_IDX_COMMON_LOCK       = 10,
	EFUSE_IDX_MAX               = 11,
} efuse_idx;

typedef enum {
	EFUSE_RESULT_C_CTRL_0         = 0,  // Enable modem sbc
	EFUSE_RESULT_SECURE_CTRL      = 1,  // Blow secure related fuses
	EFUSE_RESULT_COMMON_LOCK      = 2,  // Disable common control blow
	EFUSE_RESULT_SECURE_LOCK      = 3,  // Disable secure control blow
	EFUSE_RESULT_SBC_PUBK_HASH    = 4,  // Use oem public key hash
	EFUSE_RESULT_CUSTOM_PUB_KEY   = 5,  // Prevent cross-download
	EFUSE_RESULT_COMMON_CTRL      = 6,  // Blow common fuses such as boot, USB
	EFUSE_RESULT_C_LOCK           = 7,  // Disable C ctrl & data blow
	EFUSE_RESULT_MAX
} efuse_result_etype;

typedef struct {
	u32 idx;
	u32 r_type;    // result_type
	u32 k_index;   // kernel index
	u32 b_mask;    // mask bits
} struct_efuse_list;

typedef struct {
	u32 seq;
	u32 k_index;             // kernel index
	u32 b_mask;              // mask bits
	u32 mtk_key;             // mtk default value
	u32 lge_default_key;     // lge default value
	u32 huaqin_default_key;
} struct_pubk_list;

typedef struct {
	u32 magic[2];
	u32 control;
	u32 ac_key[4];
	u32 sbc_pubk_hash[8];
	u32 usb_id[2];
	u32 c_data[6];
	u32 custk[4];
	u32 config[11];
	u32 padding[82];
	u32 partition_hash[8];
} struct_efuse_all_partition_all;

typedef struct {
	u32 pubk_hash[8];
	u32 config[11];
} struct_efuse_using_partition;

// -------------------------------
// define
// -------------------------------
#define SHA256_HASH_SIZE 32

#define SBC_KERNEL_INDEX   27
#define SBC_KERNEL_MASKING 0x2

#define MODULE_NAME "lge-qfprom"
#define MODULE_TABLE_NAME "lge,lge-qfprom"

#define EFUSE_PARTITION_NAME "efuse"
#define EFUSE_PARTITION_READ_SIZE 8    // 32 bits
#define EFUSE_PARTITION_USED_SIZE 512  // Efuse partition size : 512KB. But, real used size is 512 bytes.

#define EFUSE_PARTITION_OFFSET_SBC_PUBK_HASH 0x1C
#define EFUSE_PARTITION_OFFSET_CONFIG        0x6C
#define EFUSE_MAGIC_CODE1                    0x32715131
#define EFUSE_MAGIC_CODE2                    0x501409A7

#define RET_OK 0
#define RET_ERR 1

#define IDX_ARB_GROUP_0 24
#define IDX_ARB_GROUP_1 25

// -------------------------------
// Variables
// -------------------------------

static struct_efuse_list fuse_list[EFUSE_IDX_MAX] = {
	{EFUSE_IDX_COMMON_CTRL,      EFUSE_RESULT_COMMON_CTRL,       0, 0x00000017},
	{EFUSE_IDX_SECURE_CTRL,      EFUSE_RESULT_SECURE_CTRL,      27, 0x00000006},
	{EFUSE_IDX_C_CTRL_0,         EFUSE_RESULT_C_CTRL_0,         89, 0x00000006},
	{EFUSE_IDX_C_LOCK,           EFUSE_RESULT_C_LOCK,           91, 0x00000007},
	{EFUSE_IDX_SECURE_LOCK,      EFUSE_RESULT_SECURE_LOCK,      22, 0x00000033},
	{EFUSE_IDX_COMMON_LOCK,      EFUSE_RESULT_COMMON_LOCK,      23, 0x00000007}
};

static struct_pubk_list pubk_list[8] = {
	{0, 80, 0xFFFFFFFF, 0xE7E272C4, 0x0F2BDBEF, 0x76B0B05B},
	{1, 81, 0xFFFFFFFF, 0xD42B217D, 0xC4E52A01, 0x769F2652},
	{2, 82, 0xFFFFFFFF, 0x71E33163, 0x2CFF2C27, 0x02390BCA},
	{3, 83, 0xFFFFFFFF, 0x84BEE069, 0x1AAAA98B, 0xF9329FEA},
	{4, 84, 0xFFFFFFFF, 0x33BDA647, 0x5F662ACE, 0x7601D108},
	{5, 85, 0xFFFFFFFF, 0x80C50624, 0xC7BE7765, 0xF6076DCC},
	{6, 86, 0xFFFFFFFF, 0x6FF05606, 0x1B77F79C, 0x84FA28AF},
	{7, 87, 0xFFFFFFFF, 0x7A098E8E, 0xBDA8A101, 0xF30A13F3}
};

static bool b_efuse_read = false;
static struct mutex secdat_lock;

static struct_efuse_using_partition efuse_data;
static unsigned char hash_p[SHA256_HASH_SIZE] = {0x0, };

// -------------------------------
// functions
// -------------------------------

#endif // __MACH_EFUSE_MT6735_53_H__
