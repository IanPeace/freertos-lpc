
#include <bsp_ser.h>
#include <bsp_i2c_rom.h>

#define DEF_DISABLED                            0u
#define DEF_ENABLED                             1u

#define CPU_WORD_SIZE_08                        1    /*  8-bit word size (in octets).                                */
#define CPU_WORD_SIZE_16                        2    /* 16-bit word size (in octets).                                */
#define CPU_WORD_SIZE_32                        4    /* 32-bit word size (in octets).                                */
#define CPU_WORD_SIZE_64                        8    /* 64-bit word size (in octets).                                */
#define CPU_CFG_DATA_SIZE                       CPU_WORD_SIZE_32        /* Defines CPU data    word size  (in octets).          */

#define DEF_BIT_NONE                            0x00u
#define DEF_BIT_00                              0x01u
#define DEF_BIT_01                              0x02u
#define DEF_BIT_02                              0x04u
#define DEF_BIT_03                              0x08u
#define DEF_BIT_04                              0x10u
#define DEF_BIT_05                              0x20u
#define DEF_BIT_06                              0x40u
#define DEF_BIT_07                              0x80u
#define DEF_OCTET_NBR_BITS                      8u

#define DEF_INT_32U_MIN_VAL                     0u
#define DEF_INT_32U_MAX_VAL                     4294967295u
#define DEF_INT_CPU_U_MAX_VAL                   DEF_INT_32U_MAX_VAL
#define DEF_INT_CPU_NBR_BITS                    (CPU_CFG_DATA_SIZE * DEF_OCTET_NBR_BITS)

#define DEF_BIT(bit)                            (1u << (bit))
#define DEF_BIT_SET(val, mask)                  ((val) = ((val) | (mask)))
#define DEF_BIT_FIELD(bit_field, bit_shift)     ((((bit_field) >= DEF_INT_CPU_NBR_BITS) ? (DEF_INT_CPU_U_MAX_VAL)     \
                                                 : (DEF_BIT(bit_field) - 1uL)) \
                                                 << (bit_shift))
