
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef OBJDICTIONARY_H
#define OBJDICTIONARY_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 master402_valueRangeTest (UNS8 typeValue, void * value);
const indextable * master402_scanIndexOD (CO_Data *d, UNS16 wIndex, UNS32 * errorCode);

/* Master node data struct */
extern CO_Data master402_Data;

extern UNS16 control_word_6040;		/* Mapped at index 0x2000, subindex 0x00*/
extern INTEGER8 modes_of_operation_6060;		/* Mapped at index 0x2001, subindex 0x00*/
extern INTEGER32 target_position_607a;		/* Mapped at index 0x2002, subindex 0x00*/
extern UNS32 profile_velocity_6081;		/* Mapped at index 0x2003, subindex 0x00*/
extern UNS16 status_word_6041;		/* Mapped at index 0x2004, subindex 0x00*/
extern INTEGER32 position_actual_value_6063;		/* Mapped at index 0x2005, subindex 0x00*/
extern INTEGER32 velocity_actual_value_606c;		/* Mapped at index 0x2006, subindex 0x00*/



extern UNS16 Controlword;		/* Mapped at index 0x6040, subindex 0x00 */
extern UNS16 Statusword;		/* Mapped at index 0x6041, subindex 0x00 */
extern INTEGER8 Modes_of_operation;		/* Mapped at index 0x6060, subindex 0x00 */
extern INTEGER32 Position_actual_value;		/* Mapped at index 0x6064, subindex 0x00 */
extern UNS16 Interpolation_value[6];		/* Mapped at index 0x60C1, subindex 0x00 */
extern INTEGER8 Modes_of_operation_display;		/* Mapped at index 0x6061, subindex 0x00 */



//  modbus rtu 命令变量定义
//  发送变量定义
// extern  UNS8  rtu_address_id;
// extern  UNS8  rtu_function_code;
// extern  UNS16 rtu_register_address;
// extern  UNS16 rtu_register_value;
// extern  UNS16 rtu_crc_value;
extern  UNS8  can_to_rtu_send_storage[8];

//  接收变量定义
extern  UNS8  rtu_to_can_recv_storage[8];


#endif // OBJDICTIONARY_H