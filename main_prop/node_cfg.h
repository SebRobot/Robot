/*
 * node_cfg.h
 *
 *  Created on: 14 nov. 2015
 *      Author: Sébastien Malissard
 */

#ifndef NODE_CFG_H_
#define NODE_CFG_H_

#define MYADDRX 0
#define MYADDRI 0
#define MYADDRU 0
#define MYADDRD ADDRD1_MAIN_PROP_SIMU
#define MYADDR (MYADDRX?:MYADDRI?:MYADDRU?:MYADDRD)

#define MYROLE ROLE_PRIM_PROPULSION
// MYROLE must be equal to role_get_role(MYADDR)

#define BN_INC_MSG_BUF_SIZE     4
#define BN_WAIT_XBEE_SND_FAIL   5000000
#define BN_MAX_RETRIES          2
#define BN_ACK_TIMEOUT          100    //in ms

// #define ARCH_X86_LINUX       in symbols
// #define ARCH_LITTLE_ENDIAN   in symbols
#define UART_WAITFRAME_TIMEOUT  10      //in µs
#define UART_READBYTE_TIMEOUT   100000  //in µs

#endif /* NODE_CFG_H_ */
