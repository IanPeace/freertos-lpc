/*
 * FreeRTOS+UDP V1.0.4
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_DEFAULT_IP_CONFIG_H
#define FREERTOS_DEFAULT_IP_CONFIG_H

/* This file provides default values for configuration options that are missing
from the FreeRTOSIPConfig.h configuration header file. */

#ifndef ipconfigUSE_NETWORK_EVENT_HOOK
	#define ipconfigUSE_NETWORK_EVENT_HOOK 0
#endif

#ifndef ipconfigMAX_SEND_BLOCK_TIME_TICKS
	#define ipconfigMAX_SEND_BLOCK_TIME_TICKS ( 20 / portTICK_RATE_MS )
#endif

#ifndef ipconfigARP_CACHE_ENTRIES
	#define ipconfigARP_CACHE_ENTRIES		10
#endif

#ifndef ipconfigMAX_ARP_RETRANSMISSIONS
	#define ipconfigMAX_ARP_RETRANSMISSIONS ( 5 )
#endif

#ifndef ipconfigMAX_ARP_AGE
	#define ipconfigMAX_ARP_AGE			150
#endif

#ifndef ipconfigINCLUDE_FULL_INET_ADDR
	#define ipconfigINCLUDE_FULL_INET_ADDR	1
#endif

#ifndef ipconfigNUM_NETWORK_BUFFERS
	#define ipconfigNUM_NETWORK_BUFFERS		45
#endif

#ifndef ipconfigEVENT_QUEUE_LENGTH
	#define ipconfigEVENT_QUEUE_LENGTH		( ipconfigNUM_NETWORK_BUFFERS + 5 )
#endif

#ifndef ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND
	#define ipconfigALLOW_SOCKET_SEND_WITHOUT_BIND 1
#endif

#ifndef updconfigIP_TIME_TO_LIVE
	#define updconfigIP_TIME_TO_LIVE		128
#endif

#ifndef ipconfigCAN_FRAGMENT_OUTGOING_PACKETS
	#define ipconfigCAN_FRAGMENT_OUTGOING_PACKETS 0
#endif

#ifndef ipconfigNETWORK_MTU
	#define ipconfigNETWORK_MTU 1500
#endif

#ifndef ipconfigUSE_DHCP
	#define ipconfigUSE_DHCP	1
#endif

#ifndef ipconfigMAXIMUM_DISCOVER_TX_PERIOD
	#ifdef _WINDOWS_
		#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD		( 999 / portTICK_RATE_MS )
	#else
		#define ipconfigMAXIMUM_DISCOVER_TX_PERIOD		( 30000 / portTICK_RATE_MS )
	#endif /* _WINDOWS_ */
#endif /* ipconfigMAXIMUM_DISCOVER_TX_PERIOD */

#ifndef ipconfigUSE_DNS
	#define ipconfigUSE_DNS		1
#endif

#ifndef ipconfigREPLY_TO_INCOMING_PINGS
	#define ipconfigREPLY_TO_INCOMING_PINGS				1
#endif

#ifndef ipconfigSUPPORT_OUTGOING_PINGS
	#define ipconfigSUPPORT_OUTGOING_PINGS				0
#endif

#ifndef updconfigLOOPBACK_ETHERNET_PACKETS
	#define updconfigLOOPBACK_ETHERNET_PACKETS	0
#endif

#ifndef ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES
	#define ipconfigFILTER_OUT_NON_ETHERNET_II_FRAMES 1
#endif

#ifndef ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES
	#define ipconfigETHERNET_DRIVER_FILTERS_FRAME_TYPES	1
#endif

#ifndef configINCLUDE_TRACE_RELATED_CLI_COMMANDS
	#define ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS 0
#else
	#define ipconfigINCLUDE_EXAMPLE_FREERTOS_PLUS_TRACE_CALLS configINCLUDE_TRACE_RELATED_CLI_COMMANDS
#endif

#ifndef ipconfigFREERTOS_PLUS_NABTO
	#define ipconfigFREERTOS_PLUS_NABTO 0
#endif

#ifndef ipconfigNABTO_TASK_STACK_SIZE
	#define ipconfigNABTO_TASK_STACK_SIZE ( configMINIMAL_STACK_SIZE * 2 )
#endif

#ifndef ipconfigNABTO_TASK_PRIORITY
	#define ipconfigNABTO_TASK_PRIORITY	 ( ipconfigUDP_TASK_PRIORITY + 1 )
#endif

#ifndef ipconfigSUPPORT_SELECT_FUNCTION
	#define ipconfigSUPPORT_SELECT_FUNCTION 0
#endif
		
#ifndef ipconfigETHERNET_DRIVER_ADDS_UDP_CHECKSUM
	#define ipconfigETHERNET_DRIVER_ADDS_UDP_CHECKSUM 0
#endif

#ifndef ipconfigETHERNET_DRIVER_ADDS_IP_CHECKSUM
	#define ipconfigETHERNET_DRIVER_ADDS_IP_CHECKSUM 0
#endif

#ifndef ipconfigETHERNET_DRIVER_CHECKS_IP_CHECKSUM
	#define ipconfigETHERNET_DRIVER_CHECKS_IP_CHECKSUM 0
#endif

#ifndef ipconfigETHERNET_DRIVER_CHECKS_UDP_CHECKSUM
	#define ipconfigETHERNET_DRIVER_CHECKS_UDP_CHECKSUM 0
#endif

#endif /* FREERTOS_DEFAULT_IP_CONFIG_H */
