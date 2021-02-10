/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "lwip/api.h"
#include "lwip/opt.h"

#if LWIP_IPV4 && LWIP_DHCP && LWIP_NETCONN

#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/netifapi.h"
#include "lwip/prot/dhcp.h"
#include "lwip/tcpip.h"
#include "lwip/sys.h"
#include "enet_ethernetif.h"

#include "board.h"
#include "fsl_phy.h"

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* MAC address configuration. */
#define configMAC_ADDR                     \
    {                                      \
        0x02, 0x12, 0x13, 0x10, 0x15, 0x11 \
    }

/* Address of PHY interface. */
#define EXAMPLE_PHY_ADDRESS BOARD_ENET0_PHY_ADDRESS

/* MDIO operations. */
#define EXAMPLE_MDIO_OPS enet_ops

/* PHY operations. */
#define EXAMPLE_PHY_OPS phyksz8081_ops

/* ENET clock frequency. */
#define EXAMPLE_CLOCK_FREQ CLOCK_GetFreq(kCLOCK_CoreSysClk)

/* GPIO pin configuration. */
#define BOARD_LED_GPIO       BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN   BOARD_LED_RED_GPIO_PIN
#define BOARD_SW_GPIO        BOARD_SW3_GPIO
#define BOARD_SW_GPIO_PIN    BOARD_SW3_GPIO_PIN
#define BOARD_SW_PORT        BOARD_SW3_PORT
#define BOARD_SW_IRQ         BOARD_SW3_IRQ
#define BOARD_SW_IRQ_HANDLER BOARD_SW3_IRQ_HANDLER


#ifndef EXAMPLE_NETIF_INIT_FN
/*! @brief Network interface initialization function. */
#define EXAMPLE_NETIF_INIT_FN ethernetif0_init
#endif /* EXAMPLE_NETIF_INIT_FN */

/*! @brief Stack size of the thread which prints DHCP info. */
#define PRINT_THREAD_STACKSIZE 512

/*! @brief Priority of the thread which prints DHCP info. */
#define PRINT_THREAD_PRIO DEFAULT_THREAD_PRIO

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void tcpexample_thread(void *arg);
/*******************************************************************************
 * Variables
 ******************************************************************************/

static mdio_handle_t mdioHandle = {.ops = &EXAMPLE_MDIO_OPS};
static phy_handle_t phyHandle   = {.phyAddr = EXAMPLE_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &EXAMPLE_PHY_OPS};
static ip_addr_t example_ip_addr;

static sys_sem_t semx;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Prints DHCP status of the interface when it has changed from last status.
 *
 * @param arg pointer to network interface structure
 */
static void print_dhcp_state(void *arg)
{
    struct netif *netif = (struct netif *)arg;
    struct dhcp *dhcp;
    u8_t dhcp_last_state = DHCP_STATE_OFF;

    if (sys_sem_new(&semx, 0)) {
    	LWIP_ASSERT("print_dhcp_state(): Semaphore creation failed: print_dhcp", 0);
    }

    while (netif_is_up(netif))
    {
        dhcp = netif_dhcp_data(netif);

        if (dhcp == NULL)
        {
            dhcp_last_state = DHCP_STATE_OFF;
        }
        else if (dhcp_last_state != dhcp->state)
        {
            dhcp_last_state = dhcp->state;

            PRINTF("print_dhcp_state(): DHCP state       : ");
            switch (dhcp_last_state)
            {
                case DHCP_STATE_OFF:
                    PRINTF("OFF");
                    break;
                case DHCP_STATE_REQUESTING:
                    PRINTF("REQUESTING");
                    break;
                case DHCP_STATE_INIT:
                    PRINTF("INIT");
                    break;
                case DHCP_STATE_REBOOTING:
                    PRINTF("REBOOTING");
                    break;
                case DHCP_STATE_REBINDING:
                    PRINTF("REBINDING");
                    break;
                case DHCP_STATE_RENEWING:
                    PRINTF("RENEWING");
                    break;
                case DHCP_STATE_SELECTING:
                    PRINTF("SELECTING");
                    break;
                case DHCP_STATE_INFORMING:
                    PRINTF("INFORMING");
                    break;
                case DHCP_STATE_CHECKING:
                    PRINTF("CHECKING");
                    break;
                case DHCP_STATE_BOUND:
                    PRINTF("BOUND");
                    break;
                case DHCP_STATE_BACKING_OFF:
                    PRINTF("BACKING_OFF");
                    break;
                default:
                    PRINTF("%u", dhcp_last_state);
                    assert(0);
                    break;
            }
            PRINTF("\r\n");

            if (dhcp_last_state == DHCP_STATE_BOUND)
            {
                PRINTF("\r\nprint_dhcp_state(): IPv4 Address     : %s\r\n", ipaddr_ntoa(&netif->ip_addr));
                PRINTF("print_dhcp_state():  IPv4 Subnet mask : %s\r\n", ipaddr_ntoa(&netif->netmask));
                PRINTF("print_dhcp_state():  IPv4 Gateway     : %s\r\n\r\n", ipaddr_ntoa(&netif->gw));
                sys_sem_signal(&semx);
            }
        }

        sys_msleep(20U);
    }

    vTaskDelete(NULL);
}

/*!
 * @brief Main function.
 */
int main(void)
{
    static struct netif netif;
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
    static mem_range_t non_dma_memory[] = NON_DMA_MEMORY_ARRAY;
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    ip4_addr_t netif_ipaddr, netif_netmask, netif_gw;
    ethernetif_config_t enet_config = {
        .phyHandle  = &phyHandle,
        .macAddress = configMAC_ADDR,
#if defined(FSL_FEATURE_SOC_LPC_ENET_COUNT) && (FSL_FEATURE_SOC_LPC_ENET_COUNT > 0)
        .non_dma_memory = non_dma_memory,
#endif /* FSL_FEATURE_SOC_LPC_ENET_COUNT */
    };

    SYSMPU_Type *base = SYSMPU;
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();
    /* Disable SYSMPU. */
    base->CESR &= ~SYSMPU_CESR_VLD_MASK;

    mdioHandle.resource.csrClock_Hz = EXAMPLE_CLOCK_FREQ;

    IP4_ADDR(&netif_ipaddr, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_netmask, 0U, 0U, 0U, 0U);
    IP4_ADDR(&netif_gw, 0U, 0U, 0U, 0U);

    tcpip_init(NULL, NULL);

    netifapi_netif_add(&netif, &netif_ipaddr, &netif_netmask, &netif_gw, &enet_config, EXAMPLE_NETIF_INIT_FN,
                       tcpip_input);
    netifapi_netif_set_default(&netif);
    netifapi_netif_set_up(&netif);

    netifapi_dhcp_start(&netif);

    PRINTF("\r\nmain():************************************************\r\n");
    PRINTF("main(): DHCP example\r\n");
    PRINTF("main():************************************************\r\n");

    if (sys_thread_new("print_dhcp", print_dhcp_state, &netif, PRINT_THREAD_STACKSIZE, 4) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed: print_dhcp", 0);
    }

    if (sys_thread_new("tcpexample_thread", tcpexample_thread, NULL, DEFAULT_THREAD_STACKSIZE, 3) == NULL)
    {
        LWIP_ASSERT("main(): Task creation failed: tcpexample_thread", 0);
    }

    vTaskStartScheduler();

    /* Will not get here unless a task calls vTaskEndScheduler ()*/
    return 0;
}

/*-----------------------------------------------------------------------------------*/
static void
tcpexample_thread(void *arg)
{
  struct netconn *conn = NULL;
  err_t err;
  struct netbuf *buf;
  void *data;
  u16_t len;
  const char* http_request =
		  "GET / HTTP/1.1\r\n"\
		  "Host: example.com\r\n"\
		  "\r\n";
  char* rcvd;
  LWIP_UNUSED_ARG(arg);

  // Wait for DHCP to complete
  PRINTF("tcpexample_thread(): Wait for DHCP provisioning to complete\n");
  sys_arch_sem_wait(&semx, 0);
  // get the example.org IP address using DNS
  PRINTF("tcpexample_thread(): DNS request\n");
  err = netconn_gethostbyname("example.org", &example_ip_addr);
  if (err != ERR_OK) {
	  PRINTF("tcpexample_thread(): ERROR! getting example.org IP address\r\n");
	  goto __tcp_task_end;
  }
  PRINTF("tcpexample_thread(): example.org IP address: %s\r\n\r\n", ipaddr_ntoa(&example_ip_addr));

  /* Create a new connection identifier. */
  /* Bind connection to well known port number 7. */
  conn = netconn_new(NETCONN_TCP);

  // Connect with example.org
  PRINTF("tcpexample_thread(): Connect with example.org\n");
  err = netconn_connect(conn, &example_ip_addr, 80);
  if (err != ERR_OK) {
	  PRINTF("tcpexample_thread(): ERROR! getting example.org IP address\r\n");
	  goto __tcp_task_end;
  }

  // Send and HTTP request
  PRINTF("tcpexample_thread(): Send and HTTP request\n");
  err = netconn_write(conn, http_request, strlen(http_request), NETCONN_COPY);
  if (err != ERR_OK) {
	PRINTF("tcpexample_thread(): netconn_write: error \"%s\"\n", lwip_strerr(err));
    goto __tcp_task_end;
  }

  PRINTF("tcpexample_thread(): Reading the response:\n");
  // Set the receive timeout to 1s
  netconn_set_recvtimeout(conn, 1000);
  do {
	  err = netconn_recv(conn, &buf);
	  if (err == ERR_OK) {
		  do {
		       netbuf_data(buf, &data, &len);
		       rcvd = (char*)data;
		       for(int i = 0; i<=len; i++) {
		    	   PRINTF("%c", *rcvd++);
		       }
		  } while (netbuf_next(buf) >= 0);
		  netbuf_delete(buf);
	  }
  }while(err == ERR_OK);

  if (err != ERR_TIMEOUT) {
	PRINTF("tcpexample_thread(): netconn_recv: error \"%s\"\n", lwip_strerr(err));
    goto __tcp_task_end;
  }

  PRINTF("\n");

  PRINTF("tcpexample_thread(): Closing the connection\n");
__tcp_task_end:
  if (conn == NULL) {
      netconn_close(conn);
      netconn_delete(conn);
  }

  vTaskDelete(NULL);
}

#endif
