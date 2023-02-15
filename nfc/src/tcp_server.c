/* BSD Socket API Example

 This example code is in the Public Domain (or CC0 licensed, at your option.)

 Unless required by applicable law or agreed to in writing, this
 software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 CONDITIONS OF ANY KIND, either express or implied.
 */
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "json_wp.h"
#include "debug.h"

#define KEEPALIVE_IDLE              (1)
#define KEEPALIVE_INTERVAL          (1)
#define KEEPALIVE_COUNT             (1)
//#define KEEPALIVE_IDLE              (5)
//#define KEEPALIVE_INTERVAL          (5)
//#define KEEPALIVE_COUNT             (3)


static void do_retransmit(const int sock)
{
	int len;
	char rx_buffer[1024];

	do {
		len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
		if (len < 0) {
			lDebug(Error, "Error occurred during receiving: errno %d", errno);
		} else if (len == 0) {
			lDebug(Warn, "Connection closed");
		} else {
			rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string

			char *tx_buffer;

			int ack_len = json_wp(rx_buffer, &tx_buffer);

			//lDebug(InfoLocal, "To send %d bytes: %s", ack_len, tx_buffer);

			if (ack_len > 0) {
				// send() can return less bytes than supplied length.
				// Walk-around for robust implementation.
				int to_write = ack_len;

//				char ack_buff[5];
//				sprintf(ack_buff, "%04x", ack_len);
//				send(sock, ack_buff, 4, 0);

				while (to_write > 0) {
					int written = send(sock, tx_buffer + (ack_len - to_write),
							to_write, 0);
					if (written < 0) {
						lDebug(Error, "Error occurred during sending: errno %d",
								errno);
						goto free_buffer;
					}
					to_write -= written;
				}

free_buffer:
				if (tx_buffer) {
					vPortFree(tx_buffer);
					tx_buffer = NULL;
				}
			}
		}
	} while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    uint16_t port = (uintptr_t) pvParameters;

	char addr_str[128];
	int ip_protocol = 0;
	int keepAlive = 0;
	int keepIdle = KEEPALIVE_IDLE;
	int keepInterval = KEEPALIVE_INTERVAL;
	int keepCount = KEEPALIVE_COUNT;
	struct sockaddr_in dest_addr;

	struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in*) &dest_addr;
	dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
	dest_addr_ip4->sin_family = AF_INET;
	dest_addr_ip4->sin_port = htons(port);
	ip_protocol = IPPROTO_IP;

	int listen_sock = socket(AF_INET, SOCK_STREAM, ip_protocol);
	if (listen_sock < 0) {
		lDebug(Error, "Unable to create socket: errno %d", errno);
		vTaskDelete(NULL);
		return;
	}
	int opt = 1;
	setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(listen_sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

	lDebug(Info, "Socket created");

	int err = bind(listen_sock, (struct sockaddr* ) &dest_addr,
			sizeof(dest_addr));
	if (err != 0) {
		lDebug(Error, "Socket unable to bind: errno %d", errno);
		lDebug(Error, "IPPROTO: %d", AF_INET);
		goto CLEAN_UP;
	}
	lDebug(Info, "Socket bound, port %d", port);

	err = listen(listen_sock, 1);
	if (err != 0) {
		lDebug(Error, "Error occurred during listen: errno %d", errno);
		goto CLEAN_UP;
	}

	while (1) {

		lDebug(Info, "Socket listening");

		struct sockaddr source_addr; // Large enough for both IPv4 or IPv6
		socklen_t addr_len = sizeof(source_addr);
		int sock = accept(listen_sock, (struct sockaddr* ) &source_addr,
				&addr_len);
		if (sock < 0) {
			lDebug(Error, "Unable to accept connection: errno %d", errno);
			break;
		}

		// Set tcp keepalive option
		setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval,
				sizeof(int));
		setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
		// Convert ip address to string
		if (source_addr.sa_family == PF_INET) {
			inet_ntoa_r(((struct sockaddr_in* )&source_addr)->sin_addr,
					addr_str, sizeof(addr_str) - 1);
		}
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
		lDebug(Info, "Socket accepted ip address: %s", addr_str);

		do_retransmit(sock);

		shutdown(sock, 0);
		close(sock);
	}

CLEAN_UP:
	close(listen_sock);
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------------------------------*/
void stackIp_ThreadInit(uint16_t port)
{
	sys_thread_new("tcp_thread", tcp_server_task, (void*) (uintptr_t) port,
	//DEFAULT_THREAD_STACKSIZE,
	1024,
	configMAX_PRIORITIES - 2);
}
/*-----------------------------------------------------------------------------------*/
