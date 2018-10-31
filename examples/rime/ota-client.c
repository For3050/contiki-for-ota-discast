/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         Reliable single-hop unicast example
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include <stdio.h>

#include "contiki.h"
#include "net/rime/rime.h"

#include "lib/list.h"
#include "lib/memb.h"
#include "process.h"

#include "dev/button-sensor.h"
#include "dev/leds.h"

#define MAX_RETRANSMISSIONS 4
#define NUM_HISTORY_ENTRIES 4

//static uint8_t frame_buf[40] = {0x0,};
static uint8_t ack_buf[5] = {0x0,};

static uint8_t server_addr[2] = {0x0,};

process_event_t frame_error_event;
static uint16_t frame_id_old, frame_id_new = 0x0;

typedef struct FRAME
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t total_frame;
	uint8_t length;
	uint8_t payload[32];
	uint16_t crc;
}frame_st, *frame_pst;

/*---------------------------------------------------------------------------*/
PROCESS(ota_client_process, "ota client process");
AUTOSTART_PROCESSES(&ota_client_process);
/*---------------------------------------------------------------------------*/
	static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	static uint8_t i = 0;
	static frame_pst frame_ptr = NULL;

	printf("broadcast message received from %d.%d: '%s'\n",
			from->u8[0], from->u8[1], (char *)packetbuf_dataptr());

	server_addr[0] = from->u8[0];
	server_addr[1] = from->u8[1];

	frame_ptr = packetbuf_dataptr();

	frame_id_new =frame_ptr->frame_id;

	if(frame_id_new > frame_id_old)
	{
		if((frame_id_new - frame_id_old) == 1)
		{
			printf("\ngood new frame!\n");

			frame_id_old = frame_id_new;

			while(frame_ptr->length--)
			{
				printf("%d, ", frame_ptr->payload[i++]);
			}
			printf("\n");
		}
		else
		{
			printf("\nerror frame!\n");
			process_post_synch(&ota_client_process, frame_error_event, NULL);

			printf("now frame id = %d\t, received frame id = %d\n", 
					frame_ptr->frame_id, 
					((frame_pst)packetbuf_dataptr())->frame_id);
		}
	}
	else 
	{
		printf("\nold frame,discast!\n");
		printf("now frame id = %d\t, received frame id = %d\n", 
				frame_ptr->frame_id, 
				((frame_pst)packetbuf_dataptr())->frame_id);
	}


}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
	static void
recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{

	printf("runicast message received from %d.%d, seqno %d\n",
			from->u8[0], from->u8[1], seqno);
}
static const struct runicast_callbacks runicast_callbacks = {recv_runicast,};
static struct runicast_conn runicast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_client_process, ev, data)
{
	PROCESS_EXITHANDLER(runicast_close(&runicast);)
	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_callbacks);
	broadcast_open(&broadcast, 129, &broadcast_call);


	/* Receiver node: do nothing */
	if(linkaddr_node_addr.u8[0] == 1 &&
			linkaddr_node_addr.u8[1] == 0) {
		PROCESS_WAIT_EVENT_UNTIL(0);
	}

	while(1) {

		PROCESS_WAIT_EVENT_UNTIL(ev==frame_error_event);

		if(!runicast_is_transmitting(&runicast)) {
			linkaddr_t recv;
			ack_buf[0] = 0x02; //ACK
			ack_buf[1] = (frame_id_old+1)>>8;
			ack_buf[2] = (uint8_t)(frame_id_old+1);
			ack_buf[3] = 0xfe;
			ack_buf[4] = 0xed;


			packetbuf_copyfrom(ack_buf, sizeof(ack_buf));
			recv.u8[0] = server_addr[0];
			recv.u8[1] = server_addr[1];

			printf("%u.%u: sending runicast to address %u.%u\n",
					linkaddr_node_addr.u8[0],
					linkaddr_node_addr.u8[1],
					recv.u8[0],
					recv.u8[1]);

			runicast_send(&runicast, &recv, MAX_RETRANSMISSIONS);
		}
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
