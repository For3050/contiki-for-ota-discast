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
 *         Testing the broadcast layer in Rime
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "net/rime/rime.h"
#include "random.h"

#include "dev/button-sensor.h"

#include "dev/leds.h"

#include <stdio.h>

static uint8_t frame_buf[40] = {0x0,};

static uint16_t frame_id = 0x1;
static uint16_t total_frame = 0x20;

typedef struct FRAME
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t total_frame;
	uint8_t length;
	uint8_t payload[32];
	uint16_t crc;
}frame_st, *frame_pst;

typedef struct ACK
{
	uint8_t func;
	uint16_t frame_id;
	uint16_t crc;
}ack_st, *ack_pst;

/*---------------------------------------------------------------------------*/
PROCESS(ota_server_process, "ota server process");
PROCESS(ota_listening_process, "ota listening process");
AUTOSTART_PROCESSES(&ota_server_process, &ota_listening_process);
/*---------------------------------------------------------------------------*/
	static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	printf("broadcast message received from %d.%d: '%s'\n",
			from->u8[0], from->u8[1], (char *)packetbuf_dataptr());
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
	static void
recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno)
{
	static ack_pst ack_ptr = NULL;

	printf("runicast message received from %d.%d, seqno %d\n",
			from->u8[0], from->u8[1], seqno);

	ack_ptr = (ack_pst)packetbuf_dataptr();

	frame_id = ack_ptr->frame_id;

	printf("ota frame error,need frame again. frame_id = %d\n", ack_ptr->frame_id);

}
static const struct runicast_callbacks runicast_callbacks = {recv_runicast,};
static struct runicast_conn runicast;
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_server_process, ev, data)
{
	static struct etimer et;
	static uint8_t i = 0;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

		PROCESS_BEGIN();

	broadcast_open(&broadcast, 129, &broadcast_call);


	frame_id = 1;

	frame_buf[0] = 1; //ota func

	frame_buf[3] = total_frame>>8;
	frame_buf[4] = (uint8_t)total_frame;

	frame_buf[5] = 0x20; //length

	while(i<frame_buf[5])
	{
		frame_buf[i+6] = i;
		i++;
	}

	frame_buf[38] = 0xfe; //crc
	frame_buf[39] = 0xed;


	while(1) {

		etimer_set(&et, CLOCK_SECOND * 1);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		frame_buf[1] = frame_id>>8;
		frame_buf[2] = (uint8_t)frame_id;
		frame_id++;

		packetbuf_copyfrom(frame_buf, sizeof(frame_buf));
		broadcast_send(&broadcast);
		printf("broadcast message sent\n");
	}

	PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(ota_listening_process, ev, data)
{

	PROCESS_EXITHANDLER(runicast_close(&runicast);)

		PROCESS_BEGIN();

	runicast_open(&runicast, 144, &runicast_callbacks);

	while(1) {

		//etimer_set(&et, CLOCK_SECOND * 4 + random_rand() % (CLOCK_SECOND * 4));
		// etimer_set(&et, CLOCK_SECOND * 1);

		PROCESS_WAIT_EVENT_UNTIL(1);

	}

	PROCESS_END();
}
