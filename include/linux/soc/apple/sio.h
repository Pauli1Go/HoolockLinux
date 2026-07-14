/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_SOC_APPLE_SIO_H_
#define _LINUX_SOC_APPLE_SIO_H_

#include <linux/types.h>

struct apple_sio;
struct apple_sio_async;

typedef void (*apple_sio_async_callback_t)(void *data, int status,
					   u64 response);

void *apple_sio_desc0_alloc(struct apple_sio *sio, unsigned int slots,
			    u32 *first_slot);
void apple_sio_desc0_free(struct apple_sio *sio, u32 first_slot,
			  unsigned int slots);

struct apple_sio_async *
apple_sio_request_async(struct apple_sio *sio, u8 endpoint, u8 opcode,
			u8 parameter, u32 data, u8 expected_opcode,
			apple_sio_async_callback_t callback,
			void *callback_data);
struct apple_sio_async *
apple_sio_request_async_atomic(struct apple_sio *sio, u8 endpoint, u8 opcode,
			       u8 parameter, u32 data, u8 expected_opcode,
			       apple_sio_async_callback_t callback,
			       void *callback_data);
int apple_sio_async_cancel(struct apple_sio_async *request);
void apple_sio_async_put(struct apple_sio_async *request);

int apple_sio_request(struct apple_sio *sio, u8 endpoint, u8 opcode,
		      u8 parameter, u32 data, u8 expected_opcode,
		      unsigned long timeout, u64 *response);

#endif
