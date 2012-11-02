/* fifo.c - circular ring buffer FIFO routines */

/*
 * This file implements a queue (FIFO) using ring (circular) buffers.
 * Policies:
 * - No overflow protection is provided for enqueue (that is, enQ() to a
 *   full queue is permitted; oldest item is overwritten)
 * - Empty queue on dequeue is detected and returned to caller as an error;
 * - Queues require one extra cell in order to distinguish queue full
 *   (tail==head-1) from queue empty (tail==head).
 * - Queue size can be up to 256 bytes, or up to 128 words for word queues.
 *
 * The enQ() operation pre-increments,mod queueSize, the TAIL when storing.
 *   If, after the store, (TAIL==HEAD), the queue is full and HEAD is
 *   incremented mod queueSize.
 * The deQ() operation first checks for an empty queue (TAIL==HEAD); if non-empty,
 * deQ() then post-increments,mod queueSize, the HEAD when fetching.
 *
 * (c) 2011 Russ Magee from his dual rate uart implementation on attiny85
 * and bluetooth at 
 * http://dangerruss-things.blogspot.pt/2011/11/dual-rate-uart-implemented-in-software.html
 *
 * Changes:
 * 02/11/2012 - Rui Barreiros:
 * - removed the word queueing functions as they're not needed and we're trying to
 * get less space used.
 *
 */

#include <avr/io.h>
#include "fifo.h"

eFIFOStatus fifoStat;

void initQ(uint8_t size, rbq *pQ) {
//  IRQMASK_SAVE;
  pQ->left = size;    /* capacity of queue */
  pQ->size = size+1;  /* actual size of FIFO */
  pQ->tail = 0;
  pQ->head = 0;       /* tail == head means fifo is empty */
//  IRQMASK_RESTORE;
}

void enQ(rbq *pQ, uint8_t data) {
//  IRQMASK_SAVE;
  pQ->buf[pQ->tail] = data;
  pQ->tail = (pQ->tail+1) % pQ->size;
  if( pQ->tail/*+1? -rlm 2011-09-21*/ == pQ->head ) {
    pQ->head = (pQ->head+1) % pQ->size; /* full queue, head needs to move as well -- drop oldest item */
  }
  else {
    pQ->left--;
  }
//  IRQMASK_RESTORE;
}

uint8_t dQ(rbq *pQ, bool peek) {
  uint8_t ret;
//  IRQMASK_SAVE;

  if( pQ->tail == pQ->head ) {
    /* empty queue! */
    ret = 0;
    fifoStat = FIFO_EMPTY;
  }
  else {
    ret = pQ->buf[pQ->head];
    if( !peek ) {
      pQ->head = (pQ->head+1) % pQ->size;
      pQ->left++;
    }
    fifoStat = FIFO_OK;
  }
//  IRQMASK_RESTORE;
  return ret;
}
