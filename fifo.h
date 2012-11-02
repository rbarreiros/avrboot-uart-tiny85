/* fifo.h */

#if !defined(__FIFO_H__)
#define __FIFO_H__

#include "common.h"

#define qEmpty(q) ((q)->head == (q)->tail)

typedef enum _fifostatus {
  FIFO_OK = 0,
  FIFO_EMPTY = 1
} eFIFOStatus;
extern eFIFOStatus fifoStat;

/* struct rbq offsets */
typedef struct _rbq {
  uint8_t left;   /**< # of bytes free in queue */
  uint8_t size;   /**< buffer size (bytes) of queue (max of 255) */
  uint8_t tail;   /**< tail index */
  uint8_t head;   /**< head index */
  uint8_t buf[];  /**< GCC: flexible array buffer */
} rbq;
/* #define rbq_hdr_size offsetof(rbq, _rbq_buf); */

/** Initialize (clear) a queue
 *
 * @param[in] size - queue size in bytes
 * @param[in] pQ - pointer to queue
 *
 * NOTE *** THIS ROUTINE EXPECTS that the queue at _rbq_buf is one MORE byte than
 *          the size passed!! The standard circular FIFO algorithm requires
 *          one extra byte in order to distinguish an empty queue from a full one.
 *          emptyQ: (tail==head); fullQ: (modinc(tail+1)==head)
 */
void initQ(uint8_t size, rbq *pQ);

/** Atomically add byte to a queue
 *
 * @param[in] data - byte to add
 * @param[in] pQ - pointer to queue
 *
 * NOTE that adding to a full queue drops the oldest item.
 */
void enQ(rbq *pQ, uint8_t data);

/** Atomically dequeue byte from head of queue
 *
 * @param[in] pQ - pointer to queue
 *
 * @return uint8 - byte at head of queue, or SREG[C]=1 if no data available
 */
uint8_t dQ(rbq *pQ, bool peek);

#define peekQ(pQ) dQ((pQ), TRUE)
#define deQ(pQ) dQ((pQ), FALSE)

/* Queue state datatype - used to take a snapshot of a queue
 * that can be restored afterwards. Used to 'scan' the current
 * contents of a queue while it is frozen, for things like doing
 * a moving average over a series of values for an FIR filter.
 */
typedef struct _rbqstate {
  uint8_t left;
  uint8_t head;
  uint8_t tail;
} rbq_state;

#define saveQstate(q,qs) \
  (qs)->left = (q)->left; \
  (qs)->head = (q)->head; \
  (qs)->tail = (q)->tail

#define restoreQstate(q,qs) \
  (q)->tail = (qs)->tail; \
  (q)->head = (qs)->head; \
  (q)->left = (qs)->left

#endif /* __FIFO_H__ */
