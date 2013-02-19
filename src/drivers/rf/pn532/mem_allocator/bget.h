#ifndef _BGET_H
#define _BGET_H

#ifndef NDEBUG
#define NDEBUG /* disable assertions */
#endif

#include <assert.h>
#include <string.h>
#include "stdint.h"

#define MemSize     uint32_t			/* Type for size arguments to memxxx()
								functions such as memcmp(). */

typedef int32_t bufsize;

/* Queue links */

struct qlinks {
    struct bfhead *flink;	      /* Forward link */
    struct bfhead *blink;	      /* Backward link */
};

/* Header in allocated and free buffers */

struct bhead {
    bufsize prevfree;			/* Relative link back to previous
								free buffer in memory or 0 if
								previous buffer is allocated.	*/
    bufsize bsize;	/* Buffer size: positive if free,
					negative if allocated. */
};

/*  Header in directly allocated buffers (by acqfcn) */

struct bdhead {
    bufsize tsize;		      /* Total size, including overhead */
    struct bhead bh;		      /* Common header */
};

/* Header in free buffers */

struct bfhead {
    struct bhead bh;		      /* Common allocated/free header */
    struct qlinks ql;		      /* Links on free list */
};


void bpool(void *buf, bufsize length);
void *bget(bufsize requested_size);
void *bgetz(bufsize size);
void *bgetr(void *buf, bufsize size);
void brel(void *buf);
void bectl(int32_t (*compact)(bufsize sizereq, int32_t sequence),
				void *(*acquire)(bufsize size),
				void (*release)(void *buf), bufsize pool_incr);
void bstats(bufsize *curalloc, bufsize *totfree, bufsize *maxfree,
				int32_t *nget, int32_t *nrel);
void bstatse(bufsize *pool_incr, int32_t *npool, int32_t *npget,
				int32_t *nprel, int32_t *ndget, int32_t *ndrel);
int32_t	bpoolv(void* buf);

/* cast functions */
struct bhead* BH (void *p);
struct bdhead* BDH (void *p);
struct bfhead* BFH (void *p);

/* this function is used for MISRA compliancy, it avoids pointer arithmetic */
int8_t* getPointerOffset(void *p, int32_t offset);

#endif /* _BGET_H */
