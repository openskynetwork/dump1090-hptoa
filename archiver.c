// Part of dump1090, a Mode S message decoder for RTLSDR devices.
//
// archiver.c: archive raw messages to a fixed-size datastore
//  for later inspection
//
// Copyright (c) 2015 Oliver Jowett <oliver@mutability.co.uk>
//
// This file is free software: you may copy, redistribute and/or modify it
// under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 2 of the License, or (at your
// option) any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "dump1090.h"

#include <assert.h>
#include <zlib.h>
#include <sys/file.h>

#define min(x,y) ((x) < (y) ? (x) : (y))

#define DEBUG(format,...) fprintf(stderr, format "\n", ##__VA_ARGS__)

static int lockFd = -1;
static int dataFd = -1;

typedef uint32_t uint24_t;

/* one currently active aircraft, in-memory state */
typedef struct archive_inmem_s {
    uint24_t addr;                       // address of aircraft
    struct timespec last_message_time;   // when we last heard from it

#define INMEM_BUFFER_SIZE 131072
    uint8_t *data;                       // uncompressed message data, circular buffer
    uint8_t *start;                      // first used byte
    uint8_t *end;                        // next free byte
    uint8_t *limit;                      // points past end of data
    uint32_t messages_in_buffer;         // for debug more than anything

    /* nb: has to fit in 3 bits, so 8 is the best we can do */
#define MAX_RECENT_MESSAGES 8
    uint8_t recent_messages[MAX_RECENT_MESSAGES][MODES_LONG_MSG_BYTES];
    unsigned next_recent_message;

    struct archive_inmem_s *hashNext;       // hash list
    struct archive_inmem_s *lruNext;        // LRU list
    struct archive_inmem_s *lruPrev;        // LRU list
} archive_inmem;

/* hashtable of in-memory state, keyed by aircraft address */
#define INMEM_HASHTABLE_SIZE 2048
static archive_inmem *inmem_hashtable[INMEM_HASHTABLE_SIZE];
static int inmemSize;

/* LRU dummy node */
static archive_inmem lruList;

/* prototypes */

static int openFiles(void);
static void closeFiles(void);
static void shrinkInmem(int purge);

static void circularPut(archive_inmem *inmem, uint8_t thebyte);
static void circularDropOldest(archive_inmem *inmem);
static int circularMessageLengthFromByte(uint8_t b);

static archive_inmem *getInmemState(struct modesMessage *message);
static int flushInmemEntry(archive_inmem *inmem);

static int writeArchiveMessage(uint8_t *data, unsigned len);
static int writeArchiveData(uint8_t *data, unsigned len, int partial);
static int writeToCurrentBlock(void *data, uint32_t len);
static int zeroTrailingData();
static void nextBlock();
static int writeCurrentIndexEntry(uint32_t first_message_offset);

static int writeLock();
static int writeUnlock();

static uint32_t get_uint32(uint8_t *data) {
    return (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
}

static uint16_t get_uint16(uint8_t *data) {
    return (data[0] << 8) | data[1];
}

static void set_uint64(uint8_t *data, uint64_t val) {
    data[0] = (val >> 56) & 255;
    data[1] = (val >> 48) & 255;
    data[2] = (val >> 40) & 255;
    data[3] = (val >> 32) & 255;
    data[4] = (val >> 24) & 255;
    data[5] = (val >> 16) & 255;
    data[6] = (val >> 8) & 255;
    data[7] = (val) & 255;
}

static void set_uint32(uint8_t *data, uint32_t val) {
    data[0] = (val >> 24) & 255;
    data[1] = (val >> 16) & 255;
    data[2] = (val >> 8) & 255;
    data[3] = (val) & 255;
}

static void set_uint24(uint8_t *data, uint24_t val) {
    data[0] = (val >> 16) & 255;
    data[1] = (val >> 8) & 255;
    data[2] = (val) & 255;
}

static void set_uint16(uint8_t *data, uint16_t val) {
    data[0] = (val >> 8) & 255;
    data[1] = (val) & 255;
}

void archiverInit(void) {
    /* init LRU */
    lruList.lruNext = &lruList;
    lruList.lruPrev = &lruList;
    openFiles();
}

void archiverCleanup(void) {
    /* flush data, close down */
    shrinkInmem(1);
    closeFiles();
}

void archiverPeriodicWork(void) {
    /* look for old entries, flush them */
    shrinkInmem(0);
}

/* put a byte onto the circular inmem buffer */
static void circularPut(archive_inmem *inmem, uint8_t thebyte) {
    *inmem->end = thebyte;
    if (++inmem->end == inmem->limit) {
        /* wrap */
        inmem->end = inmem->data;
    }

    if (inmem->end == inmem->start) {
        circularDropOldest(inmem);
    }
}

/* given the first byte of a message, return the expected length of
 * that message (as stored in the inmem buffer)
 */
static int circularMessageLengthFromByte(uint8_t b) {
    if ((b >> 3) == 31)
        return 1; /* special marker for recent messages */
    else if (b & 0x80)
        return 14;
    else
        return 7;
}

/* drop the oldest message from an inmem buffer */
static void circularDropOldest(archive_inmem *inmem) {
    inmem->start += 4; // skip timestamp
    if (inmem->start >= inmem->limit) {
        /* wrap */
        inmem->start = inmem->data + (inmem->start - inmem->limit);
    }

    // look at the message type to work out the length
    int msglen = circularMessageLengthFromByte(*inmem->start);

    inmem->start += msglen;
    if (inmem->start >= inmem->limit) {
        /* wrap */
        inmem->start = inmem->data + (inmem->start - inmem->limit);
    }

    --inmem->messages_in_buffer;
}

/* Main call point: we received a message, do whatever is needed to archive it */
void archiverStoreMessage(struct modesMessage *message)
{
    int msglen, i;
    archive_inmem *inmem;

    if (dataFd < 0) /* archiver not active */
        return;

    inmem = getInmemState(message);
    inmem->messages_in_buffer++;

    /* store 32 bits LSB of timestamp only */
    circularPut(inmem, (message->timestampMsg >> 24) & 255);
    circularPut(inmem, (message->timestampMsg >> 16) & 255);
    circularPut(inmem, (message->timestampMsg >> 8) & 255);
    circularPut(inmem, (message->timestampMsg) & 255);

    msglen = circularMessageLengthFromByte(message->verbatim[0]);

    /* check for recent messages */
    for (i = 0; i < MAX_RECENT_MESSAGES; ++i) {
        if (!memcmp(message->verbatim, inmem->recent_messages[i], msglen)) {
            int recent_message = (inmem->next_recent_message - i + MAX_RECENT_MESSAGES) % MAX_RECENT_MESSAGES;
            /* store special marker for a recent message (DF "31") */
            circularPut(inmem, (31 << 3) | recent_message);
            return;
        }
    }

    /* store raw message */
    for (i = 0; i < msglen; ++i) {
        circularPut(inmem, message->verbatim[i]);
    }

    /* remember message in the recent-message history */
    memcpy(inmem->recent_messages[inmem->next_recent_message], message->verbatim, msglen);
    inmem->next_recent_message = (inmem->next_recent_message + 1) % MAX_RECENT_MESSAGES;
}

/* Jenkins one-at-a-time hash, unrolled for 3 bytes */
static uint32_t archiveHashFunction(uint32_t a)
{
    uint32_t hash = 0;

    hash += a & 0xff;
    hash += hash << 10;
    hash ^= hash >> 6;

    hash += (a >> 8) & 0xff;
    hash += (hash << 10);
    hash ^= (hash >> 6);

    hash += (a >> 16) & 0xff;
    hash += (hash << 10);
    hash ^= (hash >> 6);

    hash += (hash << 3);
    hash ^= (hash >> 11);
    hash += (hash << 15);

    return hash;
}

#define INMEM_MAX_AGE 300
#define INMEM_SOFT_MAX 300
#define INMEM_HARD_MAX 400

/* Find the inmem state for the aircraft for this message.
 * If there is no inmem state, create some.
 */
static archive_inmem *getInmemState(struct modesMessage *message)
{
    uint32_t h;
    archive_inmem *inmem;

    /* hashtable lookup */
    h = archiveHashFunction(message->addr) % INMEM_HASHTABLE_SIZE;
    for (inmem = inmem_hashtable[h]; inmem && inmem->addr != message->addr; inmem = inmem->hashNext)
        ;

    if (!inmem) {
        /* create new entry */
        inmem = malloc(sizeof(*inmem));
        inmem->addr = message->addr;

        inmem->data = malloc(INMEM_BUFFER_SIZE);
        inmem->start = inmem->end = inmem->data;
        inmem->limit = inmem->data + INMEM_BUFFER_SIZE;
        inmem->messages_in_buffer = 0;

        memset(inmem->recent_messages, 0, sizeof(inmem->recent_messages));
        inmem->next_recent_message = 0;

        /* link into hashtable */
        inmem->hashNext = inmem_hashtable[h];
        inmem_hashtable[h] = inmem;
        ++inmemSize;

        DEBUG("created new hashtable entry for %06x, %u entries total", message->addr, inmemSize);
    } else {
        /* extract from LRU, we will replace it in a moment */
        inmem->lruNext->lruPrev = inmem->lruPrev;
        inmem->lruPrev->lruNext = inmem->lruNext;
    }

    /* move to head of LRU, update last message time */
    lruList.lruNext->lruPrev = inmem;
    inmem->lruNext = lruList.lruNext;
    lruList.lruNext = inmem;
    inmem->lruPrev = &lruList;

    inmem->last_message_time = message->sysTimestampMsg;

    return inmem;
}

/* Try to make the in memory state smaller by flushing
 * data to disk
 */
static void shrinkInmem(int purge)
{
    time_t expiry = time(NULL) - INMEM_MAX_AGE;
    int gotLock = -1;

    while (inmemSize > 0) {
        archive_inmem *oldest = lruList.lruPrev;
        archive_inmem **p;

        int discard =
            (purge ||
             (gotLock && inmemSize > INMEM_SOFT_MAX) ||
             (gotLock && oldest->last_message_time.tv_sec < expiry) ||
             (inmemSize > INMEM_HARD_MAX));

        if (!discard)
            break;

        DEBUG("LRU: size %u, expiry %u, gotLock %d, evict %06x", inmemSize, (unsigned)expiry, gotLock, oldest->addr);

        if (gotLock < 0) {
            /* first time we want to write something */
            if (writeLock()) {
                gotLock = 1;
            } else {
                gotLock = 0;
                /* write lock failed, retry with the new rules */
                continue;
            }
        }

        if (gotLock) {
            /* flush block */
            if (!flushInmemEntry(oldest)) {
                /* problems */
                writeUnlock();
                gotLock = 0;
                continue; /* give the loop a chance to discard if we're still above HARD_MAX */
            }
        }

        /* remove from LRU */
        oldest->lruPrev->lruNext = oldest->lruNext;
        oldest->lruNext->lruPrev = oldest->lruPrev;

        /* remove from hashtable */
        for (p = &inmem_hashtable[archiveHashFunction(oldest->addr) % INMEM_HASHTABLE_SIZE]; *p && *p != oldest; p = &(*p)->hashNext)
            ;
        assert (*p == oldest);
        *p = oldest->hashNext;

        /* free it */
        free(oldest->data);
        free(oldest);
        --inmemSize;
    }

    if (gotLock > 0)
        writeUnlock();
}

/*
 * ON DISK MESSAGE FORMAT
 *
 * all datatypes bigendian
 *
 * these messages are then wrapped in the archival format which provides
 * a circular file format / length delimiting / CRC.
 *
 * Header:
 *
 *  24 bits   ICAO address
 *  64 bits   unix epoch (milliseconds) of the last (most recent) message
 *            in this block of messages
 *
 * Sequence of messages, each:
 *
 *  32 bits   12MHz or GPS timestamp, least significant bits
 *    ...     raw message data
 *
 * The raw message is the actual Mode S message (with one exception).
 * Length is determined by the first byte of the message data:
 *
 *  if the top bit is clear (DF0-15), the message is 7 bytes long
 *  if the top bit is set (DF16-31), the message is 14 bytes long
 *
 * EXCEPT:
 *
 *  if the top 5 bits are all set ("DF 31", which is an
 *  impossible/unused message type currently) then this message
 *  is an exact repeat of the N+1th previous message in the message
 *  sequence, where N is the low 3 bits of the first byte.
 *  The stored message consists of only this marker byte. This message
 *  does not itself count towards the "Nth previous" message.
 *
 * for example:
 *
 *  11 22 33 44 00 xx xx xx xx xx xx                         DF0 message at time 11223344
 *  11 22 33 55 80 xx xx xx xx xx xx xx xx xx xx xx xx xx    DF16 message at time 11223355
 *  11 22 33 66 F8                                           a repeat of the DF16 message at time 11223366
 *  11 22 33 77 F8                                           a repeat of the DF16 message at time 11223377
 *  11 22 33 88 F9                                           a repeat of the DF0 message at time 11223388
 */

/* write the inmem state for an aircraft to disk;
 * return 1 if all went well and the inmem state
 * can be freed.
 */
static int flushInmemEntry(archive_inmem *inmem)
{
    uint32_t datalen;
    uint8_t *message;
    uint32_t compressedlen;
    uint64_t millis;
    z_stream deflater;
    int result;

    DEBUG("flushing entry for %06x with %u messages to the disk archive", inmem->addr, inmem->messages_in_buffer);

    if (inmem->end == inmem->start)
        return 1; /* no data to flush */

    /* find the circular buffer length */
    if (inmem->end > inmem->start) {
        datalen = inmem->end - inmem->start;
    } else {
        datalen = (inmem->limit - inmem->start) + (inmem->end - inmem->data);
    }

    DEBUG("uncompressed length: %u", datalen);

    /* init compressor, find the upper size bound for the compressed output */
    deflater.zalloc = Z_NULL;
    deflater.zfree = Z_NULL;
    if (deflateInit(&deflater, 9) != Z_OK) {
        fprintf(stderr, "archiver: compression for %06x failed: %s\n",
                inmem->addr, deflater.msg);
        return 0;
    }

    compressedlen = deflateBound(&deflater, datalen);

    /* allocate space for our output message, build the header */
    message = alloca(11 + compressedlen);
    set_uint24(message, inmem->addr);
    millis = (uint64_t)(inmem->last_message_time.tv_sec * 1000ULL + inmem->last_message_time.tv_nsec / 1000000ULL);
    set_uint64(message+3, millis);

    /* compress the circular buffer */

    deflater.next_out = message + 11;
    deflater.avail_out = compressedlen;

    if (inmem->end > inmem->start) {
        /* simple case, contiguous block of data to compress */
        deflater.avail_in = datalen;
        deflater.next_in = inmem->start;
        result = deflate(&deflater, Z_FINISH);
    } else {
        /* the buffer has wrapped, so we need to compress in two steps */
        deflater.avail_in = inmem->limit - inmem->start;
        deflater.next_in = inmem->start;
        result = deflate(&deflater, Z_NO_FLUSH);
        if (result == Z_OK) {
            deflater.avail_in = inmem->end - inmem->data;
            deflater.next_in = inmem->data;
            result = deflate(&deflater, Z_FINISH);
        }
    }

    if (result != Z_STREAM_END) {
        /* something went wrong */
        fprintf(stderr, "archiver: compression for %06x failed: %s\n",
                inmem->addr, deflater.msg);
        deflateEnd(&deflater);
        return 0;
    }

    /* find actual output length, clean up */
    compressedlen = deflater.next_out - message;
    DEBUG("compressed length: %u", compressedlen);
    deflateEnd(&deflater);

    /* done, write it all */
    return writeArchiveMessage(message, compressedlen);
}

/*
 * ON DISK ARCHIVAL FORMAT
 *
 * There is an index file and a data file.
 * The data file has a header followed by a circularly-written sequence of messages.
 * The header is:
 *
 *   32 bits   numblocks  the total number of blocks in the archive file
 *   32 bits   blocksize  the block size of the archive file in bytes (currently, must be 4096)
 *
 * The main part of the file is a sequence of messages formatted as:
 *
 *   32 bits   length     the length of the following message, excluding the len/crc values
 *   32 bits   crc        the CRC32 of the following message
 *     <length> bytes     the message data itself (see above for format)
 *
 *   If end of the file is reached mid-message, continue reading from the start of the file.
 *   If a length of zero is seen, this is the final block of the sequence. though it is
 *   possible that you may _not_ see this at the end of sequence; you also need to check
 *   the index (below).
 *
 * The start of the message sequence is not necessarily immediately after the header
 * (and, indeed, the bytes following the header may not even by the start of a message!)
 * To correctly find the start/end of the circular file, the separate index file
 * provides information about each "block" of the datafile. The datafile is conceptually
 * subdivided into blocks of 4kB. Note that the messages above can straddle more than one
 * block. The index contains one entry per block, in order. Each index entry contains:
 *
 *   32 bits  sequence     an increasing sequence number indicating the order in which blocks were last
 *                         written
 *   16 bits  offset       first_header the offset within the block of the start of the first message in the block;
 *                         if no message starts in this block, contains FFFF.
 *
 * To find the start of the sequence, find the index entry with the lowest sequence number that
 * has an index offset that is not FFFF. This block/offset is where you should start from.
 *
 * To find the end of the sequence, find the index entry with the highest sequence number.
 * This is the last block that has valid data. Then walk forward from the index offset
 * (which points to the start of a message), reading messages until you see a message with
 * a length of zero or you find a message with a bad CRC or a message
 * with a length that would take you beyond the end of the block. Now you are pointing just
 * after the last valid message in the sequence.
 *
 * See recoverFromDisk() for an example implementation of finding the end of the sequence.
 */

#define BLOCK_SIZE 4096
#define FILE_HEADER_SIZE 12
#define MESSAGE_HEADER_SIZE 8
#define INDEX_ENTRY_SIZE 6

static uint32_t archive_file_num_blocks = 50000; /* about 200M */

static uint32_t current_block_id;
static uint32_t current_block_offset;
static uint32_t current_block_sequence;

/* Write an application message to the archive file;
 * this adds the CRC/length header and maintains the indexes etc.
 * returns 1 if all is OK
 */
static int writeArchiveMessage(uint8_t *data, unsigned len)
{
    uint8_t header[MESSAGE_HEADER_SIZE];
    uint32_t datacrc;

    datacrc = crc32(0L, data, len);
    DEBUG("writing %u bytes of archive message with crc %08x", len, datacrc);

    set_uint32(header, len);
    set_uint32(header+4, datacrc);

    if (!writeArchiveData(header, MESSAGE_HEADER_SIZE, 1))
        return 0;

    if (!writeArchiveData(data, len, 0))
        return 0;

    return 1;
}

/* write raw data to the archive file, wrapping at EOF and
 * initializing new blocks as needed.
 * returns 1 if all is OK
 */
static int writeArchiveData(uint8_t *data, unsigned len, int partial)
{
    static int index_write_pending = 0;
    unsigned int c;

    /* write the current block */
    c = min(len, BLOCK_SIZE - current_block_offset);
    writeToCurrentBlock(data, c);
    len -= c;
    data += c;

    if (current_block_offset < BLOCK_SIZE) {
        /* there is still space left in the current block */
        if (!partial && index_write_pending) {
            /* need to write an index entry, and this is a message boundary */
            if (!writeCurrentIndexEntry(current_block_offset))
                return 0;
            if (!zeroTrailingData())
                return 0;
            index_write_pending = 0;
        }
        return 1;
    }

    /* filled the current block */

    if (index_write_pending) {
        /* need to write an index entry, and this block has no message boundaries */
        writeCurrentIndexEntry(0xFFFF);
        index_write_pending = 0;
    }

    nextBlock();

    /* write some complete blocks */
    while (len >= BLOCK_SIZE) {
        if (!writeCurrentIndexEntry(0xFFFF))
            return 0;
        writeToCurrentBlock(data, BLOCK_SIZE);

        len -= BLOCK_SIZE;
        data += BLOCK_SIZE;

        nextBlock();
    }

    /* write the trailing partial block
     * nb: we do this even if len=0 deliberately (in that case, we do
     * want to write an index entry with offset 0 and zero the block,
     * because the next entry will start at 0)
     */

    if (partial) {
        /* we started a new block, but we don't know if it has message boundaries yet */
        if (!writeToCurrentBlock(data, len))
            return 0;
        index_write_pending = 1;
    } else {
        /* we started a new block, and we know where the first message boundary is */
        if (!writeCurrentIndexEntry(current_block_offset + len))
            return 0;
        if (!writeToCurrentBlock(data, len))
            return 0;
        if (!zeroTrailingData())
            return 0;
    }

    return 1;
}

/* advance to the next block */
static void nextBlock()
{
    current_block_id = (current_block_id + 1) % archive_file_num_blocks;
    current_block_offset = 0;
    ++current_block_sequence;
}

static int check_errors(int rc, const char *what)
{
    if (rc < 0) {
        fprintf(stderr, "archiver: %s failed: %s\n", what, strerror(errno));
        return 0;
    }

    return 1;
}

static int checked_read(int fd, uint8_t *data, size_t count, off_t offset, const char *what)
{
    while (count > 0) {
        ssize_t rc = pread(fd, data, count, offset);
        if (rc < 0) {
            fprintf(stderr, "archiver: %s failed: %s\n", what, strerror(errno));
            return 0;
        }

        if (rc == 0) {
            fprintf(stderr, "archiver: %s failed: short read\n", what);
            return 0;
        }

        count -= rc;
        data += rc;
        offset += rc;
    }

    return 1;
}

static int checked_write(int fd, uint8_t *data, size_t count, off_t offset, const char *what) {
    while (count > 0) {
        ssize_t rc = pwrite(fd, data, count, offset);
        if (rc < 0) {
            fprintf(stderr, "archiver: %s failed: %s\n", what, strerror(errno));
            return 0;
        }

        if (rc == 0) {
            fprintf(stderr, "archiver: %s failed: short write\n", what);
            return 0;
        }

        count -= rc;
        data += rc;
        offset += rc;
    }

    return 1;
}

#define INDEX_OFFSET(x) (FILE_HEADER_SIZE + (x) * INDEX_ENTRY_SIZE)
#define DATA_OFFSET(x) (INDEX_OFFSET(archive_file_num_blocks) + (x) * BLOCK_SIZE)

/* write an index entry with the given offset for the current block */
static int writeCurrentIndexEntry(uint32_t first_message_offset)
{
    uint8_t entry[INDEX_ENTRY_SIZE];

    assert (first_message_offset < BLOCK_SIZE || first_message_offset == 0xFFFF);

    set_uint32(entry, current_block_sequence);
    set_uint16(entry+4, (uint16_t)first_message_offset);

    DEBUG("writing index entry for block %u sequence %u offset %u", current_block_id, current_block_sequence, first_message_offset);

    if (!checked_write(dataFd, entry, INDEX_ENTRY_SIZE, INDEX_OFFSET(current_block_id), "index file write"))
        return 0;

    return 1;
}

/* write some data to the current block.
 * the data must entirely fit in the current block.
 * advances the current offset to reflect the data written.
 */
static int writeToCurrentBlock(void *data, uint32_t len)
{
    assert (len + current_block_offset <= BLOCK_SIZE);

    DEBUG("writing %u raw bytes at data block %u offset %u", len, current_block_id, current_block_offset);

    if (!len)
        return 1;

    if (!checked_write(dataFd, data, len, DATA_OFFSET(current_block_id) + current_block_offset, "data file write"))
        return 0;

    current_block_offset += len;

    return 1;
}

/* zero out the remaining space in the current block.
 * does not advance the current offset.
 */
static int zeroTrailingData()
{
    void *zeros;
    unsigned len = (BLOCK_SIZE - current_block_offset);

    if (!len)
        return 1;

    DEBUG("zeroing %u raw bytes at data block %u offset %u", len, current_block_id, current_block_offset);

    zeros = alloca(len);
    memset(zeros, 0, len);

    if (!checked_write(dataFd, zeros, len, DATA_OFFSET(current_block_id) + current_block_offset, "data file zeroing"))
        return 0;

    return 1;
}

#define INDEX_BATCH 1000
#define DISK_VERSION 0x00010000

static int validateHeader()
{
    uint8_t file_header[FILE_HEADER_SIZE];
    uint32_t stored_version;
    uint32_t stored_numblocks;
    uint32_t stored_blocksize;

    /* validate header */
    if (!checked_read(dataFd, file_header, FILE_HEADER_SIZE, 0, "data file header read"))
        return 0;

    stored_version = get_uint32(file_header);
    stored_numblocks = get_uint32(file_header+4);
    stored_blocksize = get_uint32(file_header+8);

    if (stored_version != DISK_VERSION || stored_numblocks != archive_file_num_blocks || stored_blocksize != BLOCK_SIZE) {
        fprintf(stderr, "archiver: data file header does not match current settings\n");
        return 0;
    }

    return 1;
}

/* read the index and find where we should start appending from */
static int recoverFromDisk()
{
    uint8_t entry[INDEX_ENTRY_SIZE * INDEX_BATCH];
    unsigned block_id;

    unsigned highest_sequence = 0;
    unsigned selected_block_id = 0;
    unsigned selected_block_offset = 0;

    if (!validateHeader())
        return 0;

    for (block_id = 0; block_id < archive_file_num_blocks; block_id += INDEX_BATCH) {
        unsigned i;
        unsigned len = min(INDEX_BATCH, archive_file_num_blocks - block_id);
        uint32_t sequence;
        uint16_t offset;

        if (!checked_read(dataFd, entry, INDEX_ENTRY_SIZE * len, INDEX_OFFSET(block_id), "index read"))
            return 0;

        for (i = 0; i < len; ++i) {
            sequence = get_uint32(INDEX_ENTRY_SIZE * i + entry);
            offset = get_uint16(INDEX_ENTRY_SIZE * i + entry + 4);

            if (sequence > highest_sequence && offset != 0xFFFF) {
                highest_sequence = sequence;
                selected_block_id = block_id + i;
                selected_block_offset = offset;
            }
        }
    }

    if (highest_sequence == 0) {
        DEBUG("recovery: index was empty");
        /* empty index */
        return 0;
    }

    DEBUG("recovery: highest sequence %u at block %u, index offset %u", highest_sequence, selected_block_id, selected_block_offset);

    /* look at the selected block and walk forwards looking for the end of valid data */
    while (1) {
        uint32_t len;
        uint32_t headercrc;
        uint32_t datacrc;
        uint8_t block[BLOCK_SIZE];
        uint8_t header[MESSAGE_HEADER_SIZE];

        if (selected_block_offset + 8 > BLOCK_SIZE) {
            DEBUG("not enough remaining space for another message at block %u offset %u length %u", selected_block_id, selected_block_offset, len);
            break;
        }

        if (!checked_read(dataFd, header, MESSAGE_HEADER_SIZE, DATA_OFFSET(selected_block_id) + selected_block_offset, "data read"))
            return 0;

        len = get_uint32(header);
        headercrc = get_uint32(header+4);
        if (len == 0) {
            DEBUG("found zeroed data at block %u offset %u", selected_block_id, selected_block_offset);
            break; /* hit zeroed data */
        }

        if (selected_block_offset + 8 + len >= BLOCK_SIZE) {
            DEBUG("found a message that crossed a block boundary at block %u offset %u length %u", selected_block_id, selected_block_offset, len);
            break; /* message extends beyond this block */
        }

        if (!checked_read(dataFd, block, len, DATA_OFFSET(selected_block_id) + selected_block_offset + 8, "data read"))
            return 0;

        datacrc = crc32(0L, block, len);
        if (datacrc != headercrc) {
            DEBUG("found a corrupted block at block %u offset %u length %u (expected %08x, got %08x)", selected_block_id, selected_block_offset, len,
                  headercrc, datacrc);
            break;
        } else {
            DEBUG("valid message at block %u offset %u length %u", selected_block_id, selected_block_offset, len);
        }

        selected_block_offset += len + 8;
    }

    /* all good! */
    fprintf(stderr, "archiver: recovered at block %u offset %u\n", selected_block_id, selected_block_offset);
    current_block_id = selected_block_id;
    current_block_offset = selected_block_offset;
    current_block_sequence = highest_sequence;
    return 1;
}

static int openFiles() {
    struct stat st;
    int recovery = 1;
    uint32_t expected_data_size = DATA_OFFSET(archive_file_num_blocks);

    /* open lockfile, get mega-writing lock */
    if (!check_errors(lockFd = open("/tmp/archive.lock", O_RDWR | O_CREAT, 0664), "opening lock file"))
        goto error;

    if (flock(lockFd, LOCK_EX|LOCK_NB) < 0) {
        if (errno == EWOULDBLOCK) {
            fprintf(stderr, "archiver: another process is writing to the archive file, archiving disabled\n");
        } else {
            fprintf(stderr, "archiver: locking archive file failed: %s\n", strerror(errno));
        }
        goto error;
    }

    /* open datafile */
    if (!check_errors(dataFd = open("/tmp/archive.data", O_RDWR | O_CREAT, 0664), "opening data file"))
        goto error;

    /* get little writing lock, wait for it */
    if (!check_errors(flock(dataFd, LOCK_EX), "locking data file"))
        goto error;

    /* check size */
    if (!check_errors(fstat(dataFd, &st), "checking data file size"))
        goto error;

    if (st.st_size != (off_t)expected_data_size) {
        fprintf(stderr, "archiver: adjusting data file size from %u to %u\n", (unsigned)st.st_size, (unsigned)expected_data_size);

        /* adjust size */
        if (!check_errors(ftruncate(dataFd, expected_data_size), "adjusting data file size"))
            goto error;

        recovery = 0;
    }

    if (recovery) {
        recovery = recoverFromDisk();
    }

    if (!recovery) {
        uint32_t i;
        uint8_t file_header[FILE_HEADER_SIZE];
        uint8_t index_entry[INDEX_ENTRY_SIZE * INDEX_BATCH];

        /* something went wrong during recovery (no data there, or the settings changed, etc)
         * so start from scratch
         */

        fprintf(stderr, "archiver: could not recover on-disk state, starting afresh\n");

        /* rewrite datafile header */

        set_uint32(file_header, DISK_VERSION);
        set_uint32(file_header+4, archive_file_num_blocks);
        set_uint32(file_header+8, BLOCK_SIZE);

        if (!checked_write(dataFd, file_header, FILE_HEADER_SIZE, 0, "data file header write"))
            goto error;

        /* clear index */

        memset(index_entry, 0, sizeof(index_entry));
        for (i = 0; i < archive_file_num_blocks; i += INDEX_BATCH) {
            unsigned len = min(INDEX_BATCH, archive_file_num_blocks - i);
            if (!checked_write(dataFd, index_entry, INDEX_ENTRY_SIZE * len, INDEX_OFFSET(i), "index file write"))
                goto error;
        }

        current_block_id = 0;
        current_block_offset = 0;
        current_block_sequence = 1;
    }

    /* set up the current block */
    if (!zeroTrailingData())
        goto error;

    if (!writeCurrentIndexEntry(current_block_offset))
        goto error;

    /* release the little writing lock, but keep the archive file ownership one */
    if (!check_errors(flock(dataFd, LOCK_UN), "unlocking data file"))
        goto error;

    /* done */
    return 1;

 error:
    closeFiles();
    return 0;
}

static void closeFiles() {
    if (lockFd >= 0) {
        close(lockFd);
        lockFd = -1;
    }

    if (dataFd >= 0) {
        close(dataFd);
        dataFd = -1;
    }
}

static int writeLock() {
    if (flock(dataFd, LOCK_EX|LOCK_NB) < 0) {
        if (errno != EWOULDBLOCK) {
            fprintf(stderr, "archiver: locking data file failed: %s\n", strerror(errno));
        }

        return 0;
    }

    return 1;
}

static int writeUnlock() {
    return check_errors(flock(dataFd, LOCK_UN), "unlocking data file");
}
