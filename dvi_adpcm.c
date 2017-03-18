/* ADPCM codec test program, based on Intel/DVI as per text below.
 * Operates as a standard Unix filter, with various options to
 * select encode/decode/both and 8/16 bit audio I/O. Supports a
 * custom 2-bit ADPCM format for higher compression ratio (similar
 * to G726-16). See main() for command line options :)
 *
 * typical usage:
 *  cat test_s16_le.raw | dvi_adpcm e > test.adpcm # encode
 *  cat test.adpcm | dvi_adpcm d > test_s16_le.raw # decode
 *
 * Phlash, March 2017
 */

/***********************************************************
Copyright 1992 by Stichting Mathematisch Centrum, Amsterdam, The
Netherlands.

                        All Rights Reserved

Permission to use, copy, modify, and distribute this software and its 
documentation for any purpose and without fee is hereby granted, 
provided that the above copyright notice appear in all copies and that
both that copyright notice and this permission notice appear in 
supporting documentation, and that the names of Stichting Mathematisch
Centrum or CWI not be used in advertising or publicity pertaining to
distribution of the software without specific, written prior permission.

STICHTING MATHEMATISCH CENTRUM DISCLAIMS ALL WARRANTIES WITH REGARD TO
THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS, IN NO EVENT SHALL STICHTING MATHEMATISCH CENTRUM BE LIABLE
FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

******************************************************************/

static char rcsid[] = "$Id: dvi_adpcm.c,v 1.4 1995/06/15 16:58:32 hgs Exp $";

/*
** Intel/DVI ADPCM coder/decoder.
**
** The algorithm for this coder was taken from the IMA Compatability Project
** proceedings, Vol 2, Number 2; May 1992.
**
** Version 1.2, 18-Dec-92.
**
** Change log:
** - Fixed a stupid bug, where the delta was computed as
**   stepsize*code/4 in stead of stepsize*(code+0.5)/4.
** - There was an off-by-one error causing it to pick
**   an incorrect delta once in a blue moon.
** - The NODIVMUL define has been removed. Computations are now always done
**   using shifts, adds and subtracts. It turned out that, because the standard
**   is defined using shift/add/subtract, you needed bits of fixup code
**   (because the div/mul simulation using shift/add/sub made some rounding
**   errors that real div/mul don't make) and all together the resultant code
**   ran slower than just using the shifts all the time.
** - Changed some of the variable names to be more meaningful.
*/

#ifdef SMCA
#include "types.h"
#include "audio_descr.h"
#include "dvi_adpcm.h"
#include "pcmu_l.h"
#include <netinet/in.h>   /* ntohs() */
#include <stdlib.h>       /* malloc() */
#else
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdint.h>       /* int_t etc. */
typedef int8_t int8;
typedef int16_t int16;
typedef uint8_t u_int8;
typedef struct dvi_adpcm_state {
    int valpred;
    int index;
    int dvi2bit;
} dvi_adpcm_state_t;
typedef struct {
    int encoding;
} audio_descr_t;
#define AE_L16	1
#define AE_PCMU	2
#define pcmu_l16(x)	((int16)x)
#define l16_pcmu(x)	((u_int8)x)
#endif

#ifndef __STDC__
#define signed
#endif

/* Intel ADPCM step variation table */
static int indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
};

static int stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};
/* G.726-16 (2-bit ADPCM) step variation */
static int indexTable2[4] = {
    -1, 2,
    -1, 2,
};
static int stepsizeTable2[11] = {
    256*1, 256*2, 256*3, 256*5, 256*7, 256*11, 256*15, 256*23, 256*31, 256*63, 256*127
};


/*
* Initialize encoder state.
*/
void dvi_adpcm_init_state(struct dvi_adpcm_state *state)
{
  state->valpred = 0;
  state->index   = 0;
  state->dvi2bit = 0;
} /* dvi_adpcm_init_state */


#ifdef SMCA
void *dvi_adpcm_init(void *pt, double period)
{
  if (!pt) pt = (void *)malloc(sizeof(struct dvi_adpcm_state));
  if (pt) dvi_adpcm_init_state((struct dvi_adpcm_state *)pt);
  return pt;
} /* dvi_adpcm_init */
#endif


/*
* Insert state into output packet only if 'header_flag' is true.  Not
* inserting a header allows piecing together a packet from several
* audio chunks.  If we prefix a header, we still encode the remaining
* samples.  The DVI standard says to skip the first sample, since it
* is already known to the receiver from the header.  'True' DVI blocks
* thus always contain an odd number of samples (see the definition of
* wSamplesPerBlock and the encoding routine in the DVI ADPCM Wave
* Type, e.g., found in the Microsoft Development Library.)
*/
int dvi_adpcm_encode(void *in_buf, int in_size, audio_descr_t *header,
  void *out_buf, int *out_size, void *state_, int header_flag)
{
  signed char *out_sbuf = out_buf;
  int val;              /* Current input sample value */
  int sign;             /* Current adpcm sign bit */
  int delta;            /* Current adpcm output value */
  int diff;             /* Difference between val and valpred */
  int step;             /* Stepsize */
  int valpred;          /* Predicted output value */
  int vpdiff;           /* Current change to valpred */
  int index;            /* Current step change index */
  int outputbuffer = 0; /* place to keep previous 4-bit value */
  int bufferstep;       /* toggle between outputbuffer/output */
  int16 *s;             /* output buffer for linear encoding */
  u_int8 *c;            /* output buffer for mu-law encoding */
  struct dvi_adpcm_state *state = (struct dvi_adpcm_state *)state_; 

  if (header->encoding == AE_L16) {
    in_size /= 2;
    s = (int16 *)in_buf;
    c = 0;
  } else if (header->encoding == AE_PCMU) {
    s = 0;
    c = (u_int8 *)in_buf;
  }
  else return -1;

  /* insert state into output buffer */
  *out_size = (state->dvi2bit) ? in_size / 2 : in_size / 4;
  valpred = (header->encoding == AE_PCMU) ? pcmu_l16(*c++) : *s++;
#ifdef SMCA
  if (header_flag) {
    ((struct dvi_adpcm_state *)out_buf)->valpred = htons(state->valpred);
    ((struct dvi_adpcm_state *)out_buf)->index = state->index;
    *out_size += sizeof(struct dvi_adpcm_state);
    out_sbuf  += sizeof(struct dvi_adpcm_state);
  }
#endif

  index = state->index;
  step  = state->dvi2bit ? stepsizeTable[index] : stepsizeTable2[index];
  bufferstep = 1;  /* in/out: encoder state */

  for ( ; in_size > 0; in_size--) {
    val = (header->encoding == AE_PCMU) ? pcmu_l16(*c++) : *s++;

   /* DVI (4-bit) or G.726-16 (2-bit) ADPCM? */
   if (state->dvi2bit) {
    /* DVI */
    /* Step 1 - compute difference with previous value */
    diff = val - valpred;
    sign = (diff < 0) ? 8 : 0;
    if ( sign ) diff = (-diff);

    /* Step 2 - Divide and clamp */
    /* Note:
    ** This code *approximately* computes:
    **    delta = diff*4/step;
    **    vpdiff = (delta+0.5)*step/4;
    ** but in shift step bits are dropped. The net result of this is
    ** that even if you have fast mul/div hardware you cannot put it to
    ** good use since the fixup would be too expensive.
    */
    delta = 0;
    vpdiff = (step >> 3);
  
    if ( diff >= step ) {
      delta = 4;
      diff -= step;
      vpdiff += step;
    }
    step >>= 1;
    if ( diff >= step  ) {
      delta |= 2;
      diff -= step;
      vpdiff += step;
    }
    step >>= 1;
    if ( diff >= step ) {
      delta |= 1;
      vpdiff += step;
    }

    /* Step 3 - Update previous value */
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;

    /* Step 4 - Clamp previous value to 16 bits */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;

    /* Step 5 - Assemble value, update index and step values */
    delta |= sign;
  
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;
    step = stepsizeTable[index];

    /* Step 6 - Output value */
    if ( bufferstep ) {
      outputbuffer = (delta << 4) & 0xf0;
    } else {
      *out_sbuf++ = (delta & 0x0f) | outputbuffer;
    }
    bufferstep = !bufferstep;
   } else {
    /* G726-16 */
    /* Step 1: compute difference from previous sample, record sign */
    diff = val - valpred;
    sign = (diff < 0) ? 2 : 0;
    if ( sign ) diff = (-diff);

    /* Step 2: compare difference to step size, choose magnitude bit */
    delta = 0;
    vpdiff = step >> 1;
    if (diff > step) {
      delta = 1;
      vpdiff += step;
    }

    /* Step 3: update previous sample value */
    if (sign)
      valpred -= vpdiff;
    else
      valpred += vpdiff;

    /* Step 4 - Clamp previous value to 16 bits */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;

    /* Step 5 - Assemble value, update index and step values */
    delta |= sign;
    index += indexTable2[delta];
    if ( index < 0 ) index = 0;
    if ( index > 10 ) index = 10;
    step = stepsizeTable2[index];

    /* Step 6 - Output value */
    switch ( bufferstep ) {
    case 1:
      outputbuffer = (delta << 6) & 0xc0;
      break;
    case 2:
      outputbuffer |= (delta << 4) & 0x30;
      break;
    case 3:
      outputbuffer |= (delta << 2) & 0x0c;
      break;
    case 0:
      *out_sbuf++ = (delta & 0x03) | outputbuffer;
      break;
    }
    bufferstep = (bufferstep+1)%4;
   }
  }

  /* Output last step, if needed */
  if ( state->dvi2bit && !bufferstep ) *out_sbuf++ = outputbuffer;
    
  state->valpred   = valpred;
  state->index     = index;
  return 0;
} /* dvi_adpcm_encode */

/*
* Translate from DVI/ADPCM to local audio format described by 'header'.
*/
int dvi_adpcm_decode(void *in_buf, int in_size, audio_descr_t *header,
  void *out_buf, int *out_size, void *state_)
{
  signed char *in_sbuf = in_buf;
  int sign;            /* Current adpcm sign bit */
  int delta;           /* Current adpcm output value */
  int step;            /* Stepsize */
  int valpred;         /* Predicted value */
  int vpdiff;          /* Current change to valpred */
  int index;           /* Current step change index */
  int inputbuffer = 0; /* place to keep next 4-bit value */
  int bufferstep;      /* toggle between inputbuffer/input */
  int16 *s;            /* output buffer for linear encoding */
  u_int8 *c;           /* output buffer for mu-law encoding */

#ifdef SMCA
  /* State to decode is kept in the packet */
  valpred = (short)ntohs(((struct dvi_adpcm_state *)in_buf)->valpred);
  index   = ((struct dvi_adpcm_state *)in_buf)->index;
  in_sbuf += sizeof(struct dvi_adpcm_state);
  in_size -= sizeof(struct dvi_adpcm_state);
#else
  struct dvi_adpcm_state *state = (struct dvi_adpcm_state *)state_;
  valpred = state->valpred;
  index = state->index;
#endif

  in_size *= (state->dvi2bit ? 2: 4);  /* convert to sample count */
  if (header->encoding == AE_PCMU) {
    *out_size = in_size;
    c = (u_int8 *)out_buf;
    s = 0;
  } else if (header->encoding == AE_L16) {
    *out_size = in_size * 2;
    s = (int16 *)out_buf;
    c = 0;
  }
  else return -1;

  step = (state->dvi2bit) ? stepsizeTable[index]: stepsizeTable2[index];

  bufferstep = 0;
    
  for ( ; in_size > 0 ; in_size--) {
  
   if (state->dvi2bit) {
    /* DVI */
    /* Step 1 - get the delta value */
    if ( bufferstep ) {
      delta = inputbuffer & 0xf;
    } else {
      inputbuffer = *in_sbuf++;
      delta = (inputbuffer >> 4) & 0xf;
    }
    bufferstep = !bufferstep;

    /* Step 2 - Find new index value (for later) */
    index += indexTable[delta];
    if ( index < 0 ) index = 0;
    if ( index > 88 ) index = 88;

    /* Step 3 - Separate sign and magnitude */
    sign = delta & 8;
    delta = delta & 7;

    /* Step 4 - Compute difference and new predicted value */
    /*
    ** Computes 'vpdiff = (delta+0.5)*step/4', but see comment
    ** in adpcm_coder.
    */
    vpdiff = step >> 3;
    if ( delta & 4 ) vpdiff += step;
    if ( delta & 2 ) vpdiff += step>>1;
    if ( delta & 1 ) vpdiff += step>>2;

    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;

    /* Step 5 - clamp output value */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;

    /* Step 6 - Update step value */
    step = stepsizeTable[index];

    /* Step 7 - Output value */
    if (header->encoding == AE_PCMU) 
      *c++ = l16_pcmu(valpred);
    else
      *s++ = valpred;
   } else {
    /* G726-16 */
    /* Step 1: get delta value */
    switch (bufferstep) {
    case 0:
      inputbuffer = *in_sbuf++;
      delta = (inputbuffer >> 6) & 0x3;
      break;
    case 1:
      delta = (inputbuffer >> 4) & 0x3;
      break;
    case 2:
      delta = (inputbuffer >> 2) & 0x3;
      break;
    case 3:
      delta = (inputbuffer) & 0x3;
      break;
    }
    bufferstep = (bufferstep+1)%4;

    /* Step 2: get new index value for later */
    index += indexTable2[delta];
    if (index < 0) index = 0;
    if (index > 10) index = 10;

    /* Step 3 - Separate sign and magnitude */
    sign = delta & 2;
    delta = delta & 1;

    /* Step 4 - Compute difference and new predicted value */
    vpdiff = step >> 1;
    if (delta & 1)
      vpdiff += step;
    if ( sign )
      valpred -= vpdiff;
    else
      valpred += vpdiff;

    /* Step 5 - clamp output value */
    if ( valpred > 32767 )
      valpred = 32767;
    else if ( valpred < -32768 )
      valpred = -32768;

    /* Step 6 - Update step value */
    step = stepsizeTable2[index];

    /* Step 7 - Output value */
    if (header->encoding == AE_PCMU) 
      *c++ = l16_pcmu(valpred);
    else
      *s++ = valpred;
   }
  }
#ifndef SMCA
  state->valpred = valpred;
  state->index = index;
#endif
  return 0;
} /* dvi_adpcm_decode */


/* Debug */
static int _dbg;
void debug(char *fmt, ...) {
  va_list ap;
  if (_dbg) {
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
  }
}

/* Pump bytes from stdin, through the codec and back to stdout */
#define PCM_BUF	1024
int main(int argc, char **argv) {
  struct dvi_adpcm_state encstate;
  struct dvi_adpcm_state decstate;
  audio_descr_t hdr;
  int8 *s8;
  int16 *pcm;
  u_int8 *adp;
  int pl, n, mode = 0;

  dvi_adpcm_init_state(&encstate);
  dvi_adpcm_init_state(&decstate);
  pl = PCM_BUF/4;
  s8 = NULL;
  for (n=1; n<argc; n++) {
    if (argv[n][0]=='4') {         // 4-bit ADPCM
      encstate.dvi2bit = 1;
      decstate.dvi2bit = 1;
      pl = PCM_BUF/2;
    } else if (argv[n][0]=='8') {  // 8-bit PCM (S8 format)
      s8 = malloc(PCM_BUF);
    } else if (argv[n][0]=='e') {  // Only encode to stdout
      mode = 1;
    } else if (argv[n][0]=='d') {  // Only decode from stdin
      mode = 2;
    } else if (argv[n][0]=='v') {  // Verbose
      _dbg = 1;
    }
  }
  hdr.encoding = AE_L16;
  pcm = malloc(PCM_BUF*sizeof(int16));
  adp = malloc(pl);

  // clear PCM buffer
  for (n=0; n<PCM_BUF; n++) pcm[n]=0;

  while ((n=fread((mode>1?adp:s8?s8:pcm), ((s8||mode>1)?sizeof(int8):sizeof(int16)), (mode>1)?pl:PCM_BUF, stdin))>0) {
    int l;
    debug("read: %d\n", n);
    if (mode<2) {         // We are encoding something..
      if (s8) {           // 8 bit input, map to 16 bit PCM
        for (l=0; l<n; l++)
          pcm[l] = ((int16)s8[l])<<8;
        while(l<PCM_BUF)
          pcm[l++] = 0;
      }
      if (dvi_adpcm_encode(pcm, n*sizeof(int16), &hdr, adp, &l, &encstate, 0)!=0) {
        fprintf(stderr, "encode error\n");
        break;
      }
      debug("encode: %d\n", l);
      if (mode)           // We are only encoding, write ADPCM out
        fwrite(adp, sizeof(uint8_t), l, stdout);
    } else {
      l = n;              // ADPCM buffer length (as read in)
    }
    for (n=0; n<PCM_BUF; n++) pcm[n]=0;

    if (mode!=1) {        // We are decoding something..
      if (dvi_adpcm_decode(adp, l, &hdr, pcm, &n, &decstate)!=0) {
        fprintf(stderr, "decode error\n");
        break;
      }
      n /= sizeof(int16);
      debug("decode: %d\n", n);
      if (s8) {           // map 16 bit PCM back to 8 bit
        for (l=0; l<n; l++)
          s8[l] = (int8)(pcm[l]>>8);
      }
      fwrite((s8?s8:pcm), (s8?sizeof(int8):sizeof(int16)), n, stdout);
    }
  }
  debug("read loop complete\n");
  return 0;
}
