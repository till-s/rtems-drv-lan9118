/* Simulate BPM/AFE response with a crude 2nd order iir filter */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* coldfire uc5282 has no FPU */
typedef int FiltNumber;

/* coefficients are shifted 16 bits left; signals
 * are 16 bit.
 */
#define FNUM(x)  ((FiltNumber)((float)(1<<16)*(x)))

/* renormalize after multiplication by coefficients */
#define FNORM(x) ((x)>>16)

/* Crude algorithm from random(3). This is uniformely distributed noise
 * but becomes approximately gaussian after filtering...
 */ 
#define NOISESTEP(pn) do { *(pn) = *(pn) * 1103515245 + 12345; } while (0)

/* random(3) does ((*pn) >> 16) & 0x7fff after a NOISESTEP to generate
 * a number between 0..MAXRAND
 */

/* How many bits of noise (in-band) to use */
#define IBNOISEBITS 6

/* State of the IIR filter */
struct Iir2Stat_ {
	FiltNumber y[2];
	FiltNumber x[2];
	int        i;
};

/* One step of bandpass filtering.
 * The response function is decomposed in partial fractions
 * and specialized to the case where the odd parts are of
 * opposite sign:
 */

#define IIRBP(y1a,y2a,y1b,y2b,x0,x1,x2,cn0,cn1,cn2,cd1,cd2) \
	do {	\
		FiltNumber tmp1 = (cn0)*(x0)+(cn2)*(x2); \
		FiltNumber tmp2 = (cn1)*(x1);            \
		y2a = FNORM(tmp1 + tmp2 - (cd1)*(y1a) - (cd2)*(y2a)); \
		y2b = FNORM(tmp1 - tmp2 + (cd1)*(y1b) - (cd2)*(y2b)); \
		x2 = x0; \
	} while (0)

/* Do two steps of the above filtering (wraps state around once) */
#define IIRBP2(ya,yb,x,x0,x1,cn0,cn1,cn2,cd1,cd2) \
	do {	\
		IIRBP(ya[1],ya[0],yb[1],yb[0],x0,x[1],x[0],cn0,cn1,cn2,cd1,cd2);	\
		IIRBP(ya[0],ya[1],yb[0],yb[1],x1,x[0],x[1],cn0,cn1,cn2,cd1,cd2);	\
	} while (0)

/* Coefficients for a 2nd order modified chebyshev bandpass fc = .25*fs, BW = 4/100 * fs, ripple .05
 *
 *                        2    
 *       0.0200136 ( 1 - z )    
 *       ----------------------------   
 *                              2   4  
 *       0.6908829 + 1.6066152 z + z   
 *
 * Partial fraction decomposition yields the coefficients below...
 * 
 */                                                                      
#define CN0 FNUM(0.)
#define CN1 FNUM(0.0933508)
#define CN2 FNUM(0.0120391)

#define CD1 FNUM(0.2361611)
#define CD2 FNUM(0.8311936)

unsigned Read_timer();

static inline int16_t
bswap(int16_t x)
{
	return (x<<8) | (((uint16_t)x)>>8);
}

unsigned
iir2_bpmsim(int16_t *pf, int nloops, int ini, unsigned long *pn, int swp, int stride)
{
unsigned then = Read_timer();
FiltNumber ysa[2] = { FNUM(0.), FNUM(0.) }, xsa[2] = { FNUM(0.), FNUM(0.)};
FiltNumber ysb[2] = { FNUM(0.), FNUM(0.) };
signed char n;
int16_t     *pf1 = pf+stride;
	NOISESTEP(pn);
	/* use top 4 bits of noise */
	ini += ((int)*pn)>>(32-IBNOISEBITS);
	n    = (signed char)(*pn)>>24;
	/* This is 'in-band' noise. We should also add wide-band noise
	 * at the output but we probably wouldn't have time to make it gaussian
	 */
	IIRBP2(ysa,ysb,xsa,(FiltNumber)ini,n>>(8-IBNOISEBITS),CN0,CN1,CN2,CD1,CD2);
	*pf = ysa[0] + ysb[0]; *pf1 = ysa[1] + ysb[1];
	if ( swp ) {
		*pf = bswap(ysa[0] + ysb[0]); *pf1 = bswap(ysa[1] + ysb[1]);
	} else {
		*pf = (ysa[0] + ysb[0]);      *pf1 = (ysa[1] + ysb[1]);
	}
	pf  = pf1 + stride;
	pf1 = pf  + stride;
	nloops-=2;
	if (swp) {
		while ( nloops > 0 ) {
			NOISESTEP(pn);
			ini = ((int)*pn)>>(32-IBNOISEBITS);
			n   = (signed char)(*pn)>>24;
			IIRBP2(ysa,ysb,xsa,(FiltNumber)ini,n>>(8-IBNOISEBITS),CN0,CN1,CN2,CD1,CD2);
			*pf  = bswap(ysa[0] + ysb[0]);
			*pf1 = bswap(ysa[1] + ysb[1]);
			pf  = pf1 + stride;
			pf1 = pf  + stride;
			nloops-=2;
		}
	} else {
		while ( nloops > 0 ) {
			NOISESTEP(pn);
			ini = ((int)*pn)>>(32-IBNOISEBITS);
			n   = (signed char)(*pn)>>24;
			IIRBP2(ysa,ysb,xsa,(FiltNumber)ini,n>>(8-IBNOISEBITS),CN0,CN1,CN2,CD1,CD2);
			*pf  = (ysa[0] + ysb[0]);
			*pf1 = (ysa[1] + ysb[1]);
			pf  = pf1 + stride;
			pf1 = pf  + stride;
			nloops-=2;
		}
	}
	return Read_timer() - then;
}

#ifdef MAIN

unsigned Read_timer() { return 0; }

int
main(int argc, char **argv)
{
int              i;
FiltNumber buf[128];
unsigned long nois = 1;

	if ( argc < 2 || 1!=sscanf(argv[1],"%i",&i) )
		i = 10000;
	iir2_bpmsim(buf, sizeof(buf)/sizeof(buf[0]), i, &nois);

	for ( i=0; i<sizeof(buf)/sizeof(buf[0]); i++)
		printf("%f\n",(float)buf[i]);
}
#endif
