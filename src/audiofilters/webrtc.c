
#if defined(HAVE_CONFIG_H)
#include "mediastreamer-config.h"
#endif

#include <math.h>
#include <time.h>

#include "mediastreamer2/msfilter.h"
#include "mediastreamer2/msticker.h"
#include "mediastreamer2/flowcontrol.h"
#include "ortp/b64.h"

#include <webrtc/modules/audio_processing/aecm/include/echo_control_mobile.h>
#include <webrtc/modules/audio_processing/aecm/aecm_core.h>
#include <webrtc/modules/audio_processing/ns/include/noise_suppression_x.h>

#define NsHandle NsxHandle
#define WebRtcNs_Create WebRtcNsx_Create
#define WebRtcNs_Init WebRtcNsx_Init
#define WebRtcNs_Free WebRtcNsx_Free

#define WebRtcAec_Create WebRtcAecm_Create
#define WebRtcAec_Init(handle, clock, sclock) WebRtcAecm_Init(handle, clock)
#define WebRtcAec_Free WebRtcAecm_Free
#define WebRtcAec_get_error_code WebRtcAecm_get_error_code
#define WebRtcAec_set_config WebRtcAecm_set_config
#define AecConfig AecmConfig


#define channel_count 1

typedef struct {
	float sum;
	int cnt;
	int max;
} rms_t;

typedef struct webrtc_ec {
    void *AEC_inst;
    NsHandle *NS_inst;
	MSBufferizer delayed_ref;
	MSBufferizer echo;
	MSFlowControlledBufferizer ref;
	int framesize;
	int delay_ms;
	int tail_length_ms;
	int nominal_ref_samples;
	int samplerate;
	bool_t echostarted;
	bool_t bypass_mode;
	bool_t using_zeroes;
	rms_t rms_in;
	rms_t rms_out;
} webrtc_ec;


static void configure_flow_controlled_bufferizer(webrtc_ec *s)
{
	s->framesize = (s->samplerate > 8000)? 160 : 80;
	ms_flow_controlled_bufferizer_set_samplerate(&s->ref, s->samplerate);
	ms_flow_controlled_bufferizer_set_max_size_ms(&s->ref, s->delay_ms);
	ms_flow_controlled_bufferizer_set_granularity_ms(&s->ref, (s->framesize * 1000) / s->samplerate);
}

static void webrtc_ec_init(MSFilter *f)
{
	webrtc_ec *s = ms_new0(webrtc_ec, 1);
	
	s->samplerate=8000;
	s->delay_ms=90;
	s->tail_length_ms=0;
	s->framesize=80;
	s->using_zeroes=FALSE;
	s->echostarted=FALSE;
	s->bypass_mode=FALSE;

	s->rms_in.sum = 0;
	s->rms_in.cnt = 0;
	s->rms_in.max = 80*2; // todo
	s->rms_out.sum = 0;
	s->rms_out.cnt = 0;
	s->rms_out.max = 80*1; // todo

	ms_bufferizer_init(&s->delayed_ref);
	ms_bufferizer_init(&s->echo);
	ms_flow_controlled_bufferizer_init(&s->ref, f, s->samplerate, channel_count);

	f->data=s;

	printf( "webrtc_ec_init\n");
}

static void webrtc_ec_uninit(MSFilter *f)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	ms_bufferizer_uninit(&s->delayed_ref);
	ms_free(s);
}


static void webrtc_ec_preprocess(MSFilter *f)
{
    AecConfig aec_config;
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;

	s->echostarted = FALSE;

	int delay_samples = s->delay_ms * s->samplerate / 1000;
	s->framesize = (s->samplerate > 8000)? 160 : 80;

	s->AEC_inst = WebRtcAec_Create();
    WebRtcAec_Init(s->AEC_inst, s->samplerate, s->samplerate);

	aec_config.cngMode = TRUE;
	aec_config.echoMode = 3; // todo
	WebRtcAec_set_config(s->AEC_inst, aec_config);

	s->NS_inst = WebRtcNs_Create();
	WebRtcNs_Init(s->NS_inst, s->samplerate);

	configure_flow_controlled_bufferizer(s);

	/* fill with zeroes for the time of the delay*/
	mblk_t *m = allocb(delay_samples * 2, 0);
	m->b_wptr += delay_samples * 2;
	ms_bufferizer_put(&s->delayed_ref, m);
	s->nominal_ref_samples = delay_samples;
}


static int handleRms(webrtc_ec *s, rms_t *r, int16_t *sig, float *result)
{
	for (int i = 0; i < s->framesize; ++i) {
		r->sum += (float)sig[i] * (float)sig[i];
		r->cnt++;
	}
	if (r->cnt >= r->max) {
		*result = r->sum / (float)r->cnt;
		r->sum = 0;
		r->cnt = 0;
		return 1;
	}
	return 0;
}


/*	inputs[0]= reference signal from far end (sent to soundcard)  -- from other side
 *	inputs[1]= near speech & echo signal	(read from soundcard) -- from micro
 *	outputs[0]=  is a copy of inputs[0] to be sent to soundcard   -- to speaker
 *	outputs[1]=  near end speech, echo removed - towards far end  -- to other side
*/
static void webrtc_ec_process(MSFilter *f)
{
	#define BUF_LEN 160
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	int nbytes = s->framesize * 2;

	mblk_t *refm;
	int16_t tmp_buf[BUF_LEN]; (void)tmp_buf;
	int16_t ref[BUF_LEN]; (void)ref;
	int16_t echo[BUF_LEN]; (void)echo;
	const int16_t * buf_ptr;
	int16_t * out_buf_ptr;
	float rms;

	if (s->bypass_mode) {
		while ((refm = ms_queue_get(f->inputs[0])) != NULL) {
			ms_queue_put(f->outputs[0], refm);
		}
		while ((refm = ms_queue_get(f->inputs[1])) != NULL) {
			ms_queue_put(f->outputs[1], refm);
		}
		return;
	}

	if (f->inputs[0]!=NULL) {
		if (s->echostarted) {
			while((refm=ms_queue_get(f->inputs[0]))!=NULL) {
				ms_bufferizer_put(&s->delayed_ref, dupmsg(refm));
				ms_flow_controlled_bufferizer_put(&s->ref, refm);
			}
		} else {
			ms_warning("Getting reference signal but no echo to synchronize on.");
			ms_queue_flush(f->inputs[0]);
		}
	}

	ms_bufferizer_put_from_queue(&s->echo, f->inputs[1]);

	while (ms_bufferizer_read(&s->echo, (uint8_t*)echo, nbytes) >= nbytes) {
		mblk_t *oecho = allocb(nbytes, 0);
		int avail;

		if (!s->echostarted) s->echostarted = TRUE;
		if ((avail = ms_bufferizer_get_avail(&s->delayed_ref)) < ((s->nominal_ref_samples * 2) + nbytes)) {
			/*we don't have enough to read in a reference signal buffer, inject silence instead*/
			refm = allocb(nbytes, 0);
			memset(refm->b_wptr, 0, nbytes);
			refm->b_wptr += nbytes;
			ms_bufferizer_put(&s->delayed_ref, refm);
			ms_queue_put(f->outputs[0], dupmsg(refm));
			if (!s->using_zeroes) {
				ms_warning("Not enough ref samples, using zeroes");
				s->using_zeroes = TRUE;
			}
		} else {
			if (s->using_zeroes) {
				ms_message("Samples are back.");
				s->using_zeroes = FALSE;
			}
			/* read from our no-delay buffer and output */
			refm = allocb(nbytes, 0);
			if (ms_flow_controlled_bufferizer_read(&s->ref, refm->b_wptr, nbytes) == 0) {
				ms_fatal("Should never happen");
			}

			if ( handleRms(s, &s->rms_in, (int16_t*)refm->b_wptr, &rms) ) {				
				if (rms < 150) { // todo
					memset(refm->b_wptr, 0, nbytes);
				}
			}

			refm->b_wptr += nbytes;
			ms_queue_put(f->outputs[0], refm); 
		}

		/*now read a valid buffer of delayed ref samples*/
		if (ms_bufferizer_read(&s->delayed_ref, (uint8_t*)ref, nbytes) == 0) {
			ms_fatal("Should never happen");
		}

		WebRtcAecm_BufferFarend(s->AEC_inst, ref, s->framesize);
		buf_ptr = echo;
		out_buf_ptr = tmp_buf;
		WebRtcNsx_Process(s->NS_inst, &buf_ptr, channel_count, &out_buf_ptr);
		buf_ptr = out_buf_ptr;
		out_buf_ptr = (int16_t *)oecho->b_wptr;
		WebRtcAecm_Process(s->AEC_inst, echo, buf_ptr, out_buf_ptr, s->framesize, 0);

		if ( handleRms(s, &s->rms_out, echo, &rms) ) {				
			if (rms < 150) { // todo
				memset(oecho->b_wptr, 0, nbytes);
			}
		}

		oecho->b_wptr += nbytes;
		ms_queue_put(f->outputs[1], oecho);
	}
}

static void webrtc_ec_postprocess(MSFilter *f)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	ms_bufferizer_flush (&s->delayed_ref);
	ms_bufferizer_flush (&s->echo);
	ms_flow_controlled_bufferizer_flush (&s->ref);

	if (s->AEC_inst) { 
		WebRtcAec_Free(s->AEC_inst);
		s->AEC_inst = NULL;
	}
	if (s->NS_inst) { 
		WebRtcNs_Free(s->NS_inst);
		s->NS_inst = NULL;
	}
}

static int webrtc_ec_set_sr(MSFilter *f, void *arg)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	return 0;
}

static int webrtc_ec_set_framesize(MSFilter *f, void *arg)
{
	return 0;
}

static int webrtc_ec_set_delay(MSFilter *f, void *arg)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	return 0;
}

static int webrtc_ec_set_tail_length(MSFilter *f, void *arg)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	return 0;
}

static int webrtc_ec_set_bypass_mode(MSFilter *f, void *arg)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	return 0;
}

static int webrtc_ec_get_bypass_mode(MSFilter *f, void *arg)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	*(bool_t*)arg=s->bypass_mode;
	return 0;
}

static int webrtc_ec_set_state(MSFilter *f, void *arg)
{
	return 0;
}

static int webrtc_ec_get_state(MSFilter *f, void *arg)
{
	*(char**)arg="";
	return 0;
}


static MSFilterMethod webrtc_ec_methods[]={
	{	MS_FILTER_SET_SAMPLE_RATE			,	webrtc_ec_set_sr 			},
	{	MS_ECHO_CANCELLER_SET_TAIL_LENGTH	,	webrtc_ec_set_tail_length	},
	{	MS_ECHO_CANCELLER_SET_DELAY			,	webrtc_ec_set_delay			},
	{	MS_ECHO_CANCELLER_SET_FRAMESIZE		,	webrtc_ec_set_framesize		},
	{	MS_ECHO_CANCELLER_SET_BYPASS_MODE	,	webrtc_ec_set_bypass_mode	},
	{	MS_ECHO_CANCELLER_GET_BYPASS_MODE	,	webrtc_ec_get_bypass_mode	},
	{	MS_ECHO_CANCELLER_GET_STATE_STRING	,	webrtc_ec_get_state			},
	{	MS_ECHO_CANCELLER_SET_STATE_STRING	,	webrtc_ec_set_state			},
	{	0, 0 }
};

MSFilterDesc ms_webrtc_ec_desc={
	.id=MS_WEBRTC_EC_ID,
	.name="MSWebRTCAECM",
	.text=N_("Echo canceller using webrtc library"),
	.category=MS_FILTER_OTHER,
	.ninputs=2,
	.noutputs=2,
	.init=webrtc_ec_init,
	.preprocess=webrtc_ec_preprocess,
	.process=webrtc_ec_process,
	.postprocess=webrtc_ec_postprocess,
	.uninit=webrtc_ec_uninit,
	.methods=webrtc_ec_methods
};

MS_FILTER_DESC_EXPORT(ms_webrtc_ec_desc)