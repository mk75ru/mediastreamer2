
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

#define EC_DUMP 1
#define EC_DUMP_PREFIX "/var/log/"

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
	/*
	  MSFlowControlledBufferizer — это объект, который буферизует аудиосэмплы,предоставляемые как mblk_t
	  любого размера, и позволяет считывателю считывать (в порядке FIFO) произвольный размер, точно так же,
	  как MSBufferizer, но с дополнительной функцией:
	      он отслеживает фактическое заполнение ( минимальное количество образца) внутреннего буфера
		  в течение определенного периода времени.
		  Если это количество превышает заданный максимальный размер, он может либо вызвать событие,
		  чтобы запросить выборки, которые будут отброшены вышестоящим фильтром, либо просто исключить
		  такое количество избыточных выборок. Это особенно полезно, когда
		  синхронизация нескольких потоков вместе, которые могут работать с разной скоростью.
	*/
	MSFlowControlledBufferizer ref;
	int framesize;
	int delay_ms;
	int tail_length_ms;
	int nominal_ref_samples;
	int samplerate;
#ifdef EC_DUMP
	FILE *echofile;
	FILE *reffile;
	FILE *cleanfile;
#endif
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

#ifdef EC_DUMP
	{
		char *fname = ms_strdup_printf("%s/mswebrtcaec-%p-echo.raw", EC_DUMP_PREFIX, f);
		s->echofile = fopen(fname, "w");
		ms_free(fname);
		fname = ms_strdup_printf("%s/mswebrtcaec-%p-ref.raw", EC_DUMP_PREFIX, f);
		s->reffile = fopen(fname, "w");
		ms_free(fname);
		fname = ms_strdup_printf("%s/mswebrtcaec-%p-clean.raw", EC_DUMP_PREFIX, f);
		s->cleanfile = fopen(fname, "w");
		ms_free(fname);
	}
#endif


	f->data=s;

	printf( "-- webrtc_ec_init-- \n");
}

static void webrtc_ec_uninit(MSFilter *f)
{
	webrtc_ec *s=(webrtc_ec*)f->data; (void)s;
	ms_bufferizer_uninit(&s->delayed_ref);
#ifdef EC_DUMP
	if (s->echofile)
		fclose(s->echofile);
	if (s->reffile)
		fclose(s->reffile);
#endif
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
	int16_t ref[BUF_LEN]; (void)ref;    // Опорные данные полученные от удаленного абонента
	int16_t echo[BUF_LEN]; (void)echo;  // Эхо данные полученные от микрофона
	const int16_t * buf_ptr;
	int16_t * out_buf_ptr;
	float rms;

	if (s->bypass_mode) {
		// Обходной режим
		while ((refm = ms_queue_get(f->inputs[0])) != NULL) {
		    // Принимаем от удаленного абонента и посылаем на звуковую карту
			ms_queue_put(f->outputs[0], refm);
		}
		while ((refm = ms_queue_get(f->inputs[1])) != NULL) {
		    // Принимаем от звуковой карты и посылаем  удаленному абоненту
			ms_queue_put(f->outputs[1], refm);
		}
		return;
	}

	// -------- INPUTS[0] Прием данных от  УДАЛЕННОГО АБОНЕНТА (ОПОРНЫЕ ДАННЫЕ для эхоподавления) ------
	if (f->inputs[0]!=NULL) {
		// Принимаем от удаленного абонента
		if (s->echostarted) {
			//Есть данные полученные с микрофона и содержащие эхо

			// Получаем данные от удаленного абонента
			while((refm=ms_queue_get(f->inputs[0]))!=NULL) {
				// -----------  Помещаем ОПОРНЫЕ ДАННЫЕ в очередь s->DELAYED_REF -----------------------
				ms_bufferizer_put(&s->delayed_ref, dupmsg(refm));
				// -----------  Помещаем ОПОРНЫЕ ДАННЫЕ в очередь s->REF с контролем потока ------------
				ms_flow_controlled_bufferizer_put(&s->ref, refm);
			}
		} else {
			ms_warning("Getting reference signal but no echo to synchronize on.");
			ms_queue_flush(f->inputs[0]);
		}
	}

	// ------- INPUTS[1] Прием данных от  МИКРОФОНА (ЭХО ДАННЫЕ, так как содержат эхо) -------------
	// ------- Помещаем ЭХО ДАННЫЕ в очередь s->ECHO -----------------------------------------------
	ms_bufferizer_put_from_queue(&s->echo, f->inputs[1]);

	while (ms_bufferizer_read(&s->echo, (uint8_t*)echo, nbytes) >= nbytes) {
		// ----------- Читаем ЭХО ДАННЫЕ из очереди s->ECHO в буфер  ECHO --------------------------
		mblk_t *oecho = allocb(nbytes, 0);
		int avail;
		if (!s->echostarted) s->echostarted = TRUE;
		// ----------- Читаем ОПОРНЫЕ ДАННЫЕ из s->DELAYED_REF или s->REF в mblk_t REFM ------------
		if ((avail = ms_bufferizer_get_avail(&s->delayed_ref)) < ((s->nominal_ref_samples * 2) + nbytes)) {
			// ------------ Обработка хвостика ОПОРНЫХ ДАНЫХ -----------
			/*we don't have enough to read in a reference signal buffer, inject silence instead*/
			// Недостаточно данных осталсь в буфере ОПОРНЫХ ДАННЫХ
			refm = allocb(nbytes, 0);
			memset(refm->b_wptr, 0, nbytes);
			refm->b_wptr += nbytes;
			// -----------  Помещаем хвостик  ОПОРНЫХ ДАННЫХ в в mblk_t REFM -----------------------
			ms_bufferizer_put(&s->delayed_ref, refm);
			// ------------ Отправляем данные на звуковую карту OUTPUTS[0] -------------------------
			ms_queue_put(f->outputs[0], dupmsg(refm));
			if (!s->using_zeroes) {
				//недостаточно даных для для эха , ипользуем нули
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
			// -----------  Помещаем ОПОРНЫЕ ДАННЫЕ в в mblk_t REFM -----------------------
			if (ms_flow_controlled_bufferizer_read(&s->ref, refm->b_wptr, nbytes) == 0) {
				ms_fatal("Should never happen");
			}

			if ( handleRms(s, &s->rms_in, (int16_t*)refm->b_wptr, &rms) ) {				
				if (rms < 150) { // todo
					memset(refm->b_wptr, 0, nbytes);
				}
			}

			refm->b_wptr += nbytes;
			// ------------ Отправляем данные на звуковую карту OUTPUTS[0] -------------------------
			ms_queue_put(f->outputs[0], refm); 
		}

		/*now read a valid buffer of delayed ref samples*/
		if (ms_bufferizer_read(&s->delayed_ref, (uint8_t*)ref, nbytes) == 0) {
			ms_fatal("Should never happen");
		}

#ifdef EC_DUMP
		if (s->reffile)
			fwrite(ref, nbytes, 1, s->reffile);
		if (s->echofile)
			fwrite(echo, nbytes, 1, s->echofile);
#endif


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

#ifdef EC_DUMP
		if (s->cleanfile)
			fwrite(oecho->b_wptr, nbytes, 1, s->cleanfile);
#endif

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

	.id=MS_WEBRTC_EC_ID,  /* Идентификатор типа фильтра заданный в allfilters.h или нами самими.*/
	.name="MSWebRTCAECM", /* Имя фильтра.*/
	.text=N_("Echo canceller using webrtc library"), /*Короткий текст , описывающий фильтр.*/
	.category=MS_FILTER_OTHER,  /* Категория фильтра, описывающая роль. */
	.ninputs=2, /* Количество входов. */
	.noutputs=2, /* Количество выходов. */
	.init=webrtc_ec_init, /* Функция начальной инициализации фильтра. */
	.preprocess=webrtc_ec_preprocess, /* Функция вызываемая однократно перед запуском фильтра в работу */
	.process=webrtc_ec_process,  /* Функция выполняющая основную работу фильтра ,
								  вызываемая на каждый тик тикера MSTicker. */
	.postprocess=webrtc_ec_postprocess, /* Функция завершения работы фильтра ,
										 вызывается однократно после последнего вызова process(),
										 перед удалением фильтра. */
	.uninit=webrtc_ec_uninit, /*
								Функция завершения работы фильтра,
								выполняет освобождение памяти, которая была занята при создании внутренних
								структур фильтра.
							  */
	.methods=webrtc_ec_methods /* Таблица методов фильтра. */
};

MS_FILTER_DESC_EXPORT(ms_webrtc_ec_desc)
