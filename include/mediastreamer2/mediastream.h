/*
mediastreamer2 library - modular sound and video processing and streaming
Copyright (C) 2006  Simon MORLAT (simon.morlat@linphone.org)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/


#ifndef MEDIASTREAM_H
#define MEDIASTREAM_H

#include <ortp/ortp.h>
#include <ortp/event.h>
#include <ortp/zrtp.h>
/* defined in srtp.h*/
typedef struct srtp_ctx_t *MSSrtpCtx;

#include <mediastreamer2/msfilter.h>
#include <mediastreamer2/msticker.h>
#include <mediastreamer2/mssndcard.h>
#include <mediastreamer2/mswebcam.h>
#include <mediastreamer2/msvideo.h>
#include <mediastreamer2/bitratecontrol.h>
#include <mediastreamer2/qualityindicator.h>
#include <mediastreamer2/ice.h>


#define PAYLOAD_TYPE_FLAG_CAN_RECV	PAYLOAD_TYPE_USER_FLAG_1
#define PAYLOAD_TYPE_FLAG_CAN_SEND	PAYLOAD_TYPE_USER_FLAG_2


#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup ring_api
 * @{
**/

struct _RingStream
{
	MSTicker *ticker;
	MSFilter *source;
	MSFilter *gendtmf;
	MSFilter *write_resampler;
	MSFilter *sndwrite;
};

typedef struct _RingStream RingStream;

MS2_PUBLIC RingStream *ring_start (const char * file, int interval, MSSndCard *sndcard);
MS2_PUBLIC RingStream *ring_start_with_cb(const char * file, int interval, MSSndCard *sndcard, MSFilterNotifyFunc func, void * user_data);
MS2_PUBLIC void ring_stop (RingStream * stream);

/**
 * @}
**/
/*
 * Crypto suite used configure encrypted stream*/
typedef enum _MSCryptoSuite{
	MS_CRYPTO_SUITE_INVALID=0,
	MS_AES_128_SHA1_80,
	MS_AES_128_SHA1_32,
	MS_AES_128_NO_AUTH,
	MS_NO_CIPHER_SHA1_80,
	MS_AES_256_SHA1_80,
	MS_AES_256_SHA1_32
} MSCryptoSuite;

typedef struct _MSCryptoSuiteNameParams{
	const char *name;
	const char *params;
}MSCryptoSuiteNameParams;

MS2_PUBLIC MSCryptoSuite ms_crypto_suite_build_from_name_params(const MSCryptoSuiteNameParams *nameparams);
MS2_PUBLIC int ms_crypto_suite_to_name_params(MSCryptoSuite cs, MSCryptoSuiteNameParams *nameparams);

typedef enum StreamType {
	AudioStreamType,
	VideoStreamType
} StreamType;

/**
 * The MediaStream is an object describing a stream (one of AudioStream or VideoStream).
**/
typedef struct _MediaStream MediaStream;

/*
 * internal cb to process rtcp stream
 * */
typedef void (*media_stream_process_rtcp_callback_t)(MediaStream *stream, mblk_t *m);

struct _MSMediaStreamSessions{
	RtpSession *rtp_session;
	MSSrtpCtx srtp_session;
	OrtpZrtpContext *zrtp_context;
	MSTicker *ticker;
	bool_t is_secured;
};

typedef struct _MSMediaStreamSessions MSMediaStreamSessions;

MS2_PUBLIC void ms_media_stream_sessions_uninit(MSMediaStreamSessions *sessions);

typedef enum _MSStreamState{
	MSStreamInitialized,
	MSStreamPreparing,
	MSStreamStarted
}MSStreamState;

/**
 * Base struct for both AudioStream and VideoStream structure.
**/
struct _MediaStream {
	StreamType type;
	MSStreamState state;
	MSMediaStreamSessions sessions;
	OrtpEvQueue *evq;
	MSFilter *rtprecv;
	MSFilter *rtpsend;
	MSFilter *encoder;
	MSFilter *decoder;
	MSFilter *voidsink;
	MSBitrateController *rc;
	MSQualityIndicator *qi;
	IceCheckList *ice_check_list;
	time_t start_time;
	time_t last_iterate_time;
	uint64_t last_packet_count;
	time_t last_packet_time;
	bool_t use_rc;
	bool_t owns_sessions;
	bool_t pad[1];
	/**
	 * defines encoder target network bit rate, uses #media_stream_set_target_network_bitrate() setter.
	 * */
	int target_bitrate;
	media_stream_process_rtcp_callback_t process_rtcp;
};


/**
 * @addtogroup audio_stream_api
 * @{
**/

MS2_PUBLIC bool_t media_stream_srtp_supported(void);

MS2_PUBLIC void media_stream_set_rtcp_information(MediaStream *stream, const char *cname, const char *tool);

MS2_PUBLIC void media_stream_get_local_rtp_stats(MediaStream *stream, rtp_stats_t *stats);

MS2_PUBLIC int media_stream_set_dscp(MediaStream *stream, int dscp);

MS2_PUBLIC void media_stream_enable_adaptive_bitrate_control(MediaStream *stream, bool_t enabled);

MS2_PUBLIC void media_stream_enable_adaptive_jittcomp(MediaStream *stream, bool_t enabled);

/*
 * deprecated, use media_stream_set_srtp_recv_key and media_stream_set_srtp_send_key.
**/
MS2_PUBLIC bool_t media_stream_enable_srtp(MediaStream* stream, MSCryptoSuite suite, const char* snd_key, const char* rcv_key);

MS2_PUBLIC int media_stream_set_srtp_recv_key(MediaStream *stream, MSCryptoSuite suite, const char* key);

MS2_PUBLIC int media_stream_set_srtp_send_key(MediaStream *stream, MSCryptoSuite suite, const char* key);
/**
 * @param[in] stream MediaStream object
 * @return true if stream is encrypted
 * */
MS2_PUBLIC bool_t media_stream_secured(const MediaStream *stream);
#define media_stream_is_secured media_stream_secured

/**
 * Tells whether AVPF is enabled or not.
 * @param[in] stream #MediaStream object.
 * @return True if AVPF is enabled, false otherwise.
 */
MS2_PUBLIC bool_t media_stream_avpf_enabled(const MediaStream *stream);

/**
 * Gets the AVPF Regular RTCP report interval.
 * @param[in] stream #MediaStream object.
 * @return The AVPF Regular RTCP report interval in seconds.
 */
MS2_PUBLIC uint8_t media_stream_get_avpf_rr_interval(const MediaStream *stream);


MS2_PUBLIC const MSQualityIndicator *media_stream_get_quality_indicator(MediaStream *stream);
/* *
 * returns a realtime indicator of the stream quality between 0 and 5
 * */
MS2_PUBLIC float media_stream_get_quality_rating(MediaStream *stream);

MS2_PUBLIC float media_stream_get_average_quality_rating(MediaStream *stream);

MS2_PUBLIC float media_stream_get_lq_quality_rating(MediaStream *stream);

MS2_PUBLIC float media_stream_get_average_lq_quality_rating(MediaStream *stream);

/**
 * <br>For multirate codecs like OPUS, encoder output target bitrate must be set.
 * <br>Encoder will compute output codec bitrate from this value.
 * <br> default value is the value corresponding the rtp PayloadType
 * @param stream stream to apply parameter on
 * @param target_bitrate in bit per seconds
 * @return 0 if succeed
 * */
MS2_PUBLIC int media_stream_set_target_network_bitrate(MediaStream *stream,int target_bitrate);

/**
 * get the stream target bitrate.
 * @param stream stream to apply parameter on
 * @return target_bitrate in bit per seconds
 * */
MS2_PUBLIC int media_stream_get_target_network_bitrate(const MediaStream *stream);

/**
 * get current stream  upload bitrate. Value is updated every seconds
 * @param stream
 * @return bitrate in bit per seconds
 * */
MS2_PUBLIC float media_stream_get_up_bw(const MediaStream *stream);

/**
 * get current stream download bitrate. Value is updated every seconds
 * @param stream
 * @return bitrate in bit per seconds
 * */
MS2_PUBLIC float media_stream_get_down_bw(const MediaStream *stream);

/**
 * Returns the sessions that were used in the media stream (RTP, SRTP, ZRTP...) so that they can be re-used.
 * As a result of calling this function, the media stream no longer owns the sessions and thus will not free them.
**/
MS2_PUBLIC void media_stream_reclaim_sessions(MediaStream *stream, MSMediaStreamSessions *sessions);


void media_stream_iterate(MediaStream * stream);

/**
 * Returns TRUE if stream was still actively receiving packets (RTP or RTCP) in the last period specified in timeout_seconds.
**/
MS2_PUBLIC bool_t media_stream_alive(MediaStream *stream, int timeout_seconds);

/**
 * @returns curret streams tate
 * */
MS2_PUBLIC MSStreamState media_stream_get_state(const MediaStream *stream);

typedef enum EchoLimiterType{
	ELInactive,
	ELControlMic,
	ELControlFull
} EchoLimiterType;

struct _AudioStream
{
	MediaStream ms;
	MSFilter *soundread;
	MSFilter *soundwrite;
	MSFilter *dtmfgen;
	MSFilter *dtmfgen_rtp;
	MSFilter *plc;
	MSFilter *ec;/*echo canceler*/
	MSFilter *volsend,*volrecv; /*MSVolumes*/
	MSFilter *local_mixer;
	MSFilter *local_player;
	MSFilter *local_player_resampler;
	MSFilter *read_resampler;
	MSFilter *write_resampler;
	MSFilter *equalizer;
	MSFilter *dummy;
	MSFilter *send_tee;
	MSFilter *recv_tee;
	MSFilter *recorder_mixer;
	MSFilter *recorder;
	char *recorder_file;
	EchoLimiterType el_type; /*use echo limiter: two MSVolume, measured input level controlling local output level*/
	uint32_t features;
	bool_t play_dtmfs;
	bool_t use_gc;
	bool_t use_agc;
	bool_t eq_active;
	bool_t use_ng;/*noise gate*/
	bool_t is_ec_delay_set;
};

/**
 * The AudioStream holds all resources to create and run typical VoIP audiostream.
**/
typedef struct _AudioStream AudioStream;


/* start a thread that does sampling->encoding->rtp_sending|rtp_receiving->decoding->playing */
MS2_PUBLIC AudioStream *audio_stream_start (RtpProfile * prof, int locport, const char *remip,
				 int remport, int payload_type, int jitt_comp, bool_t echo_cancel);

MS2_PUBLIC AudioStream *audio_stream_start_with_sndcards(RtpProfile * prof, int locport, const char *remip4, int remport, int payload_type, int jitt_comp, MSSndCard *playcard, MSSndCard *captcard, bool_t echocancel);


MS2_PUBLIC int audio_stream_start_with_files (AudioStream * stream, RtpProfile * prof,
					    const char *remip, int remport, int rem_rtcp_port,
					    int pt, int jitt_comp,
					    const char * infile,  const char * outfile);

/**
 * Starts an audio stream from/to local wav files or soundcards.
 * 
 * This method starts the processing of the audio stream, that is playing from wav file or soundcard, voice processing, encoding,
 * sending through RTP, receiving from RTP, decoding, voice processing and wav file recording or soundcard playback.
 * 
 * 
 * @param stream an AudioStream previously created with audio_stream_new().
 * @param profile a RtpProfile containing all PayloadType possible during the audio session.
 * @param rem_rtp_ip remote IP address where to send the encoded audio.
 * @param rem_rtp_port remote IP port where to send the encoded audio.
 * @param rem_rtcp_ip remote IP address for RTCP.
 * @param rem_rtcp_port remote port for RTCP.
 * @param payload payload type index to use for the sending stream. This index must point to a valid PayloadType in the RtpProfile.
 * @param jitt_comp Nominal jitter buffer size in milliseconds.
 * @param infile path to wav file to play out (can be NULL)
 * @param outfile path to wav file to record into (can be NULL)
 * @param playcard The soundcard to be used for playback (can be NULL)
 * @param captcard The soundcard to be used for catpure. (can be NULL)
 * @param use_ec whether echo cancellation is to be performed.
 * @returns 0 if sucessful, -1 otherwise.
**/
MS2_PUBLIC int audio_stream_start_full(AudioStream *stream, RtpProfile *profile, const char *rem_rtp_ip,int rem_rtp_port,
	const char *rem_rtcp_ip, int rem_rtcp_port, int payload,int jitt_comp, const char *infile, const char *outfile,
	MSSndCard *playcard, MSSndCard *captcard, bool_t use_ec);


MS2_PUBLIC void audio_stream_play(AudioStream *st, const char *name);
MS2_PUBLIC void audio_stream_record(AudioStream *st, const char *name);

static inline void audio_stream_set_rtcp_information(AudioStream *st, const char *cname, const char *tool) {
	media_stream_set_rtcp_information(&st->ms, cname, tool);
}

MS2_PUBLIC void audio_stream_play_received_dtmfs(AudioStream *st, bool_t yesno);

/**
 * Creates an AudioStream object listening on a RTP port.
 * @param loc_rtp_port the local UDP port to listen for RTP packets.
 * @param loc_rtcp_port the local UDP port to listen for RTCP packets
 * @param ipv6 TRUE if ipv6 must be used.
 * @returns a new AudioStream.
**/
MS2_PUBLIC AudioStream *audio_stream_new(int loc_rtp_port, int loc_rtcp_port, bool_t ipv6);

/**Creates an AudioStream object from initialized MSMediaStreamSessions.
 * @param sessions the MSMediaStreamSessions
 * @returns a new AudioStream
**/
MS2_PUBLIC AudioStream *audio_stream_new_with_sessions(const MSMediaStreamSessions *sessions);

#define AUDIO_STREAM_FEATURE_PLC 		(1 << 0)
#define AUDIO_STREAM_FEATURE_EC 		(1 << 1)
#define AUDIO_STREAM_FEATURE_EQUALIZER		(1 << 2)
#define AUDIO_STREAM_FEATURE_VOL_SND 		(1 << 3)
#define AUDIO_STREAM_FEATURE_VOL_RCV 		(1 << 4)
#define AUDIO_STREAM_FEATURE_DTMF		(1 << 5)
#define AUDIO_STREAM_FEATURE_DTMF_ECHO		(1 << 6)
#define AUDIO_STREAM_FEATURE_MIXED_RECORDING	(1 << 7)
#define AUDIO_STREAM_FEATURE_LOCAL_PLAYING	(1 << 8)

#define AUDIO_STREAM_FEATURE_ALL	(\
					AUDIO_STREAM_FEATURE_PLC | \
					AUDIO_STREAM_FEATURE_EC | \
					AUDIO_STREAM_FEATURE_EQUALIZER | \
					AUDIO_STREAM_FEATURE_VOL_SND | \
					AUDIO_STREAM_FEATURE_VOL_RCV | \
					AUDIO_STREAM_FEATURE_DTMF | \
					AUDIO_STREAM_FEATURE_DTMF_ECHO |\
					AUDIO_STREAM_FEATURE_MIXED_RECORDING |\
					AUDIO_STREAM_FEATURE_LOCAL_PLAYING \
					)


MS2_PUBLIC uint32_t audio_stream_get_features(AudioStream *st);
MS2_PUBLIC void audio_stream_set_features(AudioStream *st, uint32_t features);
	
MS2_PUBLIC void audio_stream_prepare_sound(AudioStream *st, MSSndCard *playcard, MSSndCard *captcard);
MS2_PUBLIC void audio_stream_unprepare_sound(AudioStream *st);
MS2_PUBLIC bool_t audio_stream_started(AudioStream *stream);
/**
 * Starts an audio stream from local soundcards.
 * 
 * This method starts the processing of the audio stream, that is capture from soundcard, voice processing, encoding,
 * sending through RTP, receiving from RTP, decoding, voice processing and soundcard playback.
 * 
 * @param stream an AudioStream previously created with audio_stream_new().
 * @param prof a RtpProfile containing all PayloadType possible during the audio session.
 * @param remip remote IP address where to send the encoded audio.
 * @param remport remote IP port where to send the encoded audio
 * @param rem_rtcp_port remote port for RTCP.
 * @param payload_type payload type index to use for the sending stream. This index must point to a valid PayloadType in the RtpProfile.
 * @param jitt_comp Nominal jitter buffer size in milliseconds.
 * @param playcard The soundcard to be used for playback
 * @param captcard The soundcard to be used for catpure.
 * @param echo_cancel whether echo cancellation is to be performed.
**/
MS2_PUBLIC int audio_stream_start_now(AudioStream * stream, RtpProfile * prof,  const char *remip, int remport, int rem_rtcp_port, int payload_type, int jitt_comp,MSSndCard *playcard, MSSndCard *captcard, bool_t echo_cancel);
MS2_PUBLIC void audio_stream_set_relay_session_id(AudioStream *stream, const char *relay_session_id);
/*returns true if we are still receiving some data from remote end in the last timeout seconds*/
MS2_PUBLIC bool_t audio_stream_alive(AudioStream * stream, int timeout);

/**
 * Executes background low priority tasks related to audio processing (RTP statistics analysis).
 * It should be called periodically, for example with an interval of 100 ms or so.
 */
MS2_PUBLIC void audio_stream_iterate(AudioStream *stream);

/**
 * enable echo-limiter dispositve: one MSVolume in input branch controls a MSVolume in the output branch
 * */
MS2_PUBLIC void audio_stream_enable_echo_limiter(AudioStream *stream, EchoLimiterType type);

/**
 * enable gain control, to be done before start()
 * */
MS2_PUBLIC void audio_stream_enable_gain_control(AudioStream *stream, bool_t val);

/**
 * enable automatic gain control, to be done before start()
 * */
MS2_PUBLIC void audio_stream_enable_automatic_gain_control(AudioStream *stream, bool_t val);

/**
 * to be done before start
 *  */
MS2_PUBLIC void audio_stream_set_echo_canceller_params(AudioStream *st, int tail_len_ms, int delay_ms, int framesize);

/**
 * enable adaptive rate control
 * */
static inline void audio_stream_enable_adaptive_bitrate_control(AudioStream *stream, bool_t enabled) {
	media_stream_enable_adaptive_bitrate_control(&stream->ms, enabled);
}

/**
 *  Enable adaptive jitter compensation
 *  */
static inline void audio_stream_enable_adaptive_jittcomp(AudioStream *stream, bool_t enabled) {
	media_stream_enable_adaptive_jittcomp(&stream->ms, enabled);
}

MS2_PUBLIC void audio_stream_set_mic_gain(AudioStream *stream, float gain);

/**
 *  enable/disable rtp stream
 *  */
MS2_PUBLIC void audio_stream_mute_rtp(AudioStream *stream, bool_t val);

/**
 * enable noise gate, must be done before start()
 * */
MS2_PUBLIC void audio_stream_enable_noise_gate(AudioStream *stream, bool_t val);

/**
 * enable parametric equalizer in the stream that goes to the speaker
 * */
MS2_PUBLIC void audio_stream_enable_equalizer(AudioStream *stream, bool_t enabled);

MS2_PUBLIC void audio_stream_equalizer_set_gain(AudioStream *stream, int frequency, float gain, int freq_width);

/**
 *  stop the audio streaming thread and free everything
 *  */
MS2_PUBLIC void audio_stream_stop (AudioStream * stream);

/**
 *  send a dtmf
 *  */
MS2_PUBLIC int audio_stream_send_dtmf (AudioStream * stream, char dtmf);

MS2_PUBLIC MSFilter *audio_stream_get_local_player(AudioStream *stream);

MS2_PUBLIC int audio_stream_mixed_record_open(AudioStream *st, const char*filename);

MS2_PUBLIC int audio_stream_mixed_record_start(AudioStream *st);

MS2_PUBLIC int audio_stream_mixed_record_stop(AudioStream *st);

MS2_PUBLIC void audio_stream_set_default_card(int cardindex);

/* retrieve RTP statistics*/
static inline void audio_stream_get_local_rtp_stats(AudioStream *stream, rtp_stats_t *stats) {
	media_stream_get_local_rtp_stats(&stream->ms, stats);
}

/* returns a realtime indicator of the stream quality between 0 and 5 */
MS2_PUBLIC float audio_stream_get_quality_rating(AudioStream *stream);

/* returns the quality rating as an average since the start of the streaming session.*/
MS2_PUBLIC float audio_stream_get_average_quality_rating(AudioStream *stream);

/* returns a realtime indicator of the listening quality of the stream between 0 and 5 */
MS2_PUBLIC float audio_stream_get_lq_quality_rating(AudioStream *stream);

/* returns the listening quality rating as an average since the start of the streaming session.*/
MS2_PUBLIC float audio_stream_get_average_lq_quality_rating(AudioStream *stream);

/* enable ZRTP on the audio stream */
MS2_PUBLIC void audio_stream_enable_zrtp(AudioStream *stream, OrtpZrtpParams *params);
/**
 * return TRUE if zrtp is enabled, it does not mean that stream is encrypted, but only that zrtp is configured to know encryption status, uses #
 * */
bool_t  audio_stream_zrtp_enabled(const AudioStream *stream);

/* enable SRTP on the audio stream */
static inline bool_t audio_stream_enable_srtp(AudioStream* stream, MSCryptoSuite suite, const char* snd_key, const char* rcv_key) {
	return media_stream_enable_srtp(&stream->ms, suite, snd_key, rcv_key);
}

static inline int audio_stream_set_dscp(AudioStream *stream, int dscp) {
	return media_stream_set_dscp(&stream->ms, dscp);
}

/**
 * @}
**/


/**
 * @addtogroup video_stream_api
 * @{
**/

typedef void (*VideoStreamRenderCallback)(void *user_pointer, const MSPicture *local_view, const MSPicture *remote_view);
typedef void (*VideoStreamEventCallback)(void *user_pointer, const MSFilter *f, const unsigned int event_id, const void *args);


typedef enum _VideoStreamDir{
	VideoStreamSendRecv,
	VideoStreamSendOnly,
	VideoStreamRecvOnly
}VideoStreamDir;

struct _VideoStream
{
	MediaStream ms;
	MSFilter *source;
	MSFilter *pixconv;
	MSFilter *sizeconv;
	MSFilter *tee;
	MSFilter *output;
	MSFilter *tee2;
	MSFilter *jpegwriter;
	MSFilter *output2;
	MSVideoSize sent_vsize;
	int corner; /*for selfview*/
	VideoStreamRenderCallback rendercb;
	void *render_pointer;
	VideoStreamEventCallback eventcb;
	void *event_pointer;
	char *display_name;
	unsigned long window_id;
	unsigned long preview_window_id;
	VideoStreamDir dir;
	MSWebCam *cam;
	int device_orientation; /* warning: meaning of this variable depends on the platform (Android, iOS, ...) */
	bool_t use_preview_window;
	bool_t display_filter_auto_rotate_enabled;
	bool_t source_performs_encoding;
	bool_t output_performs_decoding;
	uint64_t last_reported_decoding_error_time;
};

typedef struct _VideoStream VideoStream;


MS2_PUBLIC VideoStream *video_stream_new(int loc_rtp_port, int loc_rtcp_port, bool_t use_ipv6);
MS2_PUBLIC VideoStream *video_stream_new_with_sessions(const MSMediaStreamSessions *sessions);
MS2_PUBLIC void video_stream_set_direction(VideoStream *vs, VideoStreamDir dir);
static inline void video_stream_enable_adaptive_bitrate_control(VideoStream *stream, bool_t enabled) {
	media_stream_enable_adaptive_bitrate_control(&stream->ms, enabled);
}
static inline void video_stream_enable_adaptive_jittcomp(VideoStream *stream, bool_t enabled) {
	media_stream_enable_adaptive_jittcomp(&stream->ms, enabled);
}
MS2_PUBLIC void video_stream_set_render_callback(VideoStream *s, VideoStreamRenderCallback cb, void *user_pointer);
MS2_PUBLIC void video_stream_set_event_callback(VideoStream *s, VideoStreamEventCallback cb, void *user_pointer);
MS2_PUBLIC void video_stream_set_display_filter_name(VideoStream *s, const char *fname);
MS2_PUBLIC int video_stream_start(VideoStream * stream, RtpProfile *profile, const char *rem_rtp_ip, int rem_rtp_port, const char *rem_rtcp_ip, int rem_rtcp_port,
		int payload, int jitt_comp, MSWebCam *device);
MS2_PUBLIC void video_stream_prepare_video(VideoStream *stream);
MS2_PUBLIC void video_stream_unprepare_video(VideoStream *stream);


MS2_PUBLIC void video_stream_set_relay_session_id(VideoStream *stream, const char *relay_session_id);
static inline void video_stream_set_rtcp_information(VideoStream *st, const char *cname, const char *tool) {
	media_stream_set_rtcp_information(&st->ms, cname, tool);
}
MS2_PUBLIC void video_stream_change_camera(VideoStream *stream, MSWebCam *cam);
/* Calling video_stream_set_sent_video_size() or changing the bitrate value in the used PayloadType during a stream is running does nothing.
The following function allows to take into account new parameters by redrawing the sending graph*/
MS2_PUBLIC void video_stream_update_video_params(VideoStream *stream);
/*function to call periodically to handle various events */
MS2_PUBLIC void video_stream_iterate(VideoStream *stream);

/**
 * Ask the video stream to send a Full-Intra Request.
 * @param[in] stream The videostream object.
 */
MS2_PUBLIC void video_stream_send_fir(VideoStream *stream);

/**
 * Ask the video stream to generate a Video Fast Update (generally after receiving a Full-Intra Request.
 * @param[in] stream The videostream object.
 */
MS2_PUBLIC void video_stream_send_vfu(VideoStream *stream);

MS2_PUBLIC void video_stream_stop(VideoStream * stream);
MS2_PUBLIC void video_stream_set_sent_video_size(VideoStream *stream, MSVideoSize vsize);

/**
 * Gets the size of the video that is sent.
 * @param[in] stream The videostream for which to get the sent video size.
 * @return The sent video size or MS_VIDEO_SIZE_UNKNOWN if not available.
 */
MS2_PUBLIC MSVideoSize video_stream_get_sent_video_size(const VideoStream *stream);

/**
 * Gets the size of the video that is received.
 * @param[in] stream The videostream for which to get the received video size.
 * @return The received video size or MS_VIDEO_SIZE_UNKNOWN if not available.
 */
MS2_PUBLIC MSVideoSize video_stream_get_received_video_size(const VideoStream *stream);

/**
 * Returns the name of the video display filter on the current platform.
**/
const char *video_stream_get_default_video_renderer(void);

MS2_PUBLIC void video_stream_enable_self_view(VideoStream *stream, bool_t val);
MS2_PUBLIC unsigned long video_stream_get_native_window_id(VideoStream *stream);
MS2_PUBLIC void video_stream_set_native_window_id(VideoStream *stream, unsigned long id);
MS2_PUBLIC void video_stream_set_native_preview_window_id(VideoStream *stream, unsigned long id);
MS2_PUBLIC unsigned long video_stream_get_native_preview_window_id(VideoStream *stream);
MS2_PUBLIC void video_stream_use_preview_video_window(VideoStream *stream, bool_t yesno);
MS2_PUBLIC void video_stream_set_device_rotation(VideoStream *stream, int orientation);
MS2_PUBLIC void video_stream_show_video(VideoStream *stream, bool_t show);

/**
 * @brief Gets the camera sensor rotation.
 *
 * This is needed on some mobile platforms to get the number of degrees the camera sensor
 * is rotated relative to the screen.
 *
 * @param stream The video stream related to the operation
 * @return The camera sensor rotation in degrees (0 to 360) or -1 if it could not be retrieved
 */
MS2_PUBLIC int video_stream_get_camera_sensor_rotation(VideoStream *stream);

/*provided for compatibility, use video_stream_set_direction() instead */
MS2_PUBLIC int video_stream_recv_only_start(VideoStream *videostream, RtpProfile *profile, const char *addr, int port, int used_pt, int jitt_comp);
MS2_PUBLIC int video_stream_send_only_start(VideoStream *videostream,
				RtpProfile *profile, const char *addr, int port, int rtcp_port,
				int used_pt, int  jitt_comp, MSWebCam *device);
MS2_PUBLIC void video_stream_recv_only_stop(VideoStream *vs);
MS2_PUBLIC void video_stream_send_only_stop(VideoStream *vs);

/* enable ZRTP on the video stream using information from the audio stream */
MS2_PUBLIC void video_stream_enable_zrtp(VideoStream *vstream, AudioStream *astream, OrtpZrtpParams *param);

/* enable SRTP on the video stream */
static inline bool_t video_stream_enable_strp(VideoStream* stream, MSCryptoSuite suite, const char* snd_key, const char* rcv_key) {
	return media_stream_enable_srtp(&stream->ms, suite, snd_key, rcv_key);
}

/* if enabled, the display filter will internaly rotate the video, according to the device orientation */
MS2_PUBLIC void video_stream_enable_display_filter_auto_rotate(VideoStream* stream, bool_t enable);

/* retrieve RTP statistics*/
static inline void video_stream_get_local_rtp_stats(VideoStream *stream, rtp_stats_t *stats) {
	media_stream_get_local_rtp_stats(&stream->ms, stats);
}

static inline int video_stream_set_dscp(VideoStream *stream, int dscp) {
	return media_stream_set_dscp(&stream->ms, dscp);
}

/**
 * Ask the video stream whether a decoding error should be reported (eg. to send a VFU request).
 * @param[in] stream The VideoStream object.
 * @param[in] ms The minimum interval in milliseconds between to decoding error report.
 * @return TRUE if the decoding error should be reported, FALSE otherwise.
 */
MS2_PUBLIC bool_t video_stream_is_decoding_error_to_be_reported(VideoStream *stream, uint32_t ms);

/**
 * Tell the video stream that a decoding error has been reported.
 * @param[in] stream The VideoStream object.
 */
MS2_PUBLIC void video_stream_decoding_error_reported(VideoStream *stream);

/**
 * Small API to display a local preview window.
**/

typedef VideoStream VideoPreview;

MS2_PUBLIC VideoPreview * video_preview_new();
#define video_preview_set_size(p,s) 							video_stream_set_sent_video_size(p,s)
#define video_preview_set_display_filter_name(p,dt)	video_stream_set_display_filter_name(p,dt)
#define video_preview_set_native_window_id(p,id)		video_stream_set_native_preview_window_id (p,id)
#define video_preview_get_native_window_id(p)			video_stream_get_native_preview_window_id (p)
MS2_PUBLIC void video_preview_start(VideoPreview *stream, MSWebCam *device);
MS2_PUBLIC void video_preview_stop(VideoPreview *stream);

/**
 * @}
**/

MS2_PUBLIC bool_t ms_is_ipv6(const char *address);


#ifdef __cplusplus
}
#endif

#endif
