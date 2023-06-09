//******************************************************************************
//! \file         viewer.cpp
//! \brief        sample viewer application of tof library.
//! \copyright    Nuvoton Technology Corporation Japan
//******************************************************************************

//******************************************************************************
// Include Headers
//******************************************************************************
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdlib.h>
#include <signal.h>

#include <cstring>
#include <chrono>
#include <thread>
#include <forward_list>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "tl.h"
#include "tl_log.h"

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus



//******************************************************************************
// Definitions
//******************************************************************************
#define DEBUG								1	// debug log
#define USE_OPEN_CV_COLOR_MAP				1	// Use OpenCv ColorMap, COLORMAP_JET

#define TOF_VIEWER_VERSION					(0x0001)
#define TOF_VIEWER_STRING				"CIS ToF Viewer"
#define OPENCV_WINDOW_NAME_PANEL_VIEWER	"CIS ToF Viewer - Control Panel"

#define OPENCV_WINDOW_NAME_DPTH								"Color Depth Image"
#define OPENCV_WINDOW_NAME_IR								"IR image"
#define OPENCV_WINDOW_NAME_BG								"BG image"
#define OPENCV_WINDOW_NAME_CONFDATA							"CONFDATA image"
#define OPENCV_WINDOW_NAME_IRNRREF							"IRNRREF image"
#define OPENCV_TRACKBAR_NAME_GAMMA_CORR_IR					"IR Gamma Correction (slider/10)"
#define OPENCV_TRACKBAR_NAME_GAMMA_CORR_BG					"BG Gamma Correction (slider/10)"

#define OPENCV_TRACKBAR_NAME_TOGGLE_CONFDAT					"Conf Data View :                \t\t\t"
#define OPENCV_TRACKBAR_NAME_TOGGLE_IRNRREF					"IRNRREF View :                  \t\t\t"

// Image Size
typedef struct {
	size_t	depth;		// depth image
	size_t	ir;			// ir image
	size_t	confdata;	// confdata image
	size_t	irnrref;	// irnrref image
} apl_img_size;

// Application Paramters
typedef struct {
	TL_Handle			*handle;		// camera handle
	TL_ModeInfoGroup	mode_info_grp;	// Each ranging mode info
	TL_DeviceInfo		device_info;	// Device info
	TL_Fov				fov;			// FOV info
	TL_LensPrm			lens_info;		// Lens info
	TL_E_MODE			mode;			// User's selected ranging mode
	TL_E_IMAGE_KIND		image_kind;		// User's selected image kind, type of output image
	TL_Resolution		resolution;		// resolution of images
	apl_img_size		img_size;		// image size
	bool				view_confdat_on;	// ConfData view on/off
	bool				view_irnrref_on;	// IrNrRef view on/off
	bool				view_bef_enh_on;	// Image before Enhance feature on/off
} __attribute__((aligned(8))) apl_prm;

static apl_prm gPrm;			// application parameters
static bool bExit = false;		// false = Run Program, true = Exit Program.

static uint8_t							FRM_BUF_CNT = 1U;	// maximum of frame buffer
static std::mutex						sBufMtx;			// mutex for buffer, queue
static std::forward_list<TL_Image *>	sFreeBuf;			// free buffer list

static unsigned int start = 0;
static unsigned int end = 0;
static unsigned int fps = 0;
static unsigned int calcfps = 0;

pthread_t threadview;		// View Thread (Handle Depth View, IR View)

#if USE_OPEN_CV_COLOR_MAP
#else
class fwc::ColorTable* color_tbl;
#endif

//******************************************************************************
// Functions
//******************************************************************************
static void apl_print_error(TL_E_RESULT ret, char *function, unsigned int line);	// TODO remove

void apl_show_img(TL_E_MODE mode, TL_E_IMAGE_KIND img_kind, TL_Resolution reso, TL_Image *stData);
void *view_thread(void *);
void get_user_selection(void);


//******************************************************************************
//! \brief	allocate list of frame buffer
//! \param	[in]	buf_num		number of frame buffer
//******************************************************************************
void apl_frmbuf_alloc(uint8_t buf_num, TL_Resolution reso)
{
	uint8_t i, n;
	uint32_t siz_dp;	// depth
	uint32_t siz_ir;	// ir
	uint32_t siz_cf;	// confdata
	uint32_t siz_rf;	// irnrref

	siz_dp = reso.depth.width * reso.depth.height;
	siz_ir = reso.ir.width * reso.ir.height;
	siz_cf = reso.confdata.width * reso.confdata.height;
	siz_rf = reso.irnrref.width * reso.irnrref.height;

	for (i = 0; i < buf_num; i++) {
		TL_Image* buf = new TL_Image;

		buf->depth    = new uint16_t[siz_dp];
		std::fill_n(static_cast<uint16_t *>(buf->depth),    siz_dp, 0);

		buf->ir       = new uint16_t[siz_ir];
		std::fill_n(static_cast<uint16_t *>(buf->ir),       siz_ir, 0);

		buf->confdata = new uint16_t[siz_cf];
		std::fill_n(static_cast<uint16_t *>(buf->confdata), siz_cf, 0);

		buf->irnrref  = new uint16_t[siz_rf];
		std::fill_n(static_cast<uint16_t *>(buf->irnrref),  siz_rf, 0);

		sFreeBuf.push_front(buf);
	}
}


//******************************************************************************
//! \brief	free list of frame buffer
//******************************************************************************
void apl_frmbuf_free(void)
{
	std::lock_guard<std::mutex> lock(sBufMtx);

	while (!sFreeBuf.empty()) {
		TL_Image* buf = sFreeBuf.front();
		sFreeBuf.pop_front();

		delete[] static_cast<uint16_t *>(buf->depth);
		delete[] static_cast<uint16_t *>(buf->ir);
		delete[] static_cast<uint16_t *>(buf->confdata);
		delete[] static_cast<uint16_t *>(buf->irnrref);
		delete buf;
		buf = nullptr;
	}
}


//******************************************************************************
//! \brief	get frame buffer
//! \param	[out]	buf		frame buffer pointer
//******************************************************************************
int apl_frmbuf_get(TL_Image** buf)
{
	std::lock_guard<std::mutex> lock(sBufMtx);

	if (sFreeBuf.empty()) {
		return -1;
	}

	*buf = sFreeBuf.front();
	sFreeBuf.pop_front();

	return 0;
}


//******************************************************************************
//! \brief	release frame buffer
//! \param	[in,out]	buf		frame buffer pointer
//******************************************************************************
void apl_frmbuf_rel(TL_Image** buf)
{
	std::lock_guard<std::mutex> lock(sBufMtx);

	sFreeBuf.push_front(*buf);
	*buf = nullptr;
}


//******************************************************************************
//! \brief        Initialization of libccdtof.so Library
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_init(TL_E_MODE mode, TL_E_IMAGE_KIND image_kind)
{
	TL_E_RESULT ret;
	TL_Param    tlprm;

	TL_LGI("%s", __FUNCTION__);

	memset(&tlprm, 0, sizeof(tlprm));

	// Image Kind
	tlprm.image_kind = gPrm.image_kind;

	// Initialize libccdtof.so Library
	gPrm.handle = NULL;
	ret = TL_init(&gPrm.handle, &tlprm);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_init", __LINE__);
		return -1;
	}

	// Set Depth Mode (Ranging Mode)
	ret = TL_setProperty(gPrm.handle, TL_CMD_MODE, (void*)&gPrm.mode);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_setProperty TL_CMD_MODE", __LINE__);
		return -1;
	}

	// Get Resolution Of Output Images
	ret = TL_getProperty(gPrm.handle, TL_CMD_RESOLUTION, (void*)&gPrm.resolution);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_getProperty TL_CMD_RESOLUTION", __LINE__);
		return -1;
	}
#if DEBUG	// debug log
	else {
		printf("\n");
		printf("Resolution of output images:\n");
		printf("depth    : width=%d, height=%d, stride=%d, bit_per_pixel=%d \n",
			gPrm.resolution.depth.width,
			gPrm.resolution.depth.height,
			gPrm.resolution.depth.stride,
			gPrm.resolution.depth.bit_per_pixel);
		printf("ir       : width=%d, height=%d, stride=%d, bit_per_pixel=%d \n",
			gPrm.resolution.ir.width,
			gPrm.resolution.ir.height,
			gPrm.resolution.ir.stride,
			gPrm.resolution.ir.bit_per_pixel);
		printf("confdata : width=%d, height=%d, stride=%d, bit_per_pixel=%d \n",
			gPrm.resolution.confdata.width,
			gPrm.resolution.confdata.height,
			gPrm.resolution.confdata.stride,
			gPrm.resolution.confdata.bit_per_pixel);
		printf("irnrref  : width=%d, height=%d, stride=%d, bit_per_pixel=%d \n",
			gPrm.resolution.irnrref.width,
			gPrm.resolution.irnrref.height,
			gPrm.resolution.irnrref.stride,
			gPrm.resolution.irnrref.bit_per_pixel);
		printf("\n");
	}
#endif

	// Get Mode Information
	ret = TL_getProperty(gPrm.handle, TL_CMD_MODE_INFO, (void*)&gPrm.mode_info_grp);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_getProperty TL_CMD_MODE_INFO", __LINE__);
		return -1;
	}
#if DEBUG	// debug log
	else {
		printf("Mode Info:\n");
		printf("mode0 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[0].enable,
				gPrm.mode_info_grp.mode[0].range_near,
				gPrm.mode_info_grp.mode[0].range_far,
				gPrm.mode_info_grp.mode[0].depth_unit,
				gPrm.mode_info_grp.mode[0].fps);
		printf("mode1 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[1].enable,
				gPrm.mode_info_grp.mode[1].range_near,
				gPrm.mode_info_grp.mode[1].range_far,
				gPrm.mode_info_grp.mode[1].depth_unit,
				gPrm.mode_info_grp.mode[1].fps);
		printf("mode2 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[2].enable,
				gPrm.mode_info_grp.mode[2].range_near,
				gPrm.mode_info_grp.mode[2].range_far,
				gPrm.mode_info_grp.mode[2].depth_unit,
				gPrm.mode_info_grp.mode[2].fps);
		printf("mode3 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[3].enable,
				gPrm.mode_info_grp.mode[3].range_near,
				gPrm.mode_info_grp.mode[3].range_far,
				gPrm.mode_info_grp.mode[3].depth_unit,
				gPrm.mode_info_grp.mode[3].fps);
		printf("mode4 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[4].enable,
				gPrm.mode_info_grp.mode[4].range_near,
				gPrm.mode_info_grp.mode[4].range_far,
				gPrm.mode_info_grp.mode[4].depth_unit,
				gPrm.mode_info_grp.mode[4].fps);
		printf("mode5 : enable=%d, range_near=%d, range_far=%d, depth_unit=%d, fps=%d\n",
				gPrm.mode_info_grp.mode[5].enable,
				gPrm.mode_info_grp.mode[5].range_near,
				gPrm.mode_info_grp.mode[5].range_far,
				gPrm.mode_info_grp.mode[5].depth_unit,
				gPrm.mode_info_grp.mode[5].fps);
		printf("\n");
	}
#endif

	// Get Fov Information
	ret = TL_getProperty(gPrm.handle, TL_CMD_FOV, (void*)&gPrm.fov);
	if(ret != TL_E_SUCCESS){
		apl_print_error(ret, (char *)"TL_getProperty TL_CMD_FOV", __LINE__);
		return -1;
	}
#if DEBUG	// debug log
	else {
		printf("Fov Info:\n");
		printf("focal_length=%d, angle_h=%d, angle_v=%d\n",
				gPrm.fov.focal_length,
				gPrm.fov.angle_h,
				gPrm.fov.angle_v);
		printf("\n");
	}
#endif

	// Get Mode Information
	ret = TL_getProperty(gPrm.handle, TL_CMD_DEVICE_INFO, (void*)&gPrm.device_info);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_getProperty TL_CMD_DEVICE_INFO", __LINE__);
		return -1;
	}
#if DEBUG  // debug log
	else {
		printf("Hardware Info:\n");
		printf("%s; %s; %s; \n",
			gPrm.device_info.mod_name,
			gPrm.device_info.sns_name,
			gPrm.device_info.lns_name);
		printf("module_type:0x%x(%d hw version) 0x%x(%dnm light source wavelength) sno_l:0x%x\n",
			gPrm.device_info.mod_type1, gPrm.device_info.mod_type1,
			gPrm.device_info.mod_type2, gPrm.device_info.mod_type2,
			gPrm.device_info.sno_l);
		printf("eep_map_ver:0x%x sno_u:0x%x ajust_date:0x%x [20%d-%02d-%02d,T%d] ajust_no:0x%x\n",
			gPrm.device_info.map_ver,
			gPrm.device_info.sno_u,
			gPrm.device_info.ajust_date, (gPrm.device_info.ajust_date&0xFC00)>>10, (gPrm.device_info.ajust_date&0x03C0)>>6, (gPrm.device_info.ajust_date&0x003E)>>1, (gPrm.device_info.ajust_date&0x0001),
			gPrm.device_info.ajust_no);
		printf("\n");
	}
#endif

	// Get device information, Execute TL_getProperty (TL_CMD_LENS_INFO)
	ret = TL_getProperty(gPrm.handle, TL_CMD_LENS_INFO, (void*)&gPrm.lens_info);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_getProperty TL_CMD_LENS_INFO", __LINE__);
		return -1;
	}
#if DEBUG  // debug log
	else {
		printf("Lens Info:\n");
		printf("sns_h=%d, sns_v=%d, center_h=%d, center_v=%d pixel_pitch=%d\n",
			gPrm.lens_info.sns_h,
			gPrm.lens_info.sns_v,
			gPrm.lens_info.center_h,
			gPrm.lens_info.center_v,
			gPrm.lens_info.pixel_pitch);
		printf("planer_prm: ");
		for (int i = 0; i < 4; i++) {
			printf("%ld ", gPrm.lens_info.planer_prm[i]);
		}
		printf("\n");

		printf("distortion_prm: ");
		for (int i = 0; i < 4; i++) {
			printf("%ld ", gPrm.lens_info.distortion_prm[i]);
		}
		printf("\n");
	}
#endif

#if USE_OPEN_CV_COLOR_MAP
#else
	fwc::stRange range = {
		 static_cast<uint16_t>(static_cast<float>(gPrm.mode_info_grp.mode[mode].range_near) * 0.9F)	// 10% more of whole range
		,static_cast<uint16_t>(static_cast<float>(gPrm.mode_info_grp.mode[mode].range_far)  * 1.1F)	// 10% more of whole range
	};
	color_tbl = new fwc::ColorTable();
	color_tbl->setRange(range);
#endif

	return ret;
}

//******************************************************************************
//! \brief        Termination of libccdtof/libcistof Library
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_term(void)
{
	TL_E_RESULT ret;

	TL_LGI("%s", __FUNCTION__);

	ret = TL_term(&gPrm.handle);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_term", __LINE__);
		return -1;
	}


	return 0;
}


//******************************************************************************
//! \brief        Start Transferring
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_start(void)
{
	TL_E_RESULT ret;

	TL_LGI("%s", __FUNCTION__);

	ret = TL_start(gPrm.handle);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_start", __LINE__);
		return -1;
	}

	return 0;
}


//******************************************************************************
//! \brief        Capturing Of Images
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_capture(void)
{
	TL_E_RESULT ret;
	uint32_t notify = 0U;
	TL_Image *data = nullptr;

	TL_LGI("%s", __FUNCTION__);

	apl_frmbuf_get(&data);

	ret = TL_capture(gPrm.handle, &notify, data);

	if (ret == TL_E_SUCCESS) {
		// recieved image data
		if ((notify & (uint32_t)TL_NOTIFY_IMAGE) != 0U) {

			// Show the image
			apl_show_img(gPrm.mode, gPrm.image_kind, gPrm.resolution, data);
		}
	}
	else {
		apl_print_error(ret, (char *)"TL_capture", __LINE__);
	}

	apl_frmbuf_rel(&data);

	return ret;
}


//******************************************************************************
//! \brief        Cancelation Of Capture
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_cancel(void)
{
	TL_E_RESULT ret;

	TL_LGI("%s", __FUNCTION__);

	ret = TL_cancel(gPrm.handle);
	if (ret == TL_E_SUCCESS) {
		printf("TL_cancel success\n");
	}
	else {
		printf("TL_cancel fail\n");
	}

	return ret;
}


//******************************************************************************
//! \brief        Stop Transferring
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       0         success
//! \return       -1        failed
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static int apl_stop(void)
{
	TL_E_RESULT ret;

	TL_LGI("%s", __FUNCTION__);

	ret = TL_stop(gPrm.handle);
	if (ret != TL_E_SUCCESS) {
		apl_print_error(ret, (char *)"TL_stop", __LINE__);
		return -1;
	}

	return 0;
}


//******************************************************************************
//! \brief        Print Error From libccdtof.so Library
//! \details
//! \param[in]    ret           return value from tof library
//! \param[in]    function	    function of error happend
//! \param[in]    line          line of error happend
//! \param[out]   None
//! \return       None
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static void apl_print_error(TL_E_RESULT ret, char *function, unsigned int line)
{
	switch (ret) {
		case TL_E_SUCCESS:
			break;
		case TL_E_ERR_PARAM:
			printf("[%s L.%d] TL_E_ERR_PARAM\n", function, (int)line);
			break;
		case TL_E_ERR_SYSTEM:
			printf("[%s L.%d] TL_E_ERR_SYSTEM\n", function, (int)line);
			break;
		case TL_E_ERR_STATE:
			printf("[%s L.%d] TL_E_ERR_STATE\n", function, (int)line);
			break;
		case TL_E_ERR_TIMEOUT:
			printf("[%s L.%d] TL_E_ERR_TIMEOUT\n", function, (int)line);
			break;
		case TL_E_ERR_EMPTY:
			printf("[%s L.%d] TL_E_ERR_EMPTY\n", function, (int)line);
			break;
		case TL_E_ERR_NOT_SUPPORT:
			printf("[%s L.%d] TL_E_ERR_NOT_SUPPORT\n", function, (int)line);
			break;
		case TL_E_ERR_CANCELED:
			printf("[%s L.%d] TL_E_ERR_CANCELED\n", function, (int)line);
			break;
		case TL_E_ERR_OTHER:
			printf("[%s L.%d] TL_E_ERR_OTHER\n", function, (int)line);
			break;
		default:
			printf("[%s L.%d] unknow error(%d)\n", function, (int)line, (int)ret);
			break;
	}
}


//******************************************************************************
//! \brief        Calculate Image Size From Pixel Format
//! \details
//! \param[in]    format	pixel format (tof library)
//! \param[out]   size      image size
//! \return       None
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static void apl_calc_img_size(TL_ImageFormat *format, size_t *size)
{
	// stride = image_width * bit_per_pixel
	*size = (size_t)format->stride * (size_t)format->height;
}


//******************************************************************************
//! \brief        Calculate Data Size From Pixel Format
//! \details
//! \param[in]    format    pixel format (tof library)
//! \param[in]    bpp       bit per pixel of data
//! \param[out]   size      data size
//! \return       None
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static void apl_calc_data_size(TL_ImageFormat *format, uint16_t bpp, size_t *size)
{
	*size = (size_t)format->width * (size_t)format->height * (size_t)bpp;
}


//******************************************************************************
//! \brief        Calculate Images Size
//! \details
//! \param[in]    None
//! \param[out]   None
//! \return       None
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static void apl_images_size(void)
{
	switch (gPrm.image_kind) {
		case TL_E_IMAGE_KIND_VGA_DEPTH_IR:
			apl_calc_img_size(&gPrm.resolution.depth,    &gPrm.img_size.depth);
			apl_calc_img_size(&gPrm.resolution.ir,       &gPrm.img_size.ir);
			apl_calc_img_size(&gPrm.resolution.confdata, &gPrm.img_size.confdata);
			apl_calc_img_size(&gPrm.resolution.irnrref,  &gPrm.img_size.irnrref);
			break;
		default:
			break;
	}
}


//******************************************************************************
//! \brief        Signal Handler Function
//! \details
//! \param[in]    signal    signal number
//! \param[out]   None
//! \return       None
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
static void apl_signal_handler(int signal)
{
	(void)signal;
	bExit = true;
}


//******************************************************************************
//! \brief        Utilities Function To Get User Entry.
//! \details
//! \param[in]    size  This is the maximum number of characters to be read (including the final null-character). Usually, the length of the array passed as str is used.
//! \param[out]   buf   This is the pointer to an array of chars where the string read is stored.
//! \return       Pointer To User Entry String
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
char *apl_ext_gets(char *buf, int size)
{
	size_t n;
	char   *p;

	p = fgets(buf, size, stdin);
	n = strlen(buf);
	if (buf[n-1] == '\n') {
		buf[n-1] = '\0';
	}
	else {
		buf[size-1] = '\0';
	}

	return p;
}

//******************************************************************************
//! \brief	Get user input selection
//! \param	[in]	min_choice		minimum range
//! \param	[in]	max_choice		maximum range
//! \return         None
//! \date           2023-03-15, Wed, 02:33 PM
//******************************************************************************
int get_user_input(int min_choice, int max_choice)
{
	int user_input = 0;

	do {
		printf("Enter: ");
		scanf("%d", &user_input);
		if (user_input >= min_choice && user_input <= max_choice) {
			break;
		}
	} while (true);

	return user_input;
}

//******************************************************************************
//! \brief        Utilities Function To Calculate FPS
//! \details
//! \param[in]    tick              Elapsed Time Tick.
//! \param[out]   tickforCalcFps	Start Time Tick.
//! \param[out]   fps               Frame Per Second.
//! \return       None.
//! \date         2022-06-16, Tue, 02:00 PM
//******************************************************************************
static void apl_get_calc_fps(const std::chrono::system_clock::time_point & tick, std::chrono::system_clock::time_point & tickforCalcFps, float & fps)
{
	const float elapsed = static_cast<float>(std::chrono::duration_cast<std::chrono::microseconds>(tick - tickforCalcFps).count());

	fps = 1000 * 1000 / elapsed;
	tickforCalcFps = tick;
}


//******************************************************************************
//! \brief        Utilities Function To Wait Until A Defined Interval
//! \details
//! \param[in]    targetFps     A Defined Interval, In Term Of Fps.
//! \param[in]    tickforFixFps	Current Time Tick.
//! \param[out]   None.
//! \return       None.
//! \date         2022-06-16, Tue, 02:00 PM
//******************************************************************************
static void apl_fix_fps(const float & targetFps, const std::chrono::system_clock::time_point & tickforFixFps)
{
	const int64_t duration_ms = static_cast<int64_t>(1000 / targetFps);
	const std::chrono::system_clock::time_point until = tickforFixFps + std::chrono::milliseconds(duration_ms);

	std::this_thread::sleep_until(until);
}


//******************************************************************************
//! \brief        Utilities Function To Get Current Time Tick.
//! \details
//! \param[in]    None.
//! \param[out]   None.
//! \return       Time Tick In Miliseconds
//! \date         2022-06-16, Tue, 02:00 PM
//******************************************************************************
unsigned int apl_get_tick_cnt(void)
{
	struct timeval tv;

	if (gettimeofday(&tv, NULL) != 0) {
		return 0;
	}

	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}


//******************************************************************************
//! \brief        Utilities Function To Convert Depth Image To Color Map, By Using OpenCV API.
//! \n
//! \param[in]    img       Depth Image.
//! \param[in]    min_val   Minimum Depth Value.
//! \param[in]    max_val   Depth Value.
//! \param[out]   None.
//! \return       cv::Mat of Type CV_8UC3 (8Bits 3 Channels).
//! \date         2021-02-16, Tue, 02:33 PM
//******************************************************************************
cv::Mat apl_dpth_to_color_by_opencv(cv::Mat img, uint32_t min_val, uint32_t max_val)
{
	size_t i;
	size_t j;
	size_t w = img.cols;
	size_t h = img.rows;
	double d_min_val = min_val;
	double d_max_val = max_val;
	cv::Mat mat_8bit(h, w, CV_8UC1);
	cv::Mat mat_32bit = cv::Mat::zeros(h, w, CV_32F);

	//! \remark 1. Normalise To 32Bits Float.
	img.convertTo(mat_32bit, CV_32FC1, 1/(d_max_val - d_min_val), -d_min_val/(d_max_val - d_min_val));

	//! \remark 2. Convert To 8Bits, As applyColorMap() Only Accept CV_8U.
	mat_32bit.convertTo(mat_8bit, CV_8UC1, -255, 255);

	//! \remark 3. Convert To Rainbow Color.
	cv::applyColorMap(mat_8bit, mat_8bit, cv::COLORMAP_JET);

	//! \remark 4. Mask Off Upper And Lower Range.
	for (i = 0 ; i < h; i++) {
		for (j = 0 ; j < w; j++) {
			float y = mat_32bit.at<float>(i,j);
			if (y > 1) {
				mat_8bit.at<cv::Vec3b>(i,j) = 0;	// Change To Black, i.e. 0 if (img > 1).
			}
			else
			if (y < 0) {
				mat_8bit.at<cv::Vec3b>(i,j) = 255;	// Change To White, i.e. 255 if (img < 0).
			}
		}
	}

	return mat_8bit;
}


//******************************************************************************
//! \brief        Opencv Trackbar Callback Function.
//! \n
//! \param[in]    trk_bar_pos   Current Trackbar Position.
//! \param[in]    usr_dat       Custom User Data.
//! \param[out]   None.
//! \return       None
//! \date         2022-08-17, Tue, 02:00 PM
//******************************************************************************
static void on_trkbar_toggle(int trk_bar_pos, void* usr_dat)
{
	bool *pBool = (bool *) usr_dat;

	if (*pBool != trk_bar_pos) {
		*pBool = (bool) trk_bar_pos;
	}
}

//******************************************************************************
//! \brief        Display Opencv Trackbar As GUI Panel
//! \n
//! \param[in]    None.
//! \param[out]   None.
//! \return       None.
//! \date         2022-08-17, Tue, 02:00 PM
//  https://www.opencv-srf.com/2011/11/track-bars.html
//******************************************************************************
void apl_show_pnl(void)
{
	//! \remark Create GUI For Control Panel
	cv::Mat mat_footer(25, 350, CV_8UC3, cv::Scalar(220, 220, 220));
	cv::putText(mat_footer,								// Target Image
				"Nuvoton Technology Corporation Japan",	// Text
				cv::Point(60, 15),						// Origin
				cv::FONT_HERSHEY_SIMPLEX,				// Font Face
				0.4,									// Font Scale
				CV_RGB(128, 128, 128),					// Font Color
				1,										// Thickness
				cv::LINE_8,								// Line Type
				false);									// Bottom Left Origin
	cv::imshow(OPENCV_WINDOW_NAME_PANEL_VIEWER, mat_footer);
	//cv::namedWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, CV_WINDOW_NORMAL);
	//cv::moveWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, 650, 10);
	//cv::resizeWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, 160, 80);
	//cv::waitKey(1);  // Draw The Screen And Wait For 1 Millisecond.

	cv::createTrackbar(OPENCV_TRACKBAR_NAME_TOGGLE_CONFDAT,              OPENCV_WINDOW_NAME_PANEL_VIEWER, nullptr, 1, on_trkbar_toggle, (void *)&gPrm.view_confdat_on);
	cv::createTrackbar(OPENCV_TRACKBAR_NAME_TOGGLE_IRNRREF,              OPENCV_WINDOW_NAME_PANEL_VIEWER, nullptr, 1, on_trkbar_toggle, (void *)&gPrm.view_irnrref_on);

	cv::imshow(OPENCV_WINDOW_NAME_PANEL_VIEWER, mat_footer);
	//cv::namedWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, CV_WINDOW_NORMAL);
	cv::moveWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, 650, 10);
	cv::resizeWindow(OPENCV_WINDOW_NAME_PANEL_VIEWER, 340, 80);
	cv::waitKey(1);  // Draw The Screen And Wait For 1 Millisecond.

}

//******************************************************************************
//! \brief        Display Image In Opencv Windows
//! \n
//! \param[in]    img_kind      Image data.
//! \param[in]    reso          Depth Image Format.
//! \param[in]    stData        Image data.
//! \param[in]    temperature   Temperature.
//! \param[out]   None.
//! \return       None
//! \date         2021-11-30, Tue, 02:33 PM
//******************************************************************************
void apl_show_img(TL_E_MODE mode, TL_E_IMAGE_KIND img_kind, TL_Resolution reso, TL_Image *stData)
{
	bool show_depth = false;
	bool show_ir = false;
	bool show_bg = false;
	bool show_confdat = gPrm.view_confdat_on;
	bool show_irnrref = gPrm.view_irnrref_on;
	size_t w;
	size_t h;
	uint8_t *p_data;
	uint16_t range_min;
	uint16_t range_max;
	static int32_t gamma_corr_ir = 22;  //!< Default Gamma Value Is 2.2 (For OpenCV TrackBar Used).
	static int32_t gamma_corr_bg = 22;  //!< Default Gamma Value Is 2.2 (For OpenCV TrackBar Used).
	int32_t temperature;
	std::chrono::system_clock::time_point tick = (std::chrono::system_clock::time_point::min)();
	static std::chrono::system_clock::time_point tickforCalcFps = (std::chrono::system_clock::time_point::min)();
	float calcFPS;
	char str[256];

	tick = std::chrono::system_clock::now();
	apl_get_calc_fps(tick, tickforCalcFps, calcFPS);

	show_depth = true;
	show_ir = true;

	if (show_depth) {
		// --------------------------------------------------
		//! \remark - Process Depth Image
		//! \remark - Obtain Height, Width, Pointer To Data From Toflib-Output Message.
		h = reso.depth.height;
		w = reso.depth.width;
		p_data = (uint8_t *)stData->depth;


		cv::Mat mat_depth_color;

#if USE_OPEN_CV_COLOR_MAP
		//! \remark - Create Cv Matrix, 16 Bits.
		cv::Mat mat_depth_raw(h, w, CV_16UC1, p_data);

		//! \remark - Decide The Range For Depth Base On Range Mode.
		range_min = gPrm.mode_info_grp.mode[gPrm.mode].range_near;
		range_max = gPrm.mode_info_grp.mode[gPrm.mode].range_far;

		//! \remark - Depth To Color Conversion, Using OpenCV API.
		mat_depth_color = apl_dpth_to_color_by_opencv(mat_depth_raw, range_min, range_max);
#else
		const fwc::stRGB* tbl = color_tbl->getTbl();
		mat_depth_color = cv::Mat::zeros(static_cast<int>(h), static_cast<int>(w), CV_8UC3);
		fwc::stRGB* dst = reinterpret_cast<fwc::stRGB*>(mat_depth_color.data);

		//! \remark - Convert uint16 To RGB
		uint16_t* src = static_cast<uint16_t*>(stData->depth);
		uint16_t* end = src + (w * h);

		for (; src < end; src++, dst++) {
			*dst = *(tbl + *src);
		}
#endif

		//! \remark - Add Fps Text.
		std::snprintf(str, sizeof(str), "fps=%d [instant fps=%.1f]", calcfps, calcFPS);
		cv::putText(mat_depth_color, std::string(str), cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		//! \remark - Add Temperature Text.
		temperature = stData->temp;
		std::snprintf(str, sizeof(str), "temperature=%d.%d C", temperature/100, temperature%100);
		cv::putText(mat_depth_color, std::string(str), cv::Point(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		//! \remark - Display It.
		cv::imshow(OPENCV_WINDOW_NAME_DPTH, mat_depth_color);
		cv::waitKey(1);	// Draw The Screen And Wait For 1 Millisecond.
	}

	if (show_ir) {
		// --------------------------------------------------
		//! \remark - Proecess IR Image
		//! \remark - Obtain Height, Width, Pointer To Data From Toflib-Output Message.
		h = reso.ir.height;
		w = reso.ir.width;
		p_data = (uint8_t *)stData->ir;

		//! \remark - Create Cv Matrix, 16 Bits.
		cv::Mat mat_ir(h, w, CV_16UC1, p_data);

		//! \remark - Convert To Double For "pow()".
		cv::Mat mat_ir_64f;
		mat_ir.convertTo(mat_ir_64f, CV_64F);

		//! \remark - Apply Gamma Correction.
		cv::Mat mat_ir_pow;
		cv::pow(mat_ir_64f, (float) gamma_corr_ir/10, mat_ir_pow);

		//! \remark - Convert Back To 16Bits.
		mat_ir_pow.convertTo(mat_ir, CV_16UC1);

		//! \remark - Add Fps Text.
		std::snprintf(str, sizeof(str), "fps=%d [instant fps=%.1f]", calcfps, calcFPS);
		cv::putText(mat_ir, std::string(str), cv::Point(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		//! \remark - Add Temperature Text.
		temperature = stData->temp;
		std::snprintf(str, sizeof(str), "temperature=%d.%d C", temperature/100, temperature%100);
		cv::putText(mat_ir, std::string(str), cv::Point(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

		//! \remark - Display It.
		cv::createTrackbar(OPENCV_TRACKBAR_NAME_GAMMA_CORR_IR, OPENCV_WINDOW_NAME_IR, &gamma_corr_ir, 30);
		cv::imshow(OPENCV_WINDOW_NAME_IR, mat_ir);
		cv::waitKey(1);	// Draw The Screen And Wait For 1 Millisecond.
	}

	if (show_confdat) {
		// --------------------------------------------------
		//! \remark - Proecess CONFDATA Image
		//! \remark - Obtain Height, Width, Pointer To Data From Toflib-Output Message.
		h = reso.confdata.height;
		w = reso.confdata.width;
		p_data = (uint8_t *)stData->confdata;

		//! \remark - Create Cv Matrix, 16 Bits.
		cv::Mat mat_confdata(h, w, CV_16UC1, p_data);

		//! \remark - Convert To Double For "pow()".
		cv::Mat mat_confdata_64f;
		mat_confdata.convertTo(mat_confdata_64f, CV_64F);

		//! \remark - Apply Gamma Correction.
		cv::Mat mat_confdata_pow;
		cv::pow(mat_confdata_64f, (float) 3.0, mat_confdata_pow);

		//! \remark - Convert Back To 16Bits.
		mat_confdata_pow.convertTo(mat_confdata, CV_16UC1);

		//! \remark - Display It.
		cv::imshow(OPENCV_WINDOW_NAME_CONFDATA, mat_confdata);
		cv::waitKey(1);	// Draw The Screen And Wait For 1 Millisecond.
	}
	else {
		//! \remark - Destroy It.
		cv::destroyWindow(OPENCV_WINDOW_NAME_CONFDATA);
	}

	if (show_irnrref) {
		// --------------------------------------------------
		//! \remark - Proecess IRNRREF Image
		//! \remark - Obtain Height, Width, Pointer To Data From Toflib-Output Message.
		h = reso.irnrref.height;
		w = reso.irnrref.width;
		p_data = (uint8_t *)stData->irnrref;

		//! \remark - Create Cv Matrix, 16 Bits.
		cv::Mat mat_irnrref(h, w, CV_16UC1, p_data);

		//! \remark - Convert To Double For "pow()".
		cv::Mat mat_irnrref_64f;
		mat_irnrref.convertTo(mat_irnrref_64f, CV_64F);

		//! \remark - Apply Gamma Correction.
		cv::Mat mat_irnrref_pow;
		cv::pow(mat_irnrref_64f, (float) 6.2, mat_irnrref_pow);

		//! \remark - Convert Back To 16Bits.
		mat_irnrref_pow.convertTo(mat_irnrref, CV_16UC1);

		//! \remark - Display It.
		cv::imshow(OPENCV_WINDOW_NAME_IRNRREF, mat_irnrref);
		cv::waitKey(1);	// Draw The Screen And Wait For 1 Millisecond.
	}
	else {
		//! \remark - Destroy It.
		cv::destroyWindow(OPENCV_WINDOW_NAME_IRNRREF);
	}
}


//******************************************************************************
//! \brief        Thread to handle image capture and view
//! \n
//! \param[in]    data         Data.
//! \return       void pointer
//! \date         2021-11-30, Tue, 02:33 PM
//******************************************************************************
void *view_thread(void *data)
{
	std::chrono::system_clock::time_point tickforFixFps = (std::chrono::system_clock::time_point::min)();

	apl_show_pnl();

	start = apl_get_tick_cnt();

	while (!bExit) {
		tickforFixFps = std::chrono::system_clock::now();

		apl_capture();

		int fps_now = gPrm.mode_info_grp.mode[gPrm.mode].fps;
		apl_fix_fps(fps_now, tickforFixFps); //!< Put here to adjust the capture interval

		fps++;
		end = apl_get_tick_cnt();
		if ((end - start) >= 1000) {
			calcfps = fps;
			//std::cout << "fps = " << fps << std::endl;
			fps = 0;
			start = end;
		}
	}

	return nullptr;
}

//******************************************************************************
//! \brief	Mode and type input selection
//! \date       2023-03-16, Thur, 02:33 PM
//******************************************************************************
void get_user_selection(void)
{
	char select_temp;

	printf("----------------------------------------------\n");
	printf("%s [Ver.%04x]\n", TOF_VIEWER_STRING, (int)TOF_VIEWER_VERSION);
	printf("Press [ctrl + c] to quit. \n");
	printf("----------------------------------------------\n");

	/* Image kind */
	gPrm.image_kind = TL_E_IMAGE_KIND_VGA_DEPTH_IR;

	/* Ranging mode selection */
	printf("\n");
	printf("Ranging mode selection (Current=%d)\n", gPrm.mode+1);
	printf( "1 : MODE1\n"
			"2 : MODE2\n"
			"3 : MODE3\n"
			"4 : MODE4\n"
			"5 : MODE5\n"
			"6 : MODE6\n");

	select_temp = get_user_input(1, 6);
	gPrm.mode = (TL_E_MODE) (select_temp - 1);

}


//******************************************************************************
//! \brief        main function
//! \n
//! \param[in]    argc         number of arguments.
//! \param[in]    argv         arguments.
//! \return       -1           fail with some error.
//! \date         2021-11-30, Tue, 02:33 PM
//******************************************************************************
int main(int argc, char *argv[])
{
	int ret = 0;

	signal(SIGINT, apl_signal_handler);

	memset(&gPrm, 0, sizeof(gPrm));

	gPrm.mode = TL_E_MODE_0;
	gPrm.image_kind = TL_E_IMAGE_KIND_VGA_DEPTH_IR;

	// Get user mode and type selection
	get_user_selection();

	// User Have On Camera Streaming, Proceed.
	if ((ret = apl_init(gPrm.mode, gPrm.image_kind)) < 0) {
		printf ("apl_init failed\n");
		exit(-1);
	}

	apl_images_size();

	apl_frmbuf_alloc(FRM_BUF_CNT, gPrm.resolution);

	if (apl_start() < 0) {
		printf ("apl_start failed\n");
		(void) apl_term();
		exit(-1);
	}

	// Create Threads
	if (pthread_create(&threadview, NULL, view_thread, NULL) != 0) {
		printf("pthread_create failed\n");
		exit(-1);
	}

	// Spin Here
	while (!bExit) {
		sleep(1);
	}

	// Abort Here
	apl_cancel();

	if (apl_stop() < 0) {
		printf("app exit abnormal\n");
		(void) apl_term();
		exit(-1);
	}

	// Wait Threads Terminate
	if (threadview) {
		pthread_join(threadview, NULL);
	}

	if (apl_term() < 0) {
		printf("apl_term abnormal\n");
		exit(-1);
	}

	apl_frmbuf_free();

	return 0;
}


#ifdef __cplusplus
}
#endif // __cplusplus
