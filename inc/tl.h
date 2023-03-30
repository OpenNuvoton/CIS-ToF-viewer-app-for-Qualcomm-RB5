/********************************************************************/
/**
 * @file	tl.h
 * @brief	interfaces of tof library
 */
/********************************************************************/

#ifndef H_TL
#define H_TL

/*--------------------------------------------------------------------
	include headers
--------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>

/*--------------------------------------------------------------------
	definitions
--------------------------------------------------------------------*/
/**
 * @enum	TL_E_BOOL
 * @brief	Value of boolean.
 */
typedef enum {
	TL_E_FALSE = 0,	/*!< false */
	TL_E_TRUE,		/*!< true */
} TL_E_BOOL;


/**
 * @enum	TL_E_RESULT
 * @brief	Return value of functions.
 */
typedef enum {
	 TL_E_SUCCESS = 0		/*!< success. */
	,TL_E_ERR_PARAM			/*!< bad parameter */
	,TL_E_ERR_SYSTEM		/*!< system error */
	,TL_E_ERR_STATE			/*!< state error */
	,TL_E_ERR_TIMEOUT		/*!< timeout error */
	,TL_E_ERR_EMPTY			/*!< buffer empty error */
	,TL_E_ERR_NOT_SUPPORT	/*!< not supported function */
	,TL_E_ERR_CANCELED		/*!< canceled */
	,TL_E_ERR_OTHER			/*!< unknown error */
} TL_E_RESULT;


/**
 * @name	TL_E_IMAGE_KIND
 * @brief	Kind of images to receive
 */
typedef enum {
	 TL_E_IMAGE_KIND_VGA_DEPTH_QVGA_IR_BG = 0	/*!< VGA depth image and QVGA IR image, QVGA BG data */
	,TL_E_IMAGE_KIND_QVGA_DEPTH_IR_BG			/*!< QVGA depth image, IR image and BG data */
	,TL_E_IMAGE_KIND_VGA_DEPTH_IR				/*!< VGA depth image and IR image */
	,TL_E_IMAGE_KIND_VGA_IR_QVGA_DEPTH			/*!< VGA IR image and QVGA depth image */
	,TL_E_IMAGE_KIND_VGA_IR_BG					/*!< VGA IR image and BG data */
	,TL_E_IMAGE_KIND_MAX
} TL_E_IMAGE_KIND;


/**
 * @enum	TL_E_MODE
 * @brief	Ranging mode
 */
typedef enum {
	 TL_E_MODE_0 = 0	/*!< mode 0 */
	,TL_E_MODE_1		/*!< mode 1 */
	,TL_E_MODE_2		/*!< mode 2 */
	,TL_E_MODE_3		/*!< mode 3 */
	,TL_E_MODE_4		/*!< mode 4 FbyF of TL_E_MODE_0 and TL_E_MODE_1 */
	,TL_E_MODE_5		/*!< mode 5 FbyF of TL_E_MODE_2 and TL_E_MODE_3 */
	,TL_E_MODE_NUM		/*!< number of ranging mode */
} TL_E_MODE;


/**
 * @enum	TL_E_CMD
 * @brief	Commands of property
 */
typedef enum {
	 TL_CMD_DEVICE_INFO		/*!< device information : TL_DeviceInfo */
	,TL_CMD_FOV				/*!< field of view : TL_Fov */
	,TL_CMD_RESOLUTION		/*!< image resolution : TL_Resolution */
	,TL_CMD_MODE			/*!< ranging mode : TL_E_MODE */
	,TL_CMD_MODE_INFO		/*!< ranging mode information : TL_ModeInfoGroup */
	,TL_CMD_LENS_INFO		/*!< lens information : TL_LensPrm */
} TL_E_CMD;


/**
 * @name	TL_NOTIFY
 * @brief	Contents of the notification
 */
/* @{ */
#define TL_NOTIFY_IMAGE			(0x00000001U) /*!< notify receive image */
#define TL_NOTIFY_NO_BUFFER		(0x00000100U) /*!< notify image buffer is empty */
#define TL_NOTIFY_DISCONNECT	(0x00000200U) /*!< notify disconnected */
#define TL_NOTIFY_DEVICE_ERR	(0x00001000U) /*!< notify device error */
#define TL_NOTIFY_SYSTEM_ERR	(0x00002000U) /*!< notify system error */
#define TL_NOTIFY_STOPPED		(0x80000000U) /*!< notify stopped to receive image */
/* @} */


/**
 * @struct	TL_Handle
 * @brief	Device handle
 */
struct stTL_Handle;
typedef struct stTL_Handle TL_Handle;


/**
 * @struct	TL_Param
 * @brief	initial parameter
 */
typedef struct {
	TL_E_IMAGE_KIND		image_kind;	/*!< Kind of images to receive */
} TL_Param;


/**
 * @struct	TL_DeviceInfo
 * @brief	device information
 */
typedef struct {
	char		mod_name[32];	/*!< device name */
	char		afe_name[32];	/*!< model name of AFE */
	char		sns_name[32];	/*!< model name of Sensor */
	char		lns_name[32];	/*!< model name of Lens */
	uint16_t	mod_type1;		/*!< model type 1 (HW version) */
	uint16_t	mod_type2;		/*!< model type 2 (Wavelength of light source[nm]) */
	uint16_t	afe_ptn_id;		/*!< pattern ID of AFE setting */
	uint16_t	sno_l;			/*!< Serial No. (Control number) */
	uint16_t	map_ver;		/*!< information map version */
	uint16_t	sno_u;			/*!< Serial No. */
	uint16_t	ajust_date;		/*!< Adjusted date (year[6bit], month[4bit], day[5bit]) */
	uint16_t	ajust_no;		/*!< Adjust No. */
} TL_DeviceInfo;


/**
 * @struct	TL_Fov
 * @brief	Field of view
 */
typedef struct {
	uint16_t	focal_length;	/*!< Focal length [x100 mm] */
	uint16_t	angle_h;		/*!< Horizontal viewing angle [x100 degree] */
	uint16_t	angle_v;		/*!< Vertical viewing angle [x100 degree] */
} TL_Fov;


/**
 * @struct	TL_ImageFormat
 * @brief	Image format
 */
typedef struct {
	uint16_t	width;			/*!< image width */
	uint16_t	height;			/*!< image height */
	uint16_t	stride;			/*!< data stride */
	uint16_t	bit_per_pixel;	/*!< pixel length [bit/pixel] */
} TL_ImageFormat;


/**
 * @struct	TL_Resolution
 * @brief	Image resolution
 */
typedef struct {
	TL_ImageFormat	depth;		/*!< depth Image format */
	TL_ImageFormat	ir;			/*!< IR Image format */
	TL_ImageFormat	confdata;	/*!< Conf data format */
	TL_ImageFormat	irnrref;	/*!< IrNrRef data format */
} TL_Resolution;


/**
 * @struct	TL_ModeInfo
 * @brief	ranging mode information
 */
typedef struct {
	TL_E_BOOL	enable;			/*!< Availability of ranging mode */
	uint16_t	range_near;		/*!< near limit of range */
	uint16_t	range_far;		/*!< far limit of range */
	uint16_t	depth_unit;		/*!< Unit of Depth value */
	uint16_t	fps;			/*!< frame rate */
} TL_ModeInfo;


/**
 * @struct	TL_ModeInfoGroup
 * @brief	ranging mode informations
 */
typedef struct {
	TL_E_BOOL		fbf;					/*!< frame by frame flag */
	TL_ModeInfo		mode[TL_E_MODE_NUM];	/*!< information each mode or frame in frame by frame */
} TL_ModeInfoGroup;


/**
 * @struct	TL_LensPrm
 * @brief	Lens parameters
 */
typedef struct {
	uint16_t	sns_h;				/*!< Number of pixels of sensor output(horizontal)[pixel] */
	uint16_t	sns_v;				/*!< Number of pixels of sensor output(vertical)[pixel] */
	uint16_t	center_h;			/*!< Pixel center(horizontal)[pixel] */
	uint16_t	center_v;			/*!< Pixel center(vertical)[pixel] */
	uint16_t	pixel_pitch;		/*!< sensor pitch[um*100] */
	int64_t		planer_prm[4];		/*!< parameter of planer conversion  : fixed point */
	int64_t		distortion_prm[4];	/*!< parameter of distortion correct : fixed point */
} TL_LensPrm;

//From CisTofSensorCtrl.h
/*------------------------------------------------------------------*/
/// @brief	temperature information
/*------------------------------------------------------------------*/
typedef struct {
	uint16_t	sdata;		/*!< temperature sensor information */
	uint16_t	r1;			/*!< information of temperature conversion R1 */
	uint16_t	r2;			/*!< information of temperature conversion R2 */
	uint16_t	r3;			/*!< information of temperature conversion R3 */
	uint8_t		t1;			/*!< information of temperature conversion T1 */
	uint8_t		t2;			/*!< information of temperature conversion T2 */
	uint8_t		t3;			/*!< information of temperature conversion T3 */
} stTempInfo;

// From CisTofType.h
/*------------------------------------------------------------------*/
/// @brief	temperature correction information for multi packet drive
/*------------------------------------------------------------------*/
typedef struct {
	bool		is_updated;			/*!< temperature profile is updated */
	uint16_t	shd_ofst_gain;		/*!< shading offset gain */
	uint16_t	shd_ofst_gain_idx;	/*!< decimal digit definition of shading offset gain */
	uint16_t	slope;				/*!< slope */
	int16_t		offset;				/*!< offset */
} stMPTempCrct;

// From CisTofType.h
/*------------------------------------------------------------------*/
/// @brief	frame information
/// @see	TofLib::capture
/*------------------------------------------------------------------*/
typedef struct {
	uint8_t			frm_index;		/*!< frame index */
	bool			frame_error;	/*!< whether frame error has occurred */
	bool			fbf;			/*!< frame by frame mode */
	uint8_t			pair_idx;		/*!< index of sensor mode */
	float			temp;			/*!< temperature */
	stMPTempCrct	mp_temp_crct;	/*!< temperature correction information */
} stFrmInfo;

/**
 * @struct	TL_Image
 * @brief	Image data
 */
typedef struct {
	stFrmInfo   frm_info;	/*!< frame information */
	void		*depth;		/*!< depth image */
	void		*ir;		/*!< IR image */
	void		*confdata;	/*!< Conf data */
	void		*irnrref;	/*!< IrNrRef data */
	int32_t		temp;		/*!< temperature [x100 degree] */
} TL_Image;

// Forward Declare
namespace cis
{
	class RomData;
}

/*--------------------------------------------------------------------
	functions
--------------------------------------------------------------------*/
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   initialize devices
 * @param   [out] handle            device handle
 * @param   [in] param              initial parameter
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_STATE          state error
 */
TL_E_RESULT TL_init(TL_Handle **handle, const TL_Param *param);

/**
 * @brief	terminate devices
 * @param   [in,out] handle         device handle
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_STATE          state error
 */
TL_E_RESULT TL_term(TL_Handle **handle);

/**
 * @brief   start streaming.
 * @param   [in] handle             device handle
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_STATE          state error
 */
TL_E_RESULT TL_start(TL_Handle *handle);

/**
 * @brief    stop streaming.
 * @param     [in] handle           device handle
 * @return    TL_E_SUCCESS          success.
 * @return    TL_E_ERR_PARAM        bad argument
 * @return    TL_E_ERR_SYSTEM       system error
 * @return    TL_E_ERR_STATE        state error
 */
TL_E_RESULT TL_stop(TL_Handle *handle);

/**
 * @brief   get device parameter.
 * @param   [in] handle             device handle
 * @param   [in] command            parameter type
 * @param   [in,out]    arg         setting parameter
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_STATE          state error
 */
TL_E_RESULT TL_getProperty(TL_Handle *handle, TL_E_CMD command, void* arg);

/**
 * @brief   set device parameter.
 * @param   [in] handle             device handle
 * @param   [in] command            parameter type
 * @param   [in,out] arg            setting parameter
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_STATE          state error
 * @return  TL_E_ERR_NOT_SUPPORT    not supported function(invalid mode select)
 */
TL_E_RESULT TL_setProperty(TL_Handle *handle, TL_E_CMD command, void* arg);

/**
 * @brief   receive image data
 * @param   [in] handle             device handle
 * @param   [out] notify            notification
 * @param   [out] image             Image buffers
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 * @return  TL_E_ERR_SYSTEM         system error
 * @return  TL_E_ERR_CANCELED       canceled to receive
 * @return  TL_E_ERR_TIMEOUT        receive timeout
 */
TL_E_RESULT TL_capture(TL_Handle *handle, uint32_t *notify, TL_Image *image);

/**
 * @brief   cancel to receive image data
 * @param   [in] handle             device handle
 * @return  TL_E_SUCCESS            success.
 * @return  TL_E_ERR_PARAM          bad argument
 */
TL_E_RESULT TL_cancel(TL_Handle *handle);

cis::RomData* TL_GetRomData(TL_Handle *handle);

#ifdef __cplusplus
}
#endif

#endif	/* H_TL */
