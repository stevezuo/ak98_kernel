#ifndef __AK_2DACC_REG_H__
#define __AK_2DACC_REG_H__ __FILE__

#define GUI_SET_REG(reg_addr, value)	(*((volatile T_U32 *)(reg_addr))) = (value)
#define GUI_GET_REG(reg_addr)		(*((volatile T_U32 *)(reg_addr)))

/* register address define */
#define GUI_BASE_ADDR			0x20022000	// gui register base address
#define GUI_CMD_ADDR			(GUI_BASE_ADDR+0x04)	// Command
#define GUI_BLTSIZE_ADDR		(GUI_BASE_ADDR+0x08)	// Bitblt
#define GUI_POINT1_ADDR			(GUI_BASE_ADDR+0x0c)	// Point 1
#define GUI_DSTXY_ADDR			(GUI_BASE_ADDR+0x10)	// Dest. XY
#define GUI_POINT2_ADDR			(GUI_BASE_ADDR+0x14)	// Point 2
#define GUI_SRCXY_ADDR			(GUI_BASE_ADDR+0x18)	// Source XY
#define GUI_POINT3_ADDR			(GUI_BASE_ADDR+0x1c)	// Point 3
#define GUI_COLORCMP_ADDR		(GUI_BASE_ADDR+0x20)	// Color Compare
#define GUI_CLIPLT_ADDR			(GUI_BASE_ADDR+0x24)	// Clip Left/Top
#define GUI_CLIPRB_ADDR			(GUI_BASE_ADDR+0x28)	// Clip Right/Bottom
#define GUI_FCOLOR_ADDR			(GUI_BASE_ADDR+0x2c)	// Foreground Color
#define GUI_BCOLOR_ADDR			(GUI_BASE_ADDR+0x30)	// Background Color : Useless now
#define GUI_SRCSTRD_ADDR		(GUI_BASE_ADDR+0x34)	// Source Stride
#define GUI_DSTSTRD_ADDR		(GUI_BASE_ADDR+0x3c)	// Dest. Stride
#define GUI_DSTADDR_ADDR		(GUI_BASE_ADDR+0x40)	// Dest. Base Address
#define GUI_PATN0_ADDR			(GUI_BASE_ADDR+0x58)	// Pattern Part 0
#define GUI_PATN1_ADDR			(GUI_BASE_ADDR+0x5c)	// Pattern Part 1
#define GUI_PATNFC_ADDR			(GUI_BASE_ADDR+0x60)	// Pattern Foreground
#define GUI_PATNBC_ADDR			(GUI_BASE_ADDR+0x64)	// Pattern Background
#define GUI_ARBIOBJLID_ADDR		(GUI_BASE_ADDR+0x70)	// Arbitrary Object Line Index
#define GUI_ARBIOBJLRB_ADDR		(GUI_BASE_ADDR+0x74)	// Arbitrary Object Left/Right Boundary
#define GUI_SRCADDR_ADDR		(GUI_BASE_ADDR+0x78)	// Source Base Address
#define GUI_STATUS_ADDR			(GUI_BASE_ADDR+0x7c)	// Status

///////////////////////////////////////////////////////////////////////////////

// command register bit
#define CMD_RASTER_OP_VALUE_BIT		0	// Raster operation, 6:0
#define CMD_RASTER_OP_BIT		7	// 0-raster, 1-alpha
#define CMD_TYPE_BIT			8	// Command type, 10:8
#define CMD_MONO_SRC_BIT		14	// Monochrome source, 14
#define CMD_MONO_PAT_BIT		15	// Monochrome pattern, 15
#define CMD_COLOR_TRANS_EN_BIT		16	// Color transparency enable, 16
#define CMD_DST_TRANS_POL_BIT		17	// Destination transparency polarity, 17
#define CMD_MONO_SRC_PAT_BIT		18	// Monochrome source or pattern transparency enable, 18
#define CMD_MONO_TRANS_POL_BIT		19	// Monochrome transparency polarity, 19
#define CMD_SOLID_SRC_BIT		21	// Solid source color, 21
#define CMD_CLIP_EN_BIT			24	// Clipping enable, 24
#define CMD_SOLID_PAT_BIT		26	// Solid pattern, 26
#define CMD_TRANS_CMP_SRC_BIT		27	// Color transparency compare source, 27
//#define CMD_3D_BIT                    28  // 3D command
#define CMD_MATRIX_TRI_EN_BIT		30	// Matrix Triangle Enable

// BitBLT width and height parameter register bit
#define BITBLT_WIDTH_BIT		0	// Source or destination window width, 11:0
#define BITBLT_HEIGHT_BIT		16	// Source or destination window height, 27:16

// Bresenham line draw parameter register point1 bit
#define POINT1_X_BIT			0	// X coordination of point1, 11:0
#define POINT1_Y_BIT			16	// Y coordination of point1, 27:16

// Destination XY register bit
#define DSTXY_X_BIT			0	// Destination X position, 11:0
//#define DST_MONO_X_OFF_BIT            12  // Monochrome pattern horizontal offset, 15:12
#define DSTXY_Y_BIT			16	// Destination Y position, 27:16
//#define DST_MONO_Y_OFF_BIT            28  // Monochrome pattern horizontal offset, 31:28

// Bresenham line draw parameter register point2 bit
#define POINT2_X_BIT			0	// X coordination of point2, 11:0
#define POINT2_Y_BIT			16	// Y coordination of point2, 27:16

// Source XY register bit
#define SRCXY_X_BIT			0	// Source X position, 11:0
#define SRCXY_Y_BIT			16	// Source Y position, 27:16

// Bresenham line draw parameter register point3 bit
#define POINT3_X_BIT			0	// X coordination of point3, 11:0
#define POINT3_Y_BIT			16	// Y coordination of point3, 27:16

// Color compare register bit
#define COLCMP_TRANS_COLOR_BIT		0	// Destination transparent color, 23:0

// Clip Left/Top register bit
#define CLIPLT_LEFT_X_BIT		0	// Left edge of clipping rectangle, 11:0
#define CLIPLT_TOP_Y_BIT		16	// Top edge of clipping rectangle, 27:16

// Clip Left/Top register bit
#define CLIPRB_RIGHT_X_BIT		0	// Right edge of clipping rectangle, 11:0
#define CLIPRB_BOTTOM_Y_BIT		16	// Bottom edge of clipping rectangle, 27:16

// Source stride register bit
#define SRCSTRD_LINE_STRIDE_BIT		0	// Source line stride, 11:0
#define SRCSTRD_MONO_SRC_START_BIT	13	// Monochrome source starts, 15:13

// Destination stride and color depth register bit
#define DSTSTRD_LINE_STRIDE_BIT		0	// Destination line stride, 11:0
#define DSTSTRD_ROTATE_90		14	// Enable 90 degree clockwise rotation, 15:14

// Monochrome pattern register0 bit
#define PATN0_LINE0_BIT			0	// Line 0 of monochrome pattern, 7:0
#define PATN0_LINE1_BIT			8	// Line 1 of monochrome pattern, 15:8
#define PATN0_LINE2_BIT			16	// Line 2 of monochrome pattern, 23:16
#define PATN0_LINE3_BIT			24	// Line 3 of monochrome pattern, 31:24

// Monochrome pattern register1 bit
#define PATN0_LINE4_BIT			0	// Line 4 of monochrome pattern, 7:0
#define PATN0_LINE5_BIT			8	// Line 5 of monochrome pattern, 15:8
#define PATN0_LINE6_BIT			16	// Line 6 of monochrome pattern, 23:16
#define PATN0_LINE7_BIT			24	// Line 7 of monochrome pattern, 31:24

// Color Space conversion and scaling control bit 

// start rectangle draw operation with scaling and color conversion
#define SCALCTRL_START_BIT		0
#define SCALCTRL_BYPASS_BIT		1	// Bypass scaler, 2
// input format, 3:2=00-rgb888), 01-rgb565, 10-YUV420
#define SCALCTRL_FORMAT_BIT		2
// destination image is also a source needed in GUI operations, 4
#define SCALCTRL_NEEDED_BIT		4

// Scaling parameters, scaling ration = 8192 / ILX[8:0]
#define SCALRATIO_HORI_DIV_BIT		0	// Horizontal scaling divider ratio, 8:0
#define SCALRATIO_VERT_DIV_BIT		16	// Vertical scaling divider ratio, 24:16

// Input image rectangle dimensions
#define SCALSRCRECT_WIDTH_BIT		0	// Input image width in pixels, 9:0
#define SCALSRCRECT_HEIGHT_BIT		16	// Input image width in pixels, 24:16

// Output image rectangle dimensions
#define SCALDSTRECT_WIDTH_BIT		0	// Output image width in pixels, 9:0
#define SCALDSTRECT_HEIGHT_BIT		16	// Output image width in pixels, 24:16

/////////////////////////////////////////////////////////////////////////

// command register bit 7
#define RASTER_OP           0
#define ALPHA_OP            1

// raster operation
#define RASTER_NO_OP        0
#define RASTER_SRCAND       1
#define RASTER_SRCOR        2
#define RASTER_SRCXOR       3
#define RASTER_SRCCOPY      4

// command type
#define TYPE_SCALE_CONVT    0	// draw rectangle with scaling and color conversion capability
#define TYPE_RECT_FILL      1
#define TYPE_LINE_DRAW      2
#define TYPE_TRIANGLE_FILL  3
#define TYPE_ARBITRARY_FILL 4
#define TYPE_RECT_COPY      5

// pattern start pixel
#define PATN_START_PIXEL(n)     (n)

// pattern start line
#define PATN_START_LINE(n)      (n)

// monochrome start bit
#define MONO_START_BIT(n)       (n)

////////////////////////////////////////////////////////////////////////////////

// Gradient Color Fill
#define GUI_REFPOINT_ADDR       (GUI_BASE_ADDR+0x200)	// Reference Point
#define GUI_REFCOLOR_ADDR       (GUI_BASE_ADDR+0x204)	// Reference Color
#define GUI_GRADCOLOR_X_ADDR    (GUI_BASE_ADDR+0x208)	// X Gradient Color
#define GUI_GRADCOLOR_Y_ADDR    (GUI_BASE_ADDR+0x20c)	// Y Gradient Color
#define GUI_GRAD_RED_X_ADDR     (GUI_BASE_ADDR+0x210)	// Denominator/Nominator X for color R
#define GUI_GRAD_RED_Y_ADDR     (GUI_BASE_ADDR+0x214)	// Denominator/Nominator Y for color R
#define GUI_GRAD_GREEN_X_ADDR   (GUI_BASE_ADDR+0x220)	// Denominator/Nominator X for color G
#define GUI_GRAD_GREEN_Y_ADDR   (GUI_BASE_ADDR+0x224)	// Denominator/Nominator Y for color G
#define GUI_GRAD_BLUE_X_ADDR    (GUI_BASE_ADDR+0x230)	// Denominator/Nominator X for color B
#define GUI_GRAD_BLUE_Y_ADDR    (GUI_BASE_ADDR+0x234)	// Denominator/Nominator Y for color B

// Reference Point Bit
#define REF_POINT_ENABLE        24
#define REF_POINT_X0            12
#define REF_POINT_Y0            0

// Denominator/Nominator Bit
#define GRADFILL_ABS             20
#define GRADFILL_NOMINATOR       8
#define GRADFILL_DENOMINATOR     0

////////////////////////////////////////////////////////////////////////////////

// Matrix Triangle Fill
#define GUI_MATRIX_SRCADDR	(GUI_BASE_ADDR+0x240)
#define GUI_MATRIX_SRCSTRD	(GUI_BASE_ADDR+0x244)
#define GUI_MATRIX_REG1		(GUI_BASE_ADDR+0x250)	// sx, shx [31:22], [21:12]
#define GUI_MATRIX_REG2		(GUI_BASE_ADDR+0x254)	// sy, shy [31:22], [21:12]
#define GUI_MATRIX_REG3		(GUI_BASE_ADDR+0x258)	// w0,  w1 [31:22], [21:12]
#define GUI_MATRIX_REGTX	(GUI_BASE_ADDR+0x260)	// tx [23:0]
#define GUI_MATRIX_REGTY	(GUI_BASE_ADDR+0x264)	// ty [23:0]
#define GUI_MATRIX_REGW2	(GUI_BASE_ADDR+0x268)	// w2 [23:0]

// Shift Bit
#define GUI_MATRIX_SHIFT1	22
#define GUI_MATRIX_SHIFT2	12

////////////////////////////////////////////////////////////////////////////////

#endif				// __GUI_REG_H__
