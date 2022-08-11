/*
 *  linux/drivers/video/vfb.c -- Virtual frame buffer device
 *
 *      Copyright (C) 2002 James Simmons
 *
 *	Copyright (C) 1997 Geert Uytterhoeven
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <linux/fb.h>
#include <linux/init.h>
#include <linux/uaccess.h>

    /*
     *  RAM we reserve for the frame buffer. This defines the maximum screen
     *  size
     *
     *  The default can be overridden if the driver is compiled as a module
     */

#define VIDEOMEMSIZE	(16*1024*1024)	/* 16 MB */
#define BST_TEST_FBADDR   0xE2D00000    /*900M from 0x80000000*/
static void *videomemory;
static u_long videomemorysize = VIDEOMEMSIZE;
module_param(videomemorysize, ulong, 0);
MODULE_PARM_DESC(videomemorysize, "RAM available to frame buffer (in bytes)");

static char *mode_option = NULL;
module_param(mode_option, charp, 0);
MODULE_PARM_DESC(mode_option, "Preferred video mode (e.g. 640x480-8@60)");

static const struct fb_videomode vfb_default = {
	.xres =		1280,
	.yres =		720,
	.pixclock =	20000,
	.left_margin =	64,
	.right_margin =	64,
	.upper_margin =	32,
	.lower_margin =	32,
	.hsync_len =	64,
	.vsync_len =	2,
	.vmode =	FB_VMODE_NONINTERLACED,   
};

static struct fb_fix_screeninfo vfb_fix = {
	.id =		"bst Vscreen",
	.type =		FB_TYPE_PACKED_PIXELS,
	.visual =	FB_VISUAL_TRUECOLOR,
	.xpanstep =	1,
	.ypanstep =	1,
	.ywrapstep =	1,
	.accel =	FB_ACCEL_NONE,    
    .line_length = 1280,
};

static bool vfb_enable __initdata = 0;	/* disabled by default */
module_param(vfb_enable, bool, 0);
MODULE_PARM_DESC(vfb_enable, "Enable Virtual FB driver");
ssize_t fb_sys_read_bst(struct fb_info *info, char __user *buf, size_t count,
            loff_t *ppos);
ssize_t fb_sys_write_bst(struct fb_info *info, const char __user *buf,
             size_t count, loff_t *ppos);
static int vfb_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info);
static int vfb_set_par(struct fb_info *info);
static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info);
static int vfb_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *info);

static struct fb_ops vfb_ops = {
	.fb_read        = fb_sys_read_bst,
	.fb_write       = fb_sys_write_bst,
	.fb_check_var	= vfb_check_var,
	.fb_set_par	= vfb_set_par,
	.fb_setcolreg	= vfb_setcolreg,
	.fb_pan_display	= vfb_pan_display,
	.fb_fillrect	= sys_fillrect,
	.fb_copyarea	= sys_copyarea,
	.fb_imageblit	= sys_imageblit,
};

    /*
     *  Internal routines
     */
ssize_t fb_sys_read_bst(struct fb_info *info, char __user *buf, size_t count,
            loff_t *ppos)
{
    unsigned long p = *ppos;
    void *src;
    int err = 0;
    unsigned long total_size;

    if (info->state != FBINFO_STATE_RUNNING)
        return -EPERM;
	//fb_info(info, "BST fb_sys_read_bst,screen_size:0x%lx, smem_len :0x%x\n",info->screen_size ,info->fix.smem_len);
    fb_info(info, "cnt:%d, pos %lld\n",count ,p);
    fb_info(info, "info->var ,xoffset:%d, yoffset:%d\n",info->var.xoffset,info->var.yoffset);
    if (*ppos> info->screen_size*2){
        return  -EFAULT;
    }
    
    total_size = info->screen_size*2;

    if (total_size == 0)
        total_size = info->fix.smem_len;

    if (p >= total_size)
    {
        fb_err(info, "p >= total_size failed\n");       
        return 0;
    }
    if (count >= total_size)
        count = total_size;

    if (count + p > total_size)
        count = total_size - p;

    src = (void __force *)(info->screen_base + p + info->var.yoffset);

    if (info->fbops->fb_sync)
        info->fbops->fb_sync(info);

    //memcpy_fromio(buf, src, count);
    if (copy_to_user(buf, src, count))
    {    
        fb_err(info, "copy_to_user failed\n");        
        err = -EFAULT;
    }
   
    if  (!err)
        *ppos += count;
    
    return (err) ? err : count;
}

ssize_t fb_sys_write_bst(struct fb_info *info, const char __user *buf,
             size_t count, loff_t *ppos)
{
    unsigned long p = *ppos;
    void *dst;
    int err = 0;
    unsigned long total_size;

    if (info->state != FBINFO_STATE_RUNNING)
        return -EPERM;
	fb_info(info, "BST fb_sys_write_bst,screen_size:0x%x, smem_len :0x%x\n",info->screen_size ,info->fix.smem_len);
    total_size = info->screen_size;

    if (total_size == 0)
        total_size = info->fix.smem_len;

    if (p > total_size)
        return -EFBIG;

    if (count > total_size) {
        err = -EFBIG;
        count = total_size;
    }

    if (count + p > total_size) {
        if (!err)
            err = -ENOSPC;

        count = total_size - p;
    }

    dst = (void __force *) (info->screen_base + p);

    if (info->fbops->fb_sync)
        info->fbops->fb_sync(info);

    //memcpy_toio(dst, buf, count);      
    if (copy_from_user(dst, buf, count))
        err = -EFAULT;

    if  (!err)
        *ppos += count;
    return (err) ? err : count;
}
/*
static u_long get_line_length(int xres_virtual, int bpp)
{
	u_long length;

	length = xres_virtual * bpp;
	length = (length + 31) & ~31;
	length >>= 3;
	return (length);
}
*/
    /*
     *  Setting the video mode has been split into two parts.
     *  First part, xxxfb_check_var, must not write anything
     *  to hardware, it should only verify and adjust var.
     *  This means it doesn't alter par but it does use hardware
     *  data from it to check this var. 
     */

static int vfb_check_var(struct fb_var_screeninfo *var,
			 struct fb_info *info)
{
	u_long line_length;

	/*
	 *  FB_VMODE_CONUPDATE and FB_VMODE_SMOOTH_XPAN are equal!
	 *  as FB_VMODE_SMOOTH_XPAN is only used internally
	 */
    fb_info(info, "before var->xoffset:0x%x, var->yoffset:0x%x\n",var->xoffset ,var->yoffset);
    fb_info(info, "before info->var.xoffset:0x%x, info->var.yoffset:0x%x\n",info->var.xoffset ,info->var.yoffset);

	if (var->vmode & FB_VMODE_CONUPDATE) {
        fb_info(info, "FB_VMODE_CONUPDATE\n");
		var->vmode |= FB_VMODE_YWRAP;
		var->xoffset = info->var.xoffset;
		var->yoffset = info->var.yoffset;
	}
    
    fb_info(info, "after var->xoffset:0x%x, var->yoffset:0x%x\n",var->xoffset ,var->yoffset);
    fb_info(info, "after info->var.xoffset:0x%x, info->var.yoffset:0x%x\n",info->var.xoffset ,info->var.yoffset);

	/*
	 *  Some very basic checks
	 */
	
	fb_info(info, "before var->xres;:0x%x, var->yres:0x%x\n",var->xres ,var->yres);
    fb_info(info, "before info->var.xres:0x%x, info->var.yres:0x%x\n",info->var.xres ,info->var.yres);
	fb_info(info, "before var->xres_virtualt:0x%x, var->yres_virtual:0x%x\n",var->xres_virtual,var->yres_virtual);
	var->xres_virtual = var->xres;	
	var->yres_virtual = var->yres*2;   
    var->bits_per_pixel = 32;
	
	fb_info(info, "after var->xres_virtualt:0x%x, var->yres_virtual:0x%x\n",var->xres_virtual,var->yres_virtual);
    
  

	/*
	 *  Memory limit
	 */
    /* 
	line_length =
	    get_line_length(var->xres_virtual, var->bits_per_pixel);
	if (line_length * var->yres_virtual > videomemorysize)
		return -ENOMEM;
    */


	/*
	 * Now that we checked it we alter var. The reason being is that the video
	 * mode passed in might not work but slight changes to it might make it 
	 * work. This way we let the user know what is acceptable.
	 */
	switch (var->bits_per_pixel) {
	case 1:
	case 8:
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 0;
		var->green.length = 8;
		var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 16:		/* RGBA 5551 */
		if (var->transp.length) {
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 5;
			var->blue.offset = 10;
			var->blue.length = 5;
			var->transp.offset = 15;
			var->transp.length = 1;
		} else {	/* RGB 565 */
			var->red.offset = 0;
			var->red.length = 5;
			var->green.offset = 5;
			var->green.length = 6;
			var->blue.offset = 11;
			var->blue.length = 5;
			var->transp.offset = 0;
			var->transp.length = 0;
		}
		break;
	case 24:		/* RGB 888 */
		var->red.offset = 0;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		var->blue.offset = 16;
		var->blue.length = 8;
		var->transp.offset = 0;
		var->transp.length = 0;
		break;
	case 32:		/* RGBA 8888 */
		//var->red.offset = 0;
        var->red.offset = 16;
		var->red.length = 8;
		var->green.offset = 8;
		var->green.length = 8;
		//var->blue.offset = 16;
        var->blue.offset = 0;
		var->blue.length = 8;
		var->transp.offset = 24;
		var->transp.length = 8;
		break;
	}
	var->red.msb_right = 0;
	var->green.msb_right = 0;
	var->blue.msb_right = 0;
	var->transp.msb_right = 0;
	fb_info(info, "red.offset:0x%x, red.length:0x%x,green.offset:0x%x,green.length:0x%x,blue.offset:0x%x,blue.length:0x%x,transp.offset:0x%x,transp.length:0x%x,bits_per_pixel:0x%x\n",
		var->red.offset ,var->red.length,var->green.offset,var->green.length,var->blue.offset ,var->blue.length,var->transp.offset,var->transp.length,var->bits_per_pixel);

	return 0;
}

/* This routine actually sets the video mode. It's in here where we
 * the hardware state info->par and fix which can be affected by the 
 * change in par. For this driver it doesn't do much. 
 */
static int vfb_set_par(struct fb_info *info)
{
/*    
	switch (info->var.bits_per_pixel) {
	case 1:
		info->fix.visual = FB_VISUAL_MONO01;
		break;
	case 8:
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
        fb_info(info, "BST vfb_set_par,FB_VISUAL_PSEUDOCOLOR\n");
		break;
	case 16:
	case 24:
	case 32:
		info->fix.visual = FB_VISUAL_TRUECOLOR;
        fb_info(info, "BST vfb_set_par,FB_VISUAL_TRUECOLOR\n");
		break;
	}

	info->fix.line_length = get_line_length(info->var.xres_virtual,
						info->var.bits_per_pixel);
*/                        
	fb_info(info, "BST vfb_set_par,bits_per_pixel:0x%x, visual :0x%x,line_length:0x%x\n",info->var.bits_per_pixel ,info->fix.visual,info->fix.line_length);

	return 0;
}

    /*
     *  Set a single color register. The values supplied are already
     *  rounded down to the hardware's capabilities (according to the
     *  entries in the var structure). Return != 0 for invalid regno.
     */

static int vfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
		    (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0 unless the palette index takes less than
	 *                        bits_per_pixel bits and is stored in the upper
	 *                        bits of the pixel value
	 *    var->{color}.length is set so that 1 << length is the number of available
	 *                        palette entries
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
    
	return 0;
}

    /*
     *  Pan or Wrap the Display
     *
     *  This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
     */

static int vfb_pan_display(struct fb_var_screeninfo *var,
			   struct fb_info *info)
{
    fb_info(info, "BST vfb_pan_display in\n");
	if (var->vmode & FB_VMODE_YWRAP) {
		if (var->yoffset >= info->var.yres_virtual ||
		    var->xoffset)
			return -EINVAL;
	} else {
		if (var->xoffset + info->var.xres > info->var.xres_virtual ||
		    var->yoffset + info->var.yres > info->var.yres_virtual)
			return -EINVAL;
	}
	info->var.xoffset = var->xoffset;
	info->var.yoffset = var->yoffset;
	if (var->vmode & FB_VMODE_YWRAP)
		info->var.vmode |= FB_VMODE_YWRAP;
	else
		info->var.vmode &= ~FB_VMODE_YWRAP;
    
	fb_info(info, "BST vfb_pan_display,xoffset:%d, yoffset:%d,vmode:0x%x\n",
		info->var.xoffset,info->var.yoffset,info->var.vmode);
	return 0;
}
 
#ifndef MODULE
/*
 * The virtual framebuffer driver is only enabled if explicitly
 * requested by passing 'video=vfb:' (or any actual options).
 */
static int __init vfb_setup(char *options)
{
	char *this_opt;

	vfb_enable = 0;

	if (!options)
		return 1;

	vfb_enable = 1;

	if (!*options)
		return 1;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		/* Test disable for backwards compatibility */
		if (!strcmp(this_opt, "disable"))
			vfb_enable = 0;
		else
			mode_option = this_opt;
	}
	return 1;
}
#endif  /*  MODULE  */

    /*
     *  Initialisation
     */

static int vfb_probe(struct platform_device *dev)
{
	struct fb_info *info;
	unsigned int size = PAGE_ALIGN(videomemorysize);    
	struct resource	*res;
	int retval = -ENOMEM;
	dma_addr_t fb_mem_phys;
    void *fb_mem_virt=NULL;

	/*
	 * For real video cards we use ioremap.
	 
	if (!(videomemory = vmalloc_32_user(size)))
    {       
        //fb_err(info, "info maloc failed\n");        
		return retval;
    }
    
	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	videomemory = devm_ioremap_resource(&dev->dev, res);*/
	info = framebuffer_alloc(sizeof(u32) * 256, &dev->dev);
	if (!info)
    {       
        fb_err(info, "info maloc failed\n");        
		goto err;
    }
	info->fbops = &vfb_ops;
	fb_err(info, "mode_option %p.\n",mode_option);
	if (!fb_find_mode(&info->var, info, mode_option,
			  NULL, 0, &vfb_default, 32)){
		fb_err(info, "Unable to find usable video mode.\n");
		retval = -EINVAL;
		goto err1;
	}
    vfb_fix.smem_len = size;	
	//fb_mem_virt = dmam_alloc_coherent(&dev->dev, vfb_fix.smem_len, &fb_mem_phys,GFP_KERNEL);	
    fb_mem_virt = ioremap(BST_TEST_FBADDR,size);
    if (!fb_mem_virt) {
		fb_err(info, "ioremap fail.\n");
		retval = -EIO;
		goto err;
	}
	fb_err(info, "fb_mem_virt %x.\n",(unsigned int)(fb_mem_virt));
    vfb_fix.smem_start = BST_TEST_FBADDR;
    info->screen_base = (char __iomem *)fb_mem_virt;    
    info->screen_size= 1280*720*4;
	info->fix = vfb_fix;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_DEFAULT
				| FBINFO_VIRTFB
				| FBINFO_PARTIAL_PAN_OK;
	vfb_set_par(info);

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0)
    {   
        fb_err(info, "register_framebuffer failed\n");
		goto err1;
    }
	fb_info(info, "register_framebuffer");
	retval = register_framebuffer(info);
	if (retval < 0)
    {       
        fb_err(info, "fb_dealloc_cmapmaloc failed\n");
		goto err2;
    }
	
	fb_info(info, "set drv data");
	platform_set_drvdata(dev, info);

	fb_info(info, "BST screen_base 0x%x  , fb_mem_virt:0x%x,vfb_fix.smem_start:0x%x,info->screen_size:0x%x\n",
		(u32)(info->screen_base),(u32)fb_mem_virt,vfb_fix.smem_start,info->screen_size);


    //fb_info(info, " %s:0x%x 0x%x \n",res->name,res->start,res->end);

	fb_info(info, "BST Virtual frame buffer device, using %ldK of video memory\n",
		videomemorysize >> 10);
	return 0;
err2:    
	fb_dealloc_cmap(&info->cmap);
err1:    
	framebuffer_release(info);
err:
	iounmap(fb_mem_virt);
	return retval;
}

static int vfb_remove(struct platform_device *dev)
{
	struct fb_info *info = platform_get_drvdata(dev);

	if (info) {
		unregister_framebuffer(info);
		iounmap(info->screen_base);
		fb_dealloc_cmap(&info->cmap);
		framebuffer_release(info);
	}
	return 0;
}

static struct platform_driver vfb_driver = {
	.probe	= vfb_probe,
	.remove = vfb_remove,
	.driver = {
		.name	= "vfb",
	},
};

static struct platform_device *vfb_device;

static int __init vfb_init(void)
{
	int ret = 0;

#ifndef MODULE
	char *option = NULL;

	//if (fb_get_options("vfb", &option))
		//return -ENODEV;
	//vfb_setup(option);
	vfb_enable = 1;
#endif

	if (!vfb_enable)
		return -ENXIO;

	ret = platform_driver_register(&vfb_driver);

	if (!ret) {
		vfb_device = platform_device_alloc("vfb", 0);

		if (vfb_device)
			ret = platform_device_add(vfb_device);
		else
			ret = -ENOMEM;

		if (ret) {
			platform_device_put(vfb_device);
			platform_driver_unregister(&vfb_driver);
		}
	}

	return ret;
}

module_init(vfb_init);

#ifdef MODULE
static void __exit vfb_exit(void)
{
	platform_device_unregister(vfb_device);
	platform_driver_unregister(&vfb_driver);
}

module_exit(vfb_exit);

MODULE_LICENSE("GPL");
#endif				/* MODULE */
