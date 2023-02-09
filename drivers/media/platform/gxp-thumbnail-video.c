// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C)  2023 Hewlett Packard Enterprise


#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>




#define DEVICE_NAME			"gxp-video"
#define VCD_MODULE_NAME	    "vcd"

#define MAX_FRAME_RATE      60
#define MAX_WIDTH			1024
#define MAX_HEIGHT			768
#define MIN_WIDTH			800
#define MIN_HEIGHT			600

//Default resolution 800x600 , max resolution 1024x768

#define VCD_MAX_SRC_BUFFER_SIZE		(0x300000  + sizeof(struct v4l2_rect))/* 1024x768 x 4 bpp */


#define THUMBNAIL_CFG                0x00
#define THUMBNAIL_HORIZ_SIZE         0x08
#define THUMBNAIL_VERT_SIZE          0x0C
#define THUMBNAIL_TNAXISTS           0x14
#define THUMBNAIL_DEST_BAR           0x18
#define THUMBNAIL_DEST_PITCH         0x1C

#define TNAXISTS_AXID                0x10000
#define TNAXISTS_AXIR                0x20000


#define to_gxp_video_buffer(x) \
	container_of((x), struct gxp_video_buffer, vb)


enum {
	VIDEO_STREAMING,
	VIDEO_FRAME_INPRG,
	VIDEO_STOPPED,
};

static const struct regmap_config gxp_video_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= THUMBNAIL_DEST_PITCH,
};
struct gxp_video_addr {
	size_t size;
	dma_addr_t dma;
	void *virt;
};


struct gxp_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head link;
};

struct gxp_video {
	struct regmap *vcd_regmap;
	struct device *dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_pix_format pix_fmt;
	struct v4l2_bt_timings active_timings;
	struct v4l2_bt_timings detected_timings;
	u32 v4l2_input_status;
	struct vb2_queue queue;
	struct video_device vdev;
	struct mutex video_lock;	/* v4l2 and videobuf2 lock */

	struct list_head buffers;
	spinlock_t lock;		/* buffer list lock */
	unsigned long flags;
	unsigned int sequence;

	size_t max_buffer_size;
	struct gxp_video_addr src;
	struct reset_control *reset;

	unsigned int frame_rate;
	unsigned int vb_index;
	u32 bytesperline;
	u8 bytesperpixel;
	u8 num_buffers;

};

#define to_gxp_video(x) container_of((x), struct gxp_video, v4l2_dev)

static int gWidth;
static int gHeight;
static int gBpp;

struct gxp_tn_par {
	void __iomem *base;
	void *virt_mem;
	void *phys_mem;
	uint32_t framesize;
	u32 palette[16];
};



static bool gxp_video_alloc_buf(struct gxp_video *video,
				 struct gxp_video_addr *addr, size_t size)
{
	if (size > VCD_MAX_SRC_BUFFER_SIZE)
		size = VCD_MAX_SRC_BUFFER_SIZE;

	addr->virt = dma_alloc_coherent(video->dev, size + sizeof(struct v4l2_rect), &addr->dma,
					GFP_KERNEL);

	if (!addr->virt)
		return false;

	addr->size = size;

	return true;
}

static void gxp_video_free_buf(struct gxp_video *video,
				struct gxp_video_addr *addr)
{
	dma_free_coherent(video->dev, addr->size, addr->virt, addr->dma);
	addr->size = 0;
	addr->dma = 0ULL;
	addr->virt = NULL;
}




static int thumbnail_wait_for_axi(struct gxp_video *video)
{
	uint32_t val = 0;
    int ret;
    struct regmap *vcd = video->vcd_regmap;

	ret = regmap_read_poll_timeout(vcd, THUMBNAIL_TNAXISTS, val,
				       !(val & (TNAXISTS_AXID|TNAXISTS_AXIR)), 100,
				       500000);
	if (ret) {
		dev_err(video->dev, "<TB_VIDEO> Wait for AXI timeout\n");
		return -EBUSY;
	}
    return 0;

}


static void gxpvd_enable(struct gxp_video *video)
{
	struct regmap *vcd = video->vcd_regmap;
    uint32_t reg = 0;
    
	regmap_write(vcd, THUMBNAIL_CFG, 0x00000000);
	thumbnail_wait_for_axi(video);
	if (gBpp == 4){
        regmap_write(vcd, THUMBNAIL_CFG, 0x00000100);
    }
    regmap_write(vcd, THUMBNAIL_HORIZ_SIZE, (gWidth - 1));
    regmap_write(vcd, THUMBNAIL_VERT_SIZE,  (gHeight - 1));
    regmap_write(vcd, THUMBNAIL_DEST_BAR, video->src.dma);
    regmap_write(vcd, THUMBNAIL_DEST_PITCH, (gWidth * gBpp));
	thumbnail_wait_for_axi(video);
    regmap_read(vcd, THUMBNAIL_CFG, &reg);
	regmap_write(vcd, THUMBNAIL_CFG, 0x00000001 | reg);
	udelay(100);


}

 static void gxpvd_disable(struct gxp_video *video)
 {
   struct regmap *vcd = video->vcd_regmap;
   regmap_write(vcd, THUMBNAIL_CFG, 0x00000000);
   return;
 }

static int gxp_video_queue_setup(struct vb2_queue *q,
				  unsigned int *num_buffers,
				  unsigned int *num_planes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct gxp_video *video = vb2_get_drv_priv(q);
	if (*num_planes) {
		if (sizes[0] < video->max_buffer_size)
			return -EINVAL;

		return 0;
	}
	*num_planes = 1;
	sizes[0] = video->max_buffer_size;
	video->num_buffers = *num_buffers;

	return 0;
}

static int gxp_video_buf_prepare(struct vb2_buffer *vb)
{
	struct gxp_video *video = vb2_get_drv_priv(vb->vb2_queue);
	if (vb2_plane_size(vb, 0) < video->max_buffer_size)
		return -EINVAL;
	return 0;
}

static void gxp_video_bufs_done(struct gxp_video *video,
				 enum vb2_buffer_state state)
{
	unsigned long flags;
	struct gxp_video_buffer *buf;

	spin_lock_irqsave(&video->lock, flags);
	list_for_each_entry(buf, &video->buffers, link)
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	INIT_LIST_HEAD(&video->buffers);
	spin_unlock_irqrestore(&video->lock, flags);
}

int alpha = 255;    
int red = 60;
int green = 128;
int blue = 160;
static int iter=0;
#include <linux/random.h>

static int gxp_video_start_frame(struct gxp_video *video)
{
    struct gxp_video_buffer *buf;
	dma_addr_t vb_dma_addr;
    unsigned long flags;
    void *addr;
//	uint32_t *ptr;


    spin_lock_irqsave(&video->lock, flags);
	buf = list_first_entry_or_null(&video->buffers,struct gxp_video_buffer, link);
	if (!buf) {
		spin_unlock_irqrestore(&video->lock, flags);
		dev_info(video->dev, "No empty buffers; skip capture frame\n");
		return -EPROTO;
	}

    set_bit(VIDEO_FRAME_INPRG, &video->flags);
    vb_dma_addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);    
	addr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);

	memcpy(addr, video->src.virt, gHeight*gWidth*gBpp);
//	ptr=(uint32_t *) addr;

//	get_random_bytes(ptr, gHeight*gWidth*gBpp); 


    vb2_set_plane_payload(&buf->vb.vb2_buf, 0, gHeight*gWidth*gBpp) ;    
    buf->vb.vb2_buf.timestamp = ktime_get_ns();
	buf->vb.sequence = video->sequence++;
	buf->vb.field = V4L2_FIELD_NONE;
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
	list_del(&buf->link);
    spin_unlock(&video->lock);
	clear_bit(VIDEO_FRAME_INPRG, &video->flags);
    return 0;
}



static int gxp_video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	int rc;
	struct gxp_video *video = vb2_get_drv_priv(q);

    dev_info(video->dev,"<TB_VIDEO> Start Streaming\n");
	video->sequence = 0;
	gxpvd_enable(video);
    // fill all buffers 
	rc = gxp_video_start_frame(video);
    while (rc==0) {
        rc = gxp_video_start_frame(video);
    }
//    if (rc) {
//		gxp_video_bufs_done(video, VB2_BUF_STATE_QUEUED);
//		return rc;
//	}

	set_bit(VIDEO_STREAMING, &video->flags);
	return 0;
}


static void gxp_video_stop_streaming(struct vb2_queue *q)
{
	struct gxp_video *video = vb2_get_drv_priv(q);
    dev_info(video->dev,"<TB_VIDEO> Stop Streaming\n");

    gxpvd_disable(video);
	clear_bit(VIDEO_STREAMING, &video->flags);
    gxp_video_bufs_done(video, VB2_BUF_STATE_ERROR);

}


static void gxp_video_buf_finish(struct vb2_buffer *vb)
{
	struct gxp_video *video = vb2_get_drv_priv(vb->vb2_queue);
	
    thumbnail_wait_for_axi(video);
	video->vb_index = vb->index;
}

static void gxp_video_get_resolution(struct gxp_video *video)
{
	struct v4l2_bt_timings *act = &video->active_timings;
	struct v4l2_bt_timings *det = &video->detected_timings;
	struct regmap *gfxi;	

	video->v4l2_input_status = 0;

    det->width = gWidth;
    det->height = gHeight;


	if (det->width == 0 || det->height == 0) {
		det->width = MIN_WIDTH;
		det->height = MIN_HEIGHT;
		video->v4l2_input_status = V4L2_IN_ST_NO_SIGNAL;
	}

}

static int gxp_video_start(struct gxp_video *video)
{

    gxp_video_get_resolution(video);
	video->active_timings = video->detected_timings;
    video->max_buffer_size = VCD_MAX_SRC_BUFFER_SIZE;

	if (!gxp_video_alloc_buf(video, &video->src, video->max_buffer_size)){
       dev_err(video->dev,"Failed to allocate DMA \n");
		return -ENOMEM;
    }
	video->pix_fmt.width = video->active_timings.width;
	video->pix_fmt.height = video->active_timings.height;
	video->pix_fmt.sizeimage = video->max_buffer_size;
	video->pix_fmt.bytesperline = video->bytesperline;

	return 0;

}

static void gxp_video_buf_queue(struct vb2_buffer *vb)
{
	struct gxp_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct gxp_video_buffer *nvb = to_gxp_video_buffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&video->lock, flags);
	list_add_tail(&nvb->link, &video->buffers);
	spin_unlock_irqrestore(&video->lock, flags);

    if (test_bit(VIDEO_STREAMING, &video->flags)){
        gxp_video_start_frame(video);
    }
    
}

static void gxp_video_stop(struct gxp_video *video)
{
    set_bit(VIDEO_STOPPED, &video->flags);
	if (video->src.size){
		gxp_video_free_buf(video, &video->src);        

    }  
   dev_info(video->dev,"<TB_VIDEO> Video Stopped\n");

}

static int gxp_video_open(struct file *file)
{
	int rc;
	struct gxp_video *video = video_drvdata(file);

	mutex_lock(&video->video_lock);

	rc = v4l2_fh_open(file);
	if (rc) {
		mutex_unlock(&video->video_lock);
		return rc;
	}

	if (v4l2_fh_is_singular_file(file)){
        dev_info(video->dev,"<TB_VIDEO> Starting Video\n");
		gxp_video_start(video);
    }

	mutex_unlock(&video->video_lock);

	return 0;
}

static int gxp_video_release(struct file *file)
{
	int rc;
	struct gxp_video *video = video_drvdata(file);

	mutex_lock(&video->video_lock);

	if (v4l2_fh_is_singular_file(file)){
        dev_info(video->dev,"<TB_VIDEO> Stopping Video\n");
		gxp_video_stop(video);
    }

	rc = _vb2_fop_release(file, NULL);

	mutex_unlock(&video->video_lock);

	return rc;
}


static const struct v4l2_file_operations gxp_video_v4l2_fops = {
	.owner = THIS_MODULE,
    .open = gxp_video_open,
    .release = gxp_video_release,
    .unlocked_ioctl = video_ioctl2,
    .poll = vb2_fop_poll,
    .mmap = vb2_fop_mmap,
	.read = vb2_fop_read,
};

// Queue operations 

static const struct vb2_ops gxp_video_vb2_ops = {
	.queue_setup = gxp_video_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_prepare = gxp_video_buf_prepare,
	.buf_finish = gxp_video_buf_finish,
	.start_streaming = gxp_video_start_streaming,
	.stop_streaming = gxp_video_stop_streaming,
	.buf_queue =  gxp_video_buf_queue,
};


static int gxp_video_querycap(struct file *file, void *fh,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, DEVICE_NAME, sizeof(cap->driver));
	strscpy(cap->card, "GXP Video Thumbnail Engine", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",DEVICE_NAME);
    return 0;
}

static int gxp_video_enum_format(struct file *file, void *fh,
				  struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_RGB24;

	return 0;
}
static int gxp_video_set_format(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct gxp_video *video = video_drvdata(file);

	if (vb2_is_busy(&video->queue))
		return -EBUSY;

	switch (f->fmt.pix.pixelformat) {
	case V4L2_PIX_FMT_RGB24:		
		break;
	default:
		return -EINVAL;
	}
	video->pix_fmt.pixelformat = f->fmt.pix.pixelformat;

	return 0;
}
static int gxp_video_get_format(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct gxp_video *video = video_drvdata(file);
	
	f->fmt.pix = video->pix_fmt;

	return 0;
}


static const struct v4l2_dv_timings_cap gxp_video_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.min_width = MIN_WIDTH,
		.max_width = MAX_WIDTH,
		.min_height = MIN_HEIGHT,
		.max_height = MAX_HEIGHT,
		.standards = V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT | V4L2_DV_BT_STD_GTF,
		.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE |
				V4L2_DV_BT_CAP_REDUCED_BLANKING |
				V4L2_DV_BT_CAP_CUSTOM,
	},
};

static int gxp_video_query_dv_timings(struct file *file, void *fh,
				       struct v4l2_dv_timings *timings)
{
	struct gxp_video *video = video_drvdata(file);

	gxp_video_get_resolution(video);

	timings->type = V4L2_DV_BT_656_1120;
	timings->bt = video->detected_timings;

	return video->v4l2_input_status ? -ENOLINK : 0;
}
static int gxp_video_enum_dv_timings(struct file *file, void *fh,
				      struct v4l2_enum_dv_timings *timings)
{

    return v4l2_enum_dv_timings_cap(timings, &gxp_video_timings_cap,
					NULL, NULL);
}


static int gxp_video_dv_timings_cap(struct file *file, void *fh,
				     struct v4l2_dv_timings_cap *cap)
{

    *cap = gxp_video_timings_cap;

	return 0;
}

static int gxp_video_set_dv_timings(struct file *file, void *fh,
				     struct v4l2_dv_timings *timings)
{
	struct gxp_video *video = video_drvdata(file);
	if (timings->bt.width == video->active_timings.width &&
	    timings->bt.height == video->active_timings.height)
		return 0;

	if (vb2_is_busy(&video->queue)) {
		dev_err(video->dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	video->active_timings = timings->bt;

	//gxp_video_set_resolution(video);

	video->pix_fmt.width = timings->bt.width;
	video->pix_fmt.height = timings->bt.height;
	video->pix_fmt.sizeimage = video->max_buffer_size;
	video->pix_fmt.bytesperline = video->bytesperline;

	timings->type = V4L2_DV_BT_656_1120;

	return 0;
}



static int gxp_video_get_dv_timings(struct file *file, void *fh,
				     struct v4l2_dv_timings *timings)
{
	struct gxp_video *video = video_drvdata(file);
    dev_info(video->dev,"<TB_VIDEO> GET DV TIMINGS\n");

	timings->type = V4L2_DV_BT_656_1120;
	timings->bt = video->active_timings;

	return 0;
}
static int gxp_video_enum_frameintervals(struct file *file, void *fh,
					  struct v4l2_frmivalenum *fival)
{
	struct gxp_video *video = video_drvdata(file);
    

	if (fival->index)
		return -EINVAL;

	if (fival->width != video->detected_timings.width ||
	    fival->height != video->detected_timings.height)
		return -EINVAL;

	if (fival->pixel_format != V4L2_PIX_FMT_RGB24)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;

	fival->stepwise.min.denominator = MAX_FRAME_RATE;
	fival->stepwise.min.numerator = 1;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.max.numerator = 1;
	fival->stepwise.step = fival->stepwise.max;

	return 0;
}
static int gxp_video_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	struct gxp_video *video = video_drvdata(file);
    
	if (fsize->index)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_RGB24)
		return -EINVAL;

	fsize->discrete.width = video->pix_fmt.width;
	fsize->discrete.height = video->pix_fmt.height;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	return 0;
}

static int gxp_video_get_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	struct gxp_video *video = video_drvdata(file);
    
	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.readbuffers = 3;
	a->parm.capture.timeperframe.numerator = 1;
	if (!video->frame_rate)
		a->parm.capture.timeperframe.denominator = MAX_FRAME_RATE;
	else
		a->parm.capture.timeperframe.denominator = video->frame_rate;

	return 0;
}

static int gxp_video_set_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	unsigned int frame_rate = 0;
    

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.readbuffers = 3;

	if (a->parm.capture.timeperframe.numerator)
		frame_rate = a->parm.capture.timeperframe.denominator /
			     a->parm.capture.timeperframe.numerator;

	if (!frame_rate || frame_rate > MAX_FRAME_RATE) {
		frame_rate = 0;
		a->parm.capture.timeperframe.denominator = MAX_FRAME_RATE;
		a->parm.capture.timeperframe.numerator = 1;
	}

	return 0;
}

static int gxp_video_sub_event(struct v4l2_fh *fh,
				  const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	}

	return v4l2_ctrl_subscribe_event(fh, sub);
}




static int gxp_video_enum_input(struct file *file, void *fh,
				   struct v4l2_input *inp)
{
	struct gxp_video *video = video_drvdata(file);

	if (inp->index)
		return -EINVAL;

	strscpy(inp->name, "Host VGA capture", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	inp->status = video->v4l2_input_status;

	return 0;
}

static int gxp_video_get_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int gxp_video_set_input(struct file *file, void *fh, unsigned int i)
{  
	if (i)
		return -EINVAL;

	return 0;
}



static const struct v4l2_ioctl_ops gxp_video_ioctls = {
	.vidioc_querycap = gxp_video_querycap,

	.vidioc_enum_fmt_vid_cap = gxp_video_enum_format,
	.vidioc_g_fmt_vid_cap = gxp_video_get_format,
	.vidioc_s_fmt_vid_cap = gxp_video_set_format,
	.vidioc_try_fmt_vid_cap = gxp_video_get_format,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_enum_input = gxp_video_enum_input,
	.vidioc_g_input = gxp_video_get_input,
	.vidioc_s_input = gxp_video_set_input,
	.vidioc_g_parm = gxp_video_get_parm,
	.vidioc_s_parm = gxp_video_set_parm,
	.vidioc_enum_framesizes = gxp_video_enum_framesizes,
	.vidioc_enum_frameintervals = gxp_video_enum_frameintervals,
    .vidioc_s_dv_timings = gxp_video_set_dv_timings,

	.vidioc_g_dv_timings = gxp_video_get_dv_timings,
	.vidioc_query_dv_timings = gxp_video_query_dv_timings,
	.vidioc_enum_dv_timings = gxp_video_enum_dv_timings,
	.vidioc_dv_timings_cap = gxp_video_dv_timings_cap,

	.vidioc_subscribe_event = gxp_video_sub_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};



static int gxp_video_setup_video(struct gxp_video *video)
{
	struct v4l2_device *v4l2_dev = &video->v4l2_dev;
	struct video_device *vdev = &video->vdev;
	struct vb2_queue *vbq = &video->queue;
	int rc;

	video->pix_fmt.pixelformat = V4L2_PIX_FMT_RGB24;
	video->pix_fmt.field = V4L2_FIELD_NONE;
	video->pix_fmt.colorspace = V4L2_COLORSPACE_SRGB;
	video->v4l2_input_status = V4L2_IN_ST_NO_SIGNAL;

    video->bytesperline=gWidth * gBpp;
    video->bytesperpixel=gBpp;

    rc = v4l2_device_register(video->dev, v4l2_dev);
	if (rc) {
		dev_err(video->dev, "Failed to register v4l2 device\n");
		return rc;
	}
    vbq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vbq->io_modes = VB2_MMAP;
	vbq->dev = v4l2_dev->dev;
	vbq->lock = &video->video_lock;
	vbq->ops = &gxp_video_vb2_ops;
	vbq->mem_ops = &vb2_dma_contig_memops;
	vbq->drv_priv = video;
	vbq->buf_struct_size = sizeof(struct gxp_video_buffer);
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vbq->min_buffers_needed = 3;

	rc = vb2_queue_init(vbq);
	if (rc) {
		v4l2_device_unregister(v4l2_dev);

		dev_err(video->dev, "Failed to init vb2 queue\n");
		return rc;
	}

	vdev->queue = vbq;
	vdev->fops = &gxp_video_v4l2_fops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;
	vdev->v4l2_dev = v4l2_dev;
	strscpy(vdev->name, DEVICE_NAME, sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_VIDEO;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->release = video_device_release_empty;
	vdev->ioctl_ops = &gxp_video_ioctls;
	vdev->lock = &video->video_lock;

	video_set_drvdata(vdev, video);
	rc = video_register_device(vdev, VFL_TYPE_VIDEO, 0);
	if (rc) {
		vb2_queue_release(vbq);
		v4l2_device_unregister(v4l2_dev);
		dev_err(video->dev, "Failed to register video device\n");
		return rc;
	}

	return 0;
}

static int gxp_video_init(struct gxp_video *video)
{
	int rc;
	struct device *dev = video->dev;

	of_reserved_mem_device_init(dev);

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "Failed to set DMA mask\n");
		of_reserved_mem_device_release(dev);
	}

	return rc;
}




int gxpvd_probe(struct platform_device *pdev)
{
	struct resource *res;
	int err;
	int ret;
	int bits_per_pixel = 0;

    int rc;

    void __iomem *regs;
	struct gxp_video *video = kzalloc(sizeof(*video), GFP_KERNEL);

	if (!video)
		return -ENOMEM;


    // Get thumbnail configuration

    // Default bits-per-pixel = 32 bits
    of_property_read_u32(pdev->dev.of_node, "bits-per-pixel",
                &bits_per_pixel);
    switch (bits_per_pixel) {
    case 32:
        gBpp = 4;
        break;
    default:
        gBpp = 2;
        break;
    }

    // Default resolution 800x600 , max resolution 1024x768
    ret = of_property_read_u32(pdev->dev.of_node, "width", &gWidth);
    if (ret || gWidth > 1024 || gWidth < 0)
        gWidth = 800;

    ret = of_property_read_u32(pdev->dev.of_node, "height", &gHeight);
    if (ret || gHeight > 768 || gHeight < 0)
        gHeight = 600;

    dev_info(video->dev,"<TB_VIDEO> gBpp = %d, gWidth = %d, gHeight = %d\n",
        gBpp, gWidth, gHeight);


    video->frame_rate = MAX_FRAME_RATE;
	video->dev = &pdev->dev;
	spin_lock_init(&video->lock);
	mutex_init(&video->video_lock);
	INIT_LIST_HEAD(&video->buffers);


	// Get register based memory
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs)) {
		pr_err("<TB_VIDEO> io-map memory allocate fail ... ");
		err = PTR_ERR(regs);
		goto out;
	}

	video->vcd_regmap = devm_regmap_init_mmio(&pdev->dev, regs, &gxp_video_regmap_cfg);
	if (IS_ERR(video->vcd_regmap)) {
		pr_err("<TN_VD> Failed to initialize VCD regmap!\n");
		err= PTR_ERR(video->vcd_regmap);
        goto out;
	}

	rc = gxp_video_init(video);
	if (rc)
		return rc;

	rc = gxp_video_setup_video(video);
	if (rc)
		return rc;

	dev_info(video->dev, "GXP video driver probed\n");


// Enable thumbnail
//	gxpvd_enable(info);

	return 0;

out:
	return err;
}


static int gxpvd_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct gxp_video *video = to_gxp_video(v4l2_dev);

	video_unregister_device(&video->vdev);
	vb2_queue_release(&video->queue);

	v4l2_device_unregister(v4l2_dev);


	of_reserved_mem_device_release(dev);

	return 0;}

static const struct of_device_id gxp_video_thumbnail_match[] = {
	{.compatible = "hpe,gxp-video-thumbnail"},
	{ /* null */ },
}
MODULE_DEVICE_TABLE(of, gxp_video_thumbnail_match);

static struct platform_driver gxpvd_driver = {
	.probe		= gxpvd_probe,
	.remove		= gxpvd_remove,
	.driver		= {
		.name		= "gxp-video-thumbnail",
		.of_match_table = gxp_video_thumbnail_match,
	}
};

module_platform_driver(gxpvd_driver);

MODULE_DESCRIPTION("GXP Thumbnail video driver");
MODULE_LICENSE("GPL");
