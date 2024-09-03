#include "V4l2H264FramedSource.h"
#include "thread_pool.h"
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
int64_t getCurTime()
{
    struct timespec now;// tv_sec (s) tv_nsec (ns-纳秒)
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec*1000 + now.tv_nsec/1000000);
}
// static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
// {
//     AVFrame *picture;
//     int ret;

//     picture = av_frame_alloc();
//     if (!picture)
//         return NULL;

//     picture->format = pix_fmt;
//     picture->width  = width;
//     picture->height = height;

//     /* allocate the buffers for the frame data */
//     ret = av_frame_get_buffer(picture, 0);
//     if (ret < 0) {
//         fprintf(stderr, "Could not allocate frame data.\n");
//         exit(1);
//     }

//     return picture;
// }
static inline int startCode3(uint8_t* buf)
{
    if(buf[0] == 0 && buf[1] == 0 && buf[2] == 1)
        return 1;
    else
        return 0;
}

static inline int startCode4(uint8_t* buf)
{
    if(buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 1)
        return 1;
    else
        return 0;
}

V4l2H264FramedSource* V4l2H264FramedSource::createNew(UsageEnvironment& env, const char* dev,
                                                int width, int height, int fps,uint32_t pixelFmt)

{
    return new V4l2H264FramedSource(env,dev,width,height,fps,pixelFmt);
}

V4l2H264FramedSource::V4l2H264FramedSource(UsageEnvironment& env, const char* dev,
                                                int width, int height, int fps,uint32_t pixelFmt) :
    FramedSource(env),
    mDev(dev),mWidth(width),mHeight(height),mFps(fps),mPixelFmt(pixelFmt),mPts(0),mThreadRun(false)
{
    // mCamera = new Camera(4);
    // mAvFrame = alloc_picture(AV_PIX_FMT_YUV420P, mWidth, mHeight);
    bool ret;
    ret = videoInit();
    assert(ret == true);
    // mCamera->startSteam();
    ret = x264Init();
    assert(ret == true);

    mSPSFrame = new Frame(100);
    mPPSFrame = new Frame(100);
    for(int i = 0;i<DEFAULT_FRAME_NUM;i++)
    {
        mFrames[i] = new Frame(1000000);
        mInputFrameQueue.push(mFrames[i]);
    }
    // for(int i = 0;i<DEFAULT_FRAME_NUM;i++)
    // {
    // //TODO 开四个任务取四个inputframe处理后塞到outputframe
    //     // ThreadPool::instance()->addTask(inputCallback,this);
    // }
    pthread_create(&mThreadId, NULL, inputCallback, this);
}

V4l2H264FramedSource::~V4l2H264FramedSource()
{
    //TODO
    printf("~V4l2H264FramedSource\n");
    envir().taskScheduler().unscheduleDelayedTask(mNextTaskToken);
    v4l2_streamoff(mFd);

    v4l2_munmap(mFd, mV4l2Buf);


    v4l2_relbufs(mV4l2Buf);


    v4l2_close(mFd);
    x264Exit();
    mThreadRun = false;
    pthread_join(mThreadId, NULL);
}

void V4l2H264FramedSource::afterGetNextFrame(void* data)
{
    V4l2H264FramedSource* source = (V4l2H264FramedSource*)data;
    source->doGetNextFrame();
}

void V4l2H264FramedSource::doGetNextFrame()
{
    // printf("doGetNextFrame\n");
    Frame* frame = getFrameFromOutputQueue();
    if(!frame)
    {
        // ThreadPool::instance()->addTask(inputCallback,this);

        mNextTaskToken = envir().taskScheduler().scheduleDelayedTask(10*1000, afterGetNextFrame, this);
        // handleClosure();
        return ;
    }
    //TODO
    // printf("doGetNextFrame\n");
    if(frame->mFrameSize > fMaxSize)
    {
        fFrameSize = fMaxSize;
        fNumTruncatedBytes = frame->mFrameSize - fMaxSize;
    }
    else
    {
        fFrameSize = frame->mFrameSize;
        fNumTruncatedBytes = 0;
    }

    fPresentationTime = frame->mTime;
    fDurationInMicroseconds = frame->mDurationInMicroseconds;

    memcpy(fTo, frame->mFrame, fFrameSize);

    //最后
    putFrameIntoInputQueue(frame);
    FramedSource::afterGetting(this);
}

void V4l2H264FramedSource::doStopGettingFrames()
{
    printf("doStopGettingFrames\n");
    if(!mThreadRun)
        return ;
    mThreadRun = false;
    pthread_join(mThreadId, NULL);
}

Frame* V4l2H264FramedSource::getFrameFromOutputQueue()
{
    std::lock_guard <std::mutex> lck(m_mtx);
    if(mOutputFrameQueue.empty())
    {
        //TODO 添加一个线程
        return NULL;
    }

    Frame* frame =  mOutputFrameQueue.front();
    mOutputFrameQueue.pop();
    return frame;
}

void V4l2H264FramedSource::putFrameIntoInputQueue(Frame* frame)
{
    std::lock_guard <std::mutex> lck(m_mtx);
    mInputFrameQueue.push(frame);
    //TODO 开一个子线程处理
    // ThreadPool::instance()->addTask(inputCallback,this);
}

bool V4l2H264FramedSource::x264Init()
{
    int mCsp;
    switch (mPixelFmt)
    {
    case V4L2_PIX_FMT_YUYV:
        mCsp = X264_CSP_I422;
        break;
    
    default:
        envir()<<"can't support this fmt\n";
        return false;
    }
    // mCsp = X264_CSP_I420;
    mNals = NULL;
    mX264Handle = NULL;
    mPicIn = new x264_picture_t;
    mPicOut = new x264_picture_t;
    mParam = new x264_param_t;

    x264_param_default(mParam);
    mParam->i_threads = X264_SYNC_LOOKAHEAD_AUTO;
    mParam->i_width = mWidth;
    mParam->i_height = mHeight;
    mParam->rc.i_lookahead = 0;
    // mParam->i_keyint_max = mFps;
    mParam->i_fps_num = mFps;
    mParam->i_fps_den = 1;
    mParam->i_csp = mCsp;

x264_param_apply_profile(mParam,x264_profile_names[4]);
    mX264Handle = x264_encoder_open(mParam);
    if(!mX264Handle)
    {
        envir()<<"failed to open x264 encoder\n";
        return false;
    }

    x264_picture_init(mPicOut);
    x264_picture_alloc(mPicIn, mCsp, mWidth, mHeight);
    // mPicIn->img.plane[0] = mAvFrame->data[0];
    // mPicIn->img.plane[1] = mAvFrame->data[1];
    // mPicIn->img.plane[2] = mAvFrame->data[2];
    return true;
}
void V4l2H264FramedSource::x264Exit()
{
    x264_picture_clean(mPicIn);
    x264_encoder_close(mX264Handle);

    delete mPicIn;
    delete mPicOut;
    delete mParam;
}
bool V4l2H264FramedSource::videoInit()
{
    // return mCamera->initDev(mDev.c_str(),mWidth,mHeight);
    int ret;
    char devName1[100];
    struct v4l2_capability cap;

    mFd = v4l2_open(mDev.c_str(), O_RDWR);
    if(mFd < 0)
        return false;

    ret = v4l2_querycap(mFd, &cap);
    if(ret < 0)
        return false;

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        return false;
    
    // ret = v4l2_enuminput(mFd, 0, devName1);
    // if(ret < 0)
    //     return false;

    // ret = v4l2_s_input(mFd, 0);
    // if(ret < 0)
    //     return false;
    
    // ret = v4l2_enum_fmt(mFd, mPixelFmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    // if(ret < 0)
    //     return false;
    
    ret = v4l2_s_fmt(mFd, &mWidth, &mHeight, mPixelFmt, V4L2_BUF_TYPE_VIDEO_CAPTURE);
    if(ret < 0)
        return false;
    
    mV4l2Buf = v4l2_reqbufs(mFd, V4L2_BUF_TYPE_VIDEO_CAPTURE, 4);
    if(!mV4l2Buf)
        return false;
    
    ret = v4l2_querybuf(mFd, mV4l2Buf);
    if(ret < 0)
        return false;
    
    ret = v4l2_mmap(mFd, mV4l2Buf);
    if(ret < 0)
        return false;
    
    ret = v4l2_qbuf_all(mFd, mV4l2Buf);
    if(ret < 0)
        return false;

    ret = v4l2_streamon(mFd);
    if(ret < 0)
        return false;
    
    ret = v4l2_poll(mFd);
    if(ret < 0)
        return false;
    return true;
}
void V4l2H264FramedSource::videoExit()
{
    int ret;

    ret = v4l2_streamoff(mFd);
    if(ret < 0)
        return ;

    ret = v4l2_munmap(mFd, mV4l2Buf);
    if(ret < 0)
        return ;

    ret = v4l2_relbufs(mV4l2Buf);
    if(ret < 0)
        return ;

    v4l2_close(mFd);
    // mCamera->stopStream();
    // mCamera->freeSource();
}
bool V4l2H264FramedSource::getFrame(Frame* frame)
{
    std::lock_guard <std::mutex> lck(m_queueMtx);
    if(mNaluQueue.empty())
        return false;
    
    Nalu nalu = mNaluQueue.front();
    mNaluQueue.pop();
    memcpy(frame->mFrame, nalu.mData, nalu.mSize);
    frame->mTime = nalu.mTime;
    frame->mDurationInMicroseconds = nalu.mDurationInMicroseconds;
    frame->mFrameSize = nalu.mSize;

    return true;
}

bool V4l2H264FramedSource::encode(struct v4l2_buf_unit* v4l2BufUnit, Frame* frame)
{
    std::lock_guard <std::mutex> lck(m_queueMtx);
    int nalNum, startCode;
    uint8_t byte;
    // switch(mPixelFmt)
    // {
    // case V4L2_PIX_FMT_YUYV:
    //     memcpy(mPicIn->img.plane[0], v4l2BufUnit->start, v4l2BufUnit->length);   
    //     break;
    
    // default:
    //     return false;
    // }

    char* y=(char*)mPicIn->img.plane[0];
    char* u=(char*)mPicIn->img.plane[1];
    char* v=(char*)mPicIn->img.plane[2];
    int index_y=0,index_u=0,index_v=0;
    int num = mWidth * mHeight*2-4;
    for(int i=0;i<num;i=i+4)
    {
        *(y + (index_y++)) = *(char*)(v4l2BufUnit->start + i);
        *(u + (index_u++)) = *(char*)(v4l2BufUnit->start + i + 1);
        *(y + (index_y++)) = *(char*)(v4l2BufUnit->start + i + 2);
        *(v + (index_v++)) = *(char*)(v4l2BufUnit->start + i + 3);
    }
    mPicIn->i_type = X264_TYPE_AUTO;
    mPicIn->i_pts = mPts++;
    int ret;
    int64_t tmp = getCurTime();
    printf("before encode %lld\n",tmp);
    ret = x264_encoder_encode(mX264Handle, &mNals, &nalNum, mPicIn, mPicOut);
    int64_t tmp1 = getCurTime();
    printf("after encode %lld delta %lld\n",tmp1,tmp1-tmp);

    // printf("x264_encoder_encode ret %d num %d\n",ret,nalNum);

    if(nalNum <= 0)
    {
        printf("x264_encoder_encode failed\n");
        return false;
    }
    timeval time;
    gettimeofday(&time, NULL);
    for(int i = 0; i < nalNum; ++i)
    {
        if(startCode3(mNals[i].p_payload))
            startCode = 3;
        else
            startCode = 4;
        byte = *(mNals[i].p_payload+startCode);
        if((byte&0x1F) == 7) //sps
        {
            // printf("encode sps\n");
            memcpy(mSPSFrame->mFrame, mNals[i].p_payload+startCode, mNals[i].i_payload-startCode);
            mSPSFrame->mFrameSize = mNals[i].i_payload-startCode;
            mSPSFrame->mDurationInMicroseconds = 0;
            mSPSFrame->mTime = time;
        }
        else if((byte&0x1F) == 8) //pps
        {
            // printf("encode pps\n");
            memcpy(mPPSFrame->mFrame, mNals[i].p_payload+startCode, mNals[i].i_payload-startCode);
            mPPSFrame->mFrameSize = mNals[i].i_payload-startCode;
            mPPSFrame->mDurationInMicroseconds = 0;
            mPPSFrame->mTime = time;
        }
        else //I Frame and other
        {
            if((byte&0x1F) == 5)
            {
                // printf("encode i frame\n");
                mNaluQueue.push(Nalu(mSPSFrame->mFrame, mSPSFrame->mFrameSize, time, 0));
                mNaluQueue.push(Nalu(mPPSFrame->mFrame, mPPSFrame->mFrameSize, time, 0));
            }
            mNaluQueue.push(Nalu(mNals[i].p_payload+startCode, mNals[i].i_payload-startCode, time, 1000000/mFps));
            // printf("mNaluQueue.push\n");
        }
    }

    Nalu nalu = mNaluQueue.front();
    mNaluQueue.pop();
    memcpy(frame->mFrame, nalu.mData, nalu.mSize);
    frame->mFrameSize = nalu.mSize;
    frame->mTime = nalu.mTime;
    frame->mDurationInMicroseconds = nalu.mDurationInMicroseconds;

    return true;
}

// bool V4l2H264FramedSource::encode(AVFrame* avFrame,Frame* frame)
// {
//     int nalNum, startCode;
//     uint8_t byte;
//     printf("linesize0 %d\n",avFrame->linesize[0]);
//     printf("linesize1 %d\n",avFrame->linesize[1]);
//     printf("linesize2 %d\n",avFrame->linesize[2]);
//     memcpy(mPicIn->img.plane[0], avFrame->data[0], avFrame->linesize[0]); 
//     memcpy(mPicIn->img.plane[1], avFrame->data[1], avFrame->linesize[1]); 
//     memcpy(mPicIn->img.plane[2], avFrame->data[2], avFrame->linesize[2]); 
//     mPicIn->i_pts = mPts++;
//     x264_encoder_encode(mX264Handle, &mNals, &nalNum, mPicIn, mPicOut);
//     if(nalNum <= 0)
//     {
//         printf("x264_encoder_encode failed\n");
//         return false;
//     }
//     timeval time;
//     gettimeofday(&time, NULL);
//     for(int i = 0; i < nalNum; ++i)
//     {
//         if(startCode3(mNals[i].p_payload))
//             startCode = 3;
//         else
//             startCode = 4;
//         byte = *(mNals[i].p_payload+startCode);
//         if((byte&0x1F) == 7) //sps
//         {
//             memcpy(mSPSFrame->mFrame, mNals[i].p_payload+startCode, mNals[i].i_payload-startCode);
//             mSPSFrame->mFrameSize = mNals[i].i_payload-startCode;
//             mSPSFrame->mDurationInMicroseconds = 0;
//             mSPSFrame->mTime = time;
//         }
//         else if((byte&0x1F) == 8) //pps
//         {
//             memcpy(mPPSFrame->mFrame, mNals[i].p_payload+startCode, mNals[i].i_payload-startCode);
//             mPPSFrame->mFrameSize = mNals[i].i_payload-startCode;
//             mPPSFrame->mDurationInMicroseconds = 0;
//             mPPSFrame->mTime = time;
//         }
//         else //I Frame and other
//         {
//             if((byte&0x1F) == 5)
//             {
//                 mNaluQueue.push(Nalu(mSPSFrame->mFrame, mSPSFrame->mFrameSize, time, 0));
//                 mNaluQueue.push(Nalu(mPPSFrame->mFrame, mPPSFrame->mFrameSize, time, 0));
//             }
//             mNaluQueue.push(Nalu(mNals[i].p_payload+startCode, mNals[i].i_payload-startCode, time, 1000000/mFps));
//         }
//     }

//     Nalu nalu = mNaluQueue.front();
//     mNaluQueue.pop();
//     memcpy(frame->mFrame, nalu.mData, nalu.mSize);
//     frame->mFrameSize = nalu.mSize;
//     frame->mTime = nalu.mTime;
//     frame->mDurationInMicroseconds = nalu.mDurationInMicroseconds;

//     return true;
// }

void* V4l2H264FramedSource::inputCallback(void* arg)
{
    V4l2H264FramedSource* source = (V4l2H264FramedSource*)arg;
    if(!source)
        return NULL ;
    source->mThreadRun = true; 
    source->handleTask();
    return NULL;
}

void V4l2H264FramedSource::handleTask()
{
    while(mThreadRun)
    {
        std::lock_guard <std::mutex> lck(m_mtx);
        if(mInputFrameQueue.empty())
            continue ;
        Frame* frame = mInputFrameQueue.front();
        // int ret = getFrame(frame);
        // if(ret)
        // {
        //     mInputFrameQueue.pop();
        //     mOutputFrameQueue.push(frame);
        //     return;
        // }
        // if(!mCamera->readFrame(mAvFrame,AV_PIX_FMT_YUV420P))
        //     return ;
        // if(!encode(mAvFrame,frame))
        //     return ;
        // mInputFrameQueue.pop();
        // mOutputFrameQueue.push(frame);

        int ret = getFrame(frame);
        if(ret)
        {
            mInputFrameQueue.pop();
            mOutputFrameQueue.push(frame);
            continue;;
        }
        // printf("getFrame no frame\n");
        //TODO拿相机数据 然后填充frame
        ret = v4l2_poll(mFd);
        if(ret < 0)
            continue; ;
        mV4l2BufUnit = v4l2_dqbuf(mFd, mV4l2Buf);
        // printf("v4l2_dqbuf size %d\n",mV4l2BufUnit->length);
        ret = encode(mV4l2BufUnit, frame);
        v4l2_qbuf(mFd, mV4l2BufUnit);
        if(ret)
        {
            mInputFrameQueue.pop();
            mOutputFrameQueue.push(frame);
            continue;
        }
    }

}