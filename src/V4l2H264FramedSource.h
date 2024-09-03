#ifndef V4L2_H264_FRAMED_SOURCE_H
#define V4L2_H264_FRAMED_SOURCE_H
#include <liveMedia/liveMedia.hh>
#include <x264.h>
#include <queue>
#include <mutex>
#include <string>
#include "V4l2.h"
// extern "C" {
// #include "libavcodec/avcodec.h"
// #include "libavformat/avformat.h"
// #include "libavfilter/avfilter.h"
// #include "libswscale/swscale.h"
// #include "libavutil/pixfmt.h"
// #include "libavutil/imgutils.h"
// }
#define DEFAULT_FRAME_NUM 4
// class Camera;
class Frame{
public:
    Frame(int size) :
        mSize(size),
        mFrame(new uint8_t[size]),
        mFrameSize(0),
        mDurationInMicroseconds(0)
    {}
    Frame(){}
    ~Frame(){delete[] mFrame;}

    uint8_t* mFrame;
    int mFrameSize;
    timeval mTime;
    uint32_t mDurationInMicroseconds;
private:
    int mSize;
};
struct Nalu
{
    Nalu(uint8_t* data, uint32_t size, timeval time, uint32_t duration) :
        mData(data), mSize(size), mTime(time), mDurationInMicroseconds(duration)
        {  }

    uint8_t* mData;
    uint32_t mSize;
    timeval mTime;
    uint32_t mDurationInMicroseconds;
};
class V4l2H264FramedSource : public FramedSource
{
public:
    static V4l2H264FramedSource* createNew(UsageEnvironment& env, const char* dev,
                                                int width=640, int height=480, int fps=15,uint32_t pixelFmt=V4L2_PIX_FMT_YUYV);

    bool mThreadRun;
protected:
    V4l2H264FramedSource(UsageEnvironment& env, const char* dev,
                                                int width, int height, int fps,uint32_t pixelFmt);
    ~V4l2H264FramedSource();

    // redefined virtual functions:
    virtual void doGetNextFrame();
    virtual void doStopGettingFrames();

private:
    bool x264Init();
    void x264Exit();
    bool videoInit();
    void videoExit();
    bool getFrame(Frame* frame);

    bool encode(struct v4l2_buf_unit* v4l2BufUnit, Frame* frame);
    // bool encode(AVFrame* avFrame,Frame* frame);

    Frame* getFrameFromOutputQueue();
    void putFrameIntoInputQueue(Frame* frame);

    static void afterGetNextFrame(void* data);
    static void* inputCallback(void* arg);
    void handleTask();
private:
    std::string mDev;
    int mFd;
    int mWidth;
    int mHeight;
    int mFps;
    uint32_t mPixelFmt;
    
    x264_nal_t* mNals;
	x264_t* mX264Handle;
	x264_picture_t* mPicIn;
	x264_picture_t* mPicOut;
	x264_param_t* mParam;

    Frame* mFrames[DEFAULT_FRAME_NUM];
    std::queue<Frame*> mInputFrameQueue;
    std::queue<Frame*> mOutputFrameQueue;
    std::mutex m_mtx;
    std::mutex m_queueMtx;

    TaskToken mTaskToken;
    TaskToken mNextTaskToken;
    int mPts;

    Frame* mSPSFrame;
    Frame* mPPSFrame;

    std::queue<Nalu> mNaluQueue;

    struct v4l2_buf* mV4l2Buf;
    struct v4l2_buf_unit* mV4l2BufUnit;
    // Camera* mCamera;
    // AVFrame* mAvFrame;
    pthread_t mThreadId;
    pthread_mutex_t mMutex;
    
};

#endif