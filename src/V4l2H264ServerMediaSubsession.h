#ifndef V4L2_H264_SERVER_MEDIA_SUBSESSON_H
#define V4L2_H264_SERVER_MEDIA_SUBSESSON_H
#include <liveMedia/OnDemandServerMediaSubsession.hh>
#include <string>

class V4l2H264ServerMediaSubsession : public OnDemandServerMediaSubsession
{
public:
    static V4l2H264ServerMediaSubsession* createNew(UsageEnvironment& env,char const* dev,
                                Boolean reuseFirstSource,
                                int width=640,
                                int height=480,
                                int fps=15,
                                portNumBits initialPortNum = 6970,
                                Boolean multiplexRTCPWithRTP = False);

    // Used to implement "getAuxSDPLine()":
    void checkForAuxSDPLine1();
    void afterPlayingDummy1();
protected:
    V4l2H264ServerMediaSubsession(UsageEnvironment& env,char const* dev,
                                Boolean reuseFirstSource,
                                int width,
                                int height,
                                int fps,
                                portNumBits initialPortNum = 6970,
                                Boolean multiplexRTCPWithRTP = False);
    // called only by createNew();
    virtual ~V4l2H264ServerMediaSubsession();

    void setDoneFlag() { fDoneFlag = ~0; }
protected: // redefined virtual functions
    virtual char const* getAuxSDPLine(RTPSink* rtpSink,
                    FramedSource* inputSource);
    virtual FramedSource* createNewStreamSource(unsigned clientSessionId,
                            unsigned& estBitrate);
    virtual RTPSink* createNewRTPSink(Groupsock* rtpGroupsock,
                                    unsigned char rtpPayloadTypeIfDynamic,
                    FramedSource* inputSource);
private:
    char* fAuxSDPLine;
    char fDoneFlag; // used when setting up "fAuxSDPLine"
    RTPSink* fDummyRTPSink; // ditto

    int mWidth;
    int mHeight;
    int mFps;
    std::string mDev;
};

#endif