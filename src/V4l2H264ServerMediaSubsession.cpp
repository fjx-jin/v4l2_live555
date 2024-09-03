#include "V4l2H264ServerMediaSubsession.h"
#include "V4l2H264FramedSource.h"

V4l2H264ServerMediaSubsession* V4l2H264ServerMediaSubsession::createNew(UsageEnvironment& env,char const* dev,
                                Boolean reuseFirstSource,
                                int width,
                                int height,
                                int fps,
                                portNumBits initialPortNum,
                                Boolean multiplexRTCPWithRTP)
{
    return new V4l2H264ServerMediaSubsession(env,dev,reuseFirstSource,width,height,fps,initialPortNum,multiplexRTCPWithRTP);
}

V4l2H264ServerMediaSubsession::V4l2H264ServerMediaSubsession(UsageEnvironment& env,char const* dev,
                                Boolean reuseFirstSource,
                                int width,
                                int height,
                                int fps,
                                portNumBits initialPortNum,
                                Boolean multiplexRTCPWithRTP) :
    OnDemandServerMediaSubsession(env,reuseFirstSource,initialPortNum,multiplexRTCPWithRTP),
    mWidth(width),
    mHeight(height),
    mFps(fps),mDev(dev),
    fAuxSDPLine(NULL),fDoneFlag(0),fDummyRTPSink(NULL)

{
    
}

V4l2H264ServerMediaSubsession::~V4l2H264ServerMediaSubsession()
{
    delete[] fAuxSDPLine;
}

static void afterPlayingDummy(void* clientData) {
  V4l2H264ServerMediaSubsession* subsess = (V4l2H264ServerMediaSubsession*)clientData;
  subsess->afterPlayingDummy1();
}

void V4l2H264ServerMediaSubsession::afterPlayingDummy1() {
  // Unschedule any pending 'checking' task:
  envir().taskScheduler().unscheduleDelayedTask(nextTask());
  // Signal the event loop that we're done:
  setDoneFlag();
}

static void checkForAuxSDPLine(void* clientData) {
  V4l2H264ServerMediaSubsession* subsess = (V4l2H264ServerMediaSubsession*)clientData;
  subsess->checkForAuxSDPLine1();
}

void V4l2H264ServerMediaSubsession::checkForAuxSDPLine1() {
  nextTask() = NULL;

  char const* dasl;
  if (fAuxSDPLine != NULL) {
    // Signal the event loop that we're done:
    setDoneFlag();
  } else if (fDummyRTPSink != NULL && (dasl = fDummyRTPSink->auxSDPLine()) != NULL) {
    fAuxSDPLine = strDup(dasl);
    fDummyRTPSink = NULL;

    // Signal the event loop that we're done:
    setDoneFlag();
  } else if (!fDoneFlag) {
    // try again after a brief delay:
    int uSecsToDelay = 100000; // 100 ms
    nextTask() = envir().taskScheduler().scheduleDelayedTask(uSecsToDelay,
			      (TaskFunc*)checkForAuxSDPLine, this);
  }
}

char const* V4l2H264ServerMediaSubsession::getAuxSDPLine(RTPSink* rtpSink, FramedSource* inputSource) {
  if (fAuxSDPLine != NULL) return fAuxSDPLine; // it's already been set up (for a previous client)

  if (fDummyRTPSink == NULL) { // we're not already setting it up for another, concurrent stream
    // Note: For H264 video files, the 'config' information ("profile-level-id" and "sprop-parameter-sets") isn't known
    // until we start reading the file.  This means that "rtpSink"s "auxSDPLine()" will be NULL initially,
    // and we need to start reading data from our file until this changes.
    fDummyRTPSink = rtpSink;

    // Start reading the file:
    fDummyRTPSink->startPlaying(*inputSource, afterPlayingDummy, this);

    // Check whether the sink's 'auxSDPLine()' is ready:
    checkForAuxSDPLine(this);
  }

  envir().taskScheduler().doEventLoop(&fDoneFlag);

  return fAuxSDPLine;
}

FramedSource* V4l2H264ServerMediaSubsession::createNewStreamSource(unsigned /*clientSessionId*/, unsigned& estBitrate) {
//   estBitrate = 500; // kbps, estimate
  printf("createNewStreamSource\n");
  // Create the video source: 
    V4l2H264FramedSource* source = V4l2H264FramedSource::createNew(envir(),mDev.c_str(),mWidth,mHeight,mFps);

  // Create a framer for the Video Elementary Stream:
  return H264VideoStreamDiscreteFramer::createNew(envir(), source);
}

RTPSink* V4l2H264ServerMediaSubsession
::createNewRTPSink(Groupsock* rtpGroupsock,
		   unsigned char rtpPayloadTypeIfDynamic,
		   FramedSource* /*inputSource*/) {
  return H264VideoRTPSink::createNew(envir(), rtpGroupsock, rtpPayloadTypeIfDynamic);
}