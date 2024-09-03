#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string>
#include <BasicUsageEnvironment/BasicUsageEnvironment.hh>
#include <liveMedia/liveMedia.hh>
#include <groupsock/GroupsockHelper.hh>

#include "V4l2H264ServerMediaSubsession.h"
#include "V4l2H264FramedSource.h"
RTPSink* videoSink;
H264VideoStreamDiscreteFramer* videoSource;
UsageEnvironment* env;
void play(); // forward
int main(int argc, char* argv[])
{
    int port = 8554;
    bool multicast = true;
    int videoWidth = 960;
    int videoHeight = 720;
    int videoFps = 25;
    std::string videoDev = "/dev/video0";

    TaskScheduler* scheduler = BasicTaskScheduler::createNew();
    env = BasicUsageEnvironment::createNew(*scheduler);
    RTSPServer* rtspServer = RTSPServer::createNew(*env, port);
    if (rtspServer == NULL)
    {
        *env << "Failed to create RTSP server: " << env->getResultMsg() << "\n";
        return -1;
    }
      struct sockaddr_storage destinationAddress;
  destinationAddress.ss_family = AF_INET;
  ((struct sockaddr_in&)destinationAddress).sin_addr.s_addr = chooseRandomIPv4SSMAddress(*env);
    const unsigned short rtpPortNum = 18888;
  const unsigned short rtcpPortNum = rtpPortNum+1;
  const unsigned char ttl = 255;
    const Port rtpPort(rtpPortNum);
  const Port rtcpPort(rtcpPortNum);
    Groupsock rtpGroupsock(*env, destinationAddress, rtpPort, ttl);
  rtpGroupsock.multicastSendOnly(); // we're a SSM source
  Groupsock rtcpGroupsock(*env, destinationAddress, rtcpPort, ttl);
  rtcpGroupsock.multicastSendOnly(); // we're a SSM source
    OutPacketBuffer::maxSize = 100000;
  videoSink = H264VideoRTPSink::createNew(*env, &rtpGroupsock, 96);
    const unsigned estimatedSessionBandwidth = 10000; // in kbps; for RTCP b/w share
  const unsigned maxCNAMElen = 100;
  unsigned char CNAME[maxCNAMElen+1];
  gethostname((char*)CNAME, maxCNAMElen);
  CNAME[maxCNAMElen] = '\0'; // just in case
  RTCPInstance* rtcp
  = RTCPInstance::createNew(*env, &rtcpGroupsock,
			    estimatedSessionBandwidth, CNAME,
			    videoSink, NULL /* we're a server */,
			    True /* we're a SSM source */);
  // Note: This starts RTCP running automatically
    ServerMediaSession* sms;
    if(multicast)
    {
        sms = ServerMediaSession::createNew(*env, "live", "live rtsp server",
                "live rtsp server",True /*SSM*/);
        sms->addSubsession(PassiveServerMediaSubsession::createNew(*videoSink, rtcp));
    }
    else
    {
        sms = ServerMediaSession::createNew(*env, "live", "live rtsp server",
                                                "live rtsp server");
        sms->addSubsession(V4l2H264ServerMediaSubsession::createNew(*env, videoDev.c_str(),
                                    True, videoWidth, videoHeight, videoFps));
    }
    rtspServer->addServerMediaSession(sms);

    char* url = rtspServer->rtspURL(sms);
    *env << "Play this stream using the URL \"" << url << "\"\n";
    delete[] url;
    if(multicast)
    {
        *env << "Beginning streaming...\n";
        play();
    }
    env->taskScheduler().doEventLoop();

    return 0;
}

void afterPlaying(void* /*clientData*/) {
  *env << "...done reading from file\n";
  videoSink->stopPlaying();
  Medium::close(videoSource);
  // Note that this also closes the input file that this source read from.

  // Start playing once again:
  play();
}

void play() {
  // Open the input file as a 'byte-stream file source':
//   ByteStreamFileSource* fileSource
//     = ByteStreamFileSource::createNew(*env, inputFileName);
    V4l2H264FramedSource* fileSource = V4l2H264FramedSource::createNew(*env,"/dev/video0");
  if (fileSource == NULL) {
    *env << "\" V4l2H264FramedSource::createNew failed\n";
    exit(1);
  }

  FramedSource* videoES = fileSource;

  // Create a framer for the Video Elementary Stream:
  videoSource = H264VideoStreamDiscreteFramer::createNew(*env, videoES);

  // Finally, start playing:
  *env << "Beginning to read from file...\n";
  videoSink->startPlaying(*videoSource, afterPlaying, videoSink);
}
