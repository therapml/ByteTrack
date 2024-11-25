#include "Tracker.h"
#include "BYTETracker.h"
#include <fstream>

NvMOTContext::NvMOTContext(const NvMOTConfig &configIn, NvMOTConfigResponse &configResponse) {
    configResponse.summaryStatus = NvMOTConfigStatus_OK;
}

NvMOTContext::~NvMOTContext() {
    // Clear all trackers
    byteTrackerMap.clear();
}

NvMOTStatus NvMOTContext::processFrame(const NvMOTProcessParams *params, NvMOTTrackedObjBatch *pTrackedObjectsBatch) {
    for (uint streamIdx = 0; streamIdx < pTrackedObjectsBatch->numFilled; streamIdx++){
        NvMOTTrackedObjList   *trackedObjList = &pTrackedObjectsBatch->list[streamIdx];
        NvMOTFrame            *frame          = &params->frameList[streamIdx];
        std::vector<NvObject> nvObjects(frame->objectsIn.numFilled);
        for (uint32_t numObjects = 0; numObjects < frame->objectsIn.numFilled; numObjects++) {
            NvMOTObjToTrack *objectToTrack = &frame->objectsIn.list[numObjects];
            NvObject nvObject;
            nvObject.prob    = objectToTrack->confidence;
            nvObject.label   = objectToTrack->classId;
            nvObject.rect[0] = objectToTrack->bbox.x;
            nvObject.rect[1] = objectToTrack->bbox.y;
            nvObject.rect[2] = objectToTrack->bbox.width;
            nvObject.rect[3] = objectToTrack->bbox.height;
            nvObject.associatedObjectIn = objectToTrack;
            nvObjects.push_back(nvObject);
        }

        if (byteTrackerMap.find(frame->streamID) == byteTrackerMap.end())
            byteTrackerMap.insert(std::pair<uint64_t, std::shared_ptr<BYTETracker>>(frame->streamID, std::make_shared<BYTETracker>(15, 30)));

        std::vector<STrack> outputTracks = byteTrackerMap.at(frame->streamID)->update(nvObjects);

        std::unique_ptr<NvMOTTrackedObj[]> trackedObjs(new NvMOTTrackedObj[512]); // Consider using smart pointers for managing the array
        int             filled       = 0;

        for (STrack &sTrack: outputTracks) {
            std::vector<float> tlwh = sTrack.original_tlwh;
            NvMOTRect motRect{tlwh[0], tlwh[1], tlwh[2], tlwh[3]};
            
            // Directly construct in the array, no need for separate allocation
            trackedObjs[filled].classId = 0;
            trackedObjs[filled].trackingId = (uint64_t)sTrack.track_id;
            trackedObjs[filled].bbox = motRect;
            trackedObjs[filled].confidence = 1;
            trackedObjs[filled].age = (uint32_t)sTrack.tracklet_len;
            trackedObjs[filled].associatedObjectIn = sTrack.associatedObjectIn;
            trackedObjs[filled].associatedObjectIn->doTracking = true;
            filled++;
        }

        trackedObjList->streamID     = frame->streamID;
        trackedObjList->frameNum     = frame->frameNum;
        trackedObjList->valid        = true;
        trackedObjList->list         = trackedObjs.release();  // Transfer ownership
        trackedObjList->numFilled    = filled;
        trackedObjList->numAllocated = 512;
    }
}

NvMOTStatus NvMOTContext::processFramePast(const NvMOTProcessParams *params,
                                           NvDsPastFrameObjBatch *pPastFrameObjectsBatch) {
    return NvMOTStatus_OK;
}

NvMOTStatus NvMOTContext::removeStream(const NvMOTStreamId streamIdMask) {
    if (byteTrackerMap.find(streamIdMask) != byteTrackerMap.end()){
        std::cout << "Removing tracker for stream: " << streamIdMask << std::endl;
        byteTrackerMap.erase(streamIdMask);
    }
    return NvMOTStatus_OK;
}
