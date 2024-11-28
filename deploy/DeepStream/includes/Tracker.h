//
// Created by Mayur Kulkarni on 11/11/21.
//

#ifndef DNSTARPROD_TRACKER_H
#define DNSTARPROD_TRACKER_H

#include "nvdstracker.h"
#include "BYTETracker.h"
#include <memory>

/**
* @brief Context for input video streams
*
* The stream context holds all necessary state to perform multi-object tracking
* within the stream.
*
*/
class NvMOTContext {
public:
    NvMOTContext(const NvMOTConfig &configIn, NvMOTConfigResponse &configResponse);

    ~NvMOTContext();

    /**
     * @brief Process a batch of frames
     *
     * Internal implementation of NvMOT_Process()
     *
     * @param [in] pParam Pointer to parameters for the frame to be processed
     * @param [out] pTrackedObjectsBatch Pointer to object tracks output
     */
    NvMOTStatus processFrame(const NvMOTProcessParams *params,
                             NvMOTTrackedObjBatch *pTrackedObjectsBatch);

    /**
     * @brief Output the past-frame data if there are
     *
     *  Internal implementation of NvMOT_ProcessPast()
     *
     * @param [in] pParam Pointer to parameters for the frame to be processed
     * @param [out] pPastFrameObjectsBatch Pointer to past frame object tracks output
     */
    NvMOTStatus processFramePast(const NvMOTProcessParams *params,
                                 NvDsPastFrameObjBatch *pPastFrameObjectsBatch);

    /**
     * @brief Terminate trackers and release resources for a stream when the stream is removed
     *
     *  Internal implementation of NvMOT_RemoveStreams()
     *
     * @param [in] streamIdMask removed stream ID
     */
    NvMOTStatus removeStream(const NvMOTStreamId streamIdMask);

    /**
     * Resets the trackedObjectsPool array before every use 
    */
    void resetTrackedObjectsPool();

protected:

    /**
     * Users can include an actual tracker implementation here as a member
     * `IMultiObjectTracker` can be assumed to an user-defined interface class
     */
    std::map<uint64_t, std::shared_ptr<BYTETracker>> byteTrackerMap;

private:
    int frameRate = 5; // Frame Rate 5 FPS 
    int trackBuffer = 90; // Want to keep lost_tracks for 3 seconds. So, 3 * 30 = 90 Frames

    static const int MAX_TRACKED_OBJECTS = 512;
    NvMOTTrackedObj trackedObjectsPool[MAX_TRACKED_OBJECTS];
};

#endif //DNSTARPROD_TRACKER_H
