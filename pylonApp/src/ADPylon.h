#ifndef ADPYLON_H
#define ADPYLON_H

#include <string>
#include <vector>

#include <epicsEvent.h>
#include <epicsMessageQueue.h>

#include <ADGenICam.h>
#include <PylonFeature.h>

#include <pylon/PylonIncludes.h>

#define PYLONConvertPixelFormatString  "PYLON_CONVERT_PIXEL_FORMAT"   // asynParamInt32, R/W
#define PYLONTimeStampModeString       "PYLON_TIME_STAMP_MODE"        // asynParamInt32, R/O
#define PYLONUniqueIdModeString        "PYLON_UNIQUE_ID_MODE"         // asynParamInt32, R/O

/** Main driver class inherited from areaDetectors ADGenICam class.
 * One instance of this class will control one camera.
 */
class ADPylon : public ADGenICam
{
public:
    ADPylon(const char *portName, const char *cameraId,
            size_t maxMemory, int priority, int stackSize);

    // virtual methods to override from ADGenICam
    //virtual asynStatus writeInt32( asynUser *pasynUser, epicsInt32 value);
    //virtual asynStatus writeFloat64( asynUser *pasynUser, epicsFloat64 value);
    //virtual asynStatus readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
    //                            size_t nElements, size_t *nIn);
    virtual void report(FILE *fp, int details);
    virtual GenICamFeature *createFeature(GenICamFeatureSet *set,
                                          std::string const & asynName, asynParamType asynType, int asynIndex,
                                          std::string const & featureName, GCFeatureType_t featureType);
    virtual asynStatus startCapture();
    virtual asynStatus stopCapture();

    /* These should be private but are called from C callback functions, must be public. */
    void imageGrabTask();
    asynStatus processFrame(const Pylon::CGrabResultPtr& pGrabResult);
    void cameraDisconnected();

protected:
    asynStatus extractChunkData(const GenApi::INodeMap& nodeMap, NDAttributeList *pAttributeList);

private:
    int PYLONConvertPixelFormat;
#define FIRST_PYLON_PARAM PYLONConvertPixelFormat;
    int PYLONConvertBitAlignment;
    int PYLONConvertShiftBits;
    int PYLONTimeStampMode;
    int PYLONUniqueIdMode;

    /* Local methods to this class */
    asynStatus connectCamera();

    std::string cameraId_;
    Pylon::CInstantCamera camera_;
    Pylon::CImageDecompressor decompressor_;

    bool exiting_;
    bool acquiring_;
    epicsEventId startEventId_;
    epicsEventId newFrameEventId_;
    int uniqueId_;

    std::vector<std::string> TLStatisticsFeatureNames_;

};

#endif

