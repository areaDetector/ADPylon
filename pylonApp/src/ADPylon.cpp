/*
 * ADPylon.cpp
 *
 * This is a driver for Basler cameras using their Pylon SDK, based on ADSpinnaker driver.
 *
 * Author: Xiaoqiang Wang
 *         Paul Scherrer Institute
 *
 * Created: Janurary 26, 2023
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <algorithm>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <cantProceed.h>
#include <epicsString.h>
#include <epicsExit.h>

#include <ADGenICam.h>

#include <epicsExport.h>
#include "PylonFeature.h"
#include "ADPylon.h"

#define DRIVER_VERSION      1
#define DRIVER_REVISION     0
#define DRIVER_MODIFICATION 0

static const char *driverName = "ADPylon";

typedef enum {
    PYLONPixelConvertNone,
    PYLONPixelConvertMono8,
    PYLONPixelConvertMono16,
    PYLONPixelConvertRGB8,
    PYLONPixelConvertRGB16
} PYLONPixelConvert_t;

typedef enum {
    TimeStampCamera,
    TimeStampEPICS
} PYLONTimeStamp_t;

typedef enum {
    UniqueIdCamera,
    UniqueIdDriver
} PYLONUniqueId_t;

struct {
    Pylon::EPixelType fmt;
    NDColorMode_t colorMode;
    NDDataType_t dataType;
    NDBayerPattern_t bayerFormat;
} pix_lookup[] = {
    { Pylon::PixelType_BayerBG8,    NDColorModeBayer, NDUInt8,  NDBayerBGGR },
    { Pylon::PixelType_BayerGB8,    NDColorModeBayer, NDUInt8,  NDBayerGBRG },
    { Pylon::PixelType_BayerGR8,    NDColorModeBayer, NDUInt8,  NDBayerGRBG },
    { Pylon::PixelType_Mono8,       NDColorModeMono,  NDUInt8,  NDBayerBGGR },
    { Pylon::PixelType_RGB8packed,  NDColorModeRGB1,  NDUInt8,  NDBayerBGGR },
    { Pylon::PixelType_BayerBG16,   NDColorModeBayer, NDUInt16, NDBayerBGGR },
    { Pylon::PixelType_BayerGB16,   NDColorModeBayer, NDUInt16, NDBayerGBRG },
    { Pylon::PixelType_BayerGR16,   NDColorModeBayer, NDUInt16, NDBayerGRBG },
    { Pylon::PixelType_Mono16,      NDColorModeMono,  NDUInt16, NDBayerBGGR },
    { Pylon::PixelType_RGB16packed, NDColorModeRGB1,  NDUInt16, NDBayerBGGR }
};

/** Pylon camera event handler */
class ADPylonCameraEventHandler : public Pylon::CCameraEventHandler
{
public:
    ADPylonCameraEventHandler(ADPylon *driver)
        : Pylon::CCameraEventHandler(), driver_(driver)
    {
    }
    /* Called when a camera event has been received */
    virtual void OnCameraEvent(Pylon::CInstantCamera& /*camera*/, intptr_t userProvidedId, GenApi::INode* /*pNode*/)
    {
        driver_->lock();
        driver_->readEventData((int)userProvidedId);
        driver_->unlock();
    }

private:
    ADPylon *driver_;
};

/** Pylon camera configuration event handler */
class ADPylonConfigurationEventHandler : public Pylon::CConfigurationEventHandler
{
public:
    ADPylonConfigurationEventHandler(ADPylon *driver)
        : Pylon::CConfigurationEventHandler(), driver_(driver)
    {

    }
    /** Called when a camera device removal from the PC has been detected. */
    virtual void OnCameraDeviceRemoved( Pylon::CInstantCamera& /*camera*/ )
    {
        driver_->lock();
        driver_->cameraDisconnected();
        driver_->unlock();
    }
private:
    ADPylon *driver_;
};

/** Pylon image grabber event handler */
class ADPylonImageEventHandler : public Pylon::CImageEventHandler
{
public:
    ADPylonImageEventHandler(ADPylon *driver)
        : Pylon::CImageEventHandler(), driver_(driver)
    {
    }

    /** Called when an image has been grabbed. */
    virtual void OnImageGrabbed( Pylon::CInstantCamera& /*camera*/, const Pylon::CGrabResultPtr& ptrGrabResult )
    {
        driver_->processFrame(ptrGrabResult);
    }

private:
    ADPylon *driver_;
};

/** Configuration function to configure one camera.
 *
 * This function need to be called once for each camera to be used by the IOC. A call to this
 * function instanciates one object from the ADPylon class.
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
extern "C" int ADPylonConfig(const char *portName, const char *cameraId,
                             size_t maxMemory, int priority, int stackSize)
{
    new ADPylon(portName, cameraId, maxMemory, priority, stackSize);
    return asynSuccess;
}


static void imageGrabTaskC(void *drvPvt)
{
    ADPylon *pPvt = (ADPylon *)drvPvt;

    pPvt->imageGrabTask();
}

/** Constructor for the ADPylon class
 * \param[in] portName asyn port name to assign to the camera.
 * \param[in] cameraId The camera index or serial number; <1000 is assumed to be index, >=1000 is assumed to be serial number.
 * \param[in] maxMemory Maximum memory (in bytes) that this driver is allowed to allocate. So if max. size = 1024x768 (8bpp)
 *            and maxBuffers is, say 14. maxMemory = 1024x768x14 = 11010048 bytes (~11MB). 0=unlimited.
 * \param[in] priority The EPICS thread priority for this driver.  0=use asyn default.
 * \param[in] stackSize The size of the stack for the EPICS port thread. 0=use asyn default.
 */
ADPylon::ADPylon(const char *portName, const char *cameraId,
                         size_t maxMemory, int priority, int stackSize )
    : ADGenICam(portName, maxMemory, priority, stackSize),
    cameraId_(cameraId),
    exiting_(false),
    acquiring_(false),
    uniqueId_(0),
    pCameraEventHandler_(new ADPylonCameraEventHandler(this))
{
    static const char *functionName = "ADPylon";
    asynStatus status;
    char tempString[100];

    epicsSnprintf(tempString, sizeof(tempString), "%d.%d.%d",
                  DRIVER_VERSION, DRIVER_REVISION, DRIVER_MODIFICATION);
    setStringParam(NDDriverVersion,tempString);

    Pylon::PylonInitialize();

    setStringParam(ADSDKVersion, Pylon::GetPylonVersionString());

    status = connectCamera();
    if (status) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s:  camera connection failed (%d)\n",
            driverName, functionName, status);
        // Mark camera not reachable
        cameraDisconnected();
        // Call report() to get a list of available cameras
        report(stdout, 1);
        return;
    }

    createParam("PYLON_CONVERT_PIXEL_FORMAT",     asynParamInt32,   &PYLONConvertPixelFormat);
    createParam("PYLON_CONVERT_BIT_ALIGN",        asynParamInt32,   &PYLONConvertBitAlignment);
    createParam("PYLON_CONVERT_SHIFT_BITS",       asynParamInt32,   &PYLONConvertShiftBits);
    createParam("PYLON_TIME_STAMP_MODE",          asynParamInt32,   &PYLONTimeStampMode);
    createParam("PYLON_UNIQUE_ID_MODE",           asynParamInt32,   &PYLONUniqueIdMode);

    /* Set initial values of some parameters */
    setIntegerParam(NDDataType, NDUInt8);
    setIntegerParam(NDColorMode, NDColorModeMono);
    setIntegerParam(NDArraySizeZ, 0);
    setIntegerParam(ADMinX, 0);
    setIntegerParam(ADMinY, 0);
    setStringParam(ADStringToServer, "<not used by driver>");
    setStringParam(ADStringFromServer, "<not used by driver>");

    startEventId_ = epicsEventCreate(epicsEventEmpty);
    newFrameEventId_ = epicsEventCreate(epicsEventEmpty);

    TLStatisticsFeatureNames_.push_back("Statistic_Total_Buffer_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Failed_Buffer_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Buffer_Underrun_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Total_Packet_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Failed_Packet_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Resend_Request_Count");
    TLStatisticsFeatureNames_.push_back("Statistic_Resend_Packet_Count");

    // launch image read task
    epicsThreadCreate("PylonImageTask",
                      epicsThreadPriorityMedium,
                      epicsThreadGetStackSize(epicsThreadStackMedium),
                      imageGrabTaskC, this);

    return;
}

GenICamFeature *ADPylon::createFeature(GenICamFeatureSet *set,
                                       std::string const & asynName, asynParamType asynType, int asynIndex,
                                       std::string const & featureName, GCFeatureType_t featureType) {
    GenApi::INodeMap *nodeMap = nullptr;
    try {
	// Statistics features are from StreamGrabber
        if (std::find(TLStatisticsFeatureNames_.begin(), TLStatisticsFeatureNames_.end(), featureName) != TLStatisticsFeatureNames_.end())
            nodeMap = &camera_.GetStreamGrabberNodeMap();
        else
            nodeMap = &camera_.GetNodeMap();
    } catch (const Pylon::GenericException& /*e*/) {
        // It normally means the camera is not connected.
    }

    return new PylonFeature(set, asynName, asynType, asynIndex, featureName, featureType, nodeMap);
}

/** Called by Pylon when the camera is disconnected.
 *  Here we mark the device unreachable and any further read/write requests will be rejected.
 */
void ADPylon::cameraDisconnected()
{
    this->deviceIsReachable = false;
    this->disconnect(pasynUserSelf);
    setIntegerParam(ADStatus, ADStatusDisconnected);
    setStringParam(ADStatusMessage, "Camera disconnected");

    // Detach and close Pylon device
    camera_.Close();
}

void ADPylon::readEventData(int index)
{
    if (index >= 0 && index < (int)eventList_.size())
        mGCFeatureSet.readFeatures(eventList_[index]);
}

asynStatus ADPylon::connectCamera(void)
{
    static const char *functionName = "connectCamera";

    try {
        // If cameraId_ is a number < 1000, use it as the index
        if (cameraId_.size()<4 && std::all_of(cameraId_.begin(), cameraId_.end(), isdigit)) {
            size_t index = std::stoi(cameraId_);
            Pylon::DeviceInfoList devices;
            Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
            if (index < devices.size()) {
                camera_.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(devices[index]), Pylon::Cleanup_Delete);
            } else {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s index %s >= cameras found %zd\n", driverName, functionName, cameraId_.c_str(), devices.size());
                return asynError;
            }
        } else {
            Pylon::CDeviceInfo deviceInfo;
            deviceInfo.SetSerialNumber(cameraId_.c_str());
            camera_.Attach(Pylon::CTlFactory::GetInstance().CreateDevice(deviceInfo), Pylon::Cleanup_Delete);
        }
        camera_.RegisterImageEventHandler(new ADPylonImageEventHandler(this), Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
        camera_.RegisterConfiguration(new ADPylonConfigurationEventHandler(this), Pylon::RegistrationMode_Append, Pylon::Cleanup_Delete);
        camera_.GrabCameraEvents = true;
        camera_.Open();

        /* Register event handler for all event sources. The handler updates event data Timestamp and FrameID.
           See https://docs.baslerweb.com/event-notification
         */
        Pylon::CEnumParameter eventSelector(camera_.GetNodeMap(), "EventSelector");
        if (eventSelector.IsValid()) {
            Pylon::StringList_t sources;
            eventSelector.GetAllValues(sources);
            for (size_t i=0; i<sources.size(); i++) {
                std::vector<std::string> eventData;
                GenApi::INode *pFeature;
                std::string name;

                name = std::string("Event") + sources[i].c_str() + "Timestamp";
                pFeature = camera_.GetNodeMap().GetNode(name.c_str());
                if (pFeature) { eventData.push_back(name); }

                name = std::string("Event") + sources[i].c_str() + "FrameID";
                pFeature = camera_.GetNodeMap().GetNode(name.c_str());
                if (pFeature) { eventData.push_back(name); }

                camera_.RegisterCameraEventHandler(pCameraEventHandler_,
                    Pylon::String_t("Event") + sources[i].c_str() + "Data",
                    i,
                    Pylon::RegistrationMode_Append,
                    Pylon::Cleanup_None,
                    Pylon::CameraEventAvailability_Optional);

                eventList_.push_back(eventData);
            }
        }
    } catch (const Pylon::GenericException& e) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error opening camera %s: %s\n", driverName, functionName, cameraId_.c_str(), e.GetDescription());
       return asynError;
    }

    return asynSuccess;
}


/** Task to grab images off the camera and send them up to areaDetector
 *
 */

void ADPylon::imageGrabTask()
{
    int acquire;
    int numImages;
    int numImagesCounter;
    int imageMode;
    static const char *functionName = "imageGrabTask";

    lock();

    while (1) {
        // Is acquisition active?
        getIntegerParam(ADAcquire, &acquire);
        // If we are not acquiring then wait for a semaphore that is given when acquisition is started
        if (!acquire) {
            setIntegerParam(ADStatus, ADStatusIdle);
            callParamCallbacks();

            // Wait for a signal that tells this thread that the transmission
            // has started and we can start asking for image buffers...
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s waiting for acquire to start\n",
                driverName, functionName);
            // Release the lock while we wait for an event that says acquire has started, then lock again
            unlock();
            epicsEventWait(startEventId_);
            lock();
            asynPrint(pasynUserSelf, ASYN_TRACE_FLOW,
                "%s::%s started!\n",
                driverName, functionName);
            setIntegerParam(ADNumImagesCounter, 0);
            setIntegerParam(ADAcquire, 1);
        }

        // We are now waiting for an image
        setIntegerParam(ADStatus, ADStatusWaiting);
        // Call the callbacks to update any changes
        callParamCallbacks();

        // Wait for event saying image has been collected
        unlock();
        epicsEventWaitStatus waitStatus = epicsEventWaitWithTimeout(newFrameEventId_, 0.1);
        lock();
        if (waitStatus == epicsEventWaitOK) {
            getIntegerParam(ADNumImages, &numImages);
            getIntegerParam(ADNumImagesCounter, &numImagesCounter);
            getIntegerParam(ADImageMode, &imageMode);
            // See if acquisition is done if we are in single or multiple mode
            if ((imageMode == ADImageSingle) || ((imageMode == ADImageMultiple) && (numImagesCounter >= numImages))) {
                setIntegerParam(ADStatus, ADStatusIdle);
                stopCapture();
            }
        }
        callParamCallbacks();
    }
}

/** Convert Pylon grab result data to areaDetector NDArray
 *
 */
asynStatus ADPylon::processFrame(const Pylon::CGrabResultPtr& pGrabResult)
{
    asynStatus status = asynSuccess;
    uint32_t nRows, nCols;
    NDDataType_t dataType;
    NDColorMode_t colorMode;
    NDBayerPattern_t bayerFormat;
    Pylon::EPixelType pixelType;
    bool pixelTypeSupported = false;
    int convertPixelFormat;
    int convertBitAlignment;
    int convertShiftBits;
    size_t dims[3];
    int nDims;
    int uniqueIdMode;
    int timeStampMode;
    int imageCounter;
    int numImagesCounter;
    int arrayCallbacks;
    Pylon::CompressionInfo_t compressionInfo;
    Pylon::CPylonImage outputImage;
    NDArray *pRaw = NULL;
    static const char *functionName = "processFrame";

    lock();
    if (!pGrabResult->GrabSucceeded()) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s error in grabbing\n",
            driverName, functionName);
        status = asynError;
        goto done;
    }
    nCols = pGrabResult->GetWidth();
    nRows = pGrabResult->GetHeight();
    pixelType = pGrabResult->GetPixelType();
    outputImage.AttachGrabResultBuffer(pGrabResult);

    // Check whether the image was compressed by the camera and is still compressed (could have been decompressed by a transport layer).
    if (decompressor_.GetCompressionInfo(compressionInfo, pGrabResult) && compressionInfo.hasCompressedImage) {
        if (compressionInfo.compressionStatus == Pylon::CompressionStatus_Ok) {
            outputImage.Release();
            decompressor_.DecompressImage(outputImage, pGrabResult);
        } else {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s error in decompression\n",
                driverName, functionName);
            status = asynError;
            goto done;
        }
    }

    // Convert the pixel format if requested
    getIntegerParam(PYLONConvertPixelFormat, &convertPixelFormat);
    getIntegerParam(PYLONConvertBitAlignment, &convertBitAlignment);
    getIntegerParam(PYLONConvertShiftBits, &convertShiftBits);
    if (convertPixelFormat != PYLONPixelConvertNone) {
        Pylon::EPixelType outputPixelType;
        switch (convertPixelFormat) {
            case PYLONPixelConvertMono8:
                outputPixelType = Pylon::PixelType_Mono8;
                break;
            case PYLONPixelConvertMono16:
                outputPixelType = Pylon::PixelType_Mono16;
                break;
            case PYLONPixelConvertRGB8:
                outputPixelType = Pylon::PixelType_RGB8packed;
                break;
            case PYLONPixelConvertRGB16:
                outputPixelType = Pylon::PixelType_RGB16packed;
                break;
            default:
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                    "%s::%s Error: Unknown pixel conversion format %d\n",
                    driverName, functionName, convertPixelFormat);
                outputPixelType = Pylon::PixelType_Mono8;
                break;
        }
        Pylon::CImageFormatConverter converter;
        converter.OutputPixelFormat = outputPixelType;
        converter.OutputBitAlignment = (Pylon::OutputBitAlignmentEnums) convertBitAlignment;
        converter.MonoConversionMethod = Pylon::MonoConversionMethod_Truncate;
        converter.AdditionalLeftShift = convertShiftBits;

        try {
            /* Convert to a temporary image object and reference outputImage to it */
            Pylon::CPylonImage tempImage;
            converter.Convert(tempImage, outputImage);
            pixelType = outputPixelType;
            outputImage = tempImage;
        } catch (const Pylon::GenericException& e) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s::%s error converting, input pixel type=0x%lx, output pixel type=0x%lx: %s\n",
                driverName, functionName, pixelType, outputPixelType, e.GetDescription());
        }
    }

    /* Look up NDArray colorMode, dataType and bayerormat based on Pylon pixelType */
    for (size_t i = 0; i < sizeof(pix_lookup) / sizeof(pix_lookup[0]); i ++) {
        if (pix_lookup[i].fmt == pixelType) {
            colorMode   = (NDColorMode_t)pix_lookup[i].colorMode;
            dataType    = (NDDataType_t)pix_lookup[i].dataType;
            bayerFormat = (NDBayerPattern_t)pix_lookup[i].bayerFormat;
            pixelTypeSupported = true;
            break;
        }
    }
    if (!pixelTypeSupported) {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                "%s:%s: unsupported pixel type=0x%lx\n",
                driverName, functionName, pixelType);
        status = asynError;
        goto done;
    }

    if (colorMode == NDColorModeMono || colorMode == NDColorModeBayer) {
        nDims = 2;
        dims[0] = nCols;
        dims[1] = nRows;
    } else {
        nDims = 3;
        dims[0] = 3;
        dims[1] = nCols;
        dims[2] = nRows;
    }

    pRaw = pNDArrayPool->alloc(nDims, dims, dataType, 0, NULL);
    if (!pRaw) {
        // If we didn't get a valid buffer from the NDArrayPool we must abort
        // the acquisition as we have nowhere to dump the data...
        setIntegerParam(ADStatus, ADStatusAborting);
        callParamCallbacks();
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s [%s] ERROR: Serious problem: not enough buffers left! Aborting acquisition!\n",
            driverName, functionName, portName);
        setIntegerParam(ADAcquire, 0);
        status = asynError;
        goto done;
    }
    if (outputImage.IsValid()) {
        memcpy(pRaw->pData, outputImage.GetBuffer(), pRaw->dataSize);
    } else {
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s::%s [%s] ERROR: image is invalid!\n",
            driverName, functionName, portName);
        status = asynError;
        goto done;
    }

    // Put the frame number into the buffer
    getIntegerParam(PYLONUniqueIdMode, &uniqueIdMode);
    if (uniqueIdMode == UniqueIdCamera) {
        pRaw->uniqueId = (int)pGrabResult->GetID();
    } else {
        pRaw->uniqueId = uniqueId_;
    }
    uniqueId_++;
    updateTimeStamp(&pRaw->epicsTS);
    getIntegerParam(PYLONTimeStampMode, &timeStampMode);
    // Set the timestamps in the buffer
    if (timeStampMode == TimeStampCamera) {
        pRaw->timeStamp = pGrabResult->GetTimeStamp() / 1e9;
    } else {
        pRaw->timeStamp = pRaw->epicsTS.secPastEpoch + pRaw->epicsTS.nsec/1e9;
    }

    // Update NDArray info
    setIntegerParam(NDArraySizeX, (int)nCols);
    setIntegerParam(NDArraySizeY, (int)nRows);
    setIntegerParam(NDArraySize, (int)pRaw->dataSize);
    setIntegerParam(NDDataType, dataType);
    setIntegerParam(NDColorMode, colorMode);
    setIntegerParam(NDBayerPattern, bayerFormat);

    // Extract chunk data as NDArray attributes and optionally set to EPICS database.
    if (pGrabResult->IsChunkDataAvailable()) {
        extractChunkData(pGrabResult->GetChunkDataNodeMap(), pRaw->pAttributeList);
    }

    // Get any attributes that have been defined for this driver
    getAttributes(pRaw->pAttributeList);
    pRaw->pAttributeList->add("ColorMode", "Color mode", NDAttrInt32, &colorMode);
    pRaw->pAttributeList->add("BayerPattern", "Bayer Pattern", NDAttrInt32, &bayerFormat);
    getIntegerParam(NDArrayCounter, &imageCounter);
    getIntegerParam(ADNumImagesCounter, &numImagesCounter);
    imageCounter++;
    numImagesCounter++;
    setIntegerParam(NDArrayCounter, imageCounter);
    setIntegerParam(ADNumImagesCounter, numImagesCounter);
    getIntegerParam(NDArrayCallbacks, &arrayCallbacks);
    if (arrayCallbacks) {
        // Call the NDArray callback
        doCallbacksGenericPointer(pRaw, NDArrayData, 0);
    }

    done:

    for (size_t i=0; i<TLStatisticsFeatureNames_.size(); i++) {
         GenICamFeature *pFeature = mGCFeatureSet.getByName(TLStatisticsFeatureNames_[i]);
         if (pFeature) pFeature->read(0, true);
    }

    callParamCallbacks();
    // Release the NDArray buffer now that we are done with it.
    // After the callback just above we don't need it anymore
    if (pRaw) pRaw->release();
    //pGrabResult.Release();
    epicsEventSignal(newFrameEventId_);
    unlock();
    return status;
}

asynStatus ADPylon::extractChunkData(const GenApi::INodeMap &nodeMap, NDAttributeList *pAttributeList)
{
    const char *functionName = "extractChunkData";

    GenApi::NodeList_t nodes;
    nodeMap.GetNodes(nodes);
    for (auto &node : nodes)
    {
        /* Assume chunk parameter with "Chunk" prefix */
        if (node->GetAccessMode() != GenApi::RO ||
            epicsStrGlobMatch(node->GetName(), "Chunk*") != 1)
            continue;

        try {
            PylonFeature *pFeature = dynamic_cast<PylonFeature *>(mGCFeatureSet.getByName(node->GetName().c_str()));
            switch (node->GetPrincipalInterfaceType()) {
            case GenApi::intfIInteger:
            {
                epicsInt64 ival = dynamic_cast<GenApi::IInteger *>(node)->GetValue();
                pAttributeList->add(node->GetName(), node->GetDisplayName(), NDAttrInt64, &ival);
                if (pFeature) {
                    if (pFeature->getAsynType() == asynParamInt64)
                        setInteger64Param(pFeature->getAsynIndex(), ival);
                    else
                        setIntegerParam(pFeature->getAsynIndex(), (int)ival);
                }
            }
            break;
            case GenApi::intfIFloat:
            {
                epicsFloat64 fval = dynamic_cast<GenApi::IFloat *>(node)->GetValue();
                pAttributeList->add(node->GetName(), node->GetDisplayName(), NDAttrFloat64, &fval);
                if (pFeature) setDoubleParam(pFeature->getAsynIndex(), fval);
            }
            break;
            case GenApi::intfIEnumeration:
            {
                epicsInt64 ival = dynamic_cast<GenApi::IEnumeration *>(node)->GetIntValue();
                pAttributeList->add(node->GetName(), node->GetDisplayName(), NDAttrInt64, &ival);
                if (pFeature) setIntegerParam(pFeature->getAsynIndex(), (int)ival);
            }
            break;
            case GenApi::intfIBoolean:
            {
                epicsUInt8 bval = dynamic_cast<GenApi::IBoolean *>(node)->GetValue();
                pAttributeList->add(node->GetName(), node->GetDisplayName(), NDAttrUInt8, &bval);
                if (pFeature) setIntegerParam(pFeature->getAsynIndex(), bval);
            }
            break;
            case GenApi::intfIString:
            {
                const char *sval = dynamic_cast<GenApi::IString *>(node)->GetValue().c_str();
                pAttributeList->add(node->GetName(), node->GetDisplayName(), NDAttrString, (void*)sval);
                if (pFeature) setStringParam(pFeature->getAsynIndex(), sval);
            }
            break;
            default:
            break;
            }
        } catch (const Pylon::GenericException &e) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s error getting chunk data %s: %s\n",
                      driverName, functionName, node->GetName().c_str(), e.GetDescription());
        }
    }

    return asynSuccess;
}

asynStatus ADPylon::readEnum(asynUser *pasynUser, char *strings[], int values[], int severities[],
                               size_t nElements, size_t *nIn)
{
    int function = pasynUser->reason;
    //static const char *functionName = "readEnum";

    // Some GenICam enum features can become unavailable depending on other settings.
    // If they happend to unavailable on startup, then EPICS records will stay with 0 choices
    // even when later the features become available. So it is best to use the static
    // list of choices from the EPICS database, which is created from the camera's GenICam XML specification.
    GenICamFeature *pFeature = mGCFeatureSet.getByIndex(function);
    if (pFeature && pFeature->getFeatureName() == "LineSource") {
        return asynError;
    }

    return ADGenICam::readEnum(pasynUser, strings, values, severities, nElements, nIn);
}

asynStatus ADPylon::startCapture()
{
    static const char *functionName = "startCapture";
    int imageMode, numImages;

    // If we are already acquiring return immediately
    if (acquiring_) return asynSuccess;

    // Update Decompressor and ignore exceptions if compression is not supported or configured.
    try {
        decompressor_.SetCompressionDescriptor(camera_.GetNodeMap());
    } catch (const Pylon::GenericException& /*e*/) {
    }

    getIntegerParam(ADImageMode, &imageMode);
    getIntegerParam(ADNumImages, &numImages);

    // Start the camera transmission...
    setIntegerParam(ADNumImagesCounter, 0);
    setShutter(1);
    try {
        if (imageMode == ADImageSingle)
            camera_.StartGrabbing(1, Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
        else if (imageMode == ADImageMultiple)
            camera_.StartGrabbing(numImages, Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
        else if (imageMode == ADImageContinuous)
            camera_.StartGrabbing(Pylon::GrabStrategy_OneByOne, Pylon::GrabLoop_ProvidedByInstantCamera);
        else
            return asynError;
    } catch (const Pylon::GenericException& e) {
        setIntegerParam(ADAcquire, 0);
        asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
            "%s:%s: failed to start grabbing: %s\n",
            driverName, functionName, e.what());
        return asynError;
    }

    acquiring_ = true;
    epicsEventSignal(startEventId_);
    return asynSuccess;
}


asynStatus ADPylon::stopCapture()
{
    int status;
    //static const char *functionName = "stopCapture";

    setIntegerParam(ADAcquire, 0);
    setShutter(0);
    // Need to wait for the task to set the status to idle
    while (1) {
        getIntegerParam(ADStatus, &status);
        if (status == ADStatusIdle) break;
        unlock();
        epicsThreadSleep(.1);
        lock();
    }
    // Release the driver lock here.
    unlock();
    camera_.StopGrabbing();
    lock();
    acquiring_ = false;
    return asynSuccess;
}


void ADPylon::report(FILE *fp, int details)
{
    int numCameras;
    int i;
    //static const char *functionName = "report";

    try {
        Pylon::DeviceInfoList_t devices;
        Pylon::CTlFactory::GetInstance().EnumerateDevices(devices);
        numCameras = (int) devices.size();
        fprintf(fp, "\nNumber of cameras detected: %d\n", numCameras);
        if (details >= 1) {
            for (i=0; i<numCameras; i++) {
                const Pylon::CDeviceInfo& deviceInfo  = devices[i];
                fprintf(fp, "Camera %d\n", i);
                fprintf(fp, "            Name: %s\n", deviceInfo.GetFriendlyName().c_str());
                fprintf(fp, "           Model: %s\n", deviceInfo.GetModelName().c_str());
                fprintf(fp, "        Serial #: %s\n", deviceInfo.GetSerialNumber().c_str());
                fprintf(fp, "    Interface ID: %s\n", deviceInfo.GetInterfaceID().c_str());
            }
        }
    } catch (const Pylon::GenericException& /*e*/) {

    }

    ADGenICam::report(fp, details);
    return;
}


static const iocshArg configArg0 = {"Port name", iocshArgString};
static const iocshArg configArg1 = {"cameraId", iocshArgString};
static const iocshArg configArg2 = {"maxMemory", iocshArgInt};
static const iocshArg configArg3 = {"priority", iocshArgInt};
static const iocshArg configArg4 = {"stackSize", iocshArgInt};
static const iocshArg * const configArgs[] = {&configArg0,
                                              &configArg1,
                                              &configArg2,
                                              &configArg3,
                                              &configArg4};
static const iocshFuncDef configADPylon = {"ADPylonConfig", 5, configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    ADPylonConfig(args[0].sval, args[1].sval, args[2].ival,
                  args[3].ival, args[4].ival);
}


static void ADPylonRegister(void)
{
    iocshRegister(&configADPylon, configCallFunc);
}

extern "C" {
epicsExportRegistrar(ADPylonRegister);
}
