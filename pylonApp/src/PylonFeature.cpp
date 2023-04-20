#include <PylonFeature.h>

static const char *driverName="PylonFeature";

PylonFeature::PylonFeature(GenICamFeatureSet *set,
                           std::string const & asynName, asynParamType asynType, int asynIndex,
                           std::string const & featureName, GCFeatureType_t featureType,
                           const GenApi::INodeMap* nodeMap)
         : GenICamFeature(set, asynName, asynType, asynIndex, featureName, featureType),
         mAsynUser(set->getUser()),mFeaturePtr(nullptr),mIsImplemented(false)
{
    static const char *functionName = "PylonFeature";

    if (!nodeMap)
        return;

    try {
        mFeaturePtr = nodeMap->GetNode(featureName.c_str());
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
                "%s::%s error in get feature featurename=%s: %s\n",
                driverName, functionName, featureName.c_str(), e.GetDescription());
        mIsImplemented = false;
        return;
    }
    if (mFeaturePtr) {
        mIsImplemented = true;
        GenApi::EInterfaceType dataType = mFeaturePtr->GetPrincipalInterfaceType();
        GCFeatureType_t GCFeatureType;
        switch (dataType) {
            case GenApi::intfIInteger:
                GCFeatureType = GCFeatureTypeInteger;
                break;
            case GenApi::intfIFloat:
                GCFeatureType = GCFeatureTypeDouble;
                break;
            case GenApi::intfIEnumeration:
                GCFeatureType = GCFeatureTypeEnum;
                break;
            case GenApi::intfIString:
                GCFeatureType = GCFeatureTypeString;
                break;
            case GenApi::intfIBoolean:
                GCFeatureType = GCFeatureTypeBoolean;
                break;
            case GenApi::intfICommand:
                GCFeatureType = GCFeatureTypeCmd;
                break;
            default:
                GCFeatureType = GCFeatureTypeUnknown;
                break;
        }
        if (mFeatureType == GCFeatureTypeUnknown) {
            mFeatureType = GCFeatureType;
        } else {
            if (featureType != GCFeatureType) {
                asynPrint(mAsynUser, ASYN_TRACE_ERROR,
                    "%s::%s error input feature type=%d != Pylon feature type=%d for featurename=%s\n",
                    driverName, functionName, featureType, GCFeatureType, featureName.c_str());
            }
        }
        if (mFeatureType == GCFeatureTypeUnknown) {
            asynPrint(mAsynUser, ASYN_TRACE_ERROR,
                "%s::%s error unknown feature type for featureName=%s\n",
                driverName, functionName, featureName.c_str());
        }
    } else {
        mIsImplemented = false;
    }
}

bool PylonFeature::isImplemented() {
    return mIsImplemented;
}

bool PylonFeature::isAvailable() {
    // Pylon does not support isAvailable.  We simulate it by checking if it is readable or writable.
    if (!mIsImplemented) return false;
    return GenApi::IsAvailable(mFeaturePtr);
}

bool PylonFeature::isReadable() {
    if (!mIsImplemented) return false;
    return GenApi::IsReadable(mFeaturePtr);
}

bool PylonFeature::isWritable() {
    if (!mIsImplemented) return false;
    return GenApi::IsWritable(mFeaturePtr);
}

epicsInt64 PylonFeature::readInteger() {
    if (!mIsImplemented) return 0;
    GenApi::IInteger *pValue = dynamic_cast<GenApi::IInteger *>(mFeaturePtr);
    if (!pValue) return 0;
    return pValue->GetValue();
}

epicsInt64 PylonFeature::readIntegerMin() {
    if (!mIsImplemented) return 0;
    GenApi::IInteger *pValue = dynamic_cast<GenApi::IInteger *>(mFeaturePtr);
    if (!pValue) return 0;
    return pValue->GetMin();
}

epicsInt64 PylonFeature::readIntegerMax() {
    if (!mIsImplemented) return 0;
    GenApi::IInteger *pValue = dynamic_cast<GenApi::IInteger *>(mFeaturePtr);
    if (!pValue) return 0;
    return pValue->GetMax();
}

epicsInt64 PylonFeature::readIncrement() {
    if (!mIsImplemented) return 0;
    GenApi::IInteger *pValue = dynamic_cast<GenApi::IInteger *>(mFeaturePtr);
    if (!pValue) return 0;
    return pValue->GetInc();
}

void PylonFeature::writeInteger(epicsInt64 value) {
    const char *functionName = "writeInteger";
    if (!mIsImplemented) return;
    GenApi::IInteger *pValue = dynamic_cast<GenApi::IInteger *>(mFeaturePtr);
    if (!pValue) return;
    try {
        pValue->SetValue(value);
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

bool PylonFeature::readBoolean() {
    if (!mIsImplemented) return false;
    GenApi::IBoolean *pValue = dynamic_cast<GenApi::IBoolean *>(mFeaturePtr);
    if (!pValue) return false;
    return pValue->GetValue();
}

void PylonFeature::writeBoolean(bool value) {
    const char *functionName = "writeBoolean";
    if (!mIsImplemented) return;
    GenApi::IBoolean *pValue = dynamic_cast<GenApi::IBoolean *>(mFeaturePtr);
    if (!pValue) return;
    try {
        pValue->SetValue(value);
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

double PylonFeature::readDouble() {
    if (!mIsImplemented) return 0.0;
    GenApi::IFloat *pValue = dynamic_cast<GenApi::IFloat *>(mFeaturePtr);
    if (!pValue) return 0.0;
    return pValue->GetValue();
}

void PylonFeature::writeDouble(double value) {
    const char *functionName = "writeDouble";
    if (!mIsImplemented) return;
    GenApi::IFloat *pValue = dynamic_cast<GenApi::IFloat *>(mFeaturePtr);
    if (!pValue) return;
    try {
        pValue->SetValue(value);
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

double PylonFeature::readDoubleMin() {
    if (!mIsImplemented) return 0.0;
    GenApi::IFloat *pValue = dynamic_cast<GenApi::IFloat *>(mFeaturePtr);
    if (!pValue) return 0.0;
    return pValue->GetMin();
}

double PylonFeature::readDoubleMax() {
    if (!mIsImplemented) return 0.0;
    GenApi::IFloat *pValue = dynamic_cast<GenApi::IFloat *>(mFeaturePtr);
    if (!pValue) return 0.0;
    return pValue->GetMax();
}

int PylonFeature::readEnumIndex() {
    if (!mIsImplemented) return 0;
    GenApi::IEnumeration *pValue = dynamic_cast<GenApi::IEnumeration *>(mFeaturePtr);
    if (!pValue) return 0;
    return (int)pValue->GetIntValue();
}

void PylonFeature::writeEnumIndex(int value) {
    static const char *functionName = "writeEnumIndex";
    if (!mIsImplemented) return;
    GenApi::IEnumeration *pValue = dynamic_cast<GenApi::IEnumeration *>(mFeaturePtr);
    if (!pValue) return;
    try {
        pValue->SetIntValue(value);
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

std::string PylonFeature::readEnumString() {
    return "";
}

void PylonFeature::writeEnumString(std::string const &value) {
}

std::string PylonFeature::readString() {
    if (!mIsImplemented) return "";
    GenApi::IString *pValue = dynamic_cast<GenApi::IString *>(mFeaturePtr);
    if (!pValue) return 0;
    return pValue->GetValue().c_str();
}

void PylonFeature::writeString(std::string const & value) {
    static const char *functionName = "writeString";
    if (!mIsImplemented) return;
    GenApi::IString *pValue = dynamic_cast<GenApi::IString *>(mFeaturePtr);
    if (!pValue) return;
    try {
        pValue->SetValue(value.c_str());
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

void PylonFeature::writeCommand() {
    const char *functionName = "writeCommand";
    if (!mIsImplemented) return;
    GenApi::ICommand *pValue = dynamic_cast<GenApi::ICommand *>(mFeaturePtr);
    try {
        pValue->Execute();
    } catch (const Pylon::GenericException& e) {
        asynPrint(mAsynUser, ASYN_TRACE_ERROR,
            "%s::%s %s: %s\n",
            driverName, functionName, mFeatureName.c_str(), e.GetDescription());
    }
}

void PylonFeature::readEnumChoices(std::vector<std::string>& enumStrings, std::vector<int>& enumValues) {
    if (!mIsImplemented) return;
    GenApi::IEnumeration *pValue = dynamic_cast<GenApi::IEnumeration *>(mFeaturePtr);
    if (!pValue) return;

    GenApi::NodeList_t entries;
    pValue->GetEntries(entries);
    for (int i=0; i<(int)entries.size(); i++) {
        GenApi::IEnumEntry *entry = dynamic_cast<GenApi::IEnumEntry*>(entries[i]);
        enumStrings.push_back(entry->GetSymbolic().c_str());
        enumValues.push_back((int)entry->GetValue());
    }
}

