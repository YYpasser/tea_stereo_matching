#include "../include/camera_utils.hpp"
#include "../include/logger.h"
// DSHOW
#include <strmif.h>
#include <dshow.h>
#include <setupapi.h>
#pragma comment(lib,"strmiids.lib")
#pragma comment(lib,"quartz.lib")
#pragma comment(lib,"Setupapi.lib")
#include <initguid.h> 
#include <devpropdef.h>
#include <devpkey.h>
// MF
#include <mfidl.h>
#pragma comment(lib, "mf")
#include <mfapi.h>
#pragma comment(lib, "Mfplat")
#include <mfobjects.h>
#include <strsafe.h>
#include <mftransform.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <mfmp2dlna.h>
#include <mfplay.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#pragma comment(lib, "Mfreadwrite.lib")
#pragma comment(lib, "mfuuid.lib")
#pragma comment(lib, "evr.lib")
#define MF_E_BUFFERTOOSMALL 0xC00D36B1

class CameraListDSHOW
{
public:
    CameraListDSHOW() {};
    ~CameraListDSHOW() {};
    HRESULT getCameraInfoList(std::vector<camera::CameraInfo>& cameraInfoList);
private:
    HRESULT getCameraSupportFormat(IBaseFilter* pBaseFilter, camera::MediaProperty& list);
    void getDevPathInfo(LPCTSTR pszEnumerator, std::string& locationPath);
    LPCWSTR getGUIDNameConst(const GUID& guid);
    HRESULT getGUIDName(const GUID& guid, WCHAR** ppwsz);
    std::string wchar2String(LPWSTR lpcwszStr);
};

HRESULT CameraListDSHOW::getCameraInfoList(std::vector<camera::CameraInfo>& cameraInfoList)
{
    //-- 初始化当前线程上的 COM 库
    HRESULT hr = CoInitialize(NULL);
    if (FAILED(hr))
        return hr;
    //-- 创建和默认初始化与指定 CLSID 关联的类的单个对象
    ICreateDevEnum* pSysDevEnum = NULL;//-- 设备创建枚举器
    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, reinterpret_cast<void**>(&pSysDevEnum));
    if (FAILED(hr))
    {
        pSysDevEnum->Release();
        return hr;
    }
    //-- 为指定的设备类别创建枚举器
    IEnumMoniker* pEnumCat = NULL;//使用 IEnumMoniker 接口枚举表示设备类别中的筛选器的名字对象
    hr = pSysDevEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &pEnumCat, 0);
    if (FAILED(hr))
    {
        pSysDevEnum->Release();
        return hr;
    }
    if (pEnumCat == NULL)
    {
        pSysDevEnum->Release();
        return hr;
    }
    IMoniker* pMoniker = nullptr;
    ULONG cFetched;
    auto index = 0;
    while (pEnumCat->Next(1, &pMoniker, &cFetched) == S_OK)
    {
        IPropertyBag* pPropBag;
        hr = pMoniker->BindToStorage(nullptr, nullptr, IID_IPropertyBag, (void**)&pPropBag);
        if (SUCCEEDED(hr))
        {
            IBaseFilter* pFilter;
            hr = pMoniker->BindToObject(nullptr, nullptr, IID_IBaseFilter, (void**)&pFilter);
            if (!pFilter)
            {
                pMoniker->Release();
                break;
            }
            camera::CameraInfo info;
            VARIANT varName;
            VariantInit(&varName);
            hr = pPropBag->Read(L"FriendlyName", &varName, 0);
            if (SUCCEEDED(hr))
            {
                hr = getCameraSupportFormat(pFilter, info.prop);
                info.friendlyName = wchar2String(varName.bstrVal);
            }
            VariantClear(&varName);
            VariantInit(&varName);
            hr = pPropBag->Read(L"DevicePath", &varName, 0);
            if (SUCCEEDED(hr))
            {
                info.symbolicLink = wchar2String(varName.bstrVal);

            }
            VariantClear(&varName);

            LPCTSTR pszEnumerator = varName.bstrVal;
            getDevPathInfo(pszEnumerator, info.portLocation);
            pFilter->Release();
            pPropBag->Release();
            cameraInfoList.push_back(info);
        }
        pMoniker->Release();
    }
    pEnumCat->Release();
    return hr;
}

HRESULT CameraListDSHOW::getCameraSupportFormat(IBaseFilter* pBaseFilter, camera::MediaProperty& list)
{
    HRESULT hr = S_OK;
    std::vector<IPin*> pins;
    IEnumPins* EnumPins;
    pBaseFilter->EnumPins(&EnumPins);
    pins.clear();

    for (;;)
    {
        IPin* pin;
        hr = EnumPins->Next(1, &pin, NULL);
        if (hr != S_OK)
        {
            break;
        }
        pins.push_back(pin);
        pin->Release();
    }

    EnumPins->Release();

    PIN_INFO pInfo;
    for (int i = 0; i < pins.size(); i++)
    {
        if (nullptr == pins[i])
        {
            break;
        }
        pins[i]->QueryPinInfo(&pInfo);

        IEnumMediaTypes* emt = NULL;
        pins[i]->EnumMediaTypes(&emt);
        AM_MEDIA_TYPE* pmt;

        for (;;)
        {
            hr = emt->Next(1, &pmt, NULL);
            if (hr != S_OK)
            {
                break;
            }
            if ((pmt->formattype == FORMAT_VideoInfo)
                //&& (pmt->subtype == MEDIASUBTYPE_RGB24)
                && (pmt->cbFormat >= sizeof(VIDEOINFOHEADER))
                && (pmt->pbFormat != NULL))
            {
                VIDEOINFOHEADER* pVIH = (VIDEOINFOHEADER*)pmt->pbFormat;
                //-- 获取当前摄像头支持的帧率
                auto fps = 10000000 / pVIH->AvgTimePerFrame;
                //-- 获取当前摄像头支持的分辨率
                camera::ImageSize reso(pVIH->bmiHeader.biWidth, pVIH->bmiHeader.biHeight);
                list.resolution.push_back(reso);
                list.fps.push_back(fps);
                WCHAR* pGuidValName = NULL;
                hr = getGUIDName(pmt->subtype, &pGuidValName);
                std::string format = wchar2String(pGuidValName);

                list.encoding.push_back(format);
            }

            if (pmt->cbFormat != 0)
            {
                CoTaskMemFree((PVOID)pmt->pbFormat);
                pmt->cbFormat = 0;
                pmt->pbFormat = NULL;
            }
            if (pmt->pUnk != NULL)
            {
                // pUnk should not be used.
                pmt->pUnk->Release();
                pmt->pUnk = NULL;
            }
        }
        break;
        emt->Release();
    }
    return hr;
}

void CameraListDSHOW::getDevPathInfo(LPCTSTR pszEnumerator, std::string& locationPath)
{
    HDEVINFO hDevInfo = SetupDiCreateDeviceInfoList(NULL, NULL);
    if (INVALID_HANDLE_VALUE == hDevInfo)
    {
        return;
    }
    BYTE Buf[1024] = { 0 };
    TCHAR szTemp[MAX_PATH] = { 0 };

    SP_DEVICE_INTERFACE_DATA spdid = { 0 };
    spdid.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);


    PSP_DEVICE_INTERFACE_DETAIL_DATA pspdidd = (PSP_DEVICE_INTERFACE_DETAIL_DATA)Buf;
    pspdidd->cbSize = sizeof(*pspdidd);

    SP_DEVINFO_DATA spdd = { 0 };
    spdd.cbSize = sizeof(spdd);

    DWORD dwSize = 0;
    if (!SetupDiOpenDeviceInterface(
        hDevInfo,
        pszEnumerator,
        0,
        &spdid))
        return;
    dwSize = sizeof(Buf);
    SetupDiGetDeviceInterfaceDetail(hDevInfo, &spdid, pspdidd, dwSize, &dwSize, &spdd);
    //SetupDiClassNameFromGuid(&spdd.ClassGuid, szTemp, MAX_PATH, &dwSize);
    //SetupDiGetClassDescription(&spdd.ClassGuid, szTemp, MAX_PATH, &dwSize);
    //SetupDiGetDeviceRegistryProperty(hDevInfo, &spdd, SPDRP_DEVICEDESC, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);
    //SetupDiGetDeviceRegistryProperty(hDevInfo, &spdd, SPDRP_FRIENDLYNAME, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);

    //? => SPDRP_LOCATION_INFORMATION可以读取, 但是SPDRP_LOCATION_PATHS无法读取
    SetupDiGetDeviceRegistryPropertyW(hDevInfo, &spdd, SPDRP_LOCATION_INFORMATION, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);
    locationPath = wchar2String(szTemp);
    SetupDiDestroyDeviceInfoList(hDevInfo);
}

#ifndef IF_EQUAL_RETURN
#define IF_EQUAL_RETURN(param, val) if(val == param) return L#val
#endif
LPCWSTR CameraListDSHOW::getGUIDNameConst(const GUID& guid)
{
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_AI44); //     FCC('AI44')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_ARGB32); //   D3DFMT_A8R8G8B8 
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_AYUV); //     FCC('AYUV')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_DVSD); //     FCC('dvsd')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_H264); //     FCC('H264')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_IYUV); //     FCC('IYUV')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_MJPG);
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_NV12); //     FCC('NV12')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_P010); //     FCC('P010')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_P016); //     FCC('P016')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_P210); //     FCC('P210')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_P216); //     FCC('P216')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_RGB24); //    D3DFMT_R8G8B8 
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_RGB32); //    D3DFMT_X8R8G8B8 
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_RGB555); //   D3DFMT_X1R5G5B5 
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_RGB565); //   D3DFMT_R5G6B5 
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_RGB8);
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_UYVY); //     FCC('UYVY')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_Y210); //     FCC('Y210')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_Y216); //     FCC('Y216')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_Y41P);
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_YUY2); //     FCC('YUY2')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_YV12); //     FCC('YV12')
    IF_EQUAL_RETURN(guid, MEDIASUBTYPE_YVYU);
    return NULL;
}

HRESULT CameraListDSHOW::getGUIDName(const GUID& guid, WCHAR** ppwsz)
{
    HRESULT hr = S_OK;
    WCHAR* pName = NULL;

    LPCWSTR pcwsz = getGUIDNameConst(guid);
    if (pcwsz)
    {
        size_t cchLength = 0;

        hr = StringCchLength(pcwsz, STRSAFE_MAX_CCH, &cchLength);
        if (FAILED(hr))
        {
            goto done;
        }

        pName = (WCHAR*)CoTaskMemAlloc((cchLength + 1) * sizeof(WCHAR));

        if (pName == NULL)
        {
            hr = E_OUTOFMEMORY;
            goto done;
        }

        hr = StringCchCopy(pName, cchLength + 1, pcwsz);
        if (FAILED(hr))
        {
            goto done;
        }
    }
    else
    {
        hr = StringFromCLSID(guid, &pName);
    }

done:
    if (FAILED(hr))
    {
        *ppwsz = NULL;
        CoTaskMemFree(pName);
    }
    else
    {
        *ppwsz = pName;
    }
    return hr;
}

std::string CameraListDSHOW::wchar2String(LPWSTR lpcwszStr)
{
    std::string str;
    size_t len = WideCharToMultiByte(CP_ACP, 0, lpcwszStr, wcslen(lpcwszStr), NULL, 0, NULL, NULL);
    char* m_char = new char[len + 1];
    WideCharToMultiByte(CP_ACP, 0, lpcwszStr, wcslen(lpcwszStr), m_char, len, NULL, NULL);
    m_char[len] = '\0';
    str = m_char;
    delete[] m_char;
    return str;
}

class CameraListMF
{
public:
    CameraListMF();
    ~CameraListMF();
    HRESULT getCameraInfoList(std::vector<camera::CameraInfo>& cameraInfoList);
private:
    IMFActivate** m_ppDevices;
    UINT32 m_count;
    HRESULT enumerateDevices();
    HRESULT createVideoDeviceSource(UINT32 deviceID, IMFMediaSource** ppSource);
    HRESULT getDeviceName(UINT32 deviceID, std::string& friendlyName, std::string& symbolicLink, std::string& locationPath);
    HRESULT enumerateSupportFormat(IMFMediaSource* pSource, camera::MediaProperty& list);
    void DBGMSG(PCWSTR format, ...);
    HRESULT logMediaType(IMFMediaType* pType, camera::MediaProperty& list);
    HRESULT logAttributeValueByIndex(IMFAttributes* pAttr, DWORD index, camera::MediaProperty& list);
    LPCWSTR getGUIDNameConst(const GUID& guid);
    HRESULT getGUIDName(const GUID& guid, WCHAR** ppwsz);
    std::string wchar2String(LPWSTR lpcwszStr);
    void getDevPathInfo(LPCTSTR pszEnumerator, std::string& locationPath);
};

template <class T> void SafeRelease(T** ppT)
{
    if (*ppT)
    {
        (*ppT)->Release();
        *ppT = NULL;
    }
}

CameraListMF::CameraListMF()
{
    this->m_ppDevices = NULL;
    this->m_count = NULL;
}

CameraListMF::~CameraListMF()
{
}

HRESULT CameraListMF::getCameraInfoList(std::vector<camera::CameraInfo>& cameraInfoList)
{
    HRESULT hr = enumerateDevices();
    for (UINT32 i = 0; i < m_count; ++i)
    {
        IMFMediaSource* pSource;
        camera::CameraInfo info;
        hr = getDeviceName(i, info.friendlyName, info.symbolicLink, info.portLocation);
        hr = createVideoDeviceSource(i, &pSource);
        hr = enumerateSupportFormat(pSource, info.prop);
        cameraInfoList.push_back(info);
    }
    return hr;
}

HRESULT CameraListMF::enumerateDevices()
{
    IMFAttributes* pAttributes = NULL;
    HRESULT hr = CoInitialize(NULL);
    if (FAILED(hr))
    {
        goto done;
    }
    // Create an attribute store to specify the enumeration parameters.
    hr = MFCreateAttributes(&pAttributes, 1);
    if (FAILED(hr))
    {
        goto done;
    }
    // Source type: video capture devices
    hr = pAttributes->SetGUID(
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE,
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID
    );
    if (FAILED(hr))
    {
        goto done;
    }
    // Enumerate devices.
    hr = MFEnumDeviceSources(pAttributes, &m_ppDevices, &m_count);
    if (FAILED(hr))
    {
        goto done;
    }
done:
    SafeRelease(&pAttributes);
    return hr;
}

HRESULT CameraListMF::createVideoDeviceSource(UINT32 deviceID, IMFMediaSource** ppSource)
{
    *ppSource = NULL;
    IMFMediaSource* pSource = NULL;
    HRESULT hr = S_OK;
    hr = m_ppDevices[deviceID]->ActivateObject(IID_PPV_ARGS(&pSource));
    if (FAILED(hr))
    {
        goto done;
    }
    *ppSource = pSource;
    (*ppSource)->AddRef();
done:
    SafeRelease(&pSource);
    return hr;
}

HRESULT CameraListMF::getDeviceName(UINT32 deviceID, std::string& friendlyName, std::string& symbolicLink, std::string& locationPath)
{
    WCHAR* ppwszName;
    //-- 读取友好名称
    HRESULT hr = m_ppDevices[deviceID]->GetAllocatedString(
        MF_DEVSOURCE_ATTRIBUTE_FRIENDLY_NAME,
        &ppwszName,
        NULL
    );
    if (FAILED(hr))
    {
        CoTaskMemFree(ppwszName);
        ppwszName = NULL;
        return hr;
    }
    friendlyName = wchar2String(ppwszName);
    CoTaskMemFree(ppwszName);
    ppwszName = NULL;
    //-- 读取硬件ID
    hr = m_ppDevices[deviceID]->GetAllocatedString(
        MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK,
        &ppwszName,
        NULL
    );
    if (FAILED(hr))
    {
        CoTaskMemFree(ppwszName);
        ppwszName = NULL;
        return hr;
    }
    symbolicLink = wchar2String(ppwszName);
    LPCTSTR pszEnumerator = ppwszName;
    getDevPathInfo(pszEnumerator, locationPath);
    CoTaskMemFree(ppwszName);
    ppwszName = NULL;
    return hr;
}

HRESULT CameraListMF::enumerateSupportFormat(IMFMediaSource* pSource, camera::MediaProperty& list)
{
    IMFPresentationDescriptor* pPD = NULL;
    IMFStreamDescriptor* pSD = NULL;
    IMFMediaTypeHandler* pHandler = NULL;
    IMFMediaType* pType = NULL;

    DWORD cTypes = 0;
    //-- Step.1 创建媒体源的描述符
    HRESULT hr = pSource->CreatePresentationDescriptor(&pPD);
    if (FAILED(hr))
    {
        goto done;
    }
    //-- Step.2 获取视频流的描述符
    BOOL fSelected;
    hr = pPD->GetStreamDescriptorByIndex(0, &fSelected, &pSD);
    if (FAILED(hr))
    {
        goto done;
    }
    //-- Step.3 获取流描述符的媒体类型句柄
    hr = pSD->GetMediaTypeHandler(&pHandler);
    if (FAILED(hr))
    {
        goto done;
    }
    //-- Step.4 获取视频设备支持的格式数量
    hr = pHandler->GetMediaTypeCount(&cTypes);
    if (FAILED(hr))
    {
        goto done;
    }
    //-- Step.5 枚举设备支持的格式
    for (DWORD i = 0; i < cTypes; i++)
    {
        //-- 获取每个格式的属性
        hr = pHandler->GetMediaTypeByIndex(i, &pType);
        if (FAILED(hr))
        {
            goto done;
        }

        logMediaType(pType, list);
        OutputDebugString(L"\n");

        SafeRelease(&pType);
    }

done:
    SafeRelease(&pPD);
    SafeRelease(&pSD);
    SafeRelease(&pHandler);
    SafeRelease(&pType);
    return hr;
}

void CameraListMF::DBGMSG(PCWSTR format, ...)
{
    va_list args;
    va_start(args, format);

    WCHAR msg[MAX_PATH];

    if (SUCCEEDED(StringCbVPrintf(msg, sizeof(msg), format, args)))
    {
        OutputDebugString(msg);
    }
}

HRESULT CameraListMF::logMediaType(IMFMediaType* pType, camera::MediaProperty& list)
{
    UINT32 count = 0;
    //-- 获取属性条目数量
    HRESULT hr = pType->GetCount(&count);
    if (FAILED(hr))
    {
        return hr;
    }
    if (count == 0)
    {
        DBGMSG(L"Empty media type.\n");
        return hr;
    }
    //-- 枚举属性条目
    for (UINT32 i = 0; i < count; i++)
    {
        // 记录属性条目内容
        hr = logAttributeValueByIndex(pType, i, list);
        if (FAILED(hr))
        {
            break;
        }
    }
    return hr;
}

HRESULT CameraListMF::logAttributeValueByIndex(IMFAttributes* pAttr, DWORD index, camera::MediaProperty& list)
{
    WCHAR* pGuidName = NULL;
    WCHAR* pGuidValName = NULL;
    std::string guidName;
    GUID guid = { 0 };

    PROPVARIANT var;
    PropVariantInit(&var);
    //-- 获取条目GUID及值
    HRESULT hr = pAttr->GetItemByIndex(index, &guid, &var);
    if (FAILED(hr))
    {
        goto done;
    }
    //-- 分辨率
    if (guid == MF_MT_FRAME_SIZE)
    {
        UINT32 uHigh = 0, uLow = 0;
        Unpack2UINT32AsUINT64(var.uhVal.QuadPart, &uHigh, &uLow);
        camera::ImageSize reso(uHigh, uLow);
        list.resolution.push_back(reso);
    }
    //-- 帧率
    else if (guid == MF_MT_FRAME_RATE)
    {
        UINT32 uHigh = 0, uLow = 0;
        Unpack2UINT32AsUINT64(var.uhVal.QuadPart, &uHigh, &uLow);
        list.fps.push_back(1.f * uHigh / uLow);
    }
    //-- 压缩格式
    else if (guid == MF_MT_SUBTYPE)
    {
        if (var.vt == VT_CLSID)
        {
            hr = getGUIDName(*var.puuid, &pGuidValName);
            if (SUCCEEDED(hr))
            {
                DBGMSG(pGuidValName);
                guidName = wchar2String(pGuidValName);
                list.encoding.push_back(guidName);
            }
        }
    }

done:
    DBGMSG(L"\n");
    CoTaskMemFree(pGuidName);
    CoTaskMemFree(pGuidValName);
    PropVariantClear(&var);
    return hr;
}

#ifndef IF_EQUAL_RETURN
#define IF_EQUAL_RETURN(param, val) if(val == param) return L#val
#endif
LPCWSTR CameraListMF::getGUIDNameConst(const GUID& guid)
{
    IF_EQUAL_RETURN(guid, MF_MT_MAJOR_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_MAJOR_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_SUBTYPE);
    IF_EQUAL_RETURN(guid, MF_MT_ALL_SAMPLES_INDEPENDENT);
    IF_EQUAL_RETURN(guid, MF_MT_FIXED_SIZE_SAMPLES);
    IF_EQUAL_RETURN(guid, MF_MT_COMPRESSED);
    IF_EQUAL_RETURN(guid, MF_MT_SAMPLE_SIZE);
    IF_EQUAL_RETURN(guid, MF_MT_WRAPPED_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_NUM_CHANNELS);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_SAMPLES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_FLOAT_SAMPLES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_AVG_BYTES_PER_SECOND);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_BLOCK_ALIGNMENT);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_BITS_PER_SAMPLE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_VALID_BITS_PER_SAMPLE);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_SAMPLES_PER_BLOCK);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_CHANNEL_MASK);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_FOLDDOWN_MATRIX);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_PEAKREF);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_PEAKTARGET);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_AVGREF);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_WMADRC_AVGTARGET);
    IF_EQUAL_RETURN(guid, MF_MT_AUDIO_PREFER_WAVEFORMATEX);
    IF_EQUAL_RETURN(guid, MF_MT_AAC_PAYLOAD_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_AAC_AUDIO_PROFILE_LEVEL_INDICATION);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_SIZE);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE_RANGE_MAX);
    IF_EQUAL_RETURN(guid, MF_MT_FRAME_RATE_RANGE_MIN);
    IF_EQUAL_RETURN(guid, MF_MT_PIXEL_ASPECT_RATIO);
    IF_EQUAL_RETURN(guid, MF_MT_DRM_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_PAD_CONTROL_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_SOURCE_CONTENT_HINT);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_CHROMA_SITING);
    IF_EQUAL_RETURN(guid, MF_MT_INTERLACE_MODE);
    IF_EQUAL_RETURN(guid, MF_MT_TRANSFER_FUNCTION);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_PRIMARIES);
    IF_EQUAL_RETURN(guid, MF_MT_CUSTOM_VIDEO_PRIMARIES);
    IF_EQUAL_RETURN(guid, MF_MT_YUV_MATRIX);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_LIGHTING);
    IF_EQUAL_RETURN(guid, MF_MT_VIDEO_NOMINAL_RANGE);
    IF_EQUAL_RETURN(guid, MF_MT_GEOMETRIC_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_MINIMUM_DISPLAY_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_PAN_SCAN_APERTURE);
    IF_EQUAL_RETURN(guid, MF_MT_PAN_SCAN_ENABLED);
    IF_EQUAL_RETURN(guid, MF_MT_AVG_BITRATE);
    IF_EQUAL_RETURN(guid, MF_MT_AVG_BIT_ERROR_RATE);
    IF_EQUAL_RETURN(guid, MF_MT_MAX_KEYFRAME_SPACING);
    IF_EQUAL_RETURN(guid, MF_MT_DEFAULT_STRIDE);
    IF_EQUAL_RETURN(guid, MF_MT_PALETTE);
    IF_EQUAL_RETURN(guid, MF_MT_USER_DATA);
    IF_EQUAL_RETURN(guid, MF_MT_AM_FORMAT_TYPE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG_START_TIME_CODE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_PROFILE);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_LEVEL);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG2_FLAGS);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG_SEQUENCE_HEADER);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_SRC_PACK_0);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_CTRL_PACK_0);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_SRC_PACK_1);
    IF_EQUAL_RETURN(guid, MF_MT_DV_AAUX_CTRL_PACK_1);
    IF_EQUAL_RETURN(guid, MF_MT_DV_VAUX_SRC_PACK);
    IF_EQUAL_RETURN(guid, MF_MT_DV_VAUX_CTRL_PACK);
    IF_EQUAL_RETURN(guid, MF_MT_ARBITRARY_HEADER);
    IF_EQUAL_RETURN(guid, MF_MT_ARBITRARY_FORMAT);
    IF_EQUAL_RETURN(guid, MF_MT_IMAGE_LOSS_TOLERANT);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG4_SAMPLE_DESCRIPTION);
    IF_EQUAL_RETURN(guid, MF_MT_MPEG4_CURRENT_SAMPLE_ENTRY);
    IF_EQUAL_RETURN(guid, MF_MT_ORIGINAL_4CC);
    IF_EQUAL_RETURN(guid, MF_MT_ORIGINAL_WAVE_FORMAT_TAG);

    // Media types

    IF_EQUAL_RETURN(guid, MFMediaType_Audio);
    IF_EQUAL_RETURN(guid, MFMediaType_Video);
    IF_EQUAL_RETURN(guid, MFMediaType_Protected);
    IF_EQUAL_RETURN(guid, MFMediaType_SAMI);
    IF_EQUAL_RETURN(guid, MFMediaType_Script);
    IF_EQUAL_RETURN(guid, MFMediaType_Image);
    IF_EQUAL_RETURN(guid, MFMediaType_HTML);
    IF_EQUAL_RETURN(guid, MFMediaType_Binary);
    IF_EQUAL_RETURN(guid, MFMediaType_FileTransfer);

    IF_EQUAL_RETURN(guid, MFVideoFormat_AI44); //     FCC('AI44')
    IF_EQUAL_RETURN(guid, MFVideoFormat_ARGB32); //   D3DFMT_A8R8G8B8 
    IF_EQUAL_RETURN(guid, MFVideoFormat_AYUV); //     FCC('AYUV')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DV25); //     FCC('dv25')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DV50); //     FCC('dv50')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVH1); //     FCC('dvh1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVSD); //     FCC('dvsd')
    IF_EQUAL_RETURN(guid, MFVideoFormat_DVSL); //     FCC('dvsl')
    IF_EQUAL_RETURN(guid, MFVideoFormat_H264); //     FCC('H264')
    IF_EQUAL_RETURN(guid, MFVideoFormat_I420); //     FCC('I420')
    IF_EQUAL_RETURN(guid, MFVideoFormat_IYUV); //     FCC('IYUV')
    IF_EQUAL_RETURN(guid, MFVideoFormat_M4S2); //     FCC('M4S2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MJPG);
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP43); //     FCC('MP43')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP4S); //     FCC('MP4S')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MP4V); //     FCC('MP4V')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MPG1); //     FCC('MPG1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MSS1); //     FCC('MSS1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_MSS2); //     FCC('MSS2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_NV11); //     FCC('NV11')
    IF_EQUAL_RETURN(guid, MFVideoFormat_NV12); //     FCC('NV12')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P010); //     FCC('P010')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P016); //     FCC('P016')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P210); //     FCC('P210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_P216); //     FCC('P216')
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB24); //    D3DFMT_R8G8B8 
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB32); //    D3DFMT_X8R8G8B8 
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB555); //   D3DFMT_X1R5G5B5 
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB565); //   D3DFMT_R5G6B5 
    IF_EQUAL_RETURN(guid, MFVideoFormat_RGB8);
    IF_EQUAL_RETURN(guid, MFVideoFormat_UYVY); //     FCC('UYVY')
    IF_EQUAL_RETURN(guid, MFVideoFormat_v210); //     FCC('v210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_v410); //     FCC('v410')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV1); //     FCC('WMV1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV2); //     FCC('WMV2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WMV3); //     FCC('WMV3')
    IF_EQUAL_RETURN(guid, MFVideoFormat_WVC1); //     FCC('WVC1')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y210); //     FCC('Y210')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y216); //     FCC('Y216')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y410); //     FCC('Y410')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y416); //     FCC('Y416')
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y41P);
    IF_EQUAL_RETURN(guid, MFVideoFormat_Y41T);
    IF_EQUAL_RETURN(guid, MFVideoFormat_YUY2); //     FCC('YUY2')
    IF_EQUAL_RETURN(guid, MFVideoFormat_YV12); //     FCC('YV12')
    IF_EQUAL_RETURN(guid, MFVideoFormat_YVYU);

    IF_EQUAL_RETURN(guid, MFAudioFormat_PCM); //              WAVE_FORMAT_PCM 
    IF_EQUAL_RETURN(guid, MFAudioFormat_Float); //            WAVE_FORMAT_IEEE_FLOAT 
    IF_EQUAL_RETURN(guid, MFAudioFormat_DTS); //              WAVE_FORMAT_DTS 
    IF_EQUAL_RETURN(guid, MFAudioFormat_Dolby_AC3_SPDIF); //  WAVE_FORMAT_DOLBY_AC3_SPDIF 
    IF_EQUAL_RETURN(guid, MFAudioFormat_DRM); //              WAVE_FORMAT_DRM 
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudioV8); //        WAVE_FORMAT_WMAUDIO2 
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudioV9); //        WAVE_FORMAT_WMAUDIO3 
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMAudio_Lossless); // WAVE_FORMAT_WMAUDIO_LOSSLESS 
    IF_EQUAL_RETURN(guid, MFAudioFormat_WMASPDIF); //         WAVE_FORMAT_WMASPDIF 
    IF_EQUAL_RETURN(guid, MFAudioFormat_MSP1); //             WAVE_FORMAT_WMAVOICE9 
    IF_EQUAL_RETURN(guid, MFAudioFormat_MP3); //              WAVE_FORMAT_MPEGLAYER3 
    IF_EQUAL_RETURN(guid, MFAudioFormat_MPEG); //             WAVE_FORMAT_MPEG 
    IF_EQUAL_RETURN(guid, MFAudioFormat_AAC); //              WAVE_FORMAT_MPEG_HEAAC 
    IF_EQUAL_RETURN(guid, MFAudioFormat_ADTS); //             WAVE_FORMAT_MPEG_ADTS_AAC 

    return NULL;
}

HRESULT CameraListMF::getGUIDName(const GUID& guid, WCHAR** ppwsz)
{
    HRESULT hr = S_OK;
    WCHAR* pName = NULL;

    LPCWSTR pcwsz = getGUIDNameConst(guid);
    if (pcwsz)
    {
        size_t cchLength = 0;

        hr = StringCchLength(pcwsz, STRSAFE_MAX_CCH, &cchLength);
        if (FAILED(hr))
        {
            goto done;
        }

        pName = (WCHAR*)CoTaskMemAlloc((cchLength + 1) * sizeof(WCHAR));

        if (pName == NULL)
        {
            hr = E_OUTOFMEMORY;
            goto done;
        }

        hr = StringCchCopy(pName, cchLength + 1, pcwsz);
        if (FAILED(hr))
        {
            goto done;
        }
    }
    else
    {
        hr = StringFromCLSID(guid, &pName);
    }

done:
    if (FAILED(hr))
    {
        *ppwsz = NULL;
        CoTaskMemFree(pName);
    }
    else
    {
        *ppwsz = pName;
    }
    return hr;
}

std::string CameraListMF::wchar2String(LPWSTR lpcwszStr)
{
    std::string str;
    size_t len = WideCharToMultiByte(CP_ACP, 0, lpcwszStr, wcslen(lpcwszStr), NULL, 0, NULL, NULL);
    char* m_char = new char[len + 1];
    WideCharToMultiByte(CP_ACP, 0, lpcwszStr, wcslen(lpcwszStr), m_char, len, NULL, NULL);
    m_char[len] = '\0';
    str = m_char;
    delete[] m_char;
    return str;
}

void CameraListMF::getDevPathInfo(LPCTSTR pszEnumerator, std::string& locationPath)
{
    HDEVINFO hDevInfo = SetupDiCreateDeviceInfoList(NULL, NULL);
    if (INVALID_HANDLE_VALUE == hDevInfo)
    {
        return;
    }
    BYTE Buf[1024] = { 0 };
    TCHAR szTemp[MAX_PATH] = { 0 };

    SP_DEVICE_INTERFACE_DATA spdid = { 0 };
    spdid.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);


    PSP_DEVICE_INTERFACE_DETAIL_DATA pspdidd = (PSP_DEVICE_INTERFACE_DETAIL_DATA)Buf;
    pspdidd->cbSize = sizeof(*pspdidd);

    SP_DEVINFO_DATA spdd = { 0 };
    spdd.cbSize = sizeof(spdd);

    DWORD dwSize = 0;
    if (!SetupDiOpenDeviceInterface(
        hDevInfo,
        pszEnumerator,
        0,
        &spdid))
        return;
    dwSize = sizeof(Buf);
    SetupDiGetDeviceInterfaceDetail(hDevInfo, &spdid, pspdidd, dwSize, &dwSize, &spdd);
    //SetupDiClassNameFromGuid(&spdd.ClassGuid, szTemp, MAX_PATH, &dwSize);
    //SetupDiGetClassDescription(&spdd.ClassGuid, szTemp, MAX_PATH, &dwSize);
    //SetupDiGetDeviceRegistryProperty(hDevInfo, &spdd, SPDRP_DEVICEDESC, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);
    //SetupDiGetDeviceRegistryProperty(hDevInfo, &spdd, SPDRP_FRIENDLYNAME, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);

    //? => SPDRP_LOCATION_INFORMATION可以读取, 但是SPDRP_LOCATION_PATHS无法读取
    SetupDiGetDeviceRegistryPropertyW(hDevInfo, &spdd, SPDRP_LOCATION_INFORMATION, NULL, (PBYTE)szTemp, MAX_PATH - 1, NULL);
    locationPath = wchar2String(szTemp);
    SetupDiDestroyDeviceInfoList(hDevInfo);
}

camera::CameraList& camera::CameraList::operator=(const camera::CameraList& other)
{
    if (this != &other)
    {
        this->list = other.list;
    }
    return *this;
}

void camera::CameraList::update(const camera::MediaAPI& mediaAPI)
{
    HRESULT hr = S_OK;
    switch (mediaAPI)
    {
    case camera::DSHOW:
    {
        CameraListDSHOW camera;
        hr = camera.getCameraInfoList(this->list);
        break;
    }
    case camera::MSMF:
    {
        CameraListMF camera;
        hr = camera.getCameraInfoList(this->list);
        break;
    }
    default:
        list.clear();
        break;
    }
}

void camera::CameraList::extract(const camera::VideoEncoding& encoding)
{
    for (auto& p : this->list)
        p.extract(encoding);
}

std::vector<camera::CameraInfo> camera::CameraList::getList()
{
    return this->list;
}

void camera::CameraList::print()
{
    LOG_INFO("Found " + std::to_string(this->list.size()) + " camera devices.");
    LOG_INFO("Camera device list:");
    for (auto p = this->list.begin(); p != this->list.end(); ++p)
    {
        auto i = std::distance(this->list.begin(), p);
        LOG_INFO_MSG("["+ std::to_string(i + 1) + "] \"" + p->friendlyName + "\"");
        LOG_INFO_MSG("     device ID = " + std::to_string(i));
        LOG_INFO_MSG("     Symbolic link: " + p->symbolicLink);
        LOG_INFO_MSG("     Device Location: " + p->portLocation);
        LOG_INFO_MSG("     Support resolution with fps: ");
        for (auto it = p->prop.begin(); it != p->prop.end(); ++it)
        {
            auto [res, fps, enc] = *it;
            LOG_INFO_MSG("        " + std::to_string(res->width) + " x " + 
                std::to_string(res->height) + " with " + std::to_string(*fps) + " fps.");
        }
    }
}

