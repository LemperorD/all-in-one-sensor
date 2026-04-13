#pragma once
#include <iostream>
#include <filesystem>

#ifdef _WIN32
#include <windows.h>
std::vector<std::string> list_com_ports() {
    std::vector<std::string> ports;
    HKEY hKey;
    if (RegOpenKeyEx(HKEY_LOCAL_MACHINE,
                     _T("HARDWARE\\DEVICEMAP\\SERIALCOMM"),
                     0, KEY_READ, &hKey) == ERROR_SUCCESS)
    {
        TCHAR valueName[256];
        BYTE data[256];
        DWORD valueNameSize, dataSize, type;
        for (DWORD i = 0;; ++i) {
            valueNameSize = 256; dataSize = 256;
            if (RegEnumValue(hKey, i, valueName, &valueNameSize, NULL, &type, data, &dataSize) != ERROR_SUCCESS)
                break;
            ports.push_back(std::string(reinterpret_cast<char*>(data), dataSize - 1));
        }
        RegCloseKey(hKey);
    }
    return ports;
}
#endif

class serial_device
{
private:
    /* data */
public:
    serial_device(/* args */);
    ~serial_device();
    std::vector<std::string> list_devices(std::string postfix);
};

serial_device::serial_device(/* args */)
{
}

std::vector<std::string> serial_device::list_devices(std::string postfix)
{
    std::vector<std::string> devices;

    #ifdef _WIN32
        for (const auto& port : list_com_ports()) {
            devices.push_back(port);
        }

    #else
        for (auto& p : std::filesystem::directory_iterator("/dev")) {
        std::string name = p.path().filename();
        if (name.find(postfix) == 0) {
            devices.push_back("/dev/" + name);
        }
    }
    return devices;

#endif

}

serial_device::~serial_device()
{
}
