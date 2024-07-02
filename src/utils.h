#ifndef UTILS_H
#define UTILS_H
#include <QDebug>
#include <iostream>
#include <string>
#include <unordered_map>
#include <boost/asio.hpp>
#include <msg.h>
#include <mutex>
using boost::asio::ip::udp;

class DeviceInfo
{
public:
    bool connected = false;
    DeviceInfo(){};
    DeviceInfo(const char* _Name, const char* _ip, unsigned short _ip_len, unsigned short _udpPort):
        udpPort(_udpPort), _lenip(_ip_len)
    {
        memcpy(deviceName, _Name, std::strlen(_Name));
        deviceName[std::strlen(_Name)] = 0;
        memcpy(deviceIP, _ip, _ip_len);
        deviceIP[_ip_len] = 0;
        connectEp = udp::endpoint(boost::asio::ip::address::from_string(deviceIP), (unsigned short)_udpPort);
    }

    DeviceInfo(const connect_msg& cMsg)
    {
        new (this)DeviceInfo(cMsg.senderName, cMsg.ip_addr, cMsg.ip_len, cMsg.udp_port);
    }

    const char* getDeviceName()
    {
        return deviceName;
    }

    const char* getDeviceIP()
    {
        return deviceIP;
    }

    udp::endpoint getEndpoint()
    {
        return connectEp;
    }

    void printInfo()
    {
        
    }

private:
    char deviceName[20];
    char deviceIP[16];
    unsigned short _lenip;
    unsigned short udpPort;
    udp::endpoint connectEp;    
};

class DevicesList
{
public:
    DevicesList() {
        devices.clear();
    }

    std::unordered_map<std::string, DeviceInfo>::iterator getDevice(std::string key)
    {
        return devices.find(key);
    }

    bool isIn(const connect_msg& cMsg)
    {
        return getDevice(cMsg.senderName) != devices.end();
    }

    int addDevice(const connect_msg& cMsg)
    {
        if (this->isIn(cMsg)) return -1;
        devices.emplace(cMsg.senderName, cMsg);
        return 0;
    }

    void setConnectedDeivce(std::string name)
    {
        connectedKey = name;
        if (getDevice(name) == devices.end())
        {
            std::cout << "error";
        }
    }

    udp::endpoint getConnectedDevice()
    {
        return getDevice(connectedKey)->second.getEndpoint();
    }

    void printDevices()
    {   
        int cnt = 1;
        qDebug() << "--------Devices Detected-------";
        for (auto iter = devices.begin(); iter != devices.end(); iter++)
        {
            DeviceInfo& _info = iter->second;
            qDebug() << "Device " << cnt++;
            qDebug() << "Device Name: " << iter->first.c_str();
            qDebug() << "Device IP: "  << _info.getDeviceIP();
        }
    }

private:
    std::unordered_map<std::string, DeviceInfo> devices;
    std::string connectedKey;
};

// int getSelfIPAddresses(boost::asio::io_service& ios, boost::asio::ip::address* selfAddrs)
// {
//     tcp::resolver resolver(ios);
//     tcp::resolver::query query(boost::asio::ip::host_name(),"");
//     tcp::resolver::iterator it=resolver.resolve(query);
//     int cnt = 0;
//     while(it!=tcp::resolver::iterator())
//     {
//         boost::asio::ip::address addr=(it++)->endpoint().address();
//         if(addr.is_v4())
//         {
//             selfAddrs[cnt++] = addr;
            
//             // <<"ipv6 address: ";
//         }
//             // std::cout<<"ipv4 address: ";

//         // std::cout<<addr.to_string()<<std::endl;

//     }
//     return cnt;
// }



#endif