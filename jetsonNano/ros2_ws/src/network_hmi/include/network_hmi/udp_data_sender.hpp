#pragma once

#include <string>
#include <vector>
#include <atomic>
#include <netinet/in.h> // For sockaddr_in
#include "shared_client_info.hpp"

// Handles sending all outgoing UDP packets
class UdpDataSender
{
public:
    UdpDataSender();
    ~UdpDataSender();

    UdpDataSender(const UdpDataSender&) = delete;
    UdpDataSender& operator=(const UdpDataSender&) = delete;

    void send_json(const std::string& payload, const SharedClientInfo::UdpAddress& dest);

private:
    int send_socket_ = -1;
    std::atomic<uint32_t> frame_id_counter_{0};
};