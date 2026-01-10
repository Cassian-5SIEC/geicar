#include "network_hmi/udp_data_sender.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <cstring> // For memcpy

UdpDataSender::UdpDataSender()
{
    send_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
    // Note: No bind() needed, we are only sending
}

UdpDataSender::~UdpDataSender()
{
    if (send_socket_ != -1) {
        close(send_socket_);
    }
}

void UdpDataSender::send_json(const std::string& payload, const SharedClientInfo::UdpAddress& dest)
{
    if (send_socket_ == -1 || !dest.valid) {
        return;
    }
    sendto(
        send_socket_, payload.c_str(), payload.length(), 0,
        (const struct sockaddr *)&dest.addr, sizeof(dest.addr)
    );
}
