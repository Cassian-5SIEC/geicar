#pragma once
#include <cstdint>

// Header for fragmented image packets.
// We use network byte order (htonl, htons) before sending.
#pragma pack(push, 1) // Ensure tight packing
struct ImagePacketHeader {
    uint32_t frame_id;
    uint16_t packet_index;
    uint16_t total_packets;
};
#pragma pack(pop)