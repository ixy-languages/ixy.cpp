#include "ixy/ixy.hpp"
#include "ixy/stats.hpp"

using namespace ixy;

const unsigned int BATCH_SIZE = 32;

// number of packets in our mempool
const unsigned int NUM_PACKETS = 2048;
// excluding CRC (offloaded by default)
const unsigned int PACKET_SIZE = 60;

static const uint8_t pkt_data[] = {
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, // dst MAC
        0x10, 0x10, 0x10, 0x10, 0x10, 0x10, // src MAC
        0x08, 0x00,                         // ether type: IPv4
        0x45, 0x00,                         // Version, IHL, TOS
        (PACKET_SIZE - 14) >> 8,            // ip len excluding ethernet, high byte
        (PACKET_SIZE - 14) & 0xFF,          // ip len exlucding ethernet, low byte
        0x00, 0x00, 0x00, 0x00,             // id, flags, fragmentation
        0x40, 0x11, 0x00, 0x00,             // TTL (64), protocol (UDP), checksum
        0x0A, 0x00, 0x00, 0x01,             // src ip (10.0.0.1)
        0x0A, 0x00, 0x00, 0x02,             // dst ip (10.0.0.2)
        0x00, 0x2A, 0x05, 0x39,             // src and dst ports (42 -> 1337)
        (PACKET_SIZE - 20 - 14) >> 8,       // udp len excluding ip & ethernet, high byte
        (PACKET_SIZE - 20 - 14) & 0xFF,     // udp len exlucding ip & ethernet, low byte
        0x00, 0x00,                         // udp checksum, optional
        'i', 'x', 'y'                       // payload
        // rest of the payload is zero-filled because mempools guarantee empty bufs
};

// calculate a IP/TCP/UDP checksum
static uint16_t calc_ip_checksum(const uint8_t *data, uint32_t len) {
    uint32_t cs = 0;
    for (uint32_t i = 0; i < len / 2; i++) {
        cs += ((uint16_t *) data)[i];
        if (cs > 0xFFFF) {
            cs = (cs & 0xFFFF) + 1; // 16 bit one's complement
        }
    }
    return ~((uint16_t) cs);
}

auto main(int argc, char *argv[]) -> int {
    if (argc != 2) {
        std::cout << argv[0] << " generates packets on a single port." << std::endl;
        std::cout << "Usage: " << argv[0] << " <pci bus id1>" << std::endl;
        return 1;
    }

    auto dev = ixy_init(argv[1], 1, 1);

    auto pool = Mempool::allocate(NUM_PACKETS, 0);

    // pre-fill all packet buffer in the pool with data and return them to the packet pool
    {
        std::deque<Packet> buffer;

        Mempool::alloc_pkt_batch(pool, buffer, NUM_PACKETS, PACKET_SIZE);

        for (auto &p : buffer) {
            for (uint8_t i = 0; i < sizeof(pkt_data); i++) {
                p.at(i) = pkt_data[i];
            }

            auto checksum = calc_ip_checksum(p.get_virt_addr() + 14, 20);

            p.at(24) = (checksum >> 8);
            p.at(25) = (checksum & 0xff);
        }
    }

    struct device_stats stats1{*dev}, stats1_old{*dev};

    std::deque<Packet> buffer;
    uint64_t counter = 0;
    uint32_t seq_num = 0;

    auto last_stats_printed = monotonic_time();

    while (true) {
        // re-fill our packet queue with new packets to send out
        Mempool::alloc_pkt_batch(pool, buffer, BATCH_SIZE, PACKET_SIZE);

        // update sequence number of all packets (and checksum if necessary)
        for (auto &p : buffer) {
            *(uint32_t *) (p.get_virt_addr() + PACKET_SIZE - 4) = seq_num++;
        }

        dev->tx_batch(0, buffer);

        // don't poll the time unnecessarily
        if ((counter++ & 0xfff) == 0) {
            auto time = monotonic_time();
            if (time - last_stats_printed > 1000 * 1000 * 1000) {
                // every second
                dev->read_stats(stats1);
                stats1_old.print_stats_diff(stats1, time - last_stats_printed);
                stats1_old = stats1;

                last_stats_printed = time;
            }
        }

        buffer.clear();
    }
}
