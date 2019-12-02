#include <iostream>

#include "ixy/ixy.hpp"
#include "ixy/stats.hpp"

using namespace ixy;

const unsigned int BATCH_SIZE = 32;

static void
forward(std::deque<Packet> &buffer, IxyDevice &rx_dev, uint16_t rx_queue, IxyDevice &tx_dev, uint16_t tx_queue) {
    auto num_rx = rx_dev.rx_batch(rx_queue, buffer, BATCH_SIZE);

    if (num_rx > 0) {
        for (auto &p : buffer) {
            p.at(1)++;
        }

        tx_dev.tx_batch(tx_queue, buffer);

        buffer.clear();
    }
}

auto main(int argc, char *argv[]) -> int {
    if (argc != 3) {
        std::cout << argv[0] << " forwards packets between two ports." << std::endl;
        std::cout << "Usage: " << argv[0] << " <pci bus id2> <pci bus id1>" << std::endl;
        return 1;
    }

    auto dev1 = ixy_init(argv[1], 1, 1);
    auto dev2 = ixy_init(argv[2], 1, 1);

    struct device_stats stats1{*dev1}, stats1_old{*dev1};
    struct device_stats stats2{*dev2}, stats2_old{*dev2};

    std::deque<Packet> buffer;
    uint64_t counter = 0;

    auto last_stats_printed = monotonic_time();

    while (true) {
        forward(buffer, *dev1, 0, *dev2, 0);
        forward(buffer, *dev2, 0, *dev1, 0);

        // don't poll the time unnecessarily
        if ((counter++ & 0xfff) == 0) {
            auto time = monotonic_time();
            if (time - last_stats_printed > 1000 * 1000 * 1000) {
                // every second
                dev1->read_stats(stats1);
                stats1_old.print_stats_diff(stats1, time - last_stats_printed);
                stats1_old = stats1;
                if (dev1 != dev2) {
                    dev2->read_stats(stats2);
                    stats2_old.print_stats_diff(stats2, time - last_stats_printed);
                    stats2_old = stats2;
                }
                last_stats_printed = time;
            }
        }
    }
}
