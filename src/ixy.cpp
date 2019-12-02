#include <memory>
#include <fstream>

#include "ixy/ixy.hpp"
#include "pci.hpp"
#include "ixgbe.hpp"

namespace ixy {

    auto ixy_init(const std::string &pci_addr, uint16_t rx_queues, uint16_t tx_queues) -> std::unique_ptr<IxyDevice> {
        auto config = pci_open_resource(pci_addr, "config", std::ifstream::in);

        auto vendor_id = read_io16(config, 0);
        auto device_id = read_io16(config, 2);
        auto class_id = read_io32(config, 8) >> 24u;

        config.close();

        if (class_id != 2) {
            error("device " << pci_addr << " is not a NIC");
        }

        if (vendor_id == 0x1af4 && device_id >= 0x1000) {
            error("virtio is not implemented yet");
        } else {
            // our best guess is to try ixgbe
            return IxgbeDevice::init(pci_addr, rx_queues, tx_queues);
        }

        return {};
    }

}  // namespace ixy
