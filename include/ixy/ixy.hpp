#ifndef IXY_IXY_HPP
#define IXY_IXY_HPP

#include <deque>

#include "packet.hpp"

namespace ixy {

/// Abstract class for driver implementations like ixgbe or virtio.
class IxyDevice {
public:
    /// Virtual destructor for derived classes.
    virtual ~IxyDevice() = default;

    /// Returns the driver's name, e.g. "ixgbe".
    /// @return     Name of the driver
    [[nodiscard]] virtual auto get_driver_name() -> const std::string & = 0;

    /// Returns the device's PCI address, e.g. "0000:01:00.0".
    /// @return     PCI address of the device
    [[nodiscard]] virtual auto get_pci_addr() -> const std::string & = 0;

    /// Returns the layer 2 address of the device.
    /// @return     MAC address of the device
    [[nodiscard]] virtual auto get_mac_addr() -> std::array<uint8_t, 6> = 0;

    /// Sets the layer 2 address of the device.
    /// @param mac  MAC address to be set
    virtual void set_mac_addr(const std::array<uint8_t, 6> &mac) = 0;

    /// Receives a batch of packets.
    /// @param queue_id     Receive (RX) queue to be used
    /// @param buffer       Buffer for the received packets
    /// @param packets      Maximum number of packets to be received
    /// @return             Number of packets that were actually received
    virtual auto rx_batch(uint16_t queue_id, std::deque<Packet> &buffer, uint32_t packets) -> uint32_t = 0;

    /// Transmits a batch of packets.
    /// @param queue_id     Transmit (TX) queue to be used
    /// @param buffer       Buffer of packets that shall be transmitted
    /// @return             Number of packets that were actually transmitted
    virtual auto tx_batch(uint16_t queue_id, std::deque<Packet> &buffer) -> uint32_t = 0;

    /// Reads device statistics like number of received bytes, transmitted packets, ...
    /// @param device_stats     Struct to store the read values
    virtual void read_stats(struct device_stats &device_stats) = 0;

    /// Resets device statistic registers.
    virtual void reset_stats() = 0;

    /// Returns the link speed of the device.
    /// @return     Link speed of the device
    [[nodiscard]] virtual auto get_link_speed() -> uint32_t = 0;
};

/// Initializes the device at the given PCI address.
///
/// @param pci_addr         PCI address of the device
/// @param num_rx_queues    Number of receive  (RX) queues to be used
/// @param num_tx_queues    Number of transmit (TX) queues to be used
/// @return                 Unique pointer to an implementation of IxyDevice, e.g. IxgbeDevice
auto ixy_init(const std::string &pci_addr, uint16_t num_rx_queues, uint16_t num_tx_queues) -> std::unique_ptr<IxyDevice>;

}  // namespace ixy

#endif //IXY_IXY_HPP
