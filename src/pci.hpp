#ifndef IXY_PCI_HPP
#define IXY_PCI_HPP

#include <cstdint>
#include <ios>

namespace ixy {

    void remove_driver(const std::string &pci_addr);

    void enable_dma(const std::string &pci_addr);

    auto pci_map_resource(const std::string &pci_addr) -> std::pair<uint8_t *, size_t>;

    auto pci_open_resource(const std::string &pci_addr, const std::string &resource,
                           std::ios_base::openmode flags) -> std::fstream;

    auto read_io8(std::ifstream &s, size_t offset) -> uint8_t;

    auto read_io16(std::istream &s, size_t offset) -> uint16_t;

    auto read_io32(std::istream &s, size_t offset) -> uint32_t;

    void write_io8(std::ostream &s, size_t offset, uint8_t value);

    void write_io16(std::ostream &s, size_t offset, uint16_t value);

    void write_io32(std::ostream &s, size_t offset, uint32_t value);

}  // namespace ixy

#endif //IXY_PCI_HPP
