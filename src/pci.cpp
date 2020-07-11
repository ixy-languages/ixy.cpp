#include <cassert>
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fstream>

#include "pci.hpp"
#include "log.hpp"
#include "fs.hpp"

namespace ixy {

void remove_driver(const std::string &pci_addr) {
    fs::path path{"/sys/bus/pci/devices/" + pci_addr + "/driver/unbind"};
    auto s = std::ofstream(path, std::ofstream::out);
    if (!s) {
        debug("no driver loaded");
        return;
    }
    s.write(pci_addr.c_str(), pci_addr.length());
    if (!s) {
        error("failed to unload driver for device");
    }
    s.close();
}

void enable_dma(const std::string &pci_addr) {
    fs::path path{"/sys/bus/pci/devices/" + pci_addr + "/config"};
    auto s = std::fstream(path, std::fstream::in | std::fstream::out);
    if (!s) {
        error("failed to open pci config");
    }
    // write to the command register (offset 4) in the PCIe config space
    // bit 2 is "bus master enable", see PCIe 3.0 specification section 7.5.1.1
    s.seekg(4);
    assert(s.tellg() == 4);
    uint16_t dma = 0;
    s.read(reinterpret_cast<char *>(&dma), sizeof(dma));
    if (!s) {
        error("failed to read from pci config");
    }
    dma |= 1u << 2u;
    s.seekp(4);
    assert(s.tellp() == 4);
    s.write(reinterpret_cast<char *>(&dma), sizeof(dma));
    if (!s) {
        error("failed to write to pci config");
    }
    s.close();
}

auto pci_map_resource(const std::string &pci_addr) -> std::pair<uint8_t *, size_t> {
    fs::path path{"/sys/bus/pci/devices/" + pci_addr + "/resource0"};
    debug("mapping PCI resource at " << path);
    remove_driver(pci_addr);
    enable_dma(pci_addr);
    int fd = open(path.c_str(), O_RDWR);
    if (fd == -1) {
        error("failed to open pci resource");
    }
    struct stat stat{};
    if (fstat(fd, &stat) == -1) {
        error("failed to stat pci resource");
    }
    auto *hw = reinterpret_cast<uint8_t *>(
            mmap(nullptr, static_cast<size_t>(stat.st_size), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0)
    );
    if (hw == MAP_FAILED) {
        error("failed to mmap pci resource");
    }
    close(fd);
    return std::make_pair(hw, stat.st_size);
}

auto
pci_open_resource(const std::string &pci_addr, const std::string &resource,
                  std::ios_base::openmode flags) -> std::fstream {
    fs::path path{"/sys/bus/pci/devices/" + pci_addr + "/" + resource};
    debug("opening PCI resource at " << path);
    auto s = std::fstream(path, flags);
    if (!s) {
        error("failed to open pci resource");
    }
    return s;
}

auto read_io8(std::istream &s, size_t offset) -> uint8_t {
    uint8_t tmp;
    s.seekg(offset);
    assert((size_t) s.tellg() == offset);
    s.read(reinterpret_cast<char *>(&tmp), sizeof(tmp));
    if (!s) {
        error("failed to read IO resource");
    }
    return tmp;
}

auto read_io16(std::istream &s, size_t offset) -> uint16_t {
    uint16_t tmp;
    s.seekg(offset);
    assert((size_t) s.tellg() == offset);
    s.read(reinterpret_cast<char *>(&tmp), sizeof(tmp));
    if (!s) {
        error("failed to read IO resource");
    }
    return tmp;
}

auto read_io32(std::istream &s, size_t offset) -> uint32_t {
    uint32_t tmp;
    s.seekg(offset);
    assert((size_t) s.tellg() == offset);
    s.read(reinterpret_cast<char *>(&tmp), sizeof(tmp));
    if (!s) {
        error("failed to read IO resource");
    }
    return tmp;
}

void write_io8(std::ostream &s, size_t offset, uint8_t value) {
    s.seekp(offset);
    assert((size_t) s.tellp() == offset);
    s.write(reinterpret_cast<char *>(&value), sizeof(value));
    if (!s) {
        error("failed to write IO resource");
    }
}

void write_io16(std::ostream &s, size_t offset, uint16_t value) {
    s.seekp(offset);
    assert((size_t) s.tellp() == offset);
    s.write(reinterpret_cast<char *>(&value), sizeof(value));
    if (!s) {
        error("failed to write IO resource");
    }
}

void write_io32(std::ostream &s, size_t offset, uint32_t value) {
    s.seekp(offset);
    assert((size_t) s.tellp() == offset);
    s.write(reinterpret_cast<char *>(&value), sizeof(value));
    if (!s) {
        error("failed to write IO resource");
    }
}

}  // namespace ixy
