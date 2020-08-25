#ifndef IXY_MEMORY_HPP
#define IXY_MEMORY_HPP

#include <fcntl.h>
#include <unistd.h>

#include "log.hpp"

namespace ixy {

const unsigned int HUGE_PAGE_BITS = 21;
const unsigned int HUGE_PAGE_SIZE = (1u << HUGE_PAGE_BITS); // 2_097_152 = 2MiB

struct dma_memory {
    void *virt;
    uintptr_t phy;
};

// translate a virtual address to a physical one via /proc/self/pagemap
[[nodiscard]] static auto virt_to_phys(void *virt) -> uintptr_t {
    uint32_t pagesize = sysconf(_SC_PAGESIZE);
    int fd = open("/proc/self/pagemap", O_RDONLY);

    if (fd == -1) {
        error("failed to open pagemap");
    }

    // pagemap is an array of pointers for each normal-sized page
    if (lseek(fd, reinterpret_cast<uintptr_t>(virt) / pagesize * sizeof(uintptr_t), SEEK_SET) == -1) {
        error("failed to lseek pagemap");
    }

    uintptr_t phy = 0;
    if (read(fd, &phy, sizeof(phy)) == -1) {
        error("failed to translate address");
    }

    close(fd);

    if (!phy) {
        error("failed to translate virtual address {} to physical address", virt);
    }

    // bits 0-54 are the page number
    return (phy & 0x7fffffffffffffULL) * pagesize + (reinterpret_cast<uintptr_t>(virt)) % pagesize;
}

auto memory_allocate_dma(size_t size, bool require_contiguous) -> struct dma_memory;

}  // namespace ixy

#endif //IXY_MEMORY_HPP
