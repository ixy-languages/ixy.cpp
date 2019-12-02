#include <fcntl.h>
#include <sstream>
#include <sys/mman.h>

#include "memory.hpp"
#include "fs.hpp"

namespace ixy {

    static uint32_t huge_pg_id;

    auto memory_allocate_dma(size_t size, bool require_contiguous) -> dma_memory {
        debug("allocating dma memory via huge page");
        // round up to multiples of 2 MB if necessary, this is the wasteful part
        // this could be fixed by co-locating allocations on the same page until a request would be too large
        // when fixing this: make sure to align on 128 byte boundaries (82599 dma requirement)
        if (size % HUGE_PAGE_SIZE) {
            size = ((size >> HUGE_PAGE_BITS) + 1) << HUGE_PAGE_BITS;
        }
        if (require_contiguous && size > HUGE_PAGE_SIZE) {
            // this is the place to implement larger contiguous physical mappings if that's ever needed
            error("could not map physically contiguous memory");
        }
        // unique filename, C11 stdatomic.h requires a too recent gcc, we want to support gcc 4.8
        uint32_t id = __sync_fetch_and_add(&huge_pg_id, 1);

        fs::path path{"/mnt/huge/ixy-" + std::to_string(getpid()) + "-" + std::to_string(id)};
        int fd = open(path.c_str(), O_CREAT | O_RDWR, S_IRWXU);

        if (fd == -1) {
            error("failed to open hugetlbfs file, check that /mnt/huge is mounted");
        }

        if (ftruncate(fd, static_cast<off_t>(size)) == -1) {
            error("failed to allocate huge page memory, check hugetlbfs configuration");
        }

        void *virt_addr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_HUGETLB, fd, 0);

        if (virt_addr == MAP_FAILED) {
            error("failed to mmap huge page");
        }

        // never swap out DMA memory
        if (mlock(virt_addr, size) == -1) {
            error("disable swap for DMA memory");
        }

        // don't keep it around in the hugetlbfs
        close(fd);
        unlink(path.c_str());

        return (dma_memory{virt_addr, virt_to_phys(virt_addr)});
    }

}  // namespace ixy
