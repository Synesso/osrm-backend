#ifndef INTERNAL_DATAFACADE_HPP
#define INTERNAL_DATAFACADE_HPP

// implements all data storage when shared memory is _NOT_ used

#include "engine/datafacade/memory_datafacade_base.hpp"
#include "storage/storage.hpp"

namespace osrm
{
namespace engine
{
namespace datafacade
{

/**
 * This datafacade uses a process-local memory block to load
 * data into.  The logic is otherwise identical to the SharedMemoryFacade,
 * so we can just extend from that class.  This class holds a unique_ptr
 * to the memory blocks, so they are auto-freed upon destruction.
 */
class InternalDataFacade final : public MemoryDataFacadeBase
{

  private:
    std::unique_ptr<char[]> internal_memory;
    std::unique_ptr<storage::DataLayout> internal_layout;

  public:
    explicit InternalDataFacade(const storage::StorageConfig &config)
    {
        storage::Storage storage(config);

        // Calculate the layout/size of the memory block
        internal_layout = std::make_unique<storage::DataLayout>();
        storage.LoadLayout(internal_layout.get());

        // Allocate the memory block, then load data from files into it
        internal_memory.reset(new char[internal_layout->GetSizeOfLayout()]);
        storage.LoadData(internal_layout.get(), internal_memory.get());

        // Set up the SharedDataFacade pointers
        data_layout = internal_layout.get();
        memory_block = internal_memory.get();

        // Adjust all the private m_* members to point to the right places
        LoadData();
    }
};
}
}
}

#endif // INTERNAL_DATAFACADE_HPP
