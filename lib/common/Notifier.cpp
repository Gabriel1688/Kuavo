#include "Notifier.h"
#include <condition_variable>
#include <mutex>
#include <string>
#include <vector>

typedef int32_t HAL_Handle;

namespace {
    struct Notifier {
        std::string name;
        uint64_t waitTime = UINT64_MAX;
        bool active = true;
        bool waitTimeValid = false;  // True if waitTime is set and in the future
        bool waitingForAlarm = false;// True if in HAL_WaitForNotifierAlarm()
        uint64_t waitCount = 0;      // Counts calls to HAL_WaitForNotifierAlarm()
        std::mutex mutex;
        std::condition_variable cond;
    };
}// namespace

static std::mutex notifiersWaiterMutex;
static std::condition_variable notifiersWaiterCond;

class NotifierHandleContainer {
public:
    NotifierHandleContainer() = default;

    NotifierHandleContainer(const NotifierHandleContainer &) = delete;

    NotifierHandleContainer &operator=(const NotifierHandleContainer &) = delete;

    HAL_NotifierHandle Allocate(std::shared_ptr<Notifier> structure);

    int16_t GetIndex(HAL_NotifierHandle handle) {
        return static_cast<int16_t>(handle & 0xffff);
    }

    std::shared_ptr<Notifier> Get(HAL_NotifierHandle handle);

    /* Returns structure previously at that handle (or nullptr if none) */
    std::shared_ptr<Notifier> Free(HAL_NotifierHandle handle);

    void ResetHandles();

    ~NotifierHandleContainer() {
        //        ForEach([](HAL_NotifierHandle handle, Notifier* notifier) {
        //            {
        //                std::scoped_lock lock(notifier->mutex);
        //                notifier->active = false;
        //                notifier->waitTimeValid = false;
        //            }
        //            notifier->cond.notify_all();  // wake up any waiting threads
        //        });
        //        notifiersWaiterCond.notify_all();
    }

private:
    std::vector<std::shared_ptr<Notifier>> m_structures;
    std::mutex m_handleMutex;
};

HAL_Handle createHandle(int16_t index, int handleType, int16_t version) {
    if (index < 0) {
        return HAL_kInvalidHandle;
    }
    uint8_t hType = static_cast<uint8_t>(handleType);
    if (hType == 0 || hType > 127) {
        return HAL_kInvalidHandle;
    }
    // set last 8 bits, then shift to first 8 bits
    HAL_Handle handle = hType;
    handle = handle << 8;
    handle += static_cast<uint8_t>(version);
    handle = handle << 16;
    // add index to set last 16 bits
    handle += index;
    return handle;
}

HAL_NotifierHandle NotifierHandleContainer::Allocate(std::shared_ptr<Notifier> structure) {
    int m_version = 0;
    std::scoped_lock lock(m_handleMutex);
    size_t i;
    for (i = 0; i < m_structures.size(); i++) {
        if (m_structures[i] == nullptr) {
            m_structures[i] = structure;
            return static_cast<HAL_NotifierHandle>(createHandle(i, 3, m_version));
        }
    }
    if (i >= INT16_MAX) {
        return HAL_kInvalidHandle;
    }

    m_structures.push_back(structure);
    return static_cast<HAL_NotifierHandle>(
            createHandle(static_cast<int16_t>(i), 3, m_version));
}

std::shared_ptr<Notifier> NotifierHandleContainer::Get(HAL_NotifierHandle handle) {
    int16_t index = GetIndex(handle);
    std::scoped_lock lock(m_handleMutex);
    if (index < 0 || index >= static_cast<int16_t>(m_structures.size())) {
        return nullptr;
    }
    return m_structures[index];
}

std::shared_ptr<Notifier> NotifierHandleContainer::Free(HAL_NotifierHandle handle) {
    int16_t index = GetIndex(handle);
    std::scoped_lock lock(m_handleMutex);
    if (index < 0 || index >= static_cast<int16_t>(m_structures.size())) {
        return nullptr;
    }
    return std::move(m_structures[index]);
}

static NotifierHandleContainer *notifierHandles;
static std::atomic<bool> notifiersPaused{false};

//namespace hal {
namespace init {
    void InitializeNotifier() {
        static NotifierHandleContainer nH;
        notifierHandles = &nH;
    }
}// namespace init
//  using namespace hal;

HAL_NotifierHandle HAL_InitializeNotifier(int32_t *status) {
    std::shared_ptr<Notifier> notifier = std::make_shared<Notifier>();
    HAL_NotifierHandle handle = notifierHandles->Allocate(notifier);
    return handle;
}

void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t *status) {
    auto notifier = notifierHandles->Get(notifierHandle);
    if (!notifier) {
        return;
    }

    {
        std::scoped_lock lock(notifier->mutex);
        notifier->active = false;
        notifier->waitTimeValid = false;
    }
    notifier->cond.notify_all();
}

uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle,
                                  int32_t *status) {
    auto notifier = notifierHandles->Get(notifierHandle);
    if (!notifier) {
        return 0;
    }

    std::unique_lock ulock(notifiersWaiterMutex);
    std::unique_lock lock(notifier->mutex);
    notifier->waitingForAlarm = true;
    ++notifier->waitCount;
    ulock.unlock();
    notifiersWaiterCond.notify_all();
    while (notifier->active) {
        //uint64_t curTime = HAL_GetFPGATime(status);
        uint64_t curTime = 0;
        if (notifier->waitTimeValid && curTime >= notifier->waitTime) {
            notifier->waitTimeValid = false;
            notifier->waitingForAlarm = false;
            return curTime;
        }

        double waitDuration;
        if (!notifier->waitTimeValid || notifiersPaused) {
            // If not running, wait 1000 seconds
            waitDuration = 1000.0;
        } else {
            waitDuration = (notifier->waitTime - curTime) * 1e-6;
        }

        notifier->cond.wait_for(lock, std::chrono::duration<double>(waitDuration));
    }
    notifier->waitingForAlarm = false;
    return 0;
}
