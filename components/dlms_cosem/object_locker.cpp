#include "object_locker.h"

namespace esphome::dlms_cosem {

std::vector<void *> AnyObjectLocker::locked_objects_(5);
Mutex AnyObjectLocker::lock_;

}  // namespace esphome::dlms_cosem
