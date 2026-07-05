/*
 * The Driver Station Library (LibDS)
 * Copyright (c) Lily Wang and other contributors.
 * Open Source Software; you can modify and/or share it under the terms of
 * the MIT license file in the root directory of this project.
 */

#include "FdEvent.h"
#include <unistd.h>

FdEvent::FdEvent() { fd_ = eventfd(0, EFD_NONBLOCK); }

FdEvent::~FdEvent() {
    if (fd_ != -1) {
        close(fd_);
    }
}

bool FdEvent::wait() {
    if (fd_ == -1) {
        return false;
    }
    eventfd_t value = 0;
    return (eventfd_read(fd_, &value) == 0);
}

bool FdEvent::notify() {
    if (fd_ == -1) {
        return false;
    }
    eventfd_t value = 1;
    return (eventfd_write(fd_, value) == 0);
}
