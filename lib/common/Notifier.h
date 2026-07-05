#pragma once

#include <stdint.h>

#define HAL_kInvalidHandle 0
typedef int32_t HAL_NotifierHandle;

/**
 * @defgroup hal_notifier Notifier Functions
 * @ingroup hal_capi
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initializes a notifier.
 *
 * A notifier is an FPGA controller timer that triggers at requested intervals
 * based on the FPGA time. This can be used to make precise control loops.
 *
 * @param[out] status Error status variable. 0 on success.
 * @return the created notifier
 */
HAL_NotifierHandle HAL_InitializeNotifier(int32_t *status);
/**
 * Stops a notifier from running.
 *
 * This will cause any call into HAL_WaitForNotifierAlarm to return with time =
 * 0.
 *
 * @param[in] notifierHandle the notifier handle
 * @param[out] status Error status variable. 0 on success.
 */
void HAL_StopNotifier(HAL_NotifierHandle notifierHandle, int32_t *status);

/**
 * Cleans a notifier.
 *
 * Note this also stops a notifier if it is already running.
 *
 * @param[in] notifierHandle the notifier handle
 */
void HAL_CleanNotifier(HAL_NotifierHandle notifierHandle);

/**
 * Updates the trigger time for a notifier.
 *
 * Note that this time is an absolute time relative to HAL_GetFPGATime()
 *
 * @param[in] notifierHandle the notifier handle
 * @param[in] triggerTime    the updated trigger time
 * @param[out] status        Error status variable. 0 on success.
 */
void HAL_UpdateNotifierAlarm(HAL_NotifierHandle notifierHandle, uint64_t triggerTime, int32_t *status);
/**
 * Waits for the next alarm for the specific notifier.
 *
 * This is a blocking call until either the time elapses or HAL_StopNotifier
 * gets called. If the latter occurs, this function will return zero and any
 * loops using this function should exit. Failing to do so can lead to
 * use-after-frees.
 *
 * @param[in] notifierHandle the notifier handle
 * @param[out] status        Error status variable. 0 on success.
 * @return the FPGA time the notifier returned
 */
uint64_t HAL_WaitForNotifierAlarm(HAL_NotifierHandle notifierHandle, int32_t *status);

#ifdef __cplusplus
}  // extern "C"
#endif
/** @} */
