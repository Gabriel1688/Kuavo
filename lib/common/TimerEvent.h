#pragma once
/*-------------------------------- Dependencies ------------------------------*/

#include <string>

/*----------------------------------------------------------------------------*/

/*!
 * \class TimerEvent
 * Container for timer event information.
 */
class TimerEvent {
public:
    /*!
     * Event definitions.
     */
    enum TimerEventType {
        EXPORT_CDR,
        PURGE_CDR,
        GATHER_DB_STATS,
        RE_CONN_DB,
        HTTP,
        CURL,
        CS_STATUS
    };

    /*!
     * ctor.
     * \param type - the event's type.
     * \param id - application defined indentifier.
     */
    TimerEvent(TimerEventType type, const std::string &id)
            : type_(type),
              id_(id) {
        ;
    }

    /*!
     * Get the event type.
     */
    TimerEventType getType() const {
        return type_;
    }

    /*!
     * Get the app's identifier.
     */
    const std::string &getId() const {
        return id_;
    }

private:
    TimerEventType type_;
    std::string id_;
};
