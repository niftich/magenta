#pragma once

#include <stdint.h>

#include <kernel/mutex.h>

#include <magenta/dispatcher.h>
#include <magenta/syscalls/port.h>

#include <mxtl/canary.h>
#include <mxtl/intrusive_double_list.h>
#include <mxtl/ref_counted.h>
#include <mxtl/ref_ptr.h>

class UserThread;
class ProcessDispatcher;
class PortDispatcher;

// Represents the binding of an exception port to a specific target
// (system/process/thread). Multiple ExceptionPorts may exist for a
// single underlying PortDispatcher.
class ExceptionPort : public mxtl::DoublyLinkedListable<mxtl::RefPtr<ExceptionPort>>
                    , public mxtl::RefCounted<ExceptionPort> {
public:
    enum class Type { NONE, DEBUGGER, THREAD, PROCESS, SYSTEM };

    static mx_status_t Create(Type type, mxtl::RefPtr<PortDispatcher> port,
                              uint64_t port_key,
                              mxtl::RefPtr<ExceptionPort>* eport);
    ~ExceptionPort();

    Type type() const { return type_; }

    mx_status_t SendReport(const mx_exception_report_t* packet);

    void OnThreadStart(UserThread* thread);

    void OnThreadSuspending(UserThread* thread);
    void OnThreadResuming(UserThread* thread);

    void OnProcessExit(ProcessDispatcher* process);
    void OnThreadExit(UserThread* thread);
    void OnThreadExitForDebugger(UserThread* thread);

    // Records the target that the ExceptionPort is bound to, so it can
    // unbind when the underlying PortDispatcher dies.
    void SetSystemTarget();
    void SetTarget(const mxtl::RefPtr<ProcessDispatcher>& target);
    void SetTarget(const mxtl::RefPtr<ThreadDispatcher>& target);

    // Drops the ExceptionPort's references to its target and PortDispatcher.
    // Called by the target when the port is explicitly unbound.
    void OnTargetUnbind();

private:
    friend class PortDispatcher;

    ExceptionPort(Type type, mxtl::RefPtr<PortDispatcher> port, uint64_t port_key);

    ExceptionPort(const ExceptionPort&) = delete;
    ExceptionPort& operator=(const ExceptionPort&) = delete;

    // Unbinds from the target if bound, and drops the ref to |port_|.
    // Called by |port_| when it reaches zero handles.
    void OnPortZeroHandles();

#if DEBUG_ASSERT_IMPLEMENTED
    // Lets PortDispatcher assert that this eport is associated
    // with the right instance.
    bool PortMatches(const PortDispatcher *port, bool allow_null) {
        AutoLock lock(&lock_);
        return (allow_null && port_ == nullptr) || port_.get() == port;
    }
#endif  // if DEBUG_ASSERT_IMPLEMENTED

    // Returns true if the ExceptionPort is currently bound to a target.
    bool IsBoundLocked() TA_REQ(lock_) {
        return bound_to_system_ || (target_ != nullptr);
    }

    static void BuildReport(mx_exception_report_t* report, uint32_t type,
                            mx_koid_t pid, mx_koid_t tid);

    static void BuildSuspendResumeReport(mx_exception_report_t* report,
                                         uint32_t type, UserThread* thread);

    mxtl::Canary<mxtl::magic("EXCP")> canary_;

    // These aren't locked as once the exception port is created these are
    // immutable (the io port itself has its own locking though).
    const Type type_;
    const uint64_t port_key_;

    // The underlying ioport. If null, the ExceptionPort has been unbound.
    mxtl::RefPtr<PortDispatcher> port_ TA_GUARDED(lock_);

    // The target of the exception port.
    // The system exception port doesn't have a Dispatcher, hence the bool.
    mxtl::RefPtr<Dispatcher> target_ TA_GUARDED(lock_);
    bool bound_to_system_ TA_GUARDED(lock_) = false;

    Mutex lock_;

    // NOTE: The DoublyLinkedListNodeState is guarded by |port_|'s lock,
    // and should only be touched using port_->LinkExceptionPort()
    // or port_->UnlinkExceptionPort(). This goes for ::InContainer(), too.
};
