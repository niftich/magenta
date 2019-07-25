# mx_object_get_property, mx_object_set_property

## NAME

object_get_property - Ask for various properties of various kernel objects.

object_set_property - Set various properties of various kernel objects.

## SYNOPSIS

```
#include <magenta/syscalls.h>
#include <magenta/syscalls/object.h>

mx_status_t mx_object_get_property(mx_handle_t handle, uint32_t property,
                                   void* value, size_t size);

mx_status_t mx_object_set_property(mx_handle_t handle, uint32_t property,
                                   const void* value, size_t size);
```

## DESCRIPTION

**mx_object_get_property()** requests the value of a kernel object's property.
Getting a property requires **MX_RIGHT_GET_PROPERTY** rights on the handle.

**mx_object_set_property()** modifies the value of a kernel object's property.
Setting a property requires **MX_RIGHT_SET_PROPERTY** rights on the handle.

The *handle* parameter indicates the target kernel object. Different properties
only work on certain types of kernel objects, as described below.

The *property* parameter indicates which property to get/set. Property values
have the prefix **MX_PROP_**, and are described below.

The *value* parameter holds the property value, and must be a pointer to a
buffer of *size* bytes. Different properties expect different value types/sizes
as described below.

## PROPERTIES

Property values have the prefix **MX_PROP_**, and are defined in

```
#include <magenta/syscalls/object.h>
```

### MX_PROP_NUM_STATE_KINDS

*handle* type: **Thread**

*value* type: **uint32_t**

Allowed operations: **get**

For debugger usage.

TODO: Describe the details of this property.

### MX_PROP_NAME

*handle* type: **(Most types)**

*value* type: **char\[MX_MAX_NAME_LEN\]**

Allowed operations: **get**, **set**

The name of the object, as a NUL-terminated string.

### MX_PROP_REGISTER_FS

*handle* type: **Thread**

*value* type: **uintptr_t**

Allowed operations: **set**

The value of the x86 FS segment register.

Only defined for x86-64.

### MX_PROP_PROCESS_DEBUG_ADDR

*handle* type: **Process**

*value* type: **uintptr_t**

Allowed operations: **get**, **set**

The value of ld.so's `_dl_debug_addr`.

### MX_PROP_PROCESS_VDSO_BASE_ADDRESS

*handle* type: **Process**

*value* type: **uintptr_t**

Allowed operations: **get**

The base address of the vDSO mapping, or zero.

### MX_PROP_JOB_MAX_HEIGHT

*handle* type: **Job**

*value* type: **uint32_t**

Allowed operations: **get**

The number of descendant generations that a job is allowed to have.

A job has a *MAX_HEIGHT* value equal to one less than its parent's *MAX_HEIGHT*
value.

A job with *MAX_HEIGHT* equal to zero may not have any child jobs, and calling
**mx_job_create()** on such a job will fail with **ERR_OUT_OF_RANGE**.
*MAX_HEIGHT* does not affect the creation of processes.

See also:

[job_create](job_create.md)

## RETURN VALUE

**mx_object_get_property**() returns **NO_ERROR** on success. In the event of
failure, a negative error value is returned.

## ERRORS

**ERR_BAD_HANDLE**: *handle* is not a valid handle

**ERR_WRONG_TYPE**: *handle* is not an appropriate type for *property*

**ERR_ACCESS_DENIED**: *handle* does not have the necessary rights for the
operation

**ERR_INVALID_ARGS**: *value* is an invalid pointer

**ERR_NO_MEMORY**: Temporary out of memory failure

**ERR_BUFFER_TOO_SMALL**: *size* is too small for *property*

**ERR_NOT_SUPPORTED**: *property* does not exist

## SEE ALSO

[object_set_property](object_set_property.md)
