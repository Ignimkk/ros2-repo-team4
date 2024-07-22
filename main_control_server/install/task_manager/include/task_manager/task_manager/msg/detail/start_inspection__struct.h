// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from task_manager:msg/StartInspection.idl
// generated code does not contain a copyright notice

#ifndef TASK_MANAGER__MSG__DETAIL__START_INSPECTION__STRUCT_H_
#define TASK_MANAGER__MSG__DETAIL__START_INSPECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'signal'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StartInspection in the package task_manager.
/**
  * InspectionStart.msg
 */
typedef struct task_manager__msg__StartInspection
{
  rosidl_runtime_c__String signal;
} task_manager__msg__StartInspection;

// Struct for a sequence of task_manager__msg__StartInspection.
typedef struct task_manager__msg__StartInspection__Sequence
{
  task_manager__msg__StartInspection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} task_manager__msg__StartInspection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TASK_MANAGER__MSG__DETAIL__START_INSPECTION__STRUCT_H_
