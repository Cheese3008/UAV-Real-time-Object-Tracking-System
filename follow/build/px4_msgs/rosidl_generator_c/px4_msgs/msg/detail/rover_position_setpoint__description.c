// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/RoverPositionSetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/rover_position_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__RoverPositionSetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe9, 0x44, 0x67, 0x24, 0xeb, 0x06, 0x98, 0x28,
      0x79, 0x8b, 0xd8, 0x39, 0x92, 0x69, 0x31, 0xad,
      0x0b, 0xdf, 0x9f, 0xf9, 0x68, 0x11, 0xd8, 0xa8,
      0xb1, 0xc4, 0xb6, 0x76, 0x61, 0x06, 0x98, 0x55,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME[] = "px4_msgs/msg/RoverPositionSetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__position_ned[] = "position_ned";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__cruising_speed[] = "cruising_speed";
static char px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__RoverPositionSetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__position_ned, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__cruising_speed, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__RoverPositionSetpoint__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__RoverPositionSetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME, 34, 34},
      {px4_msgs__msg__RoverPositionSetpoint__FIELDS, 4, 4},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint64 timestamp # time since system start (microseconds)\n"
  "\n"
  "float32[2] position_ned # 2-dimensional position setpoint in NED frame [m]\n"
  "float32 cruising_speed  # (Optional) Specify rover speed (Defaults to maximum speed)\n"
  "\n"
  "float32 yaw             # [rad] [-pi,pi] from North. Optional, NAN if not set. Mecanum only.";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__RoverPositionSetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__RoverPositionSetpoint__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 313, 313},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__RoverPositionSetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__RoverPositionSetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
