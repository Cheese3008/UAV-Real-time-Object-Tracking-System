// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from px4_msgs:msg/AckermannVelocitySetpoint.idl
// generated code does not contain a copyright notice

#include "px4_msgs/msg/detail/ackermann_velocity_setpoint__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_px4_msgs
const rosidl_type_hash_t *
px4_msgs__msg__AckermannVelocitySetpoint__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x62, 0x03, 0x36, 0xa2, 0x27, 0xd5, 0x46, 0x54,
      0x12, 0x8e, 0xaa, 0xa1, 0xdc, 0x2f, 0xc8, 0xcd,
      0x3c, 0x89, 0xf6, 0x77, 0x2d, 0x38, 0x35, 0xe6,
      0xc7, 0xe6, 0x84, 0x66, 0x42, 0x11, 0x76, 0x6f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char px4_msgs__msg__AckermannVelocitySetpoint__TYPE_NAME[] = "px4_msgs/msg/AckermannVelocitySetpoint";

// Define type names, field names, and default values
static char px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__timestamp[] = "timestamp";
static char px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__velocity_ned[] = "velocity_ned";
static char px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__backwards[] = "backwards";

static rosidl_runtime_c__type_description__Field px4_msgs__msg__AckermannVelocitySetpoint__FIELDS[] = {
  {
    {px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT64,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__velocity_ned, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT_ARRAY,
      2,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {px4_msgs__msg__AckermannVelocitySetpoint__FIELD_NAME__backwards, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
px4_msgs__msg__AckermannVelocitySetpoint__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {px4_msgs__msg__AckermannVelocitySetpoint__TYPE_NAME, 38, 38},
      {px4_msgs__msg__AckermannVelocitySetpoint__FIELDS, 3, 3},
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
  "float32[2] velocity_ned # 2-dimensional velocity setpoint in NED frame [m/s]\n"
  "bool backwards\\t        # Flag for backwards driving";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
px4_msgs__msg__AckermannVelocitySetpoint__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {px4_msgs__msg__AckermannVelocitySetpoint__TYPE_NAME, 38, 38},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 188, 188},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
px4_msgs__msg__AckermannVelocitySetpoint__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *px4_msgs__msg__AckermannVelocitySetpoint__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
