#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Servo.h>

// micro-ROS entities
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Servo instances
Servo servo_thumb_flex;
Servo servo_index;
Servo servo_middle;
Servo servo_ring;
Servo servo_pinky;
Servo servo_thumb_base;

// PWM pin mapping
const int PIN_THUMB_FLEX = 3;
const int PIN_INDEX      = 5;
const int PIN_MIDDLE     = 6;
const int PIN_RING       = 9;
const int PIN_PINKY      = 10;
const int PIN_THUMB_BASE = 11;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }

void error_loop() {
  while (1) {
    delay(100);
  }
}

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *incoming = (const std_msgs__msg__Int32MultiArray *)msgin;

  if (incoming->data.size == 6) {
    servo_thumb_flex.write(incoming->data.data[0]);
    servo_index.write(incoming->data.data[1]);
    servo_middle.write(incoming->data.data[2]);
    servo_ring.write(incoming->data.data[3]);
    servo_pinky.write(incoming->data.data[4]);
    servo_thumb_base.write(incoming->data.data[5]);
  }
}

void setup() {
  set_microros_transports();

  // Attach finger servos with default pulse width
  servo_thumb_flex.attach(PIN_THUMB_FLEX);
  servo_index.attach(PIN_INDEX);
  servo_middle.attach(PIN_MIDDLE);
  servo_ring.attach(PIN_RING);
  servo_pinky.attach(PIN_PINKY);

  // Extended pulse width (500-2500 us) for full 180-degree thumb base travel
  servo_thumb_base.attach(PIN_THUMB_BASE, 500, 2500);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "robotic_hand_mcu", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_array"));

  // Pre-allocate memory for 6 servo angles
  msg.data.capacity = 6;
  msg.data.size = 0;
  msg.data.data = (int32_t *)malloc(msg.data.capacity * sizeof(int32_t));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}