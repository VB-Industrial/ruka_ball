#include <cstdio>
#include <string>
#include <vector>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iomanip> 

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "ruka_joints.h"

#include "cyphal/allocators/o1/o1_allocator.h"
#include "cyphal/cyphal.h"
#include "cyphal/providers/LinuxCAN.h"
#include "cyphal/subscriptions/subscription.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "reg/udral/physics/kinematics/rotation/Planar_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/Twist_0_1.h"
#include "reg/udral/physics/kinematics/cartesian/State_0_1.h"

#include <uavcan/_register/Access_1_0.h>
#include <uavcan/_register/List_1_0.h>

TYPE_ALIAS(Twist, reg_udral_physics_kinematics_cartesian_Twist_0_1)
TYPE_ALIAS(HBeat, uavcan_node_Heartbeat_1_0)
TYPE_ALIAS(JS_msg, reg_udral_physics_kinematics_rotation_Planar_0_1)
TYPE_ALIAS(State, reg_udral_physics_kinematics_cartesian_State_0_1)

TYPE_ALIAS(RegisterListRequest, uavcan_register_List_Request_1_0)
TYPE_ALIAS(RegisterListResponse, uavcan_register_List_Response_1_0)

TYPE_ALIAS(RegisterAccessRequest, uavcan_register_Access_Request_1_0)
TYPE_ALIAS(RegisterAccessResponse, uavcan_register_Access_Response_1_0)

std::byte buffer[sizeof(CyphalInterface) + sizeof(LinuxCAN) + sizeof(O1Allocator)];

void error_handler() {std::cout << "error" << std::endl; std::exit(EXIT_FAILURE);}
uint64_t micros_64() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}
UtilityConfig utilities(micros_64, error_handler);

std::shared_ptr<CyphalInterface> cy_interface;

uint32_t uptime = 0;
void heartbeat() {
    static CanardTransferID hbeat_transfer_id = 0;
    HBeat::Type heartbeat_msg = {.uptime = uptime, .health = {uavcan_node_Health_1_0_NOMINAL}, .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};
    cy_interface->send_msg<HBeat>(&heartbeat_msg, uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_, &hbeat_transfer_id);
    uptime += 1;
}

static float j_pos[6] = {0.0};
static float j_vel[6] = {0.0};
static float j_eff[6] = {0.0};

static int controller_flag = 0;


class JSReader_01: public AbstractSubscription<JS_msg> {
public:
    JSReader_01(InterfacePtr interface): AbstractSubscription<JS_msg>(
      interface,
      AGENT_JS_SUB_PORT
    ) {};
    void handler(const reg_udral_physics_kinematics_rotation_Planar_0_1& js_read, CanardRxTransfer* transfer) override {
        j_pos[transfer->metadata.remote_node_id-1] = js_read.angular_position.radian;
        j_vel[transfer->metadata.remote_node_id-1] = js_read.angular_velocity.radian_per_second;
        j_eff[transfer->metadata.remote_node_id-1] = js_read.angular_acceleration.radian_per_second_per_second;
    }
};
JSReader_01 * JS_reader_01;

static CanardTransferID int_transfer_id = 0;

void send_JS(CanardNodeID node_id, float pos, float vel, float eff) {
	int_transfer_id++;
	reg_udral_physics_kinematics_rotation_Planar_0_1 js_msg =
	{
			.angular_position = pos,
			.angular_velocity = vel,
			.angular_acceleration = eff
	};
    cy_interface->send_msg<JS_msg>(
		&js_msg,
		js_sub_port_id[node_id-1],
		&int_transfer_id
	);
}





























void string_cb(const geometry_msgs::msg::Point &p)
{
  std::cout<<"x: "<< p.x<<"  y: "<< p.y<<std::endl;
  std::cout<<"j1_pos: "<< j_pos[0]<<std::endl;
  controller_flag = 1;
};

void controller()
{
  std::cout<<"Controller called"<<std::endl;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto sub = node->create_subscription<geometry_msgs::msg::Point>("/ball_center", 10, string_cb);

  cy_interface = CyphalInterface::create_heap<LinuxCAN, O1Allocator>(100, "can0", 1000, utilities); //Node ID, transport, queue_len, utilities
  JS_reader_01 = new JSReader_01(cy_interface);


  RCLCPP_INFO(node->get_logger(), "Ball control node started");


  while(rclcpp::ok())
  {
    
    rclcpp::spin_some(node);
    cy_interface->loop();
    if(controller_flag)
    {
      controller();
      controller_flag = 0;
    }

  }

  rclcpp::shutdown();

  return 0;
}

