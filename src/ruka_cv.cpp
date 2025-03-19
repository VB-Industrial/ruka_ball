#include <cstdio>
#include <string>
#include <vector>
#include <list>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <iomanip> 
#include <numeric>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

static int ball_x = 0;
static int ball_y = 0;

static float plat_x = 0.0;
static float plat_y = 0.0;

static float prev_x = 0.0;
static float prev_y = 0.0;

static float V_ball_x = 0.0;
static float V_ball_y = 0.0;

static std::list<int> arr_of_div_x;
static std::list<int> arr_of_div_y;


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



void cv_cb(const geometry_msgs::msg::Point &p)
{
  ball_y = p.x;
  ball_x = p.y;
  plat_x = j_pos[5];
  plat_y = j_pos[4];
  controller_flag = 1;
};

void controller(int x, int y, float plat_x, float plat_y)
{
//max X = 600, min X = 0
//max Y = 600, min Y = 0
// Левая тройка векторов х - перпендикулярен руке, y - вдоль руки
// X - управляется 5м джоинтом, поворот 5-го джоинта положительный по правилу правой руки если Z ориентировать налево смотря на руку
// Y - управляется 4м джоинтом, поворот 6-го джоинта положительный по правилу правой руки если Z ориентировать наружу смотря на руку


  float Kp = 0.0002;
  float Kd = 0.1;
  float Ki_x = 0.000002;
  float Ki_y = 0.000002;
  int Ti = 20;

  int X_zero = 10;
  int Y_zero = 80;
  int X_lim = 450;
  int Y_lim = 540;
  int X_center = 250;
  int Y_center = 450;

  V_ball_x = x - prev_x;
  V_ball_y = y - prev_y;

  std::cout<<"x: "<<x<<"  y: "<<y<<std::endl;
  std::cout<<"V_x: "<<V_ball_x<<"  V_y: "<<V_ball_y<<std::endl;
  
  arr_of_div_x.push_back(x - X_center);
  if (arr_of_div_x.size() > Ti)
  {
    arr_of_div_x.pop_front();
  }

  arr_of_div_y.push_back(y - Y_center);
  if (arr_of_div_y.size() > Ti)
  {
    arr_of_div_y.pop_front();
  } 

  int I_of_div_x = 0;
  int I_of_div_y = 0;

  I_of_div_x = std::accumulate(arr_of_div_x.begin(), arr_of_div_x.end(),0);
  I_of_div_y = std::accumulate(arr_of_div_y.begin(), arr_of_div_y.end(),0);
  std::cout<<"I_of_div_x: "<<I_of_div_x<<"  I_of_div_y: "<<I_of_div_y<<std::endl;
  float U_x = -(Kp*(x - X_center) + Kd*V_ball_x + I_of_div_x*Ki_x); //тут "-" т.к. направление угловой скорости мотора - отрицательное по оси Y платформы
  float U_y = Kp*(y - Y_center) + Kd*V_ball_y + I_of_div_y*Ki_y;

  //Робастные ограничения
  if (U_x > 0.5)
  {
    U_x = 0.5;
  }
  else if(U_x < -0.5)
  {
    U_x = -0.5;
  }

  if (U_y > 0.5)
  {
    U_y = 0.5;
  }
  else if(U_y < -0.5)
  {
    U_y = -0.5;
  }
  // off


  std::cout<<"U_x: "<< U_x<<"  U_y: "<< U_y<<std::endl;
  send_JS(4, U_x, 0.0, 0.0);
  send_JS(5, U_y, 0.0, 0.0);
  prev_x = x;
  prev_y = y;
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("my_node");
  auto sub = node->create_subscription<geometry_msgs::msg::Point>("/ball_center", 10, cv_cb);
  auto twist_pub = node->create_publisher<geometry_msgs::msg::Twist>("/ball_twist", 10);
  auto js_pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_state", 10);

  geometry_msgs::msg::Twist ball_twist;
  sensor_msgs::msg::JointState js;



  cy_interface = CyphalInterface::create_heap<LinuxCAN, O1Allocator>(100, "can0", 1000, utilities); //Node ID, transport, queue_len, utilities
  JS_reader_01 = new JSReader_01(cy_interface);


  RCLCPP_INFO(node->get_logger(), "Ball control node started");


  while(rclcpp::ok())
  {
    rclcpp::spin_some(node);
    cy_interface->loop();
    if(controller_flag)
    {
      controller(ball_x, ball_y, plat_x, plat_y);
      controller_flag = 0;

      js.name = {"4","5"};
      js.position = {j_pos[3], j_pos[4]};
      js.velocity = {0.0,0.0};
      js.effort = {0.0,0.0};

      ball_twist.linear.x = V_ball_x;
      ball_twist.linear.y = V_ball_y;

      twist_pub->publish(ball_twist);
      js_pub->publish(js);
    }

  }

  rclcpp::shutdown();

  return 0;
}