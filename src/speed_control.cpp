#include "header.h"


class JoyControl : public ParamServer
{
public:
	ros::Subscriber subJoy; //订阅joy数据
	ros::Publisher pubArmControl;  //发布joint控制程序

	double axes0, axes1, axes2, axes3, axes4, axes5, axes6, axes7;
	bool buttons0, buttons1, buttons2, buttons3, buttons4, buttons5;
	bool buttons6, buttons7, buttons8, buttons9, buttons10;

	JoyControl()
	{
		subJoy = nh.subscribe("joy", 1000, &JoyControl::JoyCallback, this, ros::TransportHints().tcpNoDelay());
		pubArmControl = nh.advertise<can_msgs::Frame>("/arm_control_send_messages", 10);

		allocateMemory();
	}

	void allocateMemory()
	{
		axes0 = axes1 = axes2 = axes3 = axes4 = axes5 =axes6 = axes7 = 0.0;
		buttons0 = buttons1 = buttons2 = buttons3 = buttons4 = buttons5 = false;
		buttons6 = buttons7 = buttons8 = buttons9 = buttons10 = false;
	}

	void JoyCallback(const sensor_msgs::Joy::ConstPtr& joyMsg)
	{
		axes0 = joyMsg->axes[0];  
		axes1 = joyMsg->axes[1];
		axes2 = joyMsg->axes[2];  // LT 默认1 -1~1
		axes3 = joyMsg->axes[3];
		axes4 = joyMsg->axes[4];
		axes5 = joyMsg->axes[5];  // RT 默认1 -1~1
		axes6 = joyMsg->axes[6];
		axes7 = joyMsg->axes[7];
		buttons0 = joyMsg->buttons[0];  // A  默认0 0/1
		buttons1 = joyMsg->buttons[1];  // B  默认0 0/1
		buttons2 = joyMsg->buttons[2];  // X  默认0 0/1
		buttons3 = joyMsg->buttons[3];  // Y  默认0 0/1
		buttons4 = joyMsg->buttons[4];  // LB 默认0 0/1
		buttons5 = joyMsg->buttons[5];  // RB 默认0 0/1
		buttons6 = joyMsg->buttons[6];
		buttons7 = joyMsg->buttons[7];
		buttons8 = joyMsg->buttons[8];
		buttons9 = joyMsg->buttons[9];  // LR  默认0 0/1
		buttons10 = joyMsg->buttons[10];  // RR  默认0 0/1

	}

	void ArmControl()
	{
		ros::Rate rate(2);
		while (ros::ok())
		{
			rate.sleep();
			
			ArmControl_base1();
		}
	}
	void ArmControl_base1()
	{
		can_msgs::Frame can_control_arm;
		can_control_arm.header.frame_id = "canFrame";
		can_control_arm.header.stamp = ros::Time::now();
		can_control_arm.id = 0x011;
		can_control_arm.is_rtr = false;
		can_control_arm.is_extended = false;
		can_control_arm.is_error = false;
		can_control_arm.dlc = 8;

		double L_speed_normal = (1 - axes2) / 2;
		double R_speed_normal = (1 - axes5) / 2;
		int L_speed_dec = L_speed_normal * 65535;
		int R_speed_dec = R_speed_normal * 65535;

		unsigned char L_speed_byte[2] = {0}; // 创建16进制速度值
		unsigned char R_speed_byte[2] = {0}; 

		// 将整数速度转换为16进制
    	L_speed_byte[0] = (L_speed_dec >> 8) & 0xFF; // 获取高8位
    	L_speed_byte[1] = L_speed_dec & 0xFF; // 获取低8位

    	R_speed_byte[0] = (R_speed_dec >> 8) & 0xFF; // 获取高8位
    	R_speed_byte[1] = R_speed_dec & 0xFF; // 获取低8位

		can_control_arm.data[0] = 0x01;
		can_control_arm.data[7] = 0x00;
		can_control_arm.data[2] = 0x20;
		can_control_arm.data[3] = 0x00;
		can_control_arm.data[4] = 0x00;

		if (buttons1)                      //使用“B”按键控制
		{
			if (L_speed_dec != 0)
			{
				can_control_arm.data[1] = 0x01; // 逆时针转动
				can_control_arm.data[5] = L_speed_byte[0];
				can_control_arm.data[6] = L_speed_byte[1];
			}
			else
			{
				can_control_arm.data[1] = 0x00; // 顺时针转动
				can_control_arm.data[5] = R_speed_byte[0];
				can_control_arm.data[6] = R_speed_byte[1];
			}

		}
		else
		{
			can_control_arm.data[1] = 0x01; // 逆时针转动
			can_control_arm.data[5] = 0x00;
			can_control_arm.data[6] = 0x00;
		}
		pubArmControl.publish(can_control_arm);
		
	}


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "joy_control");

	JoyControl JC;

	ROS_INFO("\033[1;32m----> JOY CONTROL START.\033[0M");

	std::thread ArmControlThread(&JoyControl::ArmControl, &JC); //机械臂控制线程

	ros::spin();

	ArmControlThread.join();

	return 0;
}