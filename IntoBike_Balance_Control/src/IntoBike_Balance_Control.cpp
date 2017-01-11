/*
************************************************************************
* 作者:  IntoRobot Team 
* 版本:  V1.0.0
* 版权：Copyright (c) 2016 Shenzhen MOLMC Technology Co., Ltd..
* 日期:  Dec. 21 2016
*
* The MIT License (MIT)
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*
* 描述： IntoBike: 采用PID进行平衡控制
*/
#include "IntoBike_Balance_Control.h"
void BalanceControl::begin()
{
	begin(0.01);
}
void BalanceControl::begin(float sampling_time)
{
	dt_ = sampling_time;
	wheels_speed_.left_  = 0.0f;
	wheels_speed_.right_ = 0.0f;
	control_params_.bc_kp_ = 300;
	control_params_.bc_kd_ = 0.0;
	control_params_.vc_kp_ = 1.0;
	control_params_.vc_ki_ = 0.01;	//control_params_.vc_kp_ / 100.0;
	control_params_.max_magnitude_ = MAX_MAGNITUDE;
	control_params_.max_accel_ = MAX_ACCELERATION;
	velocity_ = 0;
	vc_iterm_ = 0.0f;
}
void BalanceControl::restart(void)
{
	wheels_speed_.left_  = 0.0f;
	wheels_speed_.right_ = 0.0f;
	velocity_ = 0;
	vc_iterm_ = 0.0f;
}
void BalanceControl::setParams(BalanceControlParams params)
{
	control_params_ = params;
}
DisplayDebugInfo BalanceControl::getDebugInfo(void)
{
	return debug_info_;
}
void BalanceControl::setMoveDirection(s8 direction)
{
	move_direction_ = direction;
}
void BalanceControl::setRotateDirection(s8 direction)
{
	rotate_direction_ = direction;
}
WheelsSpeed BalanceControl::doControl(float pitch_cur, float pitch_rate_cur, WheelsSpeed encoder, bool stop_flag)
{
	float balance_pwm, velocity_pwm, rotation_pwm;
	WheelsSpeed speed;
	if(stop_flag == true)
	    vc_iterm_ = 0;
	
	balance_pwm   = balanceController(pitch_cur, pitch_rate_cur);
	velocity_pwm  = velocityController(encoder);
	rotation_pwm  = rotationController(encoder);
	speed.left_   = balance_pwm + velocity_pwm + rotation_pwm;
	speed.right_  = balance_pwm + velocity_pwm - rotation_pwm;
	speed         = boundControlOutput(speed);
	wheels_speed_ = boundWheelSpeedAccel(speed, encoder, pitch_ref_ - pitch_cur);
	//debug
	debug_info_.control_params_        = control_params_;
	debug_info_.pitch_                 = pitch_cur;
	debug_info_.pitch_rate_            = pitch_rate_cur;
	debug_info_.wheels_command_speed_  = wheels_speed_;
	debug_info_.vc_iterm_              = vc_iterm_;
	debug_info_.velocity_pwm_          = velocity_pwm;
	debug_info_.balance_pwm_           = balance_pwm;
	debug_info_.rotation_pwm_          = rotation_pwm;
	return wheels_speed_;
}
//PD balance control: consider only pitch and pitch_rate
float BalanceControl::balanceController(float pitch_cur, float pitch_rate_cur)
{
	float pitch_error, pitch_rate_error;
	float pwm_out;
	//control_params_.bc_kd_ = 0.0f;
	pitch_error      = pitch_cur - pitch_ref_;
	pitch_rate_error = pitch_rate_cur;
	pwm_out = control_params_.bc_kp_ * pitch_error + control_params_.bc_kd_ * pitch_rate_error;
	return pwm_out;
}
//PI Velocity Control
float  BalanceControl::velocityController(WheelsSpeed encoder)
{
	float velocity_average;
	float result;
	control_params_.vc_ki_ = control_params_.vc_kp_ / 100.0;
	//debug
	//control_params_.vc_kp_  = 0.0f;
	//control_params_.vc_ki_  = 0.0f;
	//vc_iterm_ = 0.0f;
	//////
	velocity_average = meanFilter(encoder.left_ / 13000.0 * 5000.0, encoder.right_  / 13000.0 * 5000.0);
	velocity_ *= 0.7;
	velocity_ += velocity_average * 0.3;
	vc_iterm_ += velocity_;
	if(move_direction_ == 1)  //forward
	{
		//velocity_ += 1700;
		vc_iterm_ += 10000;
	}
	else if(move_direction_ == -1)  //backward
	{
		//velocity_ -= 1700;
		vc_iterm_ -= 10000;
	}
	if(vc_iterm_ >  60000)  	vc_iterm_ = 60000;
	if(vc_iterm_ < -60000)	    vc_iterm_ = -60000;
	result = velocity_ * control_params_.vc_kp_ + vc_iterm_ * control_params_.vc_ki_;
	return result;
}
//rotation control
float BalanceControl::rotationController(WheelsSpeed encoder)
{
	float rotation_increase_;
	if(rotation_count_ == 0 ) rotation_count_++;
	if(rotate_direction_ == 1 || rotate_direction_ == -1)
	{
		if(rotation_count_ == 1)	rotation_encoder_init_ = abs(encoder.left_ + encoder.right_);
		rotation_increase_ = 1000 / rotation_encoder_init_;
		if(rotation_increase_ < 6)  rotation_increase_ = 6;
		if(rotation_increase_ > 50) rotation_increase_ = 50;
	}
	else
	{
		rotation_increase_=6;
		rotation_count_=0;
		rotation_encoder_init_=0;
	}
	if(rotate_direction_ == 1)	            rotation_value_ += rotation_increase_;
	else if(rotate_direction_ == -1)	    rotation_value_ -= rotation_increase_;
	else rotation_value_ = 0;
	if(rotation_value_ > 1800)  rotation_value_ = 1800;
	if(rotation_value_ < -1800) rotation_value_ = -1800;
	return rotation_value_;
}
//Bound acceleration
WheelsSpeed BalanceControl::boundWheelSpeedAccel(WheelsSpeed speed, WheelsSpeed encoder, float error)
{
	float accel;
	float threshold = control_params_.max_accel_;

	accel = speed.left_ - encoder.left_;
	if(accel > threshold)  speed.left_ = encoder.left_ + threshold;
	else if(accel < -threshold)  speed.left_ = encoder.left_ - threshold;
	
	accel = speed.right_ - encoder.right_;
	if(accel > threshold) speed.right_ = encoder.right_ + threshold;
	else if(accel < -threshold) speed.right_ = encoder.right_ - threshold;
	return speed;
}
//Bound control output
WheelsSpeed BalanceControl::boundControlOutput(WheelsSpeed speed)
{
	if(speed.right_ > control_params_.max_magnitude_) speed.right_ = control_params_.max_magnitude_;
	else if(speed.right_ < - control_params_.max_magnitude_) speed.right_ = -control_params_.max_magnitude_;
	if(speed.left_ > control_params_.max_magnitude_) speed.left_ = control_params_.max_magnitude_;
	else if(speed.left_ < - control_params_.max_magnitude_) speed.left_ = -control_params_.max_magnitude_;
	return linearizeControlOutput(speed);
}
//Bound control output
WheelsSpeed BalanceControl::linearizeControlOutput(WheelsSpeed speed)
{
	speed.right_ = (speed.right_ / control_params_.max_magnitude_) * 13000.0;
	speed.left_ =  (speed.left_ / control_params_.max_magnitude_) * 13000.0;
	return speed;
}
//control_filter
float BalanceControl::meanFilter(int16_t left,int16_t right)
{
	uint8_t i;
	float sum = 0.0f;
	float result = 0.0f;
	for (i = 1 ; i<SPEED_HISTORY_NUM; i++)
	{
		speed_buffer_[i - 1] = speed_buffer_[i];
	}
	speed_buffer_[SPEED_HISTORY_NUM - 1] = (left + right) / 2.0;
	for (i = 0 ; i < SPEED_HISTORY_NUM; i++)
	{
		sum += speed_buffer_[i];
	}
	result = sum / SPEED_HISTORY_NUM;
	return result;
}